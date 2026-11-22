"""Exercise the actual page/API with a temporary YAML; all robot traffic is mocked."""
import asyncio
import copy
import functools
import http.server
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
from unittest.mock import patch
from urllib.parse import urlparse

from playwright.async_api import async_playwright
from fastapi.testclient import TestClient
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
import agent_server as server
from costmap_settings import CostmapSettingsStore


async def main():
    web_server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(ROOT / 'frontend')))
    threading.Thread(target=web_server.serve_forever, daemon=True).start()
    tasks, errors, posts, commands = [], [], [], []
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'nav2_params.yaml'
        path.write_bytes((ROOT / 'backend/config/nav2_params.yaml').read_bytes())
        store = CostmapSettingsStore(path)
        initial = store.read()['settings']
        class Reader:
            def snapshot(self):
                return {name: {'available': True, 'values': {
                    **copy.deepcopy(initial[name]), 'footprint': initial['footprint'],
                    'footprint_padding': initial['footprint_padding']}} for name in ('global', 'local')}
        node = server.JetAutoCockpitNode()
        node.costmap_parameters = Reader()
        health = {'control': {}, 'capabilities': {}, 'nav_state': {}, 'control_ready': False,
                  'estop_engaged': False, 'hardware': {}, 'launches': []}
        with patch.object(server, 'cockpit_node', node), patch.object(server, 'costmap_settings_store', store):
            client = TestClient(server.app)
            try:
                async with async_playwright() as pw:
                    executable = os.environ.get('COCKPIT_BROWSER_EXE')
                    browser = await pw.chromium.launch(headless=True, **({'executable_path': executable} if executable else {}))
                    page = await browser.new_page(viewport={'width': 1600, 'height': 1100})
                    page.set_default_timeout(10000)
                    page.on('pageerror', lambda e: errors.append(str(e)))
                    async def route_http(route):
                        u = urlparse(route.request.url)
                        if u.hostname == '127.0.0.1' and u.port == web_server.server_port:
                            await route.continue_()
                            return
                        if u.port == 8000:
                            headers = {'Access-Control-Allow-Origin': '*', 'Access-Control-Allow-Headers': 'Content-Type',
                                       'Access-Control-Allow-Methods': 'GET,POST,OPTIONS'}
                            if route.request.method == 'OPTIONS':
                                await route.fulfill(status=204, headers=headers)
                                return
                            if u.path == '/api/navigation/costmap-settings':
                                if route.request.method == 'POST':
                                    posts.append(u.path)
                                    response = client.post(u.path, json=route.request.post_data_json)
                                else:
                                    response = client.get(u.path)
                                await route.fulfill(status=response.status_code, json=response.json(), headers=headers)
                                return
                            assert route.request.method == 'GET', 'Unexpected mutation ' + u.path
                            await route.fulfill(json={'maps': [], 'saved_map': {}}, headers=headers)
                            return
                        if os.environ.get('COCKPIT_VISUAL_CDN') == '1' and u.hostname == 'cdn.tailwindcss.com':
                            await route.continue_()
                            return
                        await route.abort()
                    async def route_ws(ws):
                        if ws.url.endswith('/ws/telemetry'):
                            async def feed():
                                while True:
                                    ws.send(json.dumps(health))
                                    await asyncio.sleep(.2)
                            tasks.append(asyncio.create_task(feed()))
                        elif ws.url.endswith('/ws/control'):
                            ws.send(json.dumps({'type': 'ready'}))
                            ws.on_message(lambda data: commands.append(json.loads(data)))
                        else:
                            ws.on_message(lambda data: None)
                    await page.route('**/*', route_http)
                    await page.route_web_socket('**/*', route_ws)
                    await page.goto('http://127.0.0.1:' + str(web_server.server_port) + '/jetauto_pro_cockpit.html', wait_until='domcontentloaded')
                    await page.locator('#costmapSettingsPanel summary').click()
                    await page.wait_for_function("!document.querySelector('#costmapSettingsInputs').disabled")
                    assert await page.locator('#cm-global-cost_scaling_factor').input_value() == '3'
                    assert '运行：3' in await page.locator('#cm-live-global-cost_scaling_factor').inner_text()
                    assert await page.locator('#btnSaveCostmapSettings').is_disabled()

                    # Server geometry rejection must preserve input and the saved file.
                    before = path.read_bytes()
                    await page.locator('#cm-local-inflation_radius').fill('0.1')
                    await page.locator('#btnSaveCostmapSettings').click()
                    await page.wait_for_function("document.querySelector('#costmapSettingsStatus').textContent.includes('内切半径')")
                    assert path.read_bytes() == before
                    assert await page.locator('#cm-local-inflation_radius').input_value() == '0.1'
                    await page.locator('#btnResetCostmapSettings').click()
                    assert await page.locator('#cm-local-inflation_radius').input_value() == '0.45'

                    # A runtime refresh while editing must leave draft inputs untouched.
                    await page.locator('#cm-global-cost_scaling_factor').fill('5')
                    await page.locator('#cm-local-cost_scaling_factor').fill('6')
                    await page.locator('#costmapSettingsPanel summary').click()
                    await page.locator('#costmapSettingsPanel summary').click()
                    await page.wait_for_function("!document.querySelector('#costmapSettingsInputs').disabled")
                    assert await page.locator('#cm-global-cost_scaling_factor').input_value() == '5'
                    await page.locator('#btnSaveCostmapSettings').click()
                    await page.wait_for_function("document.querySelector('#costmapSettingsStatus').textContent.includes('已保存。')")
                    saved = store.read()['settings']
                    assert saved['global']['cost_scaling_factor'] == 5 and saved['local']['cost_scaling_factor'] == 6
                    assert '运行：3' in await page.locator('#cm-live-global-cost_scaling_factor').inner_text()
                    assert '2 项不同' in await page.locator('#costmapSettingsStatus').inner_text()

                    # An external edit cannot silently be overwritten by an older page.
                    path.write_bytes(path.read_bytes() + b'\n# concurrent edit\n')
                    await page.locator('#cm-padding').fill('0.01')
                    await page.locator('#btnSaveCostmapSettings').click()
                    await page.wait_for_function("document.querySelector('#costmapSettingsStatus').textContent.includes('配置已被其他页面')")
                    assert await page.locator('#cm-padding').input_value() == '0.01'
                    await page.locator('#btnReadCostmapSettings').click()
                    await page.wait_for_function("document.querySelector('#cm-padding').value==='0.02'")
                    screenshot = os.environ.get('COCKPIT_SETTINGS_SCREENSHOT')
                    if screenshot:
                        await page.locator('#costmapSettingsPanel').evaluate("node => node.scrollIntoView({block:'center'})")
                        await page.locator('#costmapSettingsPanel').screenshot(path=screenshot)
                    await page.locator('#btnModeSim').click()
                    assert await page.locator('#btnSaveCostmapSettings').is_disabled()
                    assert '运行：未知' in await page.locator('#cm-live-global-cost_scaling_factor').inner_text()
                    assert not errors, errors
                    assert len(posts) == 3 and all(p == '/api/navigation/costmap-settings' for p in posts), posts
                    assert not [v for v in commands if any(v.get(k) for k in ('linear_x', 'linear_y', 'angular_z'))], commands
                    await browser.close()
                    print('Costmap settings browser PASS: persisted independent fields, runtime/draft separation, geometry rejection, conflict, disconnect; no launch or movement.')
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                client.close()
    web_server.shutdown()
    web_server.server_close()


if __name__ == '__main__':
    asyncio.run(main())
