"""Headless Chromium UI checks. Every robot/network service is mocked locally.

Install playwright; set COCKPIT_BROWSER_EXE to an existing Chrome/Edge executable,
or install the Playwright Chromium browser. No connection to a robot is made.
"""
import asyncio
import base64
import functools
import http.server
import json
import os
import threading
from pathlib import Path
from urllib.parse import urlparse
from playwright.async_api import async_playwright

ROOT = Path(__file__).resolve().parents[1]


async def main():
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0),
        functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(ROOT / 'frontend')))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    errors, calls, commands, sockets = [], [], [], []
    modules = [{'id': key, 'name': key, 'cmd': key, 'running': False, 'pid': None,
                'conflicts_with': [], 'recent_logs': [], 'managed': False, 'external': False}
               for key in ['controller', 'depth_camera', 'lidar', 'slam', 'navigation', 'web_video_server']]
    modules[2].update(running=True, pid=555, managed=True, uptime_sec=0)
    payload = {'control': {}, 'control_ready': False, 'estop_engaged': False,
        'ros_domain_id': '3', 'graph': {'nodes': ['gateway']}, 'capabilities': {'navigation': False},
        'hardware': {'cpu_percent': 12.3, 'gpu_percent': 4.5, 'cpu_temp_c': 50,
            'gpu_temp_c': None, 'ram_used_gb': 2.5, 'ram_total_gb': 15.3, 'ram_percent': 16.3,
            'disk_used_gb': 70, 'disk_total_gb': 115, 'disk_percent': 60.9,
            'battery_voltage_v': None, 'battery_sample_age_sec': None}, 'launches': modules}
    video_fail = False
    telemetry_blocked = False
    feed_tasks = []
    try:
        async with async_playwright() as pw:
            executable = os.environ.get('COCKPIT_BROWSER_EXE')
            browser = await pw.chromium.launch(headless=True, **({'executable_path': executable} if executable else {}))
            page = await browser.new_page(viewport={'width': 1600, 'height': 1100})
            page.set_default_timeout(10000)
            page.on('pageerror', lambda err: errors.append(str(err)))

            async def route_http(route):
                nonlocal video_fail
                url = urlparse(route.request.url)
                if url.hostname == '127.0.0.1' and url.port == server.server_port:
                    await route.continue_(); return
                if url.port == 8000:
                    body = route.request.post_data_json if route.request.method == 'POST' else None
                    calls.append((url.path, body))
                    if url.path == '/api/launches':
                        if body:
                            m = next(m for m in modules if m['id'] == body['module_id'])
                            m.update(running=body['action'] == 'start', pid=1234, managed=True)
                            payload['control_ready'] = modules[0]['running']
                            for ws in sockets: ws.send(json.dumps(payload))
                            answer = {'status': 'starting', 'pid': 1234}
                        else: answer = {'modules': modules}
                    elif url.path == '/api/estop':
                        payload['estop_engaged'] = 'engage=true' in url.query
                        answer = {'ok': True, 'estop_engaged': payload['estop_engaged']}
                        for ws in sockets: ws.send(json.dumps(payload))
                    else: answer = payload
                    await route.fulfill(json=answer, headers={'Access-Control-Allow-Origin': '*'}); return
                if url.port == 8080 and not video_fail:
                    # The installed legacy server does not URL-decode topic query values.
                    assert 'topic=/depth_cam/rgb/image_raw&' in route.request.url, route.request.url
                    assert '%2f' not in route.request.url.lower(), route.request.url
                    png = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jF1kAAAAASUVORK5CYII=')
                    await route.fulfill(content_type='image/png', body=png); return
                await route.abort()  # Includes CDNs: application JS must still initialize.

            async def route_ws(ws):
                if ws.url.endswith('/ws/control'):
                    ws.send(json.dumps({'type': 'ready', 'timeout_sec': 0.5}))
                    def receive(data):
                        commands.append(json.loads(data))
                        ws.send(json.dumps({'ok': True}))
                    ws.on_message(receive)
                elif ws.url.endswith('/ws/telemetry'):
                    sockets.append(ws)
                    ws.send(json.dumps(payload))
                    async def feed():
                        while not telemetry_blocked:
                            await asyncio.sleep(0.5)
                            if telemetry_blocked: return
                            modules[2]['uptime_sec'] += 0.5
                            try: ws.send(json.dumps(payload))
                            except Exception: return
                    feed_tasks.append(asyncio.create_task(feed()))
                else:
                    ws.on_message(lambda data: None)

            await page.route('**/*', route_http)
            await page.route_web_socket('**/*', route_ws)
            await page.goto(f'http://127.0.0.1:{server.server_port}/jetauto_pro_cockpit.html')
            await page.wait_for_function("document.querySelector('#badgeAgentServer').textContent.includes('控制未就绪')")
            assert await page.locator('#btnNavStart').is_disabled()
            assert await page.locator('#valCpuUsage').inner_text() == '12.3%'
            assert await page.locator('#valGpuTemp').inner_text() == '--'
            assert '暂无样本' in await page.locator('#valBatteryAge').inner_text()
            assert await page.locator('#cameraCanvas').evaluate("el => el.classList.contains('hidden')")
            await page.wait_for_function("document.querySelector('#badgeVideoServer').textContent.includes('已收到画面')")
            await page.wait_for_timeout(650)
            assert commands == [], 'Idle telemetry must not publish commands or spam estop rejections'
            button = page.locator('[data-launch-id="controller"]')
            await page.evaluate("window.__clickEvents=[]; ['mousedown','mouseup','click'].forEach(type=>document.addEventListener(type,e=>window.__clickEvents.push([type,e.target.tagName,e.target.dataset.launchId]),true))")
            await button.scroll_into_view_if_needed()
            await button.evaluate('el => { window.__originalLaunchButton = el; }')
            await button.click(force=True, delay=800)  # Telemetry changes while held.
            assert await button.evaluate('el => el === window.__originalLaunchButton')
            try:
                await page.wait_for_function("document.querySelector('#badgeAgentServer').textContent.includes('控制就绪')")
            except Exception:
                print('DEBUG', calls, errors, await page.locator('#launchLogConsole').inner_text(),
                      await button.evaluate("el => ({disabled:el.disabled, text:el.textContent,events:window.__clickEvents})"))
                raise
            assert any(path == '/api/launches' and body and body['action'] == 'start' for path, body in calls)
            await page.keyboard.press('Tab')  # no input fields should retain focus
            await page.locator('body').click(position={'x': 1, 'y': 1}, force=True)
            await page.keyboard.down('w')
            await page.wait_for_timeout(650)
            assert sum(c.get('linear_x', 0) > 0 for c in commands) >= 5, commands
            await page.keyboard.up('w')
            await page.wait_for_timeout(100)
            assert commands[-1] == {'linear_x': 0, 'linear_y': 0, 'angular_z': 0}
            await page.keyboard.down('q')
            await page.evaluate("window.dispatchEvent(new Event('blur'))")
            await page.wait_for_timeout(100)
            assert commands[-1]['angular_z'] == 0
            await page.keyboard.up('q')
            await page.locator('#btnSoftEstop').click(force=True)
            await page.wait_for_function("!document.querySelector('#btnReleaseEstop').classList.contains('hidden')")
            assert any(path == '/api/estop' for path, _ in calls)
            await page.locator('#btnReleaseEstop').click(force=True)
            await page.wait_for_timeout(100)
            payload['hardware']['battery_voltage_v'] = 12.1
            payload['hardware']['battery_sample_age_sec'] = 0.2
            for ws in sockets: ws.send(json.dumps(payload))
            await page.wait_for_function("document.querySelector('#valBatteryVolt').textContent === '12.10 V'")
            # Losing telemetry must discard commands and mark process state unknown.
            telemetry_blocked = True
            for ws in sockets: await ws.close()
            await page.wait_for_function("document.querySelector('#badgeAgentServer').textContent.includes('未连接')")
            assert await page.locator('[data-launch-id="controller"]').is_disabled()
            assert commands[-1] == {'linear_x': 0, 'linear_y': 0, 'angular_z': 0}
            video_fail = True
            await page.locator('#btnReloadStream').click(force=True)
            await page.wait_for_function("document.querySelector('#camStreamStateOverlay').textContent.includes('连接失败')")
            assert await page.locator('#cameraCanvas').evaluate("el => el.classList.contains('hidden')")
            await page.locator('#btnModeSim').click(force=True)
            before = len(calls)
            await page.locator('[data-launch-id="lidar"]').click(force=True)
            await page.wait_for_timeout(200)
            assert len(calls) == before
            assert not await page.locator('#btnNavStart').is_disabled()
            await page.locator('#btnModeLive').click(force=True)
            await page.wait_for_timeout(200)
            assert await page.locator('#btnNavStart').is_disabled()
            assert not errors, errors
            screenshot = os.environ.get('COCKPIT_TEST_SCREENSHOT')
            if screenshot: await page.screenshot(path=screenshot, full_page=True)
            await browser.close()
            print('Browser smoke OK: real API wiring, telemetry, command heartbeat, release/blur stop, estop, disconnect, video success/failure, sim/live isolation; zero page errors.')
    finally:
        for task in feed_tasks: task.cancel()
        await asyncio.gather(*feed_tasks, return_exceptions=True)
        server.shutdown()
        server.server_close()


if __name__ == '__main__': asyncio.run(main())
