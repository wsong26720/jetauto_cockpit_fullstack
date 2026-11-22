"""Configuration persistence, collision geometry checks and API isolation."""
import copy
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import MagicMock, patch

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from costmap_settings import (CostmapSettingsStore, SettingsError, RuntimeCostmapSettings,
                              PARAMETERS, differences, validate)
from fastapi.testclient import TestClient
import agent_server as server


class SettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'nav2_params.yaml'
        self.path.write_bytes((Path(__file__).resolve().parents[1] / 'backend/config/nav2_params.yaml').read_bytes())
        self.store = CostmapSettingsStore(self.path)
        self.report = self.store.read()
        self.settings = copy.deepcopy(self.report['settings'])

    def tearDown(self):
        self.temp.cleanup()

    def test_roundtrip_preserves_unrelated_params_and_synchronizes_geometry(self):
        before = yaml.safe_load(self.path.read_text())
        self.settings['global']['cost_scaling_factor'] = 5.
        self.settings['local']['resolution'] = .025
        self.settings['footprint_padding'] = .01
        report = self.store.save(self.settings, self.report['revision'])
        self.assertEqual(report['settings'], self.settings)
        after = yaml.safe_load(self.path.read_text())
        for key in set(before) - {'global_costmap', 'local_costmap'}:
            self.assertEqual(before[key], after[key])
        for name in ('global', 'local'):
            original = before[name + '_costmap'][name + '_costmap']['ros__parameters']
            actual = after[name + '_costmap'][name + '_costmap']['ros__parameters']
            for key in set(original) - {'footprint', 'footprint_padding', 'resolution', 'inflation_layer', 'obstacle_layer'}:
                self.assertEqual(original[key], actual[key])
            self.assertEqual(actual['footprint_padding'], .01)
            self.assertEqual(actual['obstacle_layer']['scan']['clearing'], True)
        self.assertNotEqual(report['revision'], self.report['revision'])
        self.assertTrue(self.path.with_suffix('.yaml.previous').exists())

    def test_stale_revision_does_not_overwrite_external_edit(self):
        self.path.write_text(self.path.read_text() + '\n# external edit\n')
        before = self.path.read_bytes()
        with self.assertRaises(SettingsError) as error:
            self.store.save(self.settings, self.report['revision'])
        self.assertEqual(error.exception.status, 409)
        self.assertEqual(self.path.read_bytes(), before)

    def test_invalid_values_never_touch_file(self):
        before = self.path.read_bytes()
        for value in (float('nan'), float('inf'), -.1, True, '5'):
            with self.subTest(value=value):
                self.settings['global']['cost_scaling_factor'] = value
                with self.assertRaises(SettingsError):
                    self.store.save(self.settings, self.report['revision'])
                self.assertEqual(self.path.read_bytes(), before)

    def test_polygon_must_be_ordered_convex_and_contain_origin(self):
        for points in (
            [[-.3, -.3], [.3, .3], [-.3, .3], [.3, -.3]],  # crossed
            [[.1, .1], [.3, .1], [.3, .3], [.1, .3]],    # displaced
            [[-.3, -.3], [.3, -.3], [.1, 0], [.3, .3], [-.3, .3]],
            [[-.3, -.3], [.3, -.3], [.3, -.3], [-.3, .3]],
        ):
            with self.subTest(points=points):
                self.settings['footprint'] = points
                with self.assertRaises(SettingsError):
                    validate(self.settings)

    def test_both_polygon_windings_are_valid(self):
        self.assertEqual(validate(self.settings), self.settings)
        self.settings['footprint'].reverse()
        self.assertEqual(validate(self.settings), self.settings)

    def test_radius_cannot_remove_inscribed_band(self):
        self.settings['local']['inflation_radius'] = .2
        with self.assertRaisesRegex(SettingsError, '内切半径'):
            validate(self.settings)

    def test_clearing_range_cannot_be_shorter_than_marking(self):
        self.settings['global']['raytrace_max_range'] = 5.
        with self.assertRaisesRegex(SettingsError, '清障距离'):
            validate(self.settings)

    def test_unknown_parameter_is_rejected(self):
        self.settings['global']['enabled'] = False
        with self.assertRaises(SettingsError):
            validate(self.settings)

    def test_failed_replace_leaves_original_file_complete(self):
        before = self.path.read_bytes()
        self.settings['local']['cost_scaling_factor'] = 6.
        with patch('costmap_settings.os.replace', side_effect=OSError('disk error')):
            with self.assertRaisesRegex(SettingsError, '保存失败'):
                self.store.save(self.settings, self.report['revision'])
        self.assertEqual(before, self.path.read_bytes())
        self.assertEqual(list(self.path.parent.glob('.costmap-*')), [])

    def test_runtime_unknown_and_changed_are_distinct(self):
        self.assertIsNone(differences(self.settings, {}))
        runtime = {name: {'available': True, 'values': {
            **self.settings[name], 'footprint': self.settings['footprint'],
            'footprint_padding': self.settings['footprint_padding']}} for name in ('global', 'local')}
        self.assertEqual(differences(self.settings, runtime), [])
        runtime['local']['values']['resolution'] = .025
        self.assertEqual(differences(self.settings, runtime), ['local.resolution'])

    def test_api_save_is_persistent_and_never_launches_or_publishes(self):
        node = server.JetAutoCockpitNode()
        with patch.object(server, 'cockpit_node', node), patch.object(server, 'costmap_settings_store', self.store), \
             patch.object(server.process_mgr, 'start') as launch, patch.object(node, '_publish_raw_twist') as motion:
            client = TestClient(server.app)  # Deliberately do not enter lifespan.
            try:
                report = client.get('/api/navigation/costmap-settings').json()
                self.assertIsNone(report['restart_required'])
                self.assertFalse(report['runtime']['global']['available'])
                self.settings['global']['cost_scaling_factor'] = 5.
                response = client.post('/api/navigation/costmap-settings', json={
                    'revision': report['revision'], 'settings': self.settings})
                self.assertEqual(response.status_code, 200)
                self.assertEqual(self.store.read()['settings'], self.settings)
                self.assertEqual(client.post('/api/navigation/costmap-settings', json={
                    'revision': report['revision'], 'settings': self.settings}).status_code, 409)
                self.settings['local']['inflation_radius'] = .1
                latest = client.get('/api/navigation/costmap-settings').json()
                response = client.post('/api/navigation/costmap-settings', json={
                    'revision': latest['revision'], 'settings': self.settings})
                self.assertEqual(response.status_code, 422)
                self.assertIn('内切半径', response.json()['detail'])
                launch.assert_not_called()
                motion.assert_not_called()
            finally:
                client.close()


class RuntimeTests(unittest.TestCase):
    def reader(self):
        r = RuntimeCostmapSettings.__new__(RuntimeCostmapSettings)
        r.clock = lambda: 100.
        r.lock = threading.RLock()
        r.rows = {}
        r.pending = {}
        r.clients = {name: S(service_is_ready=lambda: True) for name in ('global', 'local')}
        return r

    def test_parameters_expire_and_unavailable_service_is_not_applied(self):
        r = self.reader()
        r.rows['global'] = {'values': {'resolution': .05}, 'received': 99.}
        self.assertTrue(r.snapshot()['global']['available'])
        r.clock = lambda: 109.
        self.assertFalse(r.snapshot()['global']['available'])
        r.clock = lambda: 100.
        r.clients['global'] = S(service_is_ready=lambda: False)
        self.assertFalse(r.snapshot()['global']['available'])

    def test_missing_parameter_does_not_read_default_zero_as_real_value(self):
        r = self.reader()
        f = MagicMock()
        f.result.return_value = S(values=[S(type=0) for _ in PARAMETERS])
        r.pending['global'] = (f, 100.)
        r._reply('global', f)
        self.assertFalse(r.snapshot()['global']['available'])
        self.assertIn('未提供', r.snapshot()['global']['issue'])

    def test_complete_parameter_reply_maps_ros_names_to_form_values(self):
        r = self.reader()
        values = [S(type=4, string_value='[[0.25,0.22],[0.25,-0.22],[-0.25,-0.22],[-0.25,0.22]]')]
        values += [S(type=3, double_value=v) for v in (.02, .45, 3., .05, 6., 8.)]
        f = MagicMock()
        f.result.return_value = S(values=values)
        r.pending['global'] = (f, 100.)
        r._reply('global', f)
        report = r.snapshot()['global']
        self.assertTrue(report['available'])
        self.assertEqual(report['values']['cost_scaling_factor'], 3.)
        self.assertEqual(report['values']['footprint_padding'], .02)
        self.assertEqual(report['values']['raytrace_max_range'], 8.)


if __name__ == '__main__':
    unittest.main()
