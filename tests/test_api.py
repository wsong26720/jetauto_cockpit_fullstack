"""API contract checks with an in-memory publisher; no ROS launch or hardware."""
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import types
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from fastapi.testclient import TestClient
import agent_server as server


class ApiTests(unittest.TestCase):
    def test_precheck_explains_inscribed_goal_and_task_cannot_bypass_it(self):
        from test_navigation import BridgeTests
        bridge=BridgeTests().bridge();row,_=bridge.mapping_getter().costmaps.raw('global');row['grid']['data'][10]=253
        self.node.navigation=bridge
        body={'mode':'single','loop_count':1,'waypoints':[{'id':'blocked','name':'墙边','x':2.5,'y':2.5,'yaw_deg':0}]}
        response=self.client.post('/api/navigation/precheck',json=body)
        self.assertEqual(response.status_code,200);self.assertFalse(response.json()['valid'])
        self.assertEqual(response.json()['targets'][0]['code'],'INSCRIBED_OBSTACLE')
        self.assertIn('253',response.json()['targets'][0]['message'])
        response=self.client.post('/api/navigation/task',json=body)
        self.assertEqual(response.status_code,422);self.assertIn('253',response.json()['detail'])
        bridge.clients['single'].send_goal_async.assert_not_called()

    def test_stop_cleans_owned_children_after_launcher_has_exited(self):
        manager=server.ProcessManager();proc=MagicMock();proc.pid=32145;proc.poll.return_value=None
        proc.wait.side_effect=lambda **kwargs:setattr(proc.poll,'return_value',0)
        manager.processes['navigation']=proc;terminated=[False];calls=[]
        def send(group,sig):
            calls.append((group,sig))
            if sig==server.signal.SIGTERM:terminated[0]=True
            if sig==0 and terminated[0]:raise ProcessLookupError()
        with patch.object(server.os,'getpgid',return_value=32145,create=True),patch.object(server.os,'killpg',side_effect=send,create=True):
            self.assertEqual(manager.stop('navigation')['status'],'stopped')
        self.assertIn((32145,server.signal.SIGTERM),calls)
        self.assertTrue(all(group==32145 for group,_ in calls))

    def setUp(self):
        self.old_node = server.cockpit_node
        self.node = server.JetAutoCockpitNode()
        server.cockpit_node = self.node
        self.client = TestClient(server.app)  # no lifespan/ROS initialization

    def tearDown(self):
        self.client.close()
        server.cockpit_node = self.old_node

    def test_missing_ros_is_reported_and_motion_is_rejected(self):
        h = self.client.get('/api/health').json()
        self.assertFalse(h['control_ready'])
        self.assertFalse(h['ros_initialized'])
        self.assertFalse(h['capabilities']['navigation'])
        self.assertEqual(self.client.post('/api/cmd_vel', json={'client_id':'a', 'linear_x':0.1}).status_code, 503)
        self.assertEqual(self.client.post('/api/cmd_vel', json={'linear_x':0.1}).status_code, 422)

    def test_control_websocket_estop_and_disconnect(self):
        outputs = []
        self.node.control.publish = lambda *cmd: outputs.append(cmd)
        self.node.control.ready = lambda: True
        with self.client.websocket_connect('/ws/control') as a:
            self.assertEqual(a.receive_json()['type'], 'ready')
            a.send_json({'linear_x':0.1})
            self.assertTrue(a.receive_json()['ok'])
            with self.client.websocket_connect('/ws/control') as b:
                b.receive_json()
                b.send_json({'linear_x':0.1})
                self.assertEqual(b.receive_json()['status'], 409)
            self.assertEqual(outputs[-1], (0.1, 0, 0))
            self.assertEqual(self.client.post('/api/estop?engage=true').status_code, 200)
            a.send_json({'linear_x':0.1})
            self.assertEqual(a.receive_json()['status'], 423)
            self.client.post('/api/estop?engage=false')
            a.send_json({'linear_y':0.1})
            self.assertTrue(a.receive_json()['ok'])
        self.assertEqual(outputs[-1], (0, 0, 0))

    def test_map_validation_has_no_shell_side_effects(self):
        with patch.object(server.subprocess, 'run') as run:
            r = self.client.post('/api/map/save', json={'output_path':'/tmp/outside'})
            self.assertEqual(r.status_code, 422)
            run.assert_not_called()
        with patch.object(server.process_mgr, 'discover', return_value={k:[] for k in server.LAUNCH_MODULES}), patch.object(server.Path, 'exists', return_value=True):
            with patch.object(server.subprocess, 'Popen') as popen:
                r = self.client.post('/api/launches', json={'module_id':'navigation', 'action':'start', 'map_path':'x; touch /tmp/no'})
                self.assertEqual(r.status_code, 422)
                popen.assert_not_called()

    def test_external_process_is_visible_and_not_stopped(self):
        found = {k: [] for k in server.LAUNCH_MODULES}
        found['controller'] = [12345]
        with patch.object(server.process_mgr, 'discover', return_value=found):
            module = self.client.get('/api/launches').json()['modules'][0]
            self.assertTrue(module['external'])
            with patch.object(server.os, 'killpg', create=True) as kill:
                r = self.client.post('/api/launches', json={'module_id':'controller', 'action':'stop'})
                self.assertEqual(r.status_code, 409)
                kill.assert_not_called()

    def test_stopping_module_does_not_create_estop_latch(self):
        outputs = []
        self.node.control.publish = lambda *cmd: outputs.append(cmd)
        self.node.control.ready = lambda: True
        self.node.control.command_from('test', 0.1, 0, 0)
        with patch.object(server.process_mgr, 'stop', return_value={'status':'stopped'}):
            response = self.client.post('/api/launches', json={'module_id':'controller','action':'stop'})
        self.assertEqual(response.status_code, 200)
        self.assertIsNone(self.node.control.owner)
        self.assertFalse(self.node.estop_engaged)
        self.assertEqual(outputs[-1], (0, 0, 0))
        self.node.trigger_estop(True)
        with patch.object(server.process_mgr, 'stop', return_value={'status':'stopped'}):
            self.client.post('/api/launches', json={'module_id':'controller','action':'stop'})
        self.assertTrue(self.node.estop_engaged)  # Deliberate estop remains locked.

    def test_missing_device_has_explicit_error_without_launching(self):
        found = {k: [] for k in server.LAUNCH_MODULES}
        with patch.object(server.process_mgr, 'discover', return_value=found), patch.object(server.Path, 'exists', return_value=False):
            with patch.object(server.subprocess, 'Popen') as popen:
                response = self.client.post('/api/launches', json={'module_id':'controller','action':'start'})
                self.assertEqual(response.status_code, 503)
                self.assertIn('/dev/rrc', response.json()['detail'])
                popen.assert_not_called()

    def test_odom_subscriber_alone_does_not_make_chassis_ready(self):
        self.node._cmd_vel_pub = SimpleNamespace(get_subscription_count=lambda: 1)
        self.node._node = SimpleNamespace(count_publishers=lambda topic: 0)
        with patch.object(server.Path, 'exists', return_value=True):
            self.assertFalse(self.node.control_ready())
            self.node._node.count_publishers = lambda topic: 1
            self.node.battery_mv=12000;self.node.battery_ts=time.time()
            self.assertTrue(self.node.control_ready())
            self.node.battery_mv=9680
            self.assertFalse(self.node.control_ready())
            self.node.battery_mv=12000;self.node.battery_ts=time.time()-6
            self.assertFalse(self.node.control_ready())
            self.node.init_error = 'timer initialization failed'
            self.assertFalse(self.node.control_ready())

    def test_mapping_channel_reports_unavailable_without_affecting_control(self):
        with self.client.websocket_connect('/ws/mapping') as ws:
            message=ws.receive_json()
            self.assertFalse(message['available'])
        self.assertFalse(self.node.estop_engaged)
        self.assertIsNone(self.node.control.owner)

    def test_mapping_subscription_uses_a_separate_executor(self):
        control_node, map_node = MagicMock(), MagicMock()
        ros = types.ModuleType('rclpy')
        ros.init = MagicMock()
        ros.create_node = MagicMock(side_effect=[control_node,map_node])
        ros.spin = MagicMock()
        executors = types.ModuleType('rclpy.executors')
        executors.SingleThreadedExecutor = MagicMock()
        executors.ExternalShutdownException = RuntimeError
        with patch.dict(sys.modules, {'rclpy':ros,'rclpy.executors':executors}), \
             patch.object(server,'rclpy',ros,create=True), patch.object(server,'ROS2_AVAILABLE',True), \
             patch.object(server,'Twist',object,create=True), patch.object(server,'UInt16',object,create=True), \
             patch.object(server,'SignalHandlerOptions',SimpleNamespace(NO=0),create=True), \
             patch.object(server.threading,'Thread'), patch.object(server,'MappingFeed') as feed:
            self.node.start()
        feed.assert_called_once_with(map_node)
        executors.SingleThreadedExecutor.return_value.add_node.assert_called_once_with(map_node)
        control_node.create_timer.assert_called_once_with(0.05,self.node.motion.tick)
        self.assertIsNone(self.node.init_error)
        control_node.create_publisher.return_value.publish.assert_not_called()

    def test_mapping_dependency_failure_keeps_manual_gateway_initialized(self):
        control_node, map_node = MagicMock(), MagicMock()
        ros = types.ModuleType('rclpy')
        ros.init = MagicMock();ros.create_node = MagicMock(side_effect=[control_node,map_node]);ros.spin = MagicMock()
        executors = types.ModuleType('rclpy.executors')
        executors.SingleThreadedExecutor = MagicMock();executors.ExternalShutdownException = RuntimeError
        with patch.dict(sys.modules, {'rclpy':ros,'rclpy.executors':executors}), \
             patch.object(server,'rclpy',ros,create=True), patch.object(server,'ROS2_AVAILABLE',True), \
             patch.object(server,'Twist',object,create=True), patch.object(server,'UInt16',object,create=True), \
             patch.object(server,'SignalHandlerOptions',SimpleNamespace(NO=0),create=True), \
             patch.object(server.threading,'Thread'), patch.object(server,'MappingFeed',side_effect=ImportError('tf2_ros missing')):
            self.node.start()
        self.assertIsNone(self.node.init_error)
        self.assertIsNone(self.node.mapping)
        self.assertIn('tf2_ros',self.node.mapping_error)
        self.assertIs(self.node._node,control_node)
        map_node.destroy_node.assert_called_once()
        control_node.create_timer.assert_called_once()

    def test_saved_map_loading_does_not_start_navigation(self):
        with patch.object(server.saved_map_mgr,'load',return_value={'ok':True,'name':'map_501','state':'active'}) as load, \
             patch.object(server.process_mgr,'is_running',return_value=False),patch.object(server.process_mgr,'start') as start:
            response=self.client.post('/api/map/load',json={'name':'map_501'})
        self.assertEqual(response.status_code,200)
        load.assert_called_once_with('map_501');start.assert_not_called()
        self.assertFalse(self.node.estop_engaged)

    def test_slam_and_saved_map_sources_are_mutually_exclusive(self):
        with patch.object(server.saved_map_mgr,'status',return_value={'running':True}),patch.object(server.process_mgr,'start') as start:
            response=self.client.post('/api/launches',json={'module_id':'slam','action':'start'})
        self.assertEqual(response.status_code,409);start.assert_not_called()
        with patch.object(server.process_mgr,'is_running',return_value=True),patch.object(server.saved_map_mgr,'load') as load:
            response=self.client.post('/api/map/load',json={'name':'map_501'})
        self.assertEqual(response.status_code,409);load.assert_not_called()

    def test_incomplete_navigation_is_rejected_before_starting_any_drivers(self):
        package=types.ModuleType('ament_index_python.packages')
        def locate(name):
            if name=='nav2_amcl':raise LookupError('not installed')
            return '/fake/share/'+name
        package.get_package_share_directory=locate
        found={key:[] for key in server.LAUNCH_MODULES}
        with patch.dict(sys.modules,{'ament_index_python':types.ModuleType('ament_index_python'),
                                     'ament_index_python.packages':package}), \
             patch.object(server.Path,'exists',return_value=True), \
             patch.object(server.Path,'is_file',return_value=True), \
             patch.object(server.process_mgr,'discover',return_value=found),patch.object(server.subprocess,'Popen') as popen:
            response=self.client.post('/api/launches',json={'module_id':'navigation','action':'start','map_path':'map_501'})
        self.assertEqual(response.status_code,503)
        self.assertIn('nav2_amcl',response.json()['detail'])
        popen.assert_not_called()


if __name__ == '__main__': unittest.main()
