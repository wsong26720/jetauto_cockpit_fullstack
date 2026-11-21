import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import MagicMock,patch
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from fastapi import HTTPException
from saved_maps import SavedMapManager
import saved_map_worker


class SavedMapsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        (self.root/'map_501.yaml').write_text('image: map_501.pgm\n',encoding='utf-8')
        self.node=S(get_publishers_info_by_topic=lambda topic:[])
        self.manager=SavedMapManager(self.root,'source /fake/ros_env.sh',lambda:self.node)
    def tearDown(self):self.temp.cleanup()

    def test_catalog_and_invalid_names(self):
        self.assertEqual([m['name'] for m in self.manager.catalog()['maps']],['map_501'])
        for name in ['../map_501','/etc/passwd','map_501;touch x','map_501.yaml']:
            with self.assertRaises(HTTPException) as ctx:self.manager.path(name)
            self.assertEqual(ctx.exception.status_code,422)
        with self.assertRaises(HTTPException) as ctx:self.manager.path('missing')
        self.assertEqual(ctx.exception.status_code,404)

    def test_existing_slam_map_is_never_overridden(self):
        self.node.get_publishers_info_by_topic=lambda topic:[S(node_name='slam_toolbox')]
        with patch('saved_maps.subprocess.Popen') as popen:
            with self.assertRaises(HTTPException) as ctx:self.manager.load('map_501')
        self.assertEqual(ctx.exception.status_code,409);popen.assert_not_called()

    def test_success_requires_ready_marker_for_the_owned_node(self):
        proc=MagicMock();proc.poll.return_value=None
        def reader(p,ready):
            with self.manager.lock:self.manager.active=True
            ready.set()
        with patch('saved_maps.subprocess.Popen',return_value=proc) as popen,patch.object(self.manager,'_read',side_effect=reader):
            result=self.manager.load('map_501')
        self.assertTrue(result['ok']);self.assertEqual(result['state'],'active')
        command=popen.call_args.args[0][-1]
        self.assertIn('saved_map_worker.py',command)
        self.assertNotIn('navigation.launch',command);self.assertNotIn('controller.launch',command)
        self.assertTrue(self.manager.status()['active'])
        # A line emitted by another process must not activate the map.
        self.manager.active=False
        wrong=S(stdout=io.BytesIO(b'COCKPIT_MAP_READY {"node":"another_node"}\n'))
        self.manager.proc=wrong
        self.manager._read(wrong,threading.Event())
        self.assertFalse(self.manager.active)

    def test_worker_failure_is_reported_and_owned_process_cleaned_up(self):
        proc=MagicMock();proc.poll.return_value=1
        def failed(p,ready):
            self.manager.error='地图图像不存在';ready.set()
        with patch('saved_maps.subprocess.Popen',return_value=proc),patch.object(self.manager,'_read',side_effect=failed), \
             patch.object(self.manager,'_stop_process') as stop:
            with self.assertRaises(HTTPException) as ctx:self.manager.load('map_501')
        self.assertEqual(ctx.exception.status_code,503)
        self.assertIn('地图图像不存在',ctx.exception.detail)
        stop.assert_any_call(proc)
        self.assertFalse(self.manager.busy);self.assertFalse(self.manager.active)

    def test_worker_configures_then_activates_only_the_unique_map_server(self):
        ros=types.ModuleType('rclpy');node=MagicMock();client=MagicMock();ros.create_node=MagicMock(return_value=node)
        node.create_client.return_value=client;client.wait_for_service.return_value=True
        ids=[]
        def call(request):
            ids.append(request.transition.id)
            return S(done=lambda:True,result=lambda:S(success=True))
        client.call_async.side_effect=call
        ros.init=MagicMock();ros.ok=lambda:True;ros.shutdown=MagicMock();ros.spin_until_future_complete=MagicMock()
        msgs=types.ModuleType('lifecycle_msgs.msg');msgs.Transition=S(TRANSITION_CONFIGURE=1,TRANSITION_ACTIVATE=3)
        srv=types.ModuleType('lifecycle_msgs.srv');srv.ChangeState=S(Request=lambda:S(transition=S(id=0)))
        process=MagicMock();process.poll.return_value=0;process.returncode=0
        output=io.StringIO()
        with patch.dict(sys.modules,{'rclpy':ros,'lifecycle_msgs':types.ModuleType('lifecycle_msgs'),
                                   'lifecycle_msgs.msg':msgs,'lifecycle_msgs.srv':srv}), \
             patch.object(sys,'argv',['worker','--yaml',str(self.root/'map_501.yaml'),'--node-name','cockpit_saved_map_test']), \
             patch.object(saved_map_worker.subprocess,'Popen',return_value=process) as popen,contextlib.redirect_stdout(output):
            result=saved_map_worker.main()
        self.assertEqual(result,0);self.assertEqual(ids,[1,3])
        self.assertIn('COCKPIT_MAP_READY',output.getvalue())
        self.assertEqual(node.create_client.call_args.args[1],'/cockpit_saved_map_test/change_state')
        args=popen.call_args.args[0]
        self.assertIn('__node:=cockpit_saved_map_test',args);self.assertIn('map:=/map',args)
        self.assertNotIn('/map_server/change_state',str(node.mock_calls))


if __name__=='__main__':unittest.main()
