import sys,math,time,threading
from pathlib import Path
from types import SimpleNamespace as S
from unittest import TestCase,main
from unittest.mock import MagicMock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from control_safety import VelocityLease,MotionArbiter,ControlError
from navigation_bridge import NavigationBridge
from fastapi import HTTPException


class MotionTests(TestCase):
    def setUp(self):
        self.now=10;self.output=[];self.ready=True
        self.manual=VelocityLease(lambda *v:self.output.append(v),lambda:True,clock=lambda:self.now)
        self.gate=MotionArbiter(self.manual,lambda *v:self.output.append(v),lambda:True,clock=lambda:self.now)
        self.gate.nav_ready=lambda:self.ready

    def test_unowned_nav_commands_cannot_move_robot(self):
        self.gate.receive(1,1,5);self.gate.tick();self.assertEqual(self.output,[])
    def test_nav_limits_timeout_and_readiness_stop(self):
        self.gate.begin('a');self.gate.receive(10,-10,10);self.gate.tick()
        self.assertEqual(self.output[-1],(0.15,-0.1,0.4))
        self.now+=0.51;self.gate.tick();self.assertEqual(self.output[-1],(0,0,0))
        self.gate.receive(0.1,0,0);self.ready=False;self.gate.tick();self.assertEqual(self.output[-1],(0,0,0))
    def test_estop_and_manual_ownership(self):
        self.manual.command_from('manual',0.1,0,0)
        with self.assertRaises(ControlError):self.gate.begin('a')
        self.manual.disconnect('manual');self.gate.begin('a')
        self.gate.receive(0.1,0,0);self.manual.set_estop(True);self.gate.tick()
        self.assertEqual(self.output[-1],(0,0,0))
        self.gate.end('a');self.manual.set_estop(False);self.gate.tick()
        self.assertIsNone(self.gate.owner)
    def test_old_task_cannot_release_new_owner(self):
        self.gate.begin('a');self.gate.end('a');self.gate.begin('b');self.gate.end('a')
        self.assertEqual(self.gate.owner,'b')


class Future:
    def __init__(self,value=None):self.value=value;self.callbacks=[];self.complete=value is not None
    def done(self):return self.complete
    def result(self):return self.value
    def add_done_callback(self,callback):
        if self.complete:callback(self)
        else:self.callbacks.append(callback)
    def set(self,value):
        self.value=value;self.complete=True
        for cb in self.callbacks:cb(self)


def pose():return S(header=S(frame_id='',stamp=S(sec=0,nanosec=0)),pose=S(position=S(x=0.,y=0.),orientation=S(z=0.,w=1.)))


class BridgeTests(TestCase):
    def bridge(self,ready=True):
        b=NavigationBridge.__new__(NavigationBridge)
        b.lock=threading.RLock();b.confirmed=ready;b.initial_stamp=0;b.amcl=None
        b.ready_cache={'ready':ready,'reasons':[] if ready else ['请确认初始位姿'],'lifecycle':{}}
        b.ready_at=time.monotonic();b.task=None;b.thread=None;b.handle=None;b.goal_generation=0
        b.stop_event=threading.Event();b.stop_reason=None;b.unresolved=False
        b.phase='IDLE';b.code=0;b.index=0;b.loop=1;b.dwell_remaining=0;b.error=None;b.result_status=None
        b.metrics={'distance_remaining':0.,'navigation_time':0.,'estimated_time_remaining':0.,'number_of_recoveries':0}
        b.node=S(get_clock=lambda:S(now=lambda:S(to_msg=lambda:S(sec=123,nanosec=0))))
        b.PoseStamped=pose;b.actions={'single':S(Goal=lambda:S()),'through_poses':S(Goal=lambda:S())}
        grid={'width':4,'height':4,'resolution':1.,'data':[0]*16,'origin':{'position':{'x':0,'y':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}}
        b.mapping_getter=lambda:S(grid=grid)
        manual=VelocityLease(lambda *v:None,lambda:True)
        b.motion=MotionArbiter(manual,lambda *v:None,lambda:True)
        b.motion.nav_ready=lambda:True
        b.clients={'single':MagicMock(),'through_poses':MagicMock()}
        b.clients['single'].send_goal_async.side_effect=lambda *a,**k:Future(S(accepted=True,get_result_async=lambda:Future(S(status=4))))
        b.clients['through_poses'].send_goal_async.side_effect=b.clients['single'].send_goal_async.side_effect
        return b
    def test_unknown_localization_rejects_goals(self):
        b=self.bridge(False)
        with self.assertRaises(HTTPException) as ctx:b.start('single',[{'x':1,'y':1}])
        self.assertEqual(ctx.exception.status_code,409)
        b.clients['single'].send_goal_async.assert_not_called()
    def test_revoked_localization_blocks_stale_ready_cache(self):
        b=self.bridge(True);b.confirmed=False
        self.assertFalse(b.motion_ready())
        with self.assertRaises(HTTPException):b.start('single',[{'x':1,'y':1}])
    def test_waypoint_loops_are_real_sequential_actions(self):
        b=self.bridge();b.ready_at=time.monotonic()+10
        b.start('waypoints',[{'x':1,'y':1},{'x':2,'y':2}],2);b.thread.join(2)
        self.assertEqual(b.phase,'SUCCEEDED');self.assertEqual(b.clients['single'].send_goal_async.call_count,4)
        self.assertIsNone(b.motion.owner)
    def test_goal_must_be_accepted_before_velocity_gate_opens(self):
        b=self.bridge();pending=Future()
        b.clients['single'].send_goal_async.return_value=pending;b.clients['single'].send_goal_async.side_effect=None
        b.start('single',[{'x':1,'y':1}]);time.sleep(0.03)
        self.assertIsNone(b.motion.owner)
        pending.set(S(accepted=True,get_result_async=lambda:Future(S(status=4))))
        b.thread.join(1);self.assertEqual(b.phase,'SUCCEEDED')
    def test_pause_cancel_ack_and_resume(self):
        b=self.bridge();b.ready_at=time.monotonic()+10;result=Future()
        handle=S(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:result.set(S(status=5)))
        b.clients['single'].send_goal_async.side_effect=[Future(handle),Future(S(accepted=True,get_result_async=lambda:Future(S(status=4))))]
        b.start('single',[{'x':1,'y':1}]);time.sleep(0.05)
        b.cancel('pause');self.assertIsNone(b.motion.owner)
        b.thread.join(1);self.assertEqual(b.phase,'PAUSED')
        b.resume();b.thread.join(1);self.assertEqual(b.phase,'SUCCEEDED')
    def test_rejected_goal_is_not_success(self):
        b=self.bridge();b.clients['single'].send_goal_async.side_effect=lambda *a,**k:Future(S(accepted=False))
        b.start('single',[{'x':1,'y':1}]);b.thread.join(1)
        self.assertEqual(b.phase,'ABORTED');self.assertIsNone(b.motion.owner)
    def test_point_validation(self):
        b=self.bridge()
        for p in [{'x':float('nan'),'y':1},{'x':10,'y':10}]:
            with self.assertRaises(HTTPException):b.start('single',[p])
        b.mapping_getter().grid['data'][5]=100
        with self.assertRaises(HTTPException):b.start('single',[{'x':1.1,'y':1.1}])


if __name__=='__main__':main()
