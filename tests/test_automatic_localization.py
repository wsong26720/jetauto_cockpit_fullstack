import sys,math,threading,time
from pathlib import Path
from types import SimpleNamespace as S
from unittest import TestCase
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from global_scan_matcher import GlobalScanMatcher,angle_delta
from automatic_localization import AutomaticLocalization
from control_safety import VelocityLease,MotionArbiter


def fixture(asymmetric=True,origin_yaw=0):
    a=np.zeros((80,100),dtype=np.int16);a[[0,-1],:]=100;a[:,[0,-1]]=100
    if asymmetric:
        a[45:51,28:40]=100;a[12:30,75:79]=100;a[68:,60:]=100
    grid={'width':100,'height':80,'resolution':.05,'data':a.ravel().tolist(),
          'origin':{'position':{'x':-2,'y':3},'orientation':{'x':0,'y':0,'z':math.sin(origin_yaw/2),'w':math.cos(origin_yaw/2)}}}
    x,y,yaw=1.7,1.2,.37;points=[]
    for angle in np.linspace(-math.pi,math.pi,360,endpoint=False):
        for r in np.arange(.10,7,.02):
            ix=int((x+math.cos(angle+yaw)*r)/.05);iy=int((y+math.sin(angle+yaw)*r)/.05)
            if not 0<=ix<100 or not 0<=iy<80:break
            if a[iy,ix]>=65:
                points.append([r*math.cos(angle),r*math.sin(angle)]);break
    return grid,points,{'x':-2+math.cos(origin_yaw)*x-math.sin(origin_yaw)*y,
                        'y':3+math.sin(origin_yaw)*x+math.cos(origin_yaw)*y,'yaw_deg':math.degrees(yaw+origin_yaw)}


class MatcherTests(TestCase):
    def test_global_search_finds_unknown_pose_with_rotated_origin(self):
        grid,points,expected=fixture(origin_yaw=.3);r=GlobalScanMatcher(grid).match(points)
        self.assertTrue(r['accepted'],r)
        self.assertLess(math.hypot(r['x']-expected['x'],r['y']-expected['y']),.15)
        self.assertLess(abs(angle_delta(math.radians(r['yaw_deg']-expected['yaw_deg']),0)),math.radians(5))

    def test_symmetric_room_is_not_declared_localized(self):
        grid,points,_=fixture(asymmetric=False);r=GlobalScanMatcher(grid).match(points)
        self.assertFalse(r['accepted']);self.assertLess(r['margin'],.06)

    def test_insufficient_or_invalid_scan_is_rejected(self):
        grid,points,_=fixture();m=GlobalScanMatcher(grid)
        for data in [points[:20],[[float('nan'),0]]*200,[[1,.1]]*100]:
            with self.assertRaises(ValueError):m.match(data)

    def test_search_can_be_canceled(self):
        grid,points,_=fixture()
        with self.assertRaises(InterruptedError):GlobalScanMatcher(grid).match(points,lambda:True)


class AutomaticTests(TestCase):
    def test_rotation_obstacle_guard_excludes_body_returns(self):
        body = [[.25,.22],[.25,-.22],[-.25,-.22],[-.25,.22]]
        self.assertTrue(AutomaticLocalization._rotation_clear([[.1,.1],[1.,0]], body, .43))
        self.assertFalse(AutomaticLocalization._rotation_clear([[.35,0]], body, .43))
        self.assertFalse(AutomaticLocalization._rotation_clear([[.47,0]], body, .52))

    def test_localization_cannot_accept_navigation_or_translation_commands(self):
        outputs=[];now=[1.];manual=VelocityLease(lambda *v:outputs.append(v),lambda:True,clock=lambda:now[0])
        motion=MotionArbiter(manual,lambda *v:outputs.append(v),lambda:True,clock=lambda:now[0]);motion.localization_ready=lambda:True
        motion.begin('loc',localization=True);motion.receive(1,1,1);motion.tick()
        self.assertEqual(outputs[-1],(0,0,0))
        motion.localization_command('wrong-token',1);motion.tick();self.assertEqual(outputs[-1],(0,0,0))
        motion.localization_command('loc',1);motion.tick();self.assertEqual(outputs[-1],(0.,0.,.10))
        now[0]+=.31;motion.tick();self.assertEqual(outputs[-1],(0,0,0))
        motion.end('loc');self.assertFalse(motion.localizing)
    def test_small_covariance_alone_does_not_authorize_navigation(self):
        cov=[0.]*36;cov[0]=cov[7]=.01;cov[35]=.01
        pose={'x':1.,'y':2.,'yaw_deg':10.,'covariance':cov};seed=dict(pose)
        self.assertTrue(AutomaticLocalization._pose_good(pose,seed,{'hit_ratio':.9,'free_ratio':.99}))
        self.assertFalse(AutomaticLocalization._pose_good(pose,seed,{'hit_ratio':.3,'free_ratio':.99}))
        self.assertFalse(AutomaticLocalization._pose_good(dict(pose,x=4.),seed,{'hit_ratio':.9,'free_ratio':.99}))
        cov[0]=1.;self.assertFalse(AutomaticLocalization._pose_good(pose,seed,{'hit_ratio':.9,'free_ratio':.99}))

    def test_movement_during_search_is_rejected(self):
        self.assertTrue(AutomaticLocalization._stationary({'odom':{'x':0,'y':0,'yaw':0}},{'odom':{'x':.01,'y':0,'yaw':.01}}))
        self.assertFalse(AutomaticLocalization._stationary({'odom':{'x':0,'y':0,'yaw':0}},{'odom':{'x':.2,'y':0,'yaw':0}}))

    def test_failed_search_keeps_drive_gate_closed_and_never_seeds_pose(self):
        outputs=[];manual=VelocityLease(lambda *v:outputs.append(v),lambda:True);motion=MotionArbiter(manual,lambda *v:outputs.append(v),lambda:True)
        grid,points,_=fixture(asymmetric=False)
        inputs={'grid':grid,'revision':1,'sequence':1,'points':points,'odom':{'x':0,'y':0,'yaw':0}}
        published=[]
        b=S(lock=threading.RLock(),thread=None,motion=motion,confirmed=True,amcl=None,initial_stamp=0,
            ready_cache={'ready':True},states={'amcl':(3,0)},mapping_getter=lambda:S(localization_input=lambda:inputs),
            _publish_initial=lambda *a,**kw:published.append(a))
        a=AutomaticLocalization(b);a.start();a.thread.join(5)
        self.assertEqual(a.state,'FAILED');self.assertFalse(b.confirmed);self.assertFalse(b.ready_cache['ready'])
        self.assertFalse(published);self.assertTrue(all(v==(0,0,0) for v in outputs))
