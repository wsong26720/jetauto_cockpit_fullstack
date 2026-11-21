import math
import sys
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace as S

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'backend'))
from mapping_stream import MappingFeed, scan_points


def stamp(sec): return S(sec=int(sec), nanosec=round((sec-int(sec))*1e9))
def transform(q=None):
    return S(translation=S(x=10,y=20,z=0), rotation=q or S(x=0,y=0,z=0,w=1))
def scan(at=200):
    return S(header=S(frame_id='lidar_frame',stamp=stamp(at)), range_min=0.1,range_max=10,
             angle_min=0,angle_increment=math.pi/2,ranges=[1,2,float('nan'),float('inf'),0,0.05,11])
def map_msg():
    return S(header=S(frame_id='/map',stamp=stamp(200)),
        info=S(width=2,height=2,resolution=0.5,
            origin=S(position=S(x=-1,y=-2,z=0),orientation=S(x=0,y=0,z=0,w=1))),data=[0,100,-1,50])


class FakeTime:
    def __init__(self): self.stamp=None
    @classmethod
    def from_msg(cls, msg):
        obj=cls();obj.stamp=msg.sec+msg.nanosec*1e-9;return obj


class MappingTests(unittest.TestCase):
    def feed(self):
        f=MappingFeed.__new__(MappingFeed)
        f.lock=threading.RLock();f.clock=lambda:100.5
        f.grid=None;f.revision=0;f.map_received=None;f.scan_received=None;f.map_error=None
        f.scan_sequence=0;f.scans=deque(maxlen=30);f.Time=FakeTime;f.TransformException=LookupError
        f.base_frame='base_footprint'
        f.node=S(get_clock=lambda:S(now=lambda:S(nanoseconds=200_000_000_000)),count_publishers=lambda topic:1)
        return f

    def test_ray_limits_and_full_rotation(self):
        self.assertEqual(scan_points(scan(),transform()),[[11,20],[10,22]])
        q=S(x=0,y=0,z=math.sqrt(0.5),w=math.sqrt(0.5))
        self.assertEqual(scan_points(scan(),transform(q)),[[10,21],[8,20]])
        # Inverted lidar: yaw-only conversion would put the second ray on the wrong side.
        self.assertEqual(scan_points(scan(),transform(S(x=1,y=0,z=0,w=0))),[[11,20],[10,18]])
        with self.assertRaises(ValueError): scan_points(scan(),transform(S(x=0,y=0,z=0,w=0)))

    def test_latched_grid_metadata_and_revision(self):
        f=self.feed();f.on_map(map_msg())
        self.assertEqual(f.grid['frame_id'],'map');self.assertEqual(f.grid['data'],[0,100,-1,50])
        self.assertEqual(f.revision,1)
        bad=map_msg();bad.data=[];f.on_map(bad)
        self.assertEqual(f.revision,1);self.assertIsNotNone(f.map_error)
        bad=map_msg();bad.info.origin.position.x=float('nan');f.on_map(bad)
        self.assertEqual(f.revision,1)

    def test_scan_uses_its_timestamp_and_retries_previous_sample(self):
        f=self.feed();f.on_map(map_msg())
        f.scans.extend([(1,100.3,scan(199.9)),(2,100.4,scan(200.1))]);f.scan_received=100.4
        calls=[]
        def lookup(target,source,when):
            calls.append((target,source,when.stamp))
            if when.stamp is not None and when.stamp>200: raise LookupError('future')
            return S(header=S(stamp=stamp(200)),transform=transform())
        f.buffer=S(lookup_transform=lookup)
        data=f.snapshot()
        self.assertTrue(data['tf']['robot']);self.assertTrue(data['tf']['scan'])
        self.assertEqual(data['scan']['sequence'],1)
        self.assertIn(('map','lidar_frame',199.9),calls)
        self.assertEqual(data['scan']['points'],[[11,20],[10,22]])
        self.assertNotIn('map',f.snapshot(last_revision=1))  # large grid sent only on change

    def test_missing_tf_and_stale_scan_never_draw_wrong_coordinates(self):
        f=self.feed();f.on_map(map_msg());f.scans.append((1,100.4,scan()));f.scan_received=100.4
        def fail(*args): raise LookupError('disconnected TF')
        f.buffer=S(lookup_transform=fail)
        data=f.snapshot()
        self.assertIsNone(data['scan']);self.assertIsNone(data['pose'])
        self.assertFalse(data['tf']['scan']);self.assertGreaterEqual(len(data['tf']['issues']),2)
        f.scan_received=90;f.scans.clear();f.scans.append((1,90,scan()))
        data=f.snapshot();self.assertIsNone(data['scan'])

    def test_robot_pose_expiry_and_map_waiting(self):
        f=self.feed();self.assertEqual(f.snapshot()['tf']['issues'],['等待 /map；请启动 SLAM 或加载已保存地图'])
        f.on_map(map_msg())
        f.buffer=S(lookup_transform=lambda *args:S(header=S(stamp=stamp(190)),transform=transform()))
        data=f.snapshot();self.assertIsNone(data['pose']);self.assertFalse(data['tf']['robot'])


if __name__=='__main__': unittest.main()
