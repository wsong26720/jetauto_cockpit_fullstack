import math,sys,threading,copy
from pathlib import Path
from types import SimpleNamespace as S
from unittest import TestCase
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from navigation_precheck import check_pose,footprint,precheck
from costmap_stream import CostmapFeed


def grid():
    return {'frame_id':'map','width':80,'height':80,'resolution':.05,'data':[0]*6400,
            'origin':{'position':{'x':0,'y':0,'z':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}}
def profile():return {'footprint':'[[0.25,0.22],[0.25,-0.22],[-0.25,-0.22],[-0.25,0.22]]','padding':.02,'radius':.1}
def pose(**kwargs):return {'x':2.025,'y':2.025,'yaw_deg':0,**kwargs}


class PrecheckTests(TestCase):
    def test_white_static_map_is_not_enough_when_cost_is_253(self):
        g=grid();g['data'][40*80+40]=253
        r=check_pose(g,pose(),footprint(profile()),'目标')
        self.assertFalse(r['valid']);self.assertEqual(r['code'],'INSCRIBED_OBSTACLE');self.assertIn('253',r['message'])
    def test_lethal_unknown_and_outside_have_distinct_reasons(self):
        for value,code in [(254,'LETHAL_OBSTACLE'),(255,'UNKNOWN_COST')]:
            g=grid();g['data'][40*80+40]=value
            self.assertEqual(check_pose(g,pose(),footprint(profile()),'目标')['code'],code)
        self.assertEqual(check_pose(grid(),pose(x=-1),footprint(profile()),'目标')['code'],'OUTSIDE_COSTMAP')
    def test_inflation_is_not_applied_twice_to_robot_footprint(self):
        g=grid();g['data']=[252]*6400;g['data'][40*80+42]=253
        self.assertTrue(check_pose(g,pose(),footprint(profile()),'目标')['valid'])
    def test_footprint_collision_depends_on_goal_orientation(self):
        g=grid();g['data'][40*80+46]=254
        body=footprint({'footprint':'[[0.4,0.1],[0.4,-0.1],[-0.4,-0.1],[-0.4,0.1]]','padding':.02})
        self.assertEqual(check_pose(g,pose(),body,'目标')['code'],'FOOTPRINT_COLLISION')
        self.assertTrue(check_pose(g,pose(yaw_deg=90),body,'目标')['valid'])
    def test_unknown_cells_under_body_are_rejected(self):
        g=grid();g['data'][40*80+43]=255
        self.assertEqual(check_pose(g,pose(),footprint(profile()),'目标')['code'],'FOOTPRINT_UNKNOWN')
    def test_body_cannot_extend_outside_costmap(self):
        self.assertEqual(check_pose(grid(),pose(x=.1),footprint(profile()),'目标')['code'],'FOOTPRINT_OUTSIDE')
    def test_rotated_grid_origin_uses_correct_cell(self):
        g=grid();g['origin']['position'].update(x=1,y=2);g['origin']['orientation'].update(z=math.sqrt(.5),w=math.sqrt(.5))
        g['data'][40*80+40]=253
        r=check_pose(g,pose(x=1-2.025,y=2+2.025),footprint(profile()),'目标')
        self.assertEqual(r['cost'],253)
    def test_profile_padding_and_invalid_footprint(self):
        body=footprint(profile());self.assertAlmostEqual(body[0][0],.27);self.assertAlmostEqual(body[0][1],.24)
        with self.assertRaises(ValueError):footprint({'footprint':'[]','padding':.02,'radius':0})
    def test_stale_or_missing_map_never_passes(self):
        feed=S(lock=threading.RLock(), grid=None, revision=0, costmaps=S(raw=lambda _: (None,'全局代价地图已过期')))
        r=precheck(feed,[pose()]);self.assertFalse(r['valid']);self.assertIn('过期',r['issues'][0])
    def test_start_collision_and_multiple_targets_are_reported(self):
        g=grid();g['data'][20*80+20]=253;g['data'][40*80+40]=254
        costs=S(raw=lambda _:({'grid':g,'revision':3,'age_sec':.2},None),footprint=lambda:(profile(),None))
        f=S(lock=threading.RLock(),grid=grid(),revision=1,costmaps=costs,snapshot=lambda **kw:{'map_valid':True,'pose':{'x':1.025,'y':1.025,'yaw':0},'tf':{'robot':True}})
        r=precheck(f,[pose(),pose(x=3,y=3)])
        self.assertFalse(r['valid']);self.assertIn('机器人当前位置',r['issues'][0]);self.assertFalse(r['targets'][0]['valid']);self.assertTrue(r['targets'][1]['valid'])


class CostFeedTests(TestCase):
    def feed(self):
        f=CostmapFeed.__new__(CostmapFeed);f.clock=lambda:10.;f.lock=threading.RLock();f.generation=0
        f.layers={};f.pending={};f.revisions={'global':0,'local':0};f.profile=None;f.profile_at=0;f.profile_error='等待外廓'
        return f
    def reply(self,feed,data):
        i=S(size_x=2,size_y=2,resolution=.05,origin=S(position=S(x=0,y=0,z=0),orientation=S(x=0,y=0,z=0,w=1)))
        future = S(result=lambda:S(map=S(metadata=i,header=S(frame_id='map'),data=data)))
        feed.pending['global'] = (future, feed.clock())
        return future
    def test_raw_costs_preserved_and_revision_survives_reset(self):
        f=self.feed();f._reply('global',self.reply(f,[0,253,254,255]),0)
        row,_=f.raw();self.assertEqual(row['grid']['data'],[0,253,254,255]);self.assertEqual(row['revision'],1)
        f._reply('global',self.reply(f,[0,253,254,255]),0);self.assertEqual(f.raw()[0]['revision'],1)
        f.reset();f._reply('global',self.reply(f,[0,0,0,0]),1);self.assertEqual(f.raw()[0]['revision'],2)
    def test_stale_callbacks_cannot_replace_new_map(self):
        f=self.feed();f.reset();f._reply('global',self.reply(f,[0,0,0,0]),0);self.assertIsNone(f.raw()[0])

    def test_late_reply_after_timeout_cannot_overwrite_newer_obstacle(self):
        f = self.feed()
        late = self.reply(f, [0]*4)
        current = self.reply(f, [254]*4)
        f._reply('global', current, 0)
        f._reply('global', late, 0)
        self.assertEqual(f.raw()[0]['grid']['data'], [254]*4)
    def test_expired_costmap_is_not_available_for_validation(self):
        f=self.feed();f._reply('global',self.reply(f,[0]*4),0);f.clock=lambda:14
        row,error=f.raw();self.assertIsNone(row);self.assertIn('3 秒',error)
    def test_grid_only_sent_when_client_revision_changes_and_tf_is_separate(self):
        f=self.feed();f._reply('global',self.reply(f,[0]*4),0)
        first=f.snapshot('map',lambda _:None,{});self.assertIn('grid',first['global'])
        self.assertNotIn('grid',f.snapshot('map',lambda _:None,{'global':1})['global'])
        f.layers['global']['grid']['frame_id']='odom'
        packet=f.snapshot('map',lambda _:{'x':1,'y':2,'yaw':.3},{'global':1})
        self.assertTrue(packet['global']['aligned']);self.assertEqual(packet['global']['transform']['x'],1)
