"""Read raw Nav2 costmaps and footprint parameters without moving the robot."""
import math
import threading
import time


def grid_yaw(q):
    norm=math.sqrt(sum(float(q[k])**2 for k in ('x','y','z','w')))
    if not math.isfinite(norm) or norm<1e-9:raise ValueError('代价地图原点四元数无效')
    x,y,z,w=(float(q[k])/norm for k in ('x','y','z','w'))
    if abs(x)>1e-5 or abs(y)>1e-5:raise ValueError('代价地图原点不是平面旋转')
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


class CostmapFeed:
    def __init__(self,node,clock=time.monotonic):
        from nav2_msgs.srv import GetCostmap
        from rcl_interfaces.srv import GetParameters
        self.node=node;self.clock=clock;self.GetCostmap=GetCostmap;self.GetParameters=GetParameters
        self.lock=threading.RLock();self.layers={};self.pending={};self.generation=0;self.last_params=0
        self.revisions={'global':0,'local':0}
        self.profile=None;self.profile_at=0;self.profile_error='等待 Nav2 车体外廓参数'
        self.clients={name:node.create_client(GetCostmap,'/'+name+'_costmap/get_costmap') for name in ('global','local')}
        self.params=node.create_client(GetParameters,'/global_costmap/global_costmap/get_parameters')
        node.create_timer(.5,self.poll)

    def reset(self):
        with self.lock:
            self.generation+=1;self.layers={};self.profile=None;self.profile_at=0;self.last_params=0

    def _reply(self,name,future,generation):
        with self.lock:
            if self.pending.get(name, (None,))[0] is not future:
                return
            self.pending.pop(name, None)
            if generation!=self.generation:return
        try:
            r=future.result()
            if name=='profile':
                values={k:v for k,v in zip(('footprint','footprint_padding','robot_radius'),r.values)}
                if values['footprint'].type != 4 or any(values[key].type != 3 for key in ('footprint_padding','robot_radius')):
                    raise ValueError('Nav2 未返回有效的轮廓、余量和半径参数')
                profile={'footprint':values['footprint'].string_value,
                         'padding':values['footprint_padding'].double_value,
                         'radius':values['robot_radius'].double_value}
                with self.lock:
                    if generation!=self.generation:return
                    self.profile=profile;self.profile_at=self.clock();self.profile_error=None
                return
            m=r.map;i=m.metadata;p=i.origin.position;q=i.origin.orientation
            grid={'frame_id':m.header.frame_id.lstrip('/'),'width':i.size_x,'height':i.size_y,
                  'resolution':i.resolution,'origin':{'position':{'x':p.x,'y':p.y,'z':p.z},
                  'orientation':{'x':q.x,'y':q.y,'z':q.z,'w':q.w}},'data':list(m.data)}
            size=grid['width']*grid['height']
            if (not grid['frame_id'] or size<=0 or size>1_000_000 or size!=len(grid['data'])
                or not math.isfinite(grid['resolution']) or grid['resolution']<=0
                or not all(math.isfinite(v) for v in (p.x,p.y,p.z))):raise ValueError('代价地图尺寸/原点/分辨率无效，或超过 100 万格')
            grid_yaw(grid['origin']['orientation'])
            with self.lock:
                if generation!=self.generation:return
                old=self.layers.get(name,{})
                revision=self.revisions[name]+(old.get('grid')!=grid);self.revisions[name]=revision
                self.layers[name]={'grid':grid,'revision':revision,'received':self.clock(),'error':None}
        except Exception as exc:
            with self.lock:
                if generation!=self.generation:return
                if name=='profile':
                    self.profile = None
                    self.profile_error='读取车体外廓失败：'+str(exc)
                else:self.layers.setdefault(name,{})['error']='读取代价地图失败：'+str(exc)

    def poll(self):
        now=self.clock()
        for name,client in [*self.clients.items(),('profile',self.params)]:
            with self.lock:
                pending=self.pending.get(name)
                if pending:
                    if now-pending[1]>2:
                        self.pending.pop(name,None)
                        pending[0].cancel()
                    else:continue
                if name=='profile' and now-self.last_params<5:continue
                generation=self.generation
            if not client.service_is_ready():continue
            request=self.GetParameters.Request() if name=='profile' else self.GetCostmap.Request()
            if name=='profile':request.names=['footprint','footprint_padding','robot_radius'];self.last_params=now
            f=client.call_async(request)
            with self.lock:self.pending[name]=(f,now)
            f.add_done_callback(lambda result,n=name,g=generation:self._reply(n,result,g))

    def raw(self,name='global'):
        with self.lock:
            row=self.layers.get(name,{})
            age=self.clock()-row.get('received',-1e9)
            if not row.get('grid'):return None,'等待 '+name+' 代价地图'
            if age>3:return None,name+' 代价地图超过 3 秒未更新，暂时禁止下发'
            if row.get('error'):return None,row['error']
            return dict(row,age_sec=age),None

    def footprint(self):
        with self.lock:
            if not self.profile or self.clock()-self.profile_at>15:return None,self.profile_error or '车体外廓参数已过期'
            return dict(self.profile),None

    def snapshot(self,frame,resolve,revisions=None):
        output={}
        for name in ('global','local'):
            row,error=self.raw(name)
            if not row:
                output[name]={'available':False,'fresh':False,'aligned':False,'issue':error,'revision':0};continue
            grid=row['grid'];entry={'available':True,'fresh':True,'age_sec':round(row['age_sec'],3),
                 'revision':row['revision'],'frame_id':grid['frame_id'],'aligned':False,'issue':None}
            if revisions is not None and revisions.get(name)!=row['revision']:entry['grid']=grid
            try:
                entry['transform']={'x':0.,'y':0.,'yaw':0.} if grid['frame_id']==frame else resolve(grid['frame_id'])
                entry['aligned']=True
            except Exception as exc:entry['issue']='代价地图 TF 未就绪：'+str(exc)
            output[name]=entry
        return output
