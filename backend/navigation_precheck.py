"""Deterministic target/start checks against raw 0..255 Nav2 costs and footprint."""
import json
import math
from costmap_stream import grid_yaw


def coordinates(grid,x,y):
    o=grid['origin']['position'];yaw=grid_yaw(grid['origin']['orientation']);c=math.cos(yaw);s=math.sin(yaw)
    dx=x-o['x'];dy=y-o['y'];return ((c*dx+s*dy)/grid['resolution'],(-s*dx+c*dy)/grid['resolution'])


def footprint(profile):
    pad=float(profile['padding'])
    if not math.isfinite(pad) or not 0<=pad<=.5:raise ValueError('车体外廓 padding 参数无效')
    points=json.loads(profile.get('footprint') or '[]')
    if not points:
        r=float(profile.get('radius',0))
        if not .02<=r<=2:raise ValueError('未取得有效的车体 footprint 或 radius')
        points=[[r*math.cos(i*math.pi/16),r*math.sin(i*math.pi/16)] for i in range(32)]
    if not 3<=len(points)<=64:raise ValueError('车体外廓顶点数无效')
    out=[]
    for p in points:
        if len(p)!=2 or not all(math.isfinite(float(v)) and abs(float(v))<=3 for v in p):raise ValueError('车体外廓坐标无效')
        out.append([float(v)+(pad if float(v)>0 else -pad if float(v)<0 else 0) for v in p])
    area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(out,out[1:]+out[:1])))/2
    if area<.001:raise ValueError('车体外廓面积无效')
    return out


def cross(a,b,p):return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])


def on_segment(a,b,p):
    return abs(cross(a,b,p))<1e-8 and min(a[0],b[0])-1e-8<=p[0]<=max(a[0],b[0])+1e-8 and min(a[1],b[1])-1e-8<=p[1]<=max(a[1],b[1])+1e-8


def inside(poly,p):
    result=False
    for a,b in zip(poly,poly[1:]+poly[:1]):
        if on_segment(a,b,p):return True
        if (a[1]>p[1])!=(b[1]>p[1]) and p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:result=not result
    return result


def intersects(poly,x,y):
    box=[(x,y),(x+1,y),(x+1,y+1),(x,y+1)]
    if any(x<=p[0]<=x+1 and y<=p[1]<=y+1 for p in poly) or any(inside(poly,p) for p in box):return True
    for a,b in zip(poly,poly[1:]+poly[:1]):
        for c,d in zip(box,box[1:]+box[:1]):
            if any((on_segment(a,b,c),on_segment(a,b,d),on_segment(c,d,a),on_segment(c,d,b))):return True
            if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0:return True
    return False


def check_pose(grid,point,body,label):
    result={'valid':False,'x':point['x'],'y':point['y'],'yaw_deg':point.get('yaw_deg',0),'cost':None}
    def fail(code,message):return dict(result,code=code,message=label+'：'+message)
    gx,gy=coordinates(grid,point['x'],point['y']);ix=math.floor(gx);iy=math.floor(gy)
    if not 0<=ix<grid['width'] or not 0<=iy<grid['height']:return fail('OUTSIDE_COSTMAP','坐标超出全局代价地图范围')
    value=int(grid['data'][iy*grid['width']+ix]);result.update(cost=value,cell=[ix,iy])
    if value==255:return fail('UNKNOWN_COST','位于未知区域（代价 255），无法确认可通行')
    if value==254:return fail('LETHAL_OBSTACLE','位于实际障碍格（代价 254），请避开障碍')
    if value>=253:return fail('INSCRIBED_OBSTACLE','太靠近障碍，已进入车体膨胀禁入区（代价 253），'+
                             ('请先检查定位与车体周围空间' if label=='机器人当前位置' else '请向开阔区域移动目标'))
    a=math.radians(point.get('yaw_deg',0));c=math.cos(a);s=math.sin(a)
    poly=[coordinates(grid,point['x']+c*x-s*y,point['y']+s*x+c*y) for x,y in body]
    if any(not 0<=x<grid['width'] or not 0<=y<grid['height'] for x,y in poly):return fail('FOOTPRINT_OUTSIDE','车体外廓超出代价地图范围')
    minx=math.floor(min(x for x,y in poly));maxx=math.floor(max(x for x,y in poly));miny=math.floor(min(y for x,y in poly));maxy=math.floor(max(y for x,y in poly))
    if (maxx-minx+1)*(maxy-miny+1)>20000:return fail('FOOTPRINT_TOO_LARGE','车体外廓/分辨率超出预检查上限')
    for y in range(miny,maxy+1):
        for x in range(minx,maxx+1):
            cost=int(grid['data'][y*grid['width']+x])
            # Inflation already includes body radius: do not inflate the footprint twice.
            if cost<254 or not intersects(poly,x,y):continue
            return fail('FOOTPRINT_UNKNOWN' if cost==255 else 'FOOTPRINT_COLLISION',
                        '中心可通行，但按'+('当前' if label=='机器人当前位置' else '目标')+'朝向放置的车体外廓覆盖'+
                        ('未知区域' if cost==255 else '实际障碍')+f'（格 {x},{y}），'+
                        ('请先核对定位与实际环境' if label=='机器人当前位置' else '请调整位置或朝向'))
    return dict(result,valid=True,code='CLEAR',message=label+'：起终点碰撞预检查通过（尚未检查整条路径）')


def precheck(feed,points):
    result={'ok':True,'valid':False,'path_checked':False,'targets':[],'start':None,'issues':[]}
    if not feed or not getattr(feed,'costmaps',None):result['issues']=['代价地图组件尚未就绪'];return result
    with feed.lock:
        row,error=feed.costmaps.raw('global')
        revision = feed.revision
        base_map = feed.grid
    if not row:result['issues']=[error];return result
    grid=row['grid'];result.update(costmap_revision=row['revision'],costmap_age_sec=round(row['age_sec'],3))
    if not base_map or grid['frame_id']!=base_map['frame_id'] or grid['frame_id']!='map':
        result['issues']=['全局代价地图与导航 map 坐标系不一致'];return result
    profile,error=feed.costmaps.footprint()
    if not profile:result['issues']=[error];return result
    try:body=footprint(profile)
    except (ValueError,TypeError,KeyError) as exc:result['issues']=['无法检查车体外廓：'+str(exc)];return result
    result['footprint']={'points':body,'padding':profile['padding']}
    def inspect(point,label):
        mx,my=coordinates(base_map,point['x'],point['y']);x=math.floor(mx);y=math.floor(my)
        prefix={'valid':False,'x':point['x'],'y':point['y'],'yaw_deg':point.get('yaw_deg',0),'cost':None}
        if not 0<=x<base_map['width'] or not 0<=y<base_map['height']:
            return dict(prefix,code='OUTSIDE_MAP',message=label+'：坐标超出原始地图范围')
        occupancy=base_map['data'][y*base_map['width']+x]
        if occupancy<0:return dict(prefix,code='UNKNOWN_MAP',message=label+'：原始地图此处未知，不能确认可通行')
        if occupancy>=65:return dict(prefix,code='MAP_OBSTACLE',message=label+'：原始地图此处已标记为障碍')
        return check_pose(grid,point,body,label)
    sample=feed.snapshot(last_revision=revision, include_scan_points=False)
    if not sample.get('map_valid'):
        result['issues'] = ['导航需要一个有效的 /map 发布者和可用底图']
        return result
    pose=sample.get('pose')
    if not pose or not sample['tf']['robot']:result['issues'].append('机器人当前位置 TF 未就绪，无法检查导航起点')
    else:
        result['start']=inspect(dict(pose,yaw_deg=math.degrees(pose['yaw'])),'机器人当前位置')
        if not result['start']['valid']:result['issues'].append(result['start']['message'])
    for i,p in enumerate(points):
        report=inspect(p,'目标 '+str(i+1)+' '+str(p.get('name','')))
        report.update(index=i,id=p.get('id'));result['targets'].append(report)
        if not report['valid']:result['issues'].append(report['message'])
    if feed.revision != revision:
        result['issues'].append('预检查期间地图已更新，请重新检查')
    result['valid']=not result['issues']
    return result
