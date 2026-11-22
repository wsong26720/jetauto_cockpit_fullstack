"""Bounded, stationary 2D global scan matching; never publishes robot commands."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


def angle_delta(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


class GlobalScanMatcher:
    def __init__(self,grid):
        self.grid=grid;self.res=float(grid['resolution'])
        if grid['width'] * grid['height'] > 1_000_000:
            raise ValueError('地图超过自动定位的 100 万格上限')
        self.a=np.asarray(grid['data'],dtype=np.int16).reshape(grid['height'],grid['width'])
        if not np.any(self.a>=65):raise ValueError('地图缺少已知障碍，无法自动定位')
        self.distance=distance_transform_edt(self.a<65)*self.res
        # Each beam repeatedly hits the same cells across thousands of poses.
        self.likelihood = {sigma: np.exp(-0.5 * (self.distance / sigma)**2)
                           for sigma in (.25, .12)}
        self.clearance=distance_transform_edt(np.pad((self.a>=0)&(self.a<20),1))[1:-1,1:-1]*self.res
        o=grid['origin'];q=o['orientation'];p=o['position']
        self.ox=p['x'];self.oy=p['y']
        self.oyaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))

    def points(self,points):
        p=np.asarray(points,dtype=float)
        if p.ndim!=2 or p.shape[1]!=2:raise ValueError('激光点格式无效')
        p=p[np.isfinite(p).all(axis=1)]
        ranges=np.linalg.norm(p,axis=1);p=p[(ranges>.15)&(ranges<9.8)]
        if len(p)<60:raise ValueError('有效激光射线少于 60 条，无法可靠自动定位')
        angles=np.arctan2(p[:,1],p[:,0]);sectors=np.unique(np.floor((angles+math.pi)/(math.pi/4)).astype(int)%8)
        ordered=np.sort(angles);coverage=2*math.pi-float(np.max(np.diff(np.r_[ordered,ordered[0]+2*math.pi])))
        if len(sectors)<3 or coverage<math.radians(120):raise ValueError('有效激光覆盖角度不足 120°，请检查雷达遮挡')
        indices = np.linspace(0, len(p), min(len(p), 120), endpoint=False, dtype=int)
        return p[indices]

    def _evaluate(self,poses,points,sigma):
        poses=np.asarray(poses,dtype=float)
        # poses and points are expressed in the map's own unrotated grid axes.
        c=np.cos(poses[:,2,None]);s=np.sin(poses[:,2,None])
        x=poses[:,0,None]+c*points[None,:,0]-s*points[None,:,1]
        y=poses[:,1,None]+s*points[None,:,0]+c*points[None,:,1]
        ix=np.floor(x/self.res).astype(int);iy=np.floor(y/self.res).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<self.a.shape[1])&(iy<self.a.shape[0])
        clipped_x=np.clip(ix,0,self.a.shape[1]-1);clipped_y=np.clip(iy,0,self.a.shape[0]-1)
        return (self.likelihood[sigma][clipped_y, clipped_x] * valid).mean(axis=1)

    def _refine_peak(self, pose, points):
        # Search below the 5 cm / 2 degree refinement grid. Interpolate the
        # likelihood at cell centres so sub-cell shifts change the objective.
        step = min(.025, self.res / 2)
        offsets = np.array([(x * step, y * step, math.radians(a))
                            for x in (-1, 0, 1) for y in (-1, 0, 1) for a in (-1, 0, 1)])
        best = pose
        field = self.likelihood[.12]
        h, w = field.shape
        for scale in (1., .5):
            candidates = best + offsets * scale
            gx, gy = np.floor(candidates[:, :2] / self.res).astype(int).T
            valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
            candidates = candidates[valid & (self.clearance[gy.clip(0,h-1), gx.clip(0,w-1)] >= .25)]
            c, s = np.cos(candidates[:, 2:3]), np.sin(candidates[:, 2:3])
            x = (candidates[:, 0:1] + c*points[:, 0] - s*points[:, 1]) / self.res - .5
            y = (candidates[:, 1:2] + s*points[:, 0] + c*points[:, 1]) / self.res - .5
            ix, iy = np.floor(x).astype(int), np.floor(y).astype(int)
            fx, fy = x-ix, y-iy
            scores = np.zeros(len(candidates))
            for dx, wx in ((0, 1-fx), (1, fx)):
                for dy, wy in ((0, 1-fy), (1, fy)):
                    xx, yy = ix+dx, iy+dy
                    valid = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
                    scores += (wx*wy*valid*field[yy.clip(0,h-1), xx.clip(0,w-1)]).sum(axis=1)
            # Retain the centre on a flat peak.
            tied = np.flatnonzero(np.isclose(scores, scores.max(), rtol=0, atol=1e-10))
            best = candidates[tied[np.argmin(np.linalg.norm(candidates[tied] - best, axis=1))]]
        return best

    @staticmethod
    def _distinct(candidates,limit):
        kept=[]
        for score,p in sorted(candidates,key=lambda item:item[0],reverse=True):
            if all(math.hypot(p[0]-q[0],p[1]-q[1])>.45 or abs(angle_delta(p[2],q[2]))>math.radians(20) for _,q in kept):
                kept.append((score,p))
                if len(kept)>=limit:break
        return kept

    def match(self,points,canceled=lambda:False):
        p=self.points(points)
        stride=max(1,round(.20/self.res));mask=(self.a>=0)&(self.a<20)&(self.clearance>=.25)
        gy,gx=np.nonzero(mask[::stride,::stride]);xy=np.column_stack(((gx*stride+.5)*self.res,(gy*stride+.5)*self.res))
        if not len(xy):raise ValueError('地图没有足够宽的已知可通行区域')
        if len(xy)>12000:raise ValueError('地图搜索范围过大，请使用分区地图')
        candidates=[]
        for yaw in np.arange(-math.pi,math.pi,math.radians(10)):
            if canceled():raise InterruptedError('自动定位已取消')
            poses=np.column_stack((xy,np.full(len(xy),yaw)))
            scores = np.empty(len(poses))
            for begin in range(0, len(poses), 512):
                if canceled():raise InterruptedError('自动定位已取消')
                scores[begin:begin+512] = self._evaluate(poses[begin:begin+512], p, .25)
            top=np.argsort(scores)[-min(100,len(scores)):]
            candidates.extend((float(scores[i]),poses[i]) for i in top)
        seeds=self._distinct(candidates,16);refined=[]
        offsets=np.array([(x,y,a) for x in np.arange(-.2,.201,.05) for y in np.arange(-.2,.201,.05)
                          for a in np.arange(-math.radians(10),math.radians(10.1),math.radians(2))])
        for _,seed in seeds:
            if canceled():raise InterruptedError('自动定位已取消')
            poses=seed+offsets;gx=np.floor(poses[:,0]/self.res).astype(int);gy=np.floor(poses[:,1]/self.res).astype(int)
            valid=(gx>=0)&(gy>=0)&(gx<self.a.shape[1])&(gy<self.a.shape[0])
            valid &= self.clearance[np.clip(gy,0,self.a.shape[0]-1),np.clip(gx,0,self.a.shape[1]-1)]>=.25
            poses=poses[valid]
            if not len(poses):continue
            scores=self._evaluate(poses,p,.12);i=int(np.argmax(scores));refined.append((float(scores[i]),poses[i]))
        ranked=self._distinct(refined,16)
        if not ranked:raise ValueError('无法找到可用的地图匹配候选')
        score,best=ranked[0];runner_up=ranked[1][0] if len(ranked)>1 else 0.
        coarse_best = best
        if score >= .68 and score - runner_up >= .06:
            candidate = self._refine_peak(best, p)
            quality = self.assess_grid_pose(candidate, p)
            if quality['hit_ratio'] >= .65 and quality['free_ratio'] >= .95:
                best = candidate
        result=self.assess_grid_pose(best,p)
        c=math.cos(self.oyaw);s=math.sin(self.oyaw)
        result.update(x=self.ox+c*best[0]-s*best[1],y=self.oy+s*best[0]+c*best[1],
                      yaw_deg=math.degrees(angle_delta(best[2]+self.oyaw,0)),score=score,
                      runner_up_score=runner_up,margin=score-runner_up,beams=len(p))
        neighborhood=coarse_best+offsets
        local_scores=self._evaluate(neighborhood,p,.12)
        near=neighborhood[local_scores>=max(score-.03,.65)]
        if len(near):
            spread=np.mean((near-best)**2,axis=0)
            result['seed_variance_xy']=float(max(.0125,spread[0],spread[1]))
            result['seed_variance_yaw']=float(max(math.radians(5)**2,spread[2]))
        else:result.update(seed_variance_xy=.04,seed_variance_yaw=math.radians(10)**2)
        result['accepted']=(score>=.68 and result['hit_ratio']>=.65 and result['free_ratio']>=.95 and score-runner_up>=.06
                            and result['seed_variance_xy']<=.04 and result['seed_variance_yaw']<=math.radians(10)**2)
        result['reason']='匹配通过' if result['accepted'] else ('环境存在相似位置，无法唯一确定定位' if score-runner_up<.06 else '激光与地图匹配不足')
        return result

    def assess_grid_pose(self,pose,p):
        c=math.cos(pose[2]);s=math.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
        endpoints=p@rot.T+pose[:2]
        ix=np.floor(endpoints[:,0]/self.res).astype(int);iy=np.floor(endpoints[:,1]/self.res).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<self.a.shape[1])&(iy<self.a.shape[0])
        d=self.distance[np.clip(iy,0,self.a.shape[0]-1),np.clip(ix,0,self.a.shape[1]-1)]
        hit=float(np.mean(valid&(d<=.18)))
        rays=(p[None,:,:]*np.array([.2,.4,.6,.8])[:,None,None])@rot.T+pose[:2]
        ix=np.floor(rays[:,:,0]/self.res).astype(int);iy=np.floor(rays[:,:,1]/self.res).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<self.a.shape[1])&(iy<self.a.shape[0])
        free=valid&(self.a[np.clip(iy,0,self.a.shape[0]-1),np.clip(ix,0,self.a.shape[1]-1)]>=0)&(self.a[np.clip(iy,0,self.a.shape[0]-1),np.clip(ix,0,self.a.shape[1]-1)]<65)
        return {'hit_ratio':hit,'free_ratio':float(np.mean(free))}

    def assess(self,pose,points):
        dx=pose['x']-self.ox;dy=pose['y']-self.oy;c=math.cos(self.oyaw);s=math.sin(self.oyaw)
        p=np.array([c*dx+s*dy,-s*dx+c*dy,math.radians(pose['yaw_deg'])-self.oyaw])
        return self.assess_grid_pose(p,self.points(points))
