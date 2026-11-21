/* Real OccupancyGrid + TF-aligned laser/pose viewer. Never publishes ROS commands. */
(function (global) {
  'use strict';
  function yaw(q) {
    const norm = Math.hypot(q.x, q.y, q.z, q.w);
    if (!Number.isFinite(norm) || norm < 1e-9) throw new Error('地图原点四元数无效');
    const x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm;
    return Math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
  }
  function gridGeometry(map) {
    if (!map || !Number.isInteger(map.width) || !Number.isInteger(map.height) || map.width <= 0 || map.height <= 0 ||
        map.width*map.height > 16000000 || !Array.isArray(map.data) || map.data.length !== map.width*map.height ||
        !Number.isFinite(map.resolution) || map.resolution <= 0 || !map.frame_id) throw new Error('地图数据无效');
    const origin = map.origin.position, angle = yaw(map.origin.orientation);
    if (![origin.x, origin.y].every(Number.isFinite)) throw new Error('地图原点无效');
    const c=Math.cos(angle), s=Math.sin(angle), w=map.width*map.resolution, h=map.height*map.resolution;
    const corners = [[0,0],[w,0],[0,h],[w,h]].map(([x,y]) => [origin.x+c*x-s*y, origin.y+s*x+c*y]);
    return { angle, minX:Math.min(...corners.map(p=>p[0])), maxX:Math.max(...corners.map(p=>p[0])),
      minY:Math.min(...corners.map(p=>p[1])), maxY:Math.max(...corners.map(p=>p[1])) };
  }
  function raster(map) {
    // ROS row 0 is the bottom row; Canvas row 0 is the top row.
    const pixels = new Uint8ClampedArray(map.width*map.height*4);
    for (let y=0; y<map.height; y++) for (let x=0; x<map.width; x++) {
      const value=map.data[y*map.width+x];
      const shade = value < 0 || !Number.isFinite(value) ? 205 : Math.round(255*(1-Math.min(100,value)/100));
      const i=((map.height-1-y)*map.width+x)*4;
      pixels[i]=pixels[i+1]=pixels[i+2]=shade; pixels[i+3]=255;
    }
    return pixels;
  }
  class LiveMapView {
    constructor(canvas, notify, clock = Date.now) {
      this.canvas=canvas; this.notify=notify; this.clock=clock; this.epoch=0;
      this.cache=document.createElement('canvas');
      this.clear();
    }
    clear() {
      this.map=null; this.geometry=null; this.pose=null; this.scan=null; this.plan=null;this.latest=null;
      this.lastMessage=0; this.mapReceived=0; this.sampleReceived=0;
      this.connected=false; this.autoFit=true; this.needsFit=true; this.issue='等待地图连接';
    }
    close() {
      this.running=false; this.epoch++; clearTimeout(this.retry);
      if (this.ws) { this.ws.onclose=null; this.ws.close(); this.ws=null; }
      this.clear(); this.notify(this.status());
    }
    start(config) {
      this.close(); this.config=config; this.running=true; this.connect();
    }
    connect() {
      if (!this.running) return;
      const epoch=++this.epoch;
      this.started=this.clock(); this.connected=false; this.lastMessage=0;
      const ws=new WebSocket('ws://'+this.config.ip+':'+this.config.agentPort+'/ws/mapping');
      this.ws=ws;
      ws.onmessage=e => {
        if (epoch!==this.epoch) return;
        try { this.accept(JSON.parse(e.data)); }
        catch (err) { this.issue=err.message; this.notify(this.status()); }
      };
      ws.onerror=() => { if (epoch===this.epoch) { this.issue='地图连接失败，请确认完整后端已更新并重启网关'; this.notify(this.status()); } };
      ws.onclose=() => {
        if (epoch!==this.epoch) return;
        this.connected=false; this.pose=null; this.scan=null;
        this.issue=this.lastMessage ? '地图连接断开，正在重连；底图为最后收到的数据' :
          '地图服务未连接，请确认完整后端已更新并重启网关';
        this.notify(this.status());
        this.retry=setTimeout(()=>this.connect(),3000);
      };
    }
    accept(message) {
      if (message.type!=='mapping') return;
      this.lastMessage=this.clock();
      if (!message.available) {
        this.connected=false; this.pose=null; this.scan=null;
        this.issue=message.detail || '地图订阅未就绪'; this.notify(this.status()); return;
      }
      if (message.map) {
        const geometry=gridGeometry(message.map);
        const changed=!this.map || this.map.frame_id!==message.map.frame_id;
        this.map=message.map; this.geometry=geometry; this.mapReceived=this.clock();
        this.cache.width=this.map.width; this.cache.height=this.map.height;
        const ctx=this.cache.getContext('2d');
        const data=ctx.createImageData(this.map.width,this.map.height); data.data.set(raster(this.map));
        ctx.putImageData(data,0,0);
        if (changed) { this.autoFit=true; this.needsFit=true; }
        else if (this.autoFit) this.needsFit=true;
      }
      this.connected=true; this.latest=message; this.sampleReceived=this.clock();
      this.pose=message.pose && message.tf?.robot && ['x','y','yaw','age_sec'].every(k=>Number.isFinite(message.pose[k])) ? message.pose : null;
      this.scan=message.scan && message.tf?.scan && message.scan.frame_id===this.map?.frame_id ? message.scan : null;
      this.plan=message.plan && message.plan.frame_id===this.map?.frame_id ? message.plan:null;
      this.issue=(message.tf?.issues || []).join('；');
      this.notify(this.status());
    }
    tick() {
      if (!this.running) return;
      if (this.clock()-(this.lastMessage || this.started)>3500) {
        this.issue='地图数据超时，正在重连；底图为最后收到的数据';
        this.connected=false; this.pose=null; this.scan=null;
        if (this.ws) this.ws.close();
        this.lastMessage=this.clock();
      }
      this.notify(this.status());
    }
    validPose() {
      return this.connected && this.pose && this.pose.age_sec+(this.clock()-this.sampleReceived)/1000<2 ? this.pose : null;
    }
    validScan() {
      return this.connected && this.scan && Number.isFinite(this.scan.age_sec) &&
        this.scan.age_sec+(this.clock()-this.sampleReceived)/1000<2 ? this.scan : null;
    }
    status() {
      const pose=this.validPose(), scan=this.validScan();
      let text=this.issue;
      if (!text) text=this.map ? '实时地图 · '+this.map.width+' × '+this.map.height+' · '+(this.map.resolution*100).toFixed(1)+'cm/格' : '等待 /map；请启动 2D SLAM';
      else if (this.map && this.connected) text='地图已收到 · '+this.map.width+' × '+this.map.height+' · '+text;
      return { text, connected:this.connected, map:!!this.map, pose, scan,
        frame:this.map?.frame_id || 'map', mapAge:this.latest?.map_age_sec,
        tfText:'机器人 TF：'+(pose?'正常':'等待/过期')+' · 激光 TF：'+(scan?'正常':'等待/过期') };
    }
    fit(pan) {
      if (!this.geometry) return false;
      const g=this.geometry;
      pan.zoom=Math.max(0.1,Math.min(300,Math.min((this.canvas.width-60)/(g.maxX-g.minX),(this.canvas.height-100)/(g.maxY-g.minY))));
      pan.x=-(g.minX+g.maxX)*0.5*pan.zoom; pan.y=(g.minY+g.maxY)*0.5*pan.zoom;
      this.autoFit=true; this.needsFit=false; return true;
    }
    center(pan) {
      const p=this.validPose(); if (!p) return false;
      pan.x=-p.x*pan.zoom; pan.y=p.y*pan.zoom; this.autoFit=false; this.needsFit=false; return true;
    }
    render(pan,layers,estop,overlays={}) {
      const canvas=this.canvas, ctx=canvas.getContext('2d');
      const rect=canvas.parentElement.getBoundingClientRect();
      const width=Math.max(1,Math.floor(rect.width)), height=Math.max(1,Math.floor(rect.height));
      if (canvas.width!==width || canvas.height!==height) {
        canvas.width=width; canvas.height=height;
        if (this.autoFit) this.needsFit=true;
      }
      if (this.needsFit && this.map) this.fit(pan);
      ctx.fillStyle='#e2e8f0'; ctx.fillRect(0,0,width,height);
      const project=(x,y)=>[width/2+pan.x+x*pan.zoom,height/2+pan.y-y*pan.zoom];
      if (this.map && layers.map!==false) {
        const p=project(this.map.origin.position.x,this.map.origin.position.y);
        ctx.save(); ctx.translate(...p); ctx.rotate(-this.geometry.angle);
        const scale=this.map.resolution*pan.zoom; ctx.scale(scale,scale); ctx.imageSmoothingEnabled=false;
        ctx.drawImage(this.cache,0,-this.map.height); ctx.restore();
      }
      if (layers.plan && this.connected && this.plan && this.plan.age_sec+(this.clock()-this.sampleReceived)/1000<3) {
        ctx.strokeStyle='#10b981';ctx.lineWidth=3;ctx.beginPath();
        (this.plan.points || []).forEach((point,i)=>{const p=project(...point);if(i)ctx.lineTo(...p);else ctx.moveTo(...p);});ctx.stroke();
      }
      if (layers.waypoints) (overlays.waypoints || []).forEach((wp,i)=>{
        if (![wp.x,wp.y].every(Number.isFinite))return;
        const p=project(wp.x,wp.y);ctx.fillStyle=i===overlays.activeIndex?'#f59e0b':'#7c3aed';
        ctx.beginPath();ctx.arc(...p,7,0,Math.PI*2);ctx.fill();
        ctx.fillStyle='white';ctx.font='10px sans-serif';ctx.textAlign='center';ctx.fillText(String(i+1),p[0],p[1]+3);
      });
      if (overlays.initial) {
        const p=project(overlays.initial.x,overlays.initial.y),a=overlays.initial.yaw_deg*Math.PI/180;
        ctx.save();ctx.translate(...p);ctx.rotate(-a);ctx.strokeStyle='#f59e0b';ctx.lineWidth=3;
        ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(30,0);ctx.lineTo(20,-7);ctx.moveTo(30,0);ctx.lineTo(20,7);ctx.stroke();
        ctx.beginPath();ctx.arc(0,0,7,0,Math.PI*2);ctx.stroke();ctx.restore();
      }
      const scan=this.validScan();
      if (scan && layers.scan) {
        ctx.fillStyle='#ef4444';
        for (const point of scan.points || []) {
          if (!Array.isArray(point) || !point.every(Number.isFinite)) continue;
          const p=project(...point); ctx.fillRect(p[0]-1.5,p[1]-1.5,3,3);
        }
      }
      const pose=this.validPose();
      if (pose) {
        ctx.save(); ctx.translate(...project(pose.x,pose.y)); ctx.rotate(-pose.yaw);
        const w=Math.max(14,0.42*pan.zoom), h=Math.max(10,0.32*pan.zoom);
        ctx.fillStyle=estop?'#dc2626':'#2563eb'; ctx.strokeStyle='white';ctx.lineWidth=2;
        ctx.fillRect(-w/2,-h/2,w,h); ctx.strokeRect(-w/2,-h/2,w,h);
        ctx.fillStyle='white';ctx.beginPath();ctx.moveTo(w/2,0);ctx.lineTo(-w/6,-h/3);ctx.lineTo(-w/6,h/3);ctx.closePath();ctx.fill();
        ctx.restore();
      }
      if (!this.map) {
        ctx.fillStyle='#475569';ctx.font='13px sans-serif';ctx.textAlign='center';
        ctx.fillText('等待真实地图 /map',width/2,height/2);
      } else if (!this.connected) {
        ctx.fillStyle='rgba(255,255,255,0.35)';ctx.fillRect(0,0,width,height);
        ctx.fillStyle='#b45309';ctx.font='13px sans-serif';ctx.textAlign='center';
        ctx.fillText('连接已断开 · 最后收到的地图',width/2,height-20);
      }
      // A metric scale bar, independent of zoom level.
      const target=70/pan.zoom, power=10**Math.floor(Math.log10(target));
      const meters=[1,2,5,10].map(x=>x*power).find(x=>x>=target) || power*10;
      ctx.strokeStyle='#334155';ctx.fillStyle='#334155';ctx.lineWidth=2;
      ctx.beginPath();ctx.moveTo(16,height-22);ctx.lineTo(16+meters*pan.zoom,height-22);ctx.stroke();
      ctx.font='11px sans-serif';ctx.textAlign='left';ctx.fillText(meters.toPrecision(2)+' m',16,height-28);
    }
  }
  global.CockpitMapMath={yaw,gridGeometry,raster};
  global.LiveMapView=LiveMapView;
})(typeof window==='undefined'?globalThis:window);
