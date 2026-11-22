const assert = require('node:assert/strict');
require('../frontend/live_map.js');
const {gridGeometry,raster,yaw,costRaster} = globalThis.CockpitMapMath;
const map={frame_id:'map',width:2,height:2,resolution:1,data:[0,100,-1,50],
  origin:{position:{x:10,y:20,z:0},orientation:{x:0,y:0,z:0,w:1}}};
assert.deepEqual(Array.from(raster(map)),[205,205,205,255,128,128,128,255,255,255,255,255,0,0,0,255]);
let g=gridGeometry(map);assert.equal(g.minX,10);assert.equal(g.maxY,22);
map.origin.orientation={x:0,y:0,z:Math.sqrt(0.5),w:Math.sqrt(0.5)};
g=gridGeometry(map);assert.ok(Math.abs(g.minX-8)<1e-10);assert.ok(Math.abs(g.maxY-22)<1e-10);
assert.ok(Math.abs(yaw(map.origin.orientation)-Math.PI/2)<1e-10);
assert.throws(()=>gridGeometry({...map,data:[]}),/地图数据无效/);
assert.throws(()=>yaw({x:0,y:0,z:0,w:0}),/四元数无效/);
const costs={...map,data:[0,253,254,255]};
assert.deepEqual(Array.from(costRaster(costs)),[220,38,38,215,124,58,237,70,0,0,0,0,249,115,22,190]);
console.log('Map geometry OK: ROS row orientation, occupancy shades, translated/rotated grid, invalid data.');
