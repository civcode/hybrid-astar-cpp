'use strict';
const assert=require('node:assert/strict');
const M=require('./map_geometry.js');
const origin=[5,-2,Math.PI/2];
const [x,y]=M.toWorld(8,15,origin);
assert(Math.abs(x+10)<1e-10);
assert(Math.abs(y-6)<1e-10);
const back=M.toLocal(x,y,origin);
assert(Math.abs(back[0]-8)<1e-10);
assert(Math.abs(back[1]-15)<1e-10);
for(const yaw of [0,.123,Math.PI/2,-1.7,Math.PI]) {
  const t=[-4.5,19,yaw];
  for(const p of [[0,0],[20,8],[-5,11]]) {
    const world=M.toWorld(p[0],p[1],t);
    const local=M.toLocal(...world,t);
    assert(Math.hypot(local[0]-p[0],local[1]-p[1])<1e-10);
  }
}
console.log('Map geometry round trips PASS');
