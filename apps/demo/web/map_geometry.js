'use strict';
// Map origin transforms map-local metres to world metres, yaw in radians.
const MapGeometry = Object.freeze({
  toLocal(worldX,worldY,origin) {
    const dx=worldX-origin[0],dy=worldY-origin[1];
    const c=Math.cos(origin[2]),s=Math.sin(origin[2]);
    return [c*dx+s*dy,-s*dx+c*dy];
  },
  toWorld(localX,localY,origin) {
    const c=Math.cos(origin[2]),s=Math.sin(origin[2]);
    return [origin[0]+c*localX-s*localY,origin[1]+s*localX+c*localY];
  }
});
if(typeof module!=='undefined'&&module.exports)module.exports=MapGeometry;
