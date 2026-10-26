'use strict';
const MapEditor = Object.freeze({
  // Canvas is map-local, irrespective of the map's world origin / rotation.
  cellFromCanvas(px,py,geometry,width,height,resolution) {
    const x=Math.floor((px-geometry.ox)/geometry.scale/resolution);
    const y=Math.floor((geometry.mh-(py-geometry.oy)/geometry.scale)/resolution);
    if(x<0||x>=width||y<0||y>=height)return null;
    return [x,y];
  },
  brushCells(x,y,radius,width,height) {
    const cells=[];
    for(let dy=-radius;dy<=radius;++dy)for(let dx=-radius;dx<=radius;++dx) {
      if(dx*dx+dy*dy>radius*radius)continue;
      const xx=x+dx,yy=y+dy;
      if(xx>=0&&xx<width&&yy>=0&&yy<height)cells.push(yy*width+xx);
    }
    return cells;
  },
  // Overrides are a canonical, ascending snapshot relative to the base map.
  encode(edits) {
    return [...edits.entries()].sort((a,b)=>a[0]-b[0])
      .map(([index,value])=>index+':'+value).join(',');
  },
  edit(edits,baseCells,index,mode,cap=4096) {
    if(!baseCells||index<0||index>=baseCells.length)return false;
    const value=mode==='blocked'?100:mode==='free'?0:mode==='unknown'?-1:null;
    const old=edits.has(index)?edits.get(index):baseCells[index];
    const next=value===null?baseCells[index]:value;
    if(old===next)return false;
    if(next===baseCells[index])edits.delete(index);
    else {
      if(!edits.has(index)&&edits.size>=cap)throw Error('Maximum 4096 edited cells per plan');
      edits.set(index,next);
    }
    return true;
  }
});
if(typeof module!=='undefined'&&module.exports)module.exports=MapEditor;
