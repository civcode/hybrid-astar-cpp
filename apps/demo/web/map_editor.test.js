'use strict';
const assert=require('node:assert/strict');
const MapEditor=require('./map_editor.js');
const MapGeometry=require('./map_geometry.js');
const width=20,height=10,resolution=.25;
const geometry={ox:20,oy:30,mh:height*resolution,scale:10};
const origin=[5,-2,Math.PI/2];
// Rotated world position lies on local cell (3, 4), not on an axis-aligned
// world-map cell. Canvas brushes must be indexed in map-local coordinates.
const localX=3.5*resolution,localY=4.5*resolution;
const [wx,wy]=MapGeometry.toWorld(localX,localY,origin);
const [lx,ly]=MapGeometry.toLocal(wx,wy,origin);
const px=geometry.ox+lx*geometry.scale,py=geometry.oy+(geometry.mh-ly)*geometry.scale;
assert.deepEqual(MapEditor.cellFromCanvas(px,py,geometry,width,height,resolution),[3,4]);
assert.equal(MapEditor.cellFromCanvas(geometry.ox-1,py,geometry,width,height,resolution),null);
assert.equal(MapEditor.cellFromCanvas(px,geometry.oy-1,geometry,width,height,resolution),null);
const base=new Int16Array(width*height);base[4*width+3]=100;
const edits=new Map();
assert(MapEditor.edit(edits,base,4*width+3,'free'));
assert.equal(edits.get(4*width+3),0);
assert(!MapEditor.edit(edits,base,4*width+3,'free'));
assert(MapEditor.edit(edits,base,4*width+3,'restore'));
assert.equal(edits.size,0);
assert(MapEditor.edit(edits,base,5,'unknown'));
assert(MapEditor.edit(edits,base,2,'blocked'));
assert.equal(MapEditor.encode(edits),'2:100,5:-1');
assert.deepEqual(MapEditor.brushCells(0,0,1,width,height).sort((a,b)=>a-b),[0,1,width]);
assert.throws(()=>MapEditor.edit(new Map([[0,100]]),base,3,'blocked',1),/Maximum/);
console.log('Map editor cell and sparse revision contracts PASS');
