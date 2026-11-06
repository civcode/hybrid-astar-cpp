'use strict';
// Runs the REAL Planning Lab drawing and form handlers with a headless
// canvas/DOM. No server, browser, GitHub Actions or dependencies required.
// Run: node tests/demo_articulated_ui_test.js
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const root=path.resolve(__dirname,'../apps/demo/web');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const script=fs.readFileSync(path.join(root,'app.js'),'utf8');
const MapGeometry=require(path.join(root,'map_geometry.js'));
const MapEditor=require(path.join(root,'map_editor.js'));
const ids=new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]));
for(const [,id] of script.matchAll(/\$\('([^']+)'\)/g))
  assert.ok(ids.has(id),'Missing HTML element '+id);
const controls=['artStartGamma','artGoalGamma','artLf','artLr',
  'artFrontRearX','artFrontFrontX','artFrontHalfWidth',
  'artRearRearX','artRearFrontX','artRearHalfWidth'];
for(const id of [...controls,'artHeuristicMode','artHeuristicWeight',
                  'artCoarseGridResolution','artHeadingGuidanceWeight','artCoupledMotion','artSweptAabb',
                  'artGoalConnector','artAnytimeRefinement','artAnytimeExtraExpansions'])
  assert.ok(ids.has(id),'Missing articulated input '+id);
assert.match(html, /<option value="grid" selected>/,
  'Preserve the existing grid-only default');
assert.match(html, /<option value="combined">/,
  'Expose the recommended combined mode');
const defaultChecked=new Set(
  [...html.matchAll(/<input[^>]*\bid="([^"]+)"[^>]*>/g)]
    .filter(m=>/\bchecked\b/.test(m[0])).map(m=>m[1]));
const defaultInputs=new Map([...html.matchAll(/<input[^>]*\bid="([^"]+)"[^>]*>/g)]
  .map(m=>[m[1],(m[0].match(/\bvalue="([^"]*)"/)||[])[1]||'']));
const polygons=[],arcs=[];let points=[];
const ctx={setTransform(){},clearRect(){},fillRect(){},
  beginPath(){points=[]},moveTo(x,y){points=[[x,y]]},
  lineTo(x,y){points.push([x,y])},
  closePath(){if(points.length===4)polygons.push(points.map(p=>p.slice()))},
  arc(x,y,r){arcs.push({x,y,r})},fill(){},stroke(){},
  save(){},restore(){},setLineDash(){}};
const canvas={clientWidth:1000,clientHeight:640,width:0,height:0,
  getContext(){return ctx},addEventListener(){},
  getBoundingClientRect(){return {left:0,top:0}}};
const elements=new Map();
function el(id){
  if(id==='map')return canvas;
  if(!elements.has(id))elements.set(id,{
    value:defaultInputs.get(id)||'',
    checked:defaultChecked.has(id),
    disabled:false,hidden:false,textContent:'',className:'',
    listeners:{},addEventListener(name,handler){this.listeners[name]=handler},
    classList:{add(){},remove(){}}
  });
  return elements.get(id);
}
el('scenario').value='open';el('model').value='ackermann';
el('artHeuristicMode').value='grid'; // Browser selects the 'selected' option.
el('position').value='0';el('editMode').value='start';el('brushSize').value='0';
let posted=null;
const result={model:'articulated',width:200,height:160,resolution:.25,
  origin:[0,0,0],revision:1,mapRuns:[],
  articulatedGeometry:{Lf:1,Lr:1,front:[-.5,.5,.3],rear:[-.5,.5,.3]},
  footprintInflationMeters:0,refinement:null,milliseconds:3,
  limits:{maxExpansions:1000},
  articulatedSearch:{heuristicMode:'grid',heuristicWeight:1.0,
    enableCoupledMotion:true,useSweptAabbPrefilter:true},
  original:{status:'Success',validated:true,cost:1,length:1,
    reverseLength:0,expanded:4,goalGammaErrorRadians:0,
    precomputedAnalyticDriveTemplates:30,
    precomputedStationaryRk4Templates:28,
    appliedPrecomputedTemplates:176,
    sections:[{kind:'articulation',gear:'articulation',
      points:[[10,15,0],[10.01,15,-.1]],
      articulatedPoints:[[10,15,0,0],[10.01,15,-.1,.2]]}]}};
const sandbox={document:{getElementById:el},window:{devicePixelRatio:1},
  ResizeObserver:class{observe(){}},MapGeometry,MapEditor,
  URLSearchParams,AbortController,console,
  fetch:async(url,opts)=>{
    assert.equal(url,'/api/plan');
    posted=opts.body;
    return {ok:true,json:async()=>result};
  },
  requestAnimationFrame(){},setTimeout(){}};
vm.createContext(sandbox);
// Omit only the browser auto-bootstrap so tests deterministically invoke
// actual UI functions and registered event handlers.
const bootstrap='new ResizeObserver(draw).observe(canvas);';
assert.ok(script.includes(bootstrap));
vm.runInContext(script.slice(0,script.indexOf(bootstrap)),sandbox,
  {filename:'apps/demo/web/app.js'});
const call=src=>vm.runInContext(src,sandbox);
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-8,String(a)+' != '+String(b));
const center=poly=>poly.reduce((acc,[x,y])=>
  [acc[0]+x/4,acc[1]+y/4],[0,0]);
async function main(){
  call('resetScenario();syncModelOptions()');
  assert.ok(polygons.length>=2,'rigid previews must still render');
  el('model').value='articulated';
  call('syncModelOptions()');
  assert.equal(el('articulatedControls').hidden,false);
  assert.equal(el('artHeuristicMode').value,'grid');
  assert.equal(el('artHeuristicWeight').value,'1.2');
  assert.equal(el('artCoarseGridResolution').value,'0.15');
  assert.equal(el('artCoarseGridResolution').disabled,true);
  assert.equal(el('artHeadingGuidanceWeight').value,'0.8');
  assert.equal(el('artHeadingGuidanceWeight').disabled,true);
  assert.equal(el('artCoupledMotion').checked,true);
  assert.equal(el('artSweptAabb').checked,true);
  assert.equal(el('artGoalConnector').checked,false);
  assert.equal(el('artAnytimeRefinement').checked,false);
  assert.equal(el('artAnytimeRefinement').disabled,true);
  assert.equal(el('artAnytimeExtraExpansions').value,'2000');
  assert.equal(el('artAnytimeExtraExpansions').disabled,true);
  for(const id of ['artHeuristicMode','artHeuristicWeight',
                   'artCoupledMotion','artSweptAabb','artGoalConnector'])
    assert.equal(el(id).disabled,false,id+' must be active');
  for(const id of ['ompl','refine','rsDistance','improve','curved','performance'])
    assert.equal(el(id).disabled,true,id+' must be disabled');
  const count=polygons.length;
  call('draw()');
  assert.equal(polygons.length-count,4,'two bodies at each start and goal');
  const bodies=polygons.slice(-4),scale=call('canvasGeometry().scale');
  const rear=center(bodies[0]),front=center(bodies[1]);
  near(Math.hypot(front[0]-rear[0],front[1]-rear[1])/scale,2);
  function size(poly){
    return [Math.hypot(poly[1][0]-poly[0][0],poly[1][1]-poly[0][1])/scale,
      Math.hypot(poly[2][0]-poly[1][0],poly[2][1]-poly[1][1])/scale];
  }
  near(size(bodies[0])[0],1);near(size(bodies[0])[1],.6);
  near(size(bodies[1])[0],1);near(size(bodies[1])[1],.6);
  el('artStartGamma').value='45';
  const before=polygons.length;
  el('artStartGamma').listeners.input();
  assert.ok(polygons.length>=before+4,'joint edit redraws both bodies');
  const turned=polygons.slice(-4);
  assert.ok(Math.abs(center(turned[1])[1]-center(bodies[1])[1])>2,
    'front moves with changed joint angle');
  el('artStartGamma').value='0';el('artLf').value='.7';
  el('artLf').listeners.input();
  const changed=polygons.slice(-4);
  near(Math.hypot(center(changed[1])[0]-center(changed[0])[0],
    center(changed[1])[1]-center(changed[0])[1])/scale,1.7);
  el('artLf').value='1';
  await call('replan()');
  assert.ok(posted,'actual replan() must POST');
  assert.equal(posted.get('model'),'articulated');
  for(const id of controls)assert.ok(posted.has(id),'Missing POST '+id);
  assert.equal(posted.get('artHeuristicMode'),'grid');
  assert.equal(posted.get('artHeuristicWeight'),'1.2');
  assert.equal(posted.get('artCoarseGridResolution'),'0.15');
  assert.equal(posted.get('artHeadingGuidanceWeight'),'0.8');
  assert.equal(posted.get('artCoupledMotion'),'1');
  assert.equal(posted.get('artSweptAabb'),'1');
  assert.equal(posted.get('artGoalConnector'),'0');
  assert.equal(posted.get('artAnytimeRefinement'),'0');
  assert.equal(posted.get('artAnytimeExtraExpansions'),'2000');
  for(const id of ['ompl','refine','improve'])
    assert.equal(posted.get(id),'0','Unsupported rigid feature '+id);
  assert.equal(el('exportButton').disabled,false);
  assert.equal(el('curvatureVariationLabel').textContent,'GOAL γ ERROR');
  assert.ok(el('message').textContent.includes('58 cached primitives'),
    'Planning Lab reports precomputed primitive-library statistics');
  assert.ok(el('message').textContent.includes('176 applications'),
    'Planning Lab reports reuse rather than per-expansion RK4');
  assert.ok(el('message').textContent.includes('Obstacle grid ×1'),
    'Effective heuristic should appear in the result readout');
  result.original.goalConnectorTailFittedArcs=2;
  result.original.goalConnectorTailDrivingMeters=4.25;
  result.original.goalConnectorTailStationarySeconds=1.5;
  result.original.goalConnectorTailArticulationRadians=0.8;
  result.original.goalConnectorTailArticulationReversals=1;
  result.original.goalConnectorTailGearChanges=0;
  await call('replan()');
  assert.ok(el('message').textContent.includes('tail 4.3 m'),
    'Status includes terminal driving distance');
  assert.ok(el('message').textContent.includes('stationary 1.5 s'),
    'Status includes terminal stationary articulation time');
  assert.ok(el('message').textContent.includes('reversals 1'),
    'Status includes terminal articulation reversals');
  delete result.original.goalConnectorTailFittedArcs;
  delete result.original.goalConnectorTailDrivingMeters;
  delete result.original.goalConnectorTailStationarySeconds;
  delete result.original.goalConnectorTailArticulationRadians;
  delete result.original.goalConnectorTailArticulationReversals;
  delete result.original.goalConnectorTailGearChanges;

  el('artHeuristicMode').value='combined';
  el('artHeuristicMode').listeners.change();
  assert.equal(el('artHeuristicWeight').value,'1.2',
    'Changing mode retains the initial UI default');
  assert.equal(el('exportButton').disabled,true,'Changing the heuristic invalidates old route');
  await call('replan()');
  assert.equal(posted.get('artHeuristicMode'),'combined');
  assert.equal(posted.get('artHeuristicWeight'),'1.2');
  el('artHeuristicWeight').value='1.4';
  el('artHeuristicWeight').listeners.input();
  el('artCoupledMotion').checked=false;
  el('artCoupledMotion').listeners.change();
  el('artSweptAabb').checked=false;
  el('artSweptAabb').listeners.change();
  await call('replan()');
  assert.equal(posted.get('artHeuristicWeight'),'1.4');
  assert.equal(posted.get('artCoupledMotion'),'0');
  assert.equal(posted.get('artSweptAabb'),'0');
  el('artHeuristicMode').value='heading';
  el('artHeuristicMode').listeners.change();
  assert.equal(el('artHeuristicWeight').value,'1.4',
    'User-entered weight must persist when changing heuristic modes');
  assert.equal(el('artHeadingGuidanceWeight').disabled,false);
  el('artHeadingGuidanceWeight').value='1.4';
  el('artHeadingGuidanceWeight').listeners.input();
  await call('replan()');
  assert.equal(posted.get('artHeuristicMode'),'heading');
  assert.equal(posted.get('artHeadingGuidanceWeight'),'1.4');
  el('artHeuristicMode').value='coarse';
  el('artHeuristicMode').listeners.change();
  assert.equal(el('artCoarseGridResolution').disabled,false);
  assert.equal(el('artHeuristicWeight').value,'1.4');
  el('artCoarseGridResolution').value='0.18';
  el('artCoarseGridResolution').listeners.input();
  assert.equal(el('exportButton').disabled,true);
  await call('replan()');
  assert.equal(posted.get('artHeuristicMode'),'coarse');
  assert.equal(posted.get('artCoarseGridResolution'),'0.18');
  assert.equal(posted.get('artHeuristicWeight'),'1.4');
  el('artGoalConnector').checked=true;
  el('artGoalConnector').listeners.change();
  await call('replan()');
  assert.equal(posted.get('artGoalConnector'),'1');
  assert.equal(el('artAnytimeRefinement').disabled,false);
  el('artAnytimeRefinement').checked=true;
  el('artAnytimeRefinement').listeners.change();
  assert.equal(el('artAnytimeExtraExpansions').disabled,false);
  el('artAnytimeExtraExpansions').value='80';
  el('artAnytimeExtraExpansions').listeners.input();
  await call('replan()');
  assert.equal(posted.get('artAnytimeRefinement'),'1');
  assert.equal(posted.get('artAnytimeExtraExpansions'),'80');
  el('artGoalConnector').checked=false;
  el('artGoalConnector').listeners.change();
  assert.equal(el('artAnytimeRefinement').disabled,true);
  assert.equal(el('artAnytimeExtraExpansions').disabled,true);
  await call('replan()');
  assert.equal(posted.get('artAnytimeRefinement'),'0');
  el('artHeuristicMode').value='analytic';
  el('artHeuristicMode').listeners.change();
  assert.equal(el('artHeuristicWeight').value,'1.4',
    'Custom weight must persist when selecting analytic mode');
  assert.equal(el('artHeadingGuidanceWeight').disabled,true);
  assert.equal(el('artCoarseGridResolution').disabled,true);
  await call('replan()');
  assert.equal(posted.get('artHeuristicMode'),'analytic');

  const n=polygons.length;
  el('position').value='1000';call('draw()');
  assert.equal(polygons.length-n,6,'start+goal+replay each have two bodies');
  assert.ok(arcs.some(a=>a.r===4),'draws the hinge marker');
  call('baseMap={width:200,height:160,resolution:.25,origin:[4,-3,.47],mapRuns:[],revision:1}');
  const last=polygons.length;call('draw()');
  assert.equal(polygons.length-last,6,'rotated origin preserves both bodies');
  el('model').value='ackermann';
  call('syncModelOptions()');
  assert.equal(el('articulatedControls').hidden,true);
  for(const id of ['artHeuristicMode','artHeuristicWeight',
                   'artCoarseGridResolution','artHeadingGuidanceWeight','artCoupledMotion','artSweptAabb',
                  'artGoalConnector','artAnytimeRefinement','artAnytimeExtraExpansions'])
    assert.equal(el(id).disabled,true,id+' must not affect rigid models');
  console.log('PASS: articulated UI selection, optimizations, presets, POST and replay');
}
main().catch(error=>{console.error(error);process.exitCode=1});
