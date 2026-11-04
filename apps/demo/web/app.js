'use strict';
const $ = id => document.getElementById(id);
const defaults = {
  open:      {name:'Open field',start:[10,15,0],goal:[17,15,0]},
  reverse:   {name:'Reverse maneuver',start:[10,15,0],goal:[6,15,0]},
  parking:   {name:'Parking approach',start:[10,15,0],goal:[8,16,0]},
  warehouse: {name:'Warehouse obstacles',start:[8,20,0],goal:[43,20,0]},
  blocked:   {name:'Unreachable wall',start:[10,20,0],goal:[35,20,0]}
};
let snapshot=null, baseMap=null, baseCells=null, pending=null;
let animation=false, lastFrame=0, routeValid=false, generation=0;
let editMap=new Map(), pointerPainting=false, strokeChanged=false;
let customOrigin=[0,0,0];
const canvas=$('map'), ctx=canvas.getContext('2d');
function setPose(prefix,v) {['x','y','yaw'].forEach((s,i)=>$(prefix+s).value=Number(v[i]).toFixed(i===2?3:2));}
function resetScenario() {
  const d=defaults[$('scenario').value];
  $('sceneTitle').textContent=d.name;
  setPose('start',d.start);setPose('goal',d.goal);
  $('position').value=0;draw();
}
function canvasGeometry() {
  const dpr=window.devicePixelRatio||1;
  const w=canvas.clientWidth,h=canvas.clientHeight;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)) {
    canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  const map=baseMap||snapshot;
  const mw=(map?.width||200)*(map?.resolution||.25);
  const mh=(map?.height||160)*(map?.resolution||.25);
  const scale=Math.min((w-42)/mw,(h-42)/mh);
  const ox=(w-mw*scale)/2,oy=(h-mh*scale)/2;
  return {w,h,mw,mh,scale,ox,oy};
}
function activeOrigin() {
  return baseMap?.origin||snapshot?.origin||($('scenario').value==='custom'?customOrigin:[0,0,0]);
}
function localPoint(x,y,g) {return [g.ox+x*g.scale,g.oy+(g.mh-y)*g.scale];}
function mapPoint(x,y,g) {
  const [lx,ly]=MapGeometry.toLocal(x,y,activeOrigin());
  return localPoint(lx,ly,g);
}
function worldPoint(px,py,g) {
  const lx=(px-g.ox)/g.scale,ly=g.mh-(py-g.oy)/g.scale;
  return MapGeometry.toWorld(lx,ly,activeOrigin());
}
// Draw the real model-specific rectangular body at ANY pose, including
// editable start/goal poses before a successful planning response exists.
function drawFootprint(p,color,g,customFootprint=null) {
  const isCompact=$('model').value==='differential'||$('model').value==='mecanum';
  // Never draw a stale result's footprint after a model/route change.
  const footprint=customFootprint || ((routeValid && snapshot?.model===$('model').value &&
                   Array.isArray(snapshot.footprint) &&
                   snapshot.footprint.length===3)
      ?snapshot.footprint:(isCompact?[-.35,.35,.25]:[-1,3.6,.9]));
  const [rear,frontX,halfWidth]=footprint;
  const c=Math.cos(p[2]),s=Math.sin(p[2]);
  const corners=[[rear,-halfWidth],[frontX,-halfWidth],
                 [frontX,halfWidth],[rear,halfWidth]];
  ctx.save();
  ctx.beginPath();
  corners.forEach(([u,v],i)=>{
    const [sx,sy]=mapPoint(p[0]+c*u-s*v,p[1]+s*u+c*v,g);
    if(!i)ctx.moveTo(sx,sy);else ctx.lineTo(sx,sy);
  });
  ctx.closePath();
  // Render a round-joined metric inflation stroke when selected. The
  // inflated outline is a display aid; the C++ checker owns validity.
  const inflation=routeValid && snapshot?.model===$('model').value
      ?Number(snapshot.footprintInflationMeters||0)
      :($('inflate').checked?Number($('inflationMeters').value):0);
  if(inflation>0 && Number.isFinite(inflation)) {
    ctx.save();
    ctx.lineJoin='round';ctx.lineWidth=2*inflation*g.scale;
    ctx.strokeStyle='rgba(255,188,102,.45)';
    ctx.stroke();ctx.restore();
  }
  ctx.fillStyle=color;ctx.globalAlpha=.18;ctx.fill();
  ctx.globalAlpha=.92;ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.stroke();
  ctx.restore();
}
// Front reference F = rear base_link R + Lr*heading(thetaRear)
//                  + Lf*heading(thetaRear + gamma).
// Each collision body uses its OWN reference, yaw and configured rectangle.
function articulatedGeometry() {
  if(routeValid && snapshot?.model==='articulated' &&
     snapshot.articulatedGeometry)return snapshot.articulatedGeometry;
  const n=id=>Number($(id).value);
  return {Lf:n('artLf'),Lr:n('artLr'),
    front:[n('artFrontRearX'),n('artFrontFrontX'),n('artFrontHalfWidth')],
    rear:[n('artRearRearX'),n('artRearFrontX'),n('artRearHalfWidth')]};
}
function drawArticulatedPose(p,color,g) {
  const [xr,yr,thetaRear,gamma]=p, geometry=articulatedGeometry();
  if(![xr,yr,thetaRear,gamma,geometry.Lf,geometry.Lr,
       ...geometry.front,...geometry.rear].every(Number.isFinite) ||
     geometry.Lf<=0 || geometry.Lr<=0 ||
     geometry.front[1]<=geometry.front[0] ||
     geometry.rear[1]<=geometry.rear[0] ||
     geometry.front[2]<=0 || geometry.rear[2]<=0)return;
  const thetaFront=thetaRear+gamma;
  const hx=xr+geometry.Lr*Math.cos(thetaRear);
  const hy=yr+geometry.Lr*Math.sin(thetaRear);
  const fx=hx+geometry.Lf*Math.cos(thetaFront);
  const fy=hy+geometry.Lf*Math.sin(thetaFront);
  drawFootprint([xr,yr,thetaRear],color,g,geometry.rear);
  drawFootprint([fx,fy,thetaFront],color,g,geometry.front);
  const [rx,ry]=mapPoint(xr,yr,g),[jx,jy]=mapPoint(hx,hy,g);
  const [sx,sy]=mapPoint(fx,fy,g);
  ctx.save();
  ctx.strokeStyle=color;ctx.lineWidth=1.5;
  ctx.setLineDash([3,4]);ctx.beginPath();
  ctx.moveTo(rx,ry);ctx.lineTo(jx,jy);ctx.lineTo(sx,sy);ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle='#ffc77b';ctx.beginPath();ctx.arc(jx,jy,4,0,2*Math.PI);ctx.fill();
  ctx.restore();
  // Rear is the interactive start/goal reference; show both yaw rays.
  drawPose([xr,yr,thetaRear],color,g,false);
  drawPose([fx,fy,thetaFront],color,g,false);
}
function drawPose(p,color,g,body=false) {
  if(body && $('model').value==='articulated') {
    drawArticulatedPose(p,color,g);
    return;
  }
  if(body)drawFootprint(p,color,g);
  // Marker and yaw ray belong on top of the body, retaining the existing
  // start/goal visual encoding and making the reference point unambiguous.
  const [x,y]=mapPoint(p[0],p[1],g);
  ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2.5;
  ctx.beginPath();ctx.arc(x,y,5.5,0,Math.PI*2);ctx.fill();
  const isCompact=$('model').value==='differential'||$('model').value==='mecanum';
  const headLength=isCompact||$('model').value==='articulated' ? 0.35 : 1.6;
  const [tx,ty]=mapPoint(p[0]+headLength*Math.cos(p[2]),p[1]+headLength*Math.sin(p[2]),g);
  ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(tx,ty);ctx.stroke();
}
function drawSections(path,g,muted=false) {
  if(!path)return;
  ctx.lineCap='round';ctx.lineJoin='round';
  for(const section of path.sections) {
    if(!section.points.length)continue;
    if(section.kind==='rotation') {
      const anchor=section.points[0];
      const [cx,cy]=mapPoint(anchor[0],anchor[1],g);
      ctx.beginPath();ctx.arc(cx,cy,9,0,Math.PI*1.75,section.turnRadians>0);
      ctx.strokeStyle=muted?'rgba(155,177,204,.5)':'#b69eff';
      ctx.lineWidth=2.5;ctx.stroke();
      for(const p of [section.points[0],section.points.at(-1)]) {
        const [ex,ey]=mapPoint(p[0]+.45*Math.cos(p[2]),p[1]+.45*Math.sin(p[2]),g);
        ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(ex,ey);ctx.stroke();
      }
      continue;
    }
    ctx.beginPath();section.points.forEach((p,i)=>{
      const [x,y]=mapPoint(p[0],p[1],g);if(!i)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    });
    ctx.strokeStyle=muted?'rgba(155,177,204,.5)':section.kind==='articulation'?'#b69eff':section.kind==='holonomic'?'#7db2ff':section.gear==='reverse'?'#e4ac70':'#55dac6';
    ctx.lineWidth=muted?2:3.5;ctx.setLineDash(muted?[5,6]:[]);ctx.stroke();ctx.setLineDash([]);
  }
}
function activePath() {
  if(!snapshot||!routeValid)return null;
  return snapshot.refinement?.path||snapshot.original;
}
function allPositions(path) {
  return path?.sections.flatMap((sec,i)=>{
    const points=snapshot?.model==='articulated' ? (sec.articulatedPoints||[]) : sec.points;
    return i?points.slice(1):points;
  })||[];
}
function draw() {
  const g=canvasGeometry();ctx.clearRect(0,0,g.w,g.h);
  ctx.fillStyle='#112238';ctx.fillRect(g.ox,g.oy,g.mw*g.scale,g.mh*g.scale);
  ctx.strokeStyle='#2c4157';ctx.lineWidth=.7;
  for(let x=0;x<=g.mw;x+=5){const [sx]=localPoint(x,0,g);ctx.beginPath();ctx.moveTo(sx,g.oy);ctx.lineTo(sx,g.oy+g.mh*g.scale);ctx.stroke();}
  for(let y=0;y<=g.mh;y+=5){const [,sy]=localPoint(0,y,g);ctx.beginPath();ctx.moveTo(g.ox,sy);ctx.lineTo(g.ox+g.mw*g.scale,sy);ctx.stroke();}
  if(baseMap) {
    for(const [x,y,length,value] of baseMap.mapRuns) {
      const [sx,sy]=localPoint(x*baseMap.resolution,(y+1)*baseMap.resolution,g);
      ctx.fillStyle=value===-1?'#a58b61':'#71859e';
      ctx.fillRect(sx,sy,Math.max(1,length*baseMap.resolution*g.scale),
                   Math.max(1,baseMap.resolution*g.scale));
    }
    for(const [index,value] of editMap) {
      const x=index%baseMap.width,y=Math.floor(index/baseMap.width);
      const [sx,sy]=localPoint(x*baseMap.resolution,(y+1)*baseMap.resolution,g);
      ctx.fillStyle=value===-1?'#a58b61':value===100?'#71859e':'#112238';
      ctx.fillRect(sx,sy,Math.max(1,baseMap.resolution*g.scale),
                   Math.max(1,baseMap.resolution*g.scale));
    }
  }
  const path=activePath();
  if(path&&snapshot.refinement?.path&&$('showOriginal').checked)
    drawSections(snapshot.original,g,true);
  if(path)drawSections(path,g);
  const start=['startx','starty','startyaw'].map(id=>Number($(id).value));
  const goal=['goalx','goaly','goalyaw'].map(id=>Number($(id).value));
  const art=$('model').value==='articulated';
  if(art) {
    start.push(Number($('artStartGamma').value)*Math.PI/180);
    goal.push(Number($('artGoalGamma').value)*Math.PI/180);
  }
  if(start.every(Number.isFinite))drawPose([start[0],start[1],start[2]*Math.PI/180,...(art?[start[3]]:[])],'#5dd5ff',g,true);
  if(goal.every(Number.isFinite))drawPose([goal[0],goal[1],goal[2]*Math.PI/180,...(art?[goal[3]]:[])],'#eac47d',g,true);
  const samples=allPositions(path);
  if(samples.length)drawPose(samples[Math.min(samples.length-1,Math.round(Number($('position').value)/1000*(samples.length-1)))],'#6ce8d3',g,true);
}
// Match the benchmark metrics: adjacent translated-section curvature jumps,
// plus changes in sign across nonzero steering segments.
function steeringMetrics(path) {
  let totalVariation=0, reversals=0, prior=null, priorSign=0;
  for(const sec of path?.sections||[]) {
    if(sec.kind==='translation') {
      if(prior?.kind==='translation')
        totalVariation+=Math.abs(sec.curvature-prior.curvature);
      const sign=sec.curvature>1e-9?1:sec.curvature< -1e-9?-1:0;
      if(sign && priorSign && sign!==priorSign)reversals++;
      if(sign)priorSign=sign;
    }
    prior=sec;
  }
  return {totalVariation,reversals};
}
function stats(result) {
  const path=activePath(),valid=path?.status==='Success';
  $('message').className='readout '+(valid?'good':'bad');
  const status=result.original.status;
  const refinement=result.refinement?.status;
  const acceptedLabel=result.refinement
    ?' · accepted '+result.refinement.accepted+'/'+result.refinementOptions.maxAccepted:'';
  const limit=result.limits;
  const exhausted=result.original.terminationReason==='WorkBudgetExceeded'||
    status==='BudgetExceeded'||result.original.terminationReason==='ValidationBudgetExhausted';
  const failureDetail=status==='StrictValidationFailed'
    ?(' · goal-region candidates failed independent validation'
       +' (collision '+(result.original.strictRejectedCollision||0)
       +', continuity '+(result.original.strictRejectedContinuity||0)
       +', goal '+(result.original.strictRejectedGoalMismatch||0)+')')
    :(exhausted?' · search work budget exhausted'
       :' · search exhaustion is not a proof of continuous-space infeasibility');
  const cachedArticulated=result.model==='articulated'?
    (result.original.precomputedAnalyticDriveTemplates||0)+
    (result.original.precomputedStationaryRk4Templates||0):0;
  const templateNote=cachedArticulated>0?
    ' · '+cachedArticulated.toLocaleString()+' cached primitives'+
    ', '+(result.original.appliedPrecomputedTemplates||0).toLocaleString()+
    ' applications':'';
  $('message').textContent=(valid?(result.model==='articulated'
    ?'Experimental articulated path · '+status+' · two-carriage independently sampled checks · not swept certified'
    :'Validated geometric path · '+status+(refinement?' · refinement: '+refinement+acceptedLabel:'')+' · sampled footprint checking'):
    ('No validated path · '+status+failureDetail+
     (exhausted&&limit?' · limit '+limit.maxExpansions.toLocaleString()+' expansions':'')))+
    templateNote;
  $('cost').textContent=valid?path.cost.toFixed(2)+' m':'—';
  $('length').textContent=valid?path.length.toFixed(2)+' m':'—';
  $('reverseLen').textContent=valid?path.reverseLength.toFixed(2)+' m':'—';
  $('expanded').textContent=result.original.expanded.toLocaleString();
  $('limit').textContent=limit?limit.maxExpansions.toLocaleString():'—';
  $('elapsed').textContent=result.milliseconds.toFixed(1)+' ms';
  $('curvatureVariationLabel').textContent=result.model==='articulated'?'GOAL γ ERROR':'CURVATURE VARIATION';
  $('steeringReversalsLabel').textContent=result.model==='articulated'?'ARTICULATION MOVES':'STEERING REVERSALS';
  if(result.model==='articulated') {
    $('curvatureVariation').textContent=valid?
      (path.goalGammaErrorRadians*180/Math.PI).toFixed(2)+'°':'—';
    $('steeringReversals').textContent=valid?
      String(path.sections.filter(sec=>sec.kind==='articulation').length):'—';
    return;
  }
  const before=steeringMetrics(result.original),after=steeringMetrics(path);
  const compare=!!result.refinement?.path;
  $('curvatureVariation').textContent=valid?
    (compare?before.totalVariation.toFixed(2)+' → ':'')+
    after.totalVariation.toFixed(2)+' 1/m':'—';
  $('steeringReversals').textContent=valid?
    (compare?before.reversals+' → ':'')+after.reversals:'—';
}
let modelCapabilities={ompl:false};
function syncModelOptions() {
  const model=$('model').value;
  const ackermann=model==='ackermann';
  const articulated=model==='articulated';
  $('articulatedControls').hidden=!articulated;
  if(articulated) {
    // These features are built for rigid three-state paths. Never send an
    // unsupported RS/refinement/heuristic request to the 4D planner.
    for(const id of ['ompl','rsDistance','refine','curved',
                     'steeringSmooth','improve','performance'])
      {$(id).checked=false;$(id).disabled=true;}
    $('maxAccepted').disabled=true;
    $('steeringWeight').disabled=true;
    return;
  }
  $('performance').disabled=false;
  $('improve').disabled=false;
  const mecanum=model==='mecanum';
  // OMPL stays Ackermann-only. Strict native refinement supports all three
  // models; CSC is not a mecanum candidate (holonomic twists replace it).
  if(!ackermann)$('ompl').checked=false;
  $('ompl').disabled=!ackermann||!modelCapabilities.ompl;
  const canUseRsDistance=ackermann&&modelCapabilities.ompl&&$('ompl').checked&&$('performance').checked;
  if(!canUseRsDistance)$('rsDistance').checked=false;
  $('rsDistance').disabled=!canUseRsDistance;
  $('refine').disabled=false;
  $('maxAccepted').disabled=!$('refine').checked;
  if(mecanum)$('curved').checked=false;
  $('curved').disabled=mecanum||!$('refine').checked;
  // Only Ackermann has the optional steering-variation objective.
  if(!ackermann)$('steeringSmooth').checked=false;
  const canUseSteering=ackermann&&$('refine').checked;
  $('steeringSmooth').disabled=!canUseSteering;
  $('steeringWeight').disabled=!canUseSteering||!$('steeringSmooth').checked;
}
$('model').addEventListener('change',()=>{
  syncModelOptions();invalidateRoute();replan();
});
function invalidateRoute() {
  routeValid=false;animation=false;
  $('playButton').textContent='▶';
  $('exportButton').disabled=true;
  if(pending){pending.abort();pending=null;}
  $('message').className='readout';
  $('message').textContent='Map/pose changed · previous path is stale · replan required';
  for(const id of ['cost','length','reverseLen','expanded','elapsed','curvatureVariation','steeringReversals'])$(id).textContent='—';
  draw();
}
function resetEditor() {
  editMap.clear();generation=0;baseMap=null;baseCells=null;snapshot=null;routeValid=false;
  $('resetMap').disabled=true;
  $('editCount').textContent='0 changed cells';
}
function showEditCount() {
  $('editCount').textContent=editMap.size+' changed cells · revision +'+generation;
  $('resetMap').disabled=editMap.size===0;
}
function paintAt(event) {
  if(!baseMap||!baseCells)return false;
  const rect=canvas.getBoundingClientRect(),g=canvasGeometry();
  const cell=MapEditor.cellFromCanvas(event.clientX-rect.left,event.clientY-rect.top,g,
                                      baseMap.width,baseMap.height,baseMap.resolution);
  if(!cell)return false;
  let changed=false;
  try {
    for(const index of MapEditor.brushCells(cell[0],cell[1],Number($('brushSize').value),
                                            baseMap.width,baseMap.height)) {
      if(MapEditor.edit(editMap,baseCells,index,$('editMode').value))changed=true;
    }
  }catch(e){
    $('message').className='readout bad';$('message').textContent=e.message;
  }
  if(changed) {generation++;strokeChanged=true;showEditCount();invalidateRoute();}
  return changed;
}
async function replan() {
  if(pending)pending.abort();pending=new AbortController();
  routeValid=false;draw();
  $('exportButton').disabled=true;
  $('planButton').disabled=true;$('overlay').classList.remove('hidden');
  $('overlay').textContent=$('model').value==='articulated'?
    'Computing a two-carriage RK4 route…':'Computing an exact-arc route…';
  animation=false;$('playButton').textContent='▶';
  const keys=['scenario','startx','starty','startyaw','goalx','goaly','goalyaw'];
  const params=new URLSearchParams();for(const key of keys)params.set(key,$(key).value);
  params.set('model',$('model').value);
  if($('model').value==='articulated') {
    for(const key of ['artStartGamma','artGoalGamma','artLf','artLr',
                      'artFrontRearX','artFrontFrontX','artFrontHalfWidth',
                      'artRearRearX','artRearFrontX','artRearHalfWidth'])
      params.set(key,$(key).value.trim());
  }
  if($('refine').checked)
    params.set('maxAccepted',$('maxAccepted').value);
  for(const key of ['ompl','rsDistance','refine','curved','performance','improve'])params.set(key,$(key).checked?'1':'0');
  const steeringEnabled=$('model').value==='ackermann'&&
    $('refine').checked&&$('steeringSmooth').checked;
  params.set('steeringSmooth',steeringEnabled?'1':'0');
  // Preserve a typed weight when disabled, but don't send a stale or blank
  // value to the server while the optional objective is switched off.
  if(steeringEnabled)params.set('steeringWeight',$('steeringWeight').value.trim());
  params.set('inflate',$('inflate').checked?'1':'0');
  params.set('inflationMeters',$('inflationMeters').value.trim());
  const budget=$('budget').value.trim();
  if(budget)params.set('budget',budget);
  params.set('editRevision',String(generation));
  params.set('edits',MapEditor.encode(editMap));
  const signal=pending.signal, requestedRevision=generation;
  try {
    const res=await fetch('/api/plan',{method:'POST',body:params,signal});

    const result=await res.json();
    if(!res.ok)throw new Error(result.error||'Planning request failed');
    if(signal.aborted||requestedRevision!==generation)return;
    if(!baseMap) {
      baseMap={width:result.width,height:result.height,resolution:result.resolution,
               origin:result.origin,mapRuns:result.mapRuns,revision:result.revision};
      baseCells=new Int16Array(result.width*result.height);
      for(const [x,y,length,value] of result.mapRuns)
        baseCells.fill(value,y*result.width+x,y*result.width+x+length);
    }
    if(result.revision!==baseMap.revision+generation)
      throw Error('Planner map revision differs from current brush revision');
    snapshot=result;routeValid=true;$('exportButton').disabled=false;
    $('position').value=0;stats(result);draw();
  } catch(e) {
    if(e.name==='AbortError'||signal.aborted)return;
    $('message').className='readout bad';$('message').textContent=e.message;
  } finally {
    if(pending?.signal===signal){pending=null;$('overlay').classList.add('hidden');$('planButton').disabled=false;}
  }
}
function tick(now) {
  if(!animation)return;
  if(now-lastFrame>28){lastFrame=now;const next=Math.min(1000,Number($('position').value)+5);$('position').value=next;
    $('travel').textContent=(next/10).toFixed(0)+'%';draw();
    if(next>=1000){animation=false;$('playButton').textContent='▶';return;}}
  requestAnimationFrame(tick);
}
$('playButton').addEventListener('click',()=>{
  if(!activePath()?.sections.length)return;
  animation=!animation;$('playButton').textContent=animation?'Ⅱ':'▶';
  if(animation){if(Number($('position').value)>=1000)$('position').value=0;lastFrame=0;requestAnimationFrame(tick);}
});
$('position').addEventListener('input',()=>{$('travel').textContent=(Number($('position').value)/10).toFixed(0)+'%';draw();});
$('showOriginal').addEventListener('change',draw);
// Numeric start/goal edits must update the footprint preview immediately,
// before the next explicit Plan click. The previous path is now stale.
for(const key of ['startx','starty','startyaw','goalx','goaly','goalyaw',
  'artStartGamma','artGoalGamma','artLf','artLr',
  'artFrontRearX','artFrontFrontX','artFrontHalfWidth',
  'artRearRearX','artRearFrontX','artRearHalfWidth'])
  $(key).addEventListener('input',invalidateRoute);
$('scenario').addEventListener('change',()=>{if(pending)pending.abort();resetEditor();resetScenario();replan();});
$('planButton').addEventListener('click',replan);
$('exportButton').addEventListener('click',()=>{
  if(!snapshot||!routeValid)return;
  const data=new Blob([JSON.stringify(snapshot,null,2)+'\n'],{type:'application/json'});
  const href=URL.createObjectURL(data);
  const link=document.createElement('a');
  link.href=href;link.download='hybrid-astar-'+$('scenario').value+'-plan.json';
  document.body.appendChild(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(href),2000);
});
$('refine').addEventListener('change',()=>{syncModelOptions();invalidateRoute();});
$('maxAccepted').addEventListener('change',invalidateRoute);
$('steeringSmooth').addEventListener('change',()=>{syncModelOptions();invalidateRoute();});
$('steeringWeight').addEventListener('change',invalidateRoute);
$('ompl').addEventListener('change',syncModelOptions);
$('performance').addEventListener('change',syncModelOptions);
$('inflate').addEventListener('change',()=>{
  $('inflationMeters').disabled=!$('inflate').checked;
  invalidateRoute();
});
$('inflationMeters').addEventListener('change',invalidateRoute);
$('resetMap').addEventListener('click',()=>{
  if(!editMap.size)return;
  editMap.clear();generation++;showEditCount();invalidateRoute();replan();
});
canvas.addEventListener('pointerdown',e=>{
  if(e.button!==0)return;
  const mode=$('editMode').value;
  if(['blocked','free','unknown','restore'].includes(mode)) {
    if(!baseMap)return;
    pointerPainting=true;strokeChanged=false;
    canvas.setPointerCapture(e.pointerId);
    paintAt(e);e.preventDefault();return;
  }
  const g=canvasGeometry(),rect=canvas.getBoundingClientRect();
  const px=e.clientX-rect.left,py=e.clientY-rect.top;
  const lx=(px-g.ox)/g.scale,ly=g.mh-(py-g.oy)/g.scale;
  if(lx<0||lx>=g.mw||ly<0||ly>=g.mh)return;
  const [x,y]=worldPoint(px,py,g);
  const prefix=mode==='goal'||e.shiftKey?'goal':'start';
  $(prefix+'x').value=x.toFixed(2);$(prefix+'y').value=y.toFixed(2);
  invalidateRoute();replan();e.preventDefault();
});
canvas.addEventListener('pointermove',e=>{
  if(pointerPainting) {paintAt(e);e.preventDefault();}
});
function finishStroke(e) {
  if(!pointerPainting)return;
  pointerPainting=false;
  if(canvas.hasPointerCapture(e.pointerId))canvas.releasePointerCapture(e.pointerId);
  if(strokeChanged)replan();
  strokeChanged=false;
}
canvas.addEventListener('pointerup',finishStroke);
canvas.addEventListener('pointercancel',finishStroke);

new ResizeObserver(draw).observe(canvas);
(async()=>{
  resetScenario();$('curved').disabled=true;
  try {const caps=await (await fetch('/api/capabilities')).json();
    modelCapabilities=caps;
    $('articulatedOption').hidden=!caps.articulated;
    // OMPL-equipped Ackermann Planning Lab defaults to grid + adaptive RS;
    // RS distance remains a separate, unchecked quality/compute trade-off.
    $('ompl').checked=!!caps.ompl&&$('model').value==='ackermann';
    syncModelOptions();
    $('omplInfo').textContent=caps.ompl?'(available for Ackermann)':'(build without OMPL)';
    for(const [key,option,name] of [
      ['nav2_warehouse_full','nav2FullOption','Nav2 warehouse · full resolution'],
      ['nav2_warehouse_344128','nav2SmallOption','Nav2 warehouse · 344,128 cells']
    ]) {
      const map=caps.nav2Maps?.[key];
      if(!map)continue;
      $(option).hidden=false;
      const width=map.width*map.resolution,height=map.height*map.resolution;
      const start=MapGeometry.toWorld(width*.2,height*.5,map.origin);
      const goal=MapGeometry.toWorld(width*.8,height*.5,map.origin);
      defaults[key]={name,start:[...start,map.origin[2]*180/Math.PI],
        goal:[...goal,map.origin[2]*180/Math.PI]};
    }
    if(caps.customMap) {
      $('customOption').hidden=false;
      customOrigin=caps.origin||[0,0,0];
      const start=MapGeometry.toWorld(caps.mapWidthMeters*.2,caps.mapHeightMeters*.5,customOrigin);
      const goal=MapGeometry.toWorld(caps.mapWidthMeters*.8,caps.mapHeightMeters*.5,customOrigin);
      defaults.custom={name:'Imported map',
        start:[...start,customOrigin[2]*180/Math.PI],
        goal:[...goal,customOrigin[2]*180/Math.PI]};
      $('scenario').value='custom';resetScenario();
    }
  }catch(e){$('connection').textContent='OFFLINE';}
  await replan();
})();
