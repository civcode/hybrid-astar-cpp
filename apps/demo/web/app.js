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
function drawPose(p,color,g,body=false) {
  const [x,y]=mapPoint(p[0],p[1],g);
  ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2.5;
  ctx.beginPath();ctx.arc(x,y,5.5,0,Math.PI*2);ctx.fill();
  const isCompact=$('model').value==='differential'||$('model').value==='mecanum';
  const headLength=isCompact ? 0.35 : 1.6;
  const [tx,ty]=mapPoint(p[0]+headLength*Math.cos(p[2]),p[1]+headLength*Math.sin(p[2]),g);
  ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(tx,ty);ctx.stroke();
  if(!body)return;
  const [rear,frontX,halfWidth]=(snapshot?.model===$('model').value
      ? snapshot.footprint : null) ||
      (isCompact?[-.35,.35,.25]:[-1,3.6,.9]);
  const corners=[[rear,-halfWidth],[frontX,-halfWidth],
                 [frontX,halfWidth],[rear,halfWidth]];
  ctx.fillStyle='rgba(91,221,206,.16)';ctx.strokeStyle='#70e5d5';ctx.lineWidth=1.5;
  ctx.beginPath();corners.forEach(([u,v],i)=>{
    const xx=p[0]+Math.cos(p[2])*u-Math.sin(p[2])*v;
    const yy=p[1]+Math.sin(p[2])*u+Math.cos(p[2])*v;
    const [sx,sy]=mapPoint(xx,yy,g);if(!i)ctx.moveTo(sx,sy);else ctx.lineTo(sx,sy);
  });ctx.closePath();
  // Stroke the physical polygon boundary by +/- the requested continuous
  // distance. Round joins visualize the true Minkowski offset, unlike
  // drawing a larger axis-aligned rectangle.
  const inflation=snapshot&&routeValid
      ?Number(snapshot.footprintInflationMeters||0)
      :($('inflate').checked?Number($('inflationMeters').value):0);
  if(inflation>0 && Number.isFinite(inflation)) {
    ctx.save();ctx.lineJoin='round';
    ctx.lineWidth=2*inflation*g.scale;
    ctx.strokeStyle='rgba(255,188,102,.45)';
    ctx.stroke();ctx.restore();
  }
  ctx.fill();ctx.stroke();
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
    ctx.strokeStyle=muted?'rgba(155,177,204,.5)':section.kind==='holonomic'?'#7db2ff':section.gear==='reverse'?'#e4ac70':'#55dac6';
    ctx.lineWidth=muted?2:3.5;ctx.setLineDash(muted?[5,6]:[]);ctx.stroke();ctx.setLineDash([]);
  }
}
function activePath() {
  if(!snapshot||!routeValid)return null;
  return snapshot.refinement?.path||snapshot.original;
}
function allPositions(path) {
  return path?.sections.flatMap((sec,i)=>i?sec.points.slice(1):sec.points)||[];
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
  if(start.every(Number.isFinite))drawPose([start[0],start[1],start[2]*Math.PI/180],'#5dd5ff',g);
  if(goal.every(Number.isFinite))drawPose([goal[0],goal[1],goal[2]*Math.PI/180],'#eac47d',g);
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
  $('message').textContent=valid?('Validated geometric path · '+status+(refinement?' · refinement: '+refinement:'')+' · sampled footprint checking'):
    ('No validated path · '+status+failureDetail+
     (exhausted&&limit?' · limit '+limit.maxExpansions.toLocaleString()+' expansions':''));
  $('cost').textContent=valid?path.cost.toFixed(2)+' m':'—';
  $('length').textContent=valid?path.length.toFixed(2)+' m':'—';
  $('reverseLen').textContent=valid?path.reverseLength.toFixed(2)+' m':'—';
  $('expanded').textContent=result.original.expanded.toLocaleString();
  $('limit').textContent=limit?limit.maxExpansions.toLocaleString():'—';
  $('elapsed').textContent=result.milliseconds.toFixed(1)+' ms';
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
  const mecanum=model==='mecanum';
  // OMPL stays Ackermann-only. Strict native refinement supports all three
  // models; CSC is not a mecanum candidate (holonomic twists replace it).
  if(!ackermann)$('ompl').checked=false;
  $('ompl').disabled=!ackermann||!modelCapabilities.ompl;
  const canUseRsDistance=ackermann&&modelCapabilities.ompl&&$('ompl').checked&&$('performance').checked;
  if(!canUseRsDistance)$('rsDistance').checked=false;
  $('rsDistance').disabled=!canUseRsDistance;
  $('refine').disabled=false;
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
  $('planButton').disabled=true;$('overlay').classList.remove('hidden');$('overlay').textContent='Computing an exact-arc route…';
  animation=false;$('playButton').textContent='▶';
  const keys=['scenario','startx','starty','startyaw','goalx','goaly','goalyaw'];
  const params=new URLSearchParams();for(const key of keys)params.set(key,$(key).value);
  params.set('model',$('model').value);
  for(const key of ['ompl','rsDistance','refine','curved','performance','improve'])params.set(key,$(key).checked?'1':'0');
  params.set('steeringSmooth',$('model').value==='ackermann'&&
    $('refine').checked&&$('steeringSmooth').checked?'1':'0');
  params.set('steeringWeight',$('steeringWeight').value.trim());
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
$('refine').addEventListener('change',syncModelOptions);
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
    // OMPL-equipped Ackermann Planning Lab defaults to grid + adaptive RS;
    // RS distance remains a separate, unchecked quality/compute trade-off.
    $('ompl').checked=!!caps.ompl&&$('model').value==='ackermann';
    syncModelOptions();
    $('omplInfo').textContent=caps.ompl?'(available for Ackermann)':'(build without OMPL)';
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
