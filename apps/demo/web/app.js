'use strict';
const $ = id => document.getElementById(id);
const defaults = {
  open:      {name:'Open field',start:[10,15,0],goal:[17,15,0]},
  reverse:   {name:'Reverse maneuver',start:[10,15,0],goal:[6,15,0]},
  parking:   {name:'Parking approach',start:[10,15,0],goal:[8,16,0]},
  warehouse: {name:'Warehouse obstacles',start:[8,20,0],goal:[43,20,0]},
  blocked:   {name:'Unreachable wall',start:[10,20,0],goal:[35,20,0]}
};
let snapshot=null, pending=null, animation=false, lastFrame=0;
const canvas=$('map'), ctx=canvas.getContext('2d');
function setPose(prefix,v) {['x','y','yaw'].forEach((s,i)=>$(prefix+s).value=Number(v[i]).toFixed(i===2?0:2));}
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
  const mw=(snapshot?.width||200)*(snapshot?.resolution||.25);
  const mh=(snapshot?.height||160)*(snapshot?.resolution||.25);
  const scale=Math.min((w-42)/mw,(h-42)/mh);
  const ox=(w-mw*scale)/2,oy=(h-mh*scale)/2;
  return {w,h,mw,mh,scale,ox,oy};
}
function mapPoint(x,y,g) {return [g.ox+x*g.scale,g.oy+(g.mh-y)*g.scale];}
function worldPoint(px,py,g) {return [(px-g.ox)/g.scale,g.mh-(py-g.oy)/g.scale];}
function drawPose(p,color,g,body=false) {
  const [x,y]=mapPoint(p[0],p[1],g);
  ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2.5;
  ctx.beginPath();ctx.arc(x,y,5.5,0,Math.PI*2);ctx.fill();
  const [tx,ty]=mapPoint(p[0]+1.6*Math.cos(p[2]),p[1]+1.6*Math.sin(p[2]),g);
  ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(tx,ty);ctx.stroke();
  if(!body)return;
  const corners=[[-1,-.9],[3.6,-.9],[3.6,.9],[-1,.9]];
  ctx.fillStyle='rgba(91,221,206,.16)';ctx.strokeStyle='#70e5d5';ctx.lineWidth=1.5;
  ctx.beginPath();corners.forEach(([u,v],i)=>{
    const xx=p[0]+Math.cos(p[2])*u-Math.sin(p[2])*v;
    const yy=p[1]+Math.sin(p[2])*u+Math.cos(p[2])*v;
    const [sx,sy]=mapPoint(xx,yy,g);if(!i)ctx.moveTo(sx,sy);else ctx.lineTo(sx,sy);
  });ctx.closePath();ctx.fill();ctx.stroke();
}
function drawSections(path,g,muted=false) {
  if(!path)return;
  ctx.lineCap='round';ctx.lineJoin='round';
  for(const section of path.sections) {
    if(!section.points.length)continue;
    ctx.beginPath();section.points.forEach((p,i)=>{
      const [x,y]=mapPoint(p[0],p[1],g);if(!i)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    });
    ctx.strokeStyle=muted?'rgba(155,177,204,.5)':section.gear==='reverse'?'#e4ac70':'#55dac6';
    ctx.lineWidth=muted?2:3.5;ctx.setLineDash(muted?[5,6]:[]);ctx.stroke();ctx.setLineDash([]);
  }
}
function activePath() {
  if(!snapshot)return null;
  return snapshot.refinement?.path||snapshot.original;
}
function allPositions(path) {
  return path?.sections.flatMap((sec,i)=>i?sec.points.slice(1):sec.points)||[];
}
function draw() {
  const g=canvasGeometry();ctx.clearRect(0,0,g.w,g.h);
  ctx.fillStyle='#112238';ctx.fillRect(g.ox,g.oy,g.mw*g.scale,g.mh*g.scale);
  ctx.strokeStyle='#2c4157';ctx.lineWidth=.7;
  for(let x=0;x<=g.mw;x+=5){const [sx]=mapPoint(x,0,g);ctx.beginPath();ctx.moveTo(sx,g.oy);ctx.lineTo(sx,g.oy+g.mh*g.scale);ctx.stroke();}
  for(let y=0;y<=g.mh;y+=5){const [,sy]=mapPoint(0,y,g);ctx.beginPath();ctx.moveTo(g.ox,sy);ctx.lineTo(g.ox+g.mw*g.scale,sy);ctx.stroke();}
  if(snapshot) {
    for(const [x,y,length,value] of snapshot.mapRuns) {
      const [sx,sy]=mapPoint(x*snapshot.resolution,(y+1)*snapshot.resolution,g);
      ctx.fillStyle=value===-1?'#a58b61':'#71859e';
      ctx.fillRect(sx,sy,Math.max(1,length*snapshot.resolution*g.scale),Math.max(1,snapshot.resolution*g.scale));
    }
    const path=activePath();
    if(snapshot.refinement?.path && $('showOriginal').checked)drawSections(snapshot.original,g,true);
    drawSections(path,g);
    drawPose(snapshot.start,'#5dd5ff',g);
    drawPose(snapshot.goal,'#eac47d',g);
    const samples=allPositions(path);
    if(samples.length)drawPose(samples[Math.min(samples.length-1,Math.round(Number($('position').value)/1000*(samples.length-1)))],'#6ce8d3',g,true);
  } else {
    const start=['startx','starty','startyaw'].map(id=>Number($(id).value));
    const goal=['goalx','goaly','goalyaw'].map(id=>Number($(id).value));
    if(start.every(Number.isFinite))drawPose([start[0],start[1],start[2]*Math.PI/180],'#5dd5ff',g);
    if(goal.every(Number.isFinite))drawPose([goal[0],goal[1],goal[2]*Math.PI/180],'#eac47d',g);
  }
}
function stats(result) {
  const path=activePath(),valid=path?.status==='Success';
  $('message').className='readout '+(valid?'good':'bad');
  const status=result.original.status;
  const refinement=result.refinement?.status;
  $('message').textContent=valid?('Validated geometric path · '+status+(refinement?' · refinement: '+refinement:'')+' · sampled footprint checking'):
    ('No validated path · '+status+' · a work limit or NoPath is not a swept-safety proof');
  $('cost').textContent=valid?path.cost.toFixed(2)+' m':'—';
  $('length').textContent=valid?path.length.toFixed(2)+' m':'—';
  $('reverseLen').textContent=valid?path.reverseLength.toFixed(2)+' m':'—';
  $('expanded').textContent=result.original.expanded.toLocaleString();
  $('elapsed').textContent=result.milliseconds.toFixed(1)+' ms';
}
async function replan() {
  if(pending)pending.abort();pending=new AbortController();
  $('exportButton').disabled=true;
  $('planButton').disabled=true;$('overlay').classList.remove('hidden');$('overlay').textContent='Computing an exact-arc route…';
  animation=false;$('playButton').textContent='▶';
  const keys=['scenario','startx','starty','startyaw','goalx','goaly','goalyaw'];
  const params=new URLSearchParams();for(const key of keys)params.set(key,$(key).value);
  for(const key of ['ompl','refine','curved','performance','improve'])params.set(key,$(key).checked?'1':'0');
  const signal=pending.signal;
  try {
    const res=await fetch('/api/plan?'+params,{signal});
    const result=await res.json();
    if(!res.ok)throw new Error(result.error||'Planning request failed');
    snapshot=result;$('exportButton').disabled=false;$('position').value=0;stats(result);draw();
  } catch(e) {
    if(e.name==='AbortError')return;
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
$('scenario').addEventListener('change',()=>{snapshot=null;resetScenario();replan();});
$('planButton').addEventListener('click',replan);
$('exportButton').addEventListener('click',()=>{
  if(!snapshot)return;
  const data=new Blob([JSON.stringify(snapshot,null,2)+'\n'],{type:'application/json'});
  const href=URL.createObjectURL(data);
  const link=document.createElement('a');
  link.href=href;link.download='hybrid-astar-'+$('scenario').value+'-plan.json';
  document.body.appendChild(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(href),2000);
});
$('refine').addEventListener('change',()=>{$('curved').disabled=!$('refine').checked;});
canvas.addEventListener('click',e=>{
  const g=canvasGeometry();const rect=canvas.getBoundingClientRect();
  const [x,y]=worldPoint(e.clientX-rect.left,e.clientY-rect.top,g);
  if(x<0||x>g.mw||y<0||y>g.mh)return;
  const prefix=e.shiftKey?'goal':'start';
  $(prefix+'x').value=x.toFixed(2);$(prefix+'y').value=y.toFixed(2);
  snapshot=null;draw();replan();
});
new ResizeObserver(draw).observe(canvas);
(async()=>{
  resetScenario();$('curved').disabled=true;
  try {const caps=await (await fetch('/api/capabilities')).json();
    $('ompl').disabled=!caps.ompl;$('omplInfo').textContent=caps.ompl?'(available)':'(build without OMPL)';
    if(caps.customMap) {
      $('customOption').hidden=false;
      defaults.custom={name:'Imported PGM map',
        start:[caps.mapWidthMeters*.2,caps.mapHeightMeters*.5,0],
        goal:[caps.mapWidthMeters*.8,caps.mapHeightMeters*.5,0]};
      $('scenario').value='custom';resetScenario();
    }
  }catch(e){$('connection').textContent='OFFLINE';}
  await replan();
})();
