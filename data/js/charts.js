'use strict';
(() => {
  const colors = ['#d4e5ac','#83bab5','#b4a6cf'];
  const channels = {speed:[['engRpm','Engine'],['turbRpm','Turbine'],['outRpm','Output']],
    pressure:[['mpc','MPC'],['spc','SPC'],['tccPwm','TCC']],
    slip:[['onClutch','Oncoming'],['offClutch','Offgoing'],['tccActual','TCC slip']],
    conditions:[['tps','TPS %'],['atfTemp','ATF °C']]};
  const capacity=300, samples=new Array(capacity);
  let head=0, count=0, trace=null, dirty=true;
  TCU.events.addEventListener('telemetry',e=>{
    const d=e.detail;
    samples[head]={...d,atfTemp:d.atfMeasuredC ?? (d.atfSignalOk===true?d.atfTemp:null),t:performance.now()}; head=(head+1)%capacity;
    count=Math.min(count+1,capacity); dirty=true;
  });
  TCU.events.addEventListener('connection',()=>{ head=0;count=0;dirty=true; });
  TCU.events.addEventListener('shift_trace',e=>{
    trace=e.detail;dirty=true; document.getElementById('export-trace').disabled=false;
    document.getElementById('trace-info').textContent=`Capture ${trace.from} → ${trace.to} · ${trace.n} samples · ${trace.t.at(-1) ?? 0} ms${trace.n >= 400 ? ' · buffer limit reached' : ''}. TPS, ATF and TCC are not recorded in shift captures.`;
  });
  function rows(source) {
    if(source==='trace') return trace ? trace.t.map((t,i)=>({t,phase:trace.ph[i],mpc:trace.mpc[i],spc:trace.spc[i],engRpm:trace.eng[i],turbRpm:trace.turb[i],outRpm:trace.out[i],onClutch:trace.onClutch[i],offClutch:trace.offClutch[i]})) : [];
    return Array.from({length:count},(_,i)=>samples[(head-count+i+capacity)%capacity]);
  }
  function draw(canvas, data, type, source) {
    if (!canvas.offsetWidth) return;
    const ratio=Math.min(devicePixelRatio||1,2), w=canvas.clientWidth,h=canvas.clientHeight;
    if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
    const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
    const keys=channels[type],left=44,top=15,pw=w-left-10,ph=h-top-22;
    const vals=data.flatMap(d=>keys.map(([k])=>d[k])).filter(Number.isFinite);
    let lo=Math.min(0,...vals), hi=Math.max(type==='speed'?1000:100,...vals);
    const step=type==='speed'?1000: type==='slip'?250:20;
    hi=Math.ceil(hi/step)*step;lo=Math.floor(lo/step)*step;
    const end=source==='live'?performance.now():data.at(-1)?.t||1;
    const start=source==='live'?end-30000:0;
    const x=t=>left+(t-start)/Math.max(1,end-start)*pw;
    const y=v=>top+ph-(v-lo)/(hi-lo)*ph;
    ctx.font='9px Consolas, monospace';ctx.fillStyle='#7e8b90';ctx.strokeStyle='#343b3f';ctx.lineWidth=.5;
    for(let i=0;i<=4;++i){const yy=top+ph*i/4;ctx.beginPath();ctx.moveTo(left,yy);ctx.lineTo(w,yy);ctx.stroke();ctx.fillText(String(Math.round(hi-(hi-lo)*i/4)),0,yy+3);}
    ctx.save();ctx.beginPath();ctx.rect(left,top,pw,ph);ctx.clip();
    if(source==='trace') for(let i=0;i<data.length-1;++i) if(data[i].phase>0){ctx.fillStyle=data[i].phase%2?'#d4e5ac09':'#83bab509';ctx.fillRect(x(data[i].t),top,Math.max(1,x(data[i+1].t)-x(data[i].t)),ph);}
    keys.forEach(([key],index)=>{
      ctx.strokeStyle=colors[index];ctx.lineWidth=1.5;ctx.beginPath();let begun=false,previous=0;
      for(const d of data){if(!Number.isFinite(d[key])){begun=false;continue;}if(!begun || (source==='live' && d.t-previous>1500)){ctx.moveTo(x(d.t),y(d[key]));begun=true;}else ctx.lineTo(x(d.t),y(d[key]));previous=d.t;}ctx.stroke();
    });ctx.restore();
    if(!vals.length){ctx.fillStyle='#7e8b90';ctx.fillText('Waiting for available samples',left+15,top+ph/2);}
  }
  function refresh(){
    if(document.hidden || !dirty) return;dirty=false;
    draw(document.getElementById('speed-chart'),rows('live'),'speed','live');
    const type=document.getElementById('chart-signal').value,source=document.getElementById('chart-source').value;
    draw(document.getElementById('analysis-chart'),rows(source),type,source);
    const legend=document.getElementById('analysis-legend');legend.replaceChildren(...channels[type].map(([key,label],i)=>{const span=document.createElement('span');span.textContent=label;span.style.color=colors[i];return span;}));
  }
  ['chart-signal','chart-source'].forEach(id=>document.getElementById(id).addEventListener('change',()=>{dirty=true;refresh();}));
  addEventListener('resize',()=>dirty=true);addEventListener('hashchange',()=>dirty=true);
  document.addEventListener('visibilitychange',()=>dirty=true);
  setInterval(refresh,100); // Render only dirty visible charts, at most 10 Hz.
  document.getElementById('export-trace').onclick=()=>{
    if(!trace)return;
    const keys=['t','ph','spc','mpc','ratio','eng','turb','out','clErr','onClutch','offClutch','fl'];
    const lines=['t_ms,phase,spc_pct,mpc_pct,ratio_x1000,engine_rpm,turbine_rpm,output_rpm,cl_err_x1000,on_clutch_rpm,off_clutch_rpm,flags'];
    for(let i=0;i<trace.n;++i)lines.push(keys.map(k=>trace[k][i]).join(','));
    const url=URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/csv'}));
    const a=document.createElement('a');a.href=url;a.download=`shift-${trace.from}-${trace.to}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
})();
