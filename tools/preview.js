// Offline-only transport simulator. This file is never embedded in firmware.
(() => {
  const fixtures=window.__PREVIEW_DATA;
  const profile=fixtures.profile,params=fixtures.params,network=fixtures.network;
  let cells=[...fixtures.cells],clock=120000,bench=false,atfOnly=false,selector='D',outputs=0;
  const clone=x=>JSON.parse(JSON.stringify(x));
  const baseline=clone(fixtures);
  class PreviewSocket {
    static CONNECTING=0;static OPEN=1;static CLOSING=2;static CLOSED=3;
    constructor(){this.readyState=0;setTimeout(()=>{this.readyState=1;this.onopen?.();this.interval=setInterval(()=>this.sample(),100);},30);}
    deliver(data){if(this.readyState===1)this.onmessage?.({data:JSON.stringify(data)});}
    send(raw){const d=JSON.parse(raw);let response=null,error=null;
      switch(d.cmd){
        case 'selector.atf':atfOnly=d.on;break;
        case 'get_profile':response=profile;break;
        case 'param.list':response={type:'param_list',params,n:params.length};break;
        case 'get_cells':response={type:'cell_data',classes:4,shifts:4,tbins:4,data:cells};break;
        case 'get_dtcs':response={type:'dtc_data',dtcs:fixtures.dtcs};break;
        case 'network.get':response=network;break;
        case 'param.set':{const p=params.find(p=>p.idx===d.idx);if(!p||d.row>=p.rows||d.col>=p.cols||!Number.isInteger(d.val)||d.val<p.min||d.val>p.max)error='Invalid preview parameter';else p.v[d.row*p.cols+d.col]=d.val;break;}
        case 'param.persist':break;
        case 'param.reset':params.splice(0,params.length,...clone(baseline.params));break;
        case 'set_profile':Object.keys(d).filter(k=>!['cmd','requestId'].includes(k)).forEach(k=>profile[k]=clone(d[k]));break;
        case 'set_cells':cells=[...d.data];break;
        case 'adapt_nudge':cells[2]=Math.max(-15,Math.min(15,cells[2]+d.dir*2));break;
        case 'clear_dtcs':fixtures.dtcs.forEach(x=>{x.count=0;x.active=false;});break;
        case 'limp_reset':break;
        case 'test_mode':bench=d.on;if(!bench)outputs=0;break;
        case 'test_prnd':selector=d.v;break;
        case 'test_paddle':break;
        case 'test_io':{const bit=1<<['y3','y5','y4','mpc','spc','tcc','rp','tq'].indexOf(d.id);outputs=d.on?outputs|bit:outputs&~bit;break;}
        case 'network.set':
          if(d.op==='mode')network.mode=d.mode;
          if(d.op==='remove')network.known.splice(d.index,1);
          if(d.op==='up'&&d.index>0)[network.known[d.index-1],network.known[d.index]]=[network.known[d.index],network.known[d.index-1]];
          if(d.op==='add'){const found=network.known.find(x=>x.ssid===d.ssid);if(found)found.secured=!!d.password;else if(network.known.length<5)network.known.push({ssid:d.ssid,secured:!!d.password});else error='Maximum five preview networks';}
          break;
        default:error='Unsupported preview command';
      }
      setTimeout(()=>{this.deliver({type:'command_result',requestId:d.requestId,ok:!error,message:error||'Preview only · changed in this browser'});if(response)this.deliver(response);},15);
    }
    sample(){
      clock+=100;
      const time=clock/1000,cycle=clock%16000,shifting=cycle>10000&&cycle<11000;
      const phase=shifting?cycle<10100?1:cycle<10300?2:cycle<10500?3:cycle<10800?4:7:0;
      const gear=cycle<10800?3:4,rpm=2450+550*Math.sin(time/7),tps=32+16*Math.sin(time/9);
      const ratio=gear===3?1.486:1,output=(rpm-100)/ratio;
      this.deliver({...fixtures.telemetry,atfOnly,forwardConfirmed:atfOnly,atfRange:2,modeName:atfOnly?"ATF ONLY / MANUAL":fixtures.telemetry.modeName,sampledMs:clock,prnd:bench?selector:'D',gear,tgt:shifting?4:gear,
        engRpm:Math.round(rpm),turbRpm:Math.round(rpm-100),outRpm:Math.round(output),kmh:output*.038,
        n2:Math.round(rpm-100),n3:Math.round(rpm-100),tps,map:95+tps*.5,loadPct:tps,
        phase,ratio,expectedRatio:ratio,targetRatio:shifting?1:ratio,intervalMs:100,
        mpc:Math.round(76+tps*.18),spc:shifting?Math.round(50+(cycle-10000)/20):100,
        atfTemp:82+2*Math.sin(time/30),atfMeasuredC:82+2*Math.sin(time/30),tccPwm:shifting?0:52,tccActual:shifting?250:100,
        testMode:bench,tout:outputs,onClutch:shifting?350-(cycle-10000)*.3:0,offClutch:shifting?400:0});
      if(cycle===11000){const n=300,t=Array.from({length:n},(_,i)=>i*2),r=t.map(x=>1.486-.486*Math.min(1,x/500));this.deliver({type:'shift_trace',cls:0,pd:0,from:3,to:4,n,t,ph:t.map(x=>x<60?1:x<180?2:x<300?3:x<500?4:7),spc:t.map(x=>Math.min(100,50+x/10)|0),mpc:t.map(()=>82),ratio:r.map(x=>Math.round(x*1000)),eng:r.map(x=>Math.round(x*2000+100)),turb:r.map(x=>Math.round(x*2000)),out:t.map(()=>2000),clErr:t.map(()=>0),fl:t.map(()=>0),onClutch:t.map(x=>Math.max(0,400-x)|0),offClutch:t.map(x=>Math.min(400,x)|0)});}
    }
    close(){clearInterval(this.interval);this.readyState=3;this.onclose?.();}
  }
  // No real WebSocket is constructed, including after reconnect or navigation.
  window.WebSocket=PreviewSocket;
})();
