'use strict';
(() => {
  const $=id=>document.getElementById(id);
  TCU.events.addEventListener('network',({detail:d})=>{
    const pairs=[['Network state',d.state],['Station',d.sta?`${d.ssid} · ${d.rssi} dBm`:'Disconnected'],['Station address',d.ip||'Unavailable'],['Fallback access',d.ap?`${d.apSsid} · ${d.apIp}`:'Inactive'],['Hostname',d.mdns+(d.mdnsActive?' · active':' · pending')],['Credential storage',d.storageOk?'NVS available':'Unavailable']];
    $('network-status').replaceChildren(...pairs.map(([label,value])=>{const card=document.createElement('div');card.className='health-card';const l=document.createElement('label'),v=document.createElement('b');l.textContent=label;v.textContent=value;card.append(l,v);return card;}));
    if(document.activeElement!==$('network-mode'))$('network-mode').value=d.mode;
    $('known-networks').replaceChildren(...d.known.map((k,index)=>{
      const row=document.createElement('div');row.className='network-row';const n=document.createElement('span');n.className='priority';n.textContent=String(index+1).padStart(2,'0');
      const name=document.createElement('div');name.className='network-name';name.textContent=k.ssid;const desc=document.createElement('small');desc.textContent=k.secured?'Password stored':'Open network';name.append(desc);
      const up=document.createElement('button');up.textContent='↑ Priority';up.disabled=index===0;up.onclick=()=>change({op:'up',index},'Change network priority?');
      const remove=document.createElement('button');remove.textContent='Remove';remove.className='danger';remove.onclick=()=>change({op:'remove',index},`Forget ${k.ssid}?`);row.append(n,name,up,remove);return row;
    }));
  });
  async function change(fields,title){try{if(!await TCU.confirm(title,'Network changes require engine off and P/N. This connection may be interrupted.'))return;await TCU.send({cmd:'network.set',...fields});$('network-password').value='';TCU.notify('Network settings saved. Reconnect if the address changes.');setTimeout(()=>{if(TCU.connected)TCU.send({cmd:'network.get'}).catch(()=>{});},1800);}catch(e){TCU.notify(e.message,true);}}
  $('network-mode-save').onclick=()=>change({op:'mode',mode:Number($('network-mode').value)},'Change connection mode?');
  $('network-refresh').onclick=()=>TCU.send({cmd:'network.get'}).catch(e=>TCU.notify(e.message,true));
  $('network-form').onsubmit=e=>{e.preventDefault();change({op:'add',ssid:$('network-ssid').value,password:$('network-password').value},'Remember this network?');};
})();
