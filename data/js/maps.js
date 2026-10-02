'use strict';
// Reusable heatmap editor. Only the selected cell becomes an input.
window.Heatmap = class {
  constructor(container, options) {
    this.el=container;this.o=options;this.cells=[];this.selected=0;this.original=[...options.values];
    this.build();
  }
  build() {
    const o=this.o,table=document.createElement('table');table.className='map-grid';table.setAttribute('aria-label',o.label||'Calibration values');
    const head=document.createElement('tr');
    [o.corner||'',...o.columns].forEach(x=>{const th=document.createElement('th');th.textContent=x;head.append(th);});table.append(head);
    o.rows.forEach((name,r)=>{const row=document.createElement('tr'),th=document.createElement('th');th.textContent=name;row.append(th);
      o.columns.forEach((_,c)=>{const i=r*o.columns.length+c,td=document.createElement('td');td.tabIndex=i===0?0:-1;td.setAttribute('aria-label',`${name}, ${o.columns[c]}`);
        td.onclick=()=>this.select(i);td.ondblclick=()=>this.edit(i);td.onkeydown=e=>this.key(e,i);td.onpaste=e=>this.paste(e,i);
        this.cells.push(td);row.append(td);this.paint(i);});table.append(row);});
    this.el.replaceChildren(table);
  }
  paint(i){const td=this.cells[i],o=this.o;td.textContent=o.values[i];const f=(o.values[i]-o.min)/Math.max(1,o.max-o.min);td.style.background=`rgba(150,179,128,${.05+Math.max(0,Math.min(1,f))*.28})`;td.classList.toggle('dirty',o.values[i]!==this.original[i]);}
  select(i){this.cells[this.selected]?.classList.remove('selected');this.cells[this.selected].tabIndex=-1;this.selected=i;const td=this.cells[i];td.tabIndex=0;td.classList.add('selected');td.focus();}
  set(i,raw){const value=Number(raw);if(String(raw).trim()===''||!Number.isFinite(value)||!Number.isInteger(value)||value<this.o.min||value>this.o.max)throw new Error(`Enter an integer from ${this.o.min} to ${this.o.max}`);this.o.values[i]=value;this.paint(i);this.o.onchange?.(i,value);}
  edit(i,initial){if(this.cells[i].querySelector('input'))return;const input=document.createElement('input');input.type='number';input.min=this.o.min;input.max=this.o.max;input.step=1;input.value=initial??this.o.values[i];input.setAttribute('aria-label',this.cells[i].getAttribute('aria-label'));this.cells[i].replaceChildren(input);
    let done=false;const finish=cancel=>{if(done)return;done=true;try{if(!cancel)this.set(i,input.value);else this.paint(i);}catch(e){this.paint(i);TCU.notify(e.message,true);}this.cells[i].focus();};
    input.onclick=e=>e.stopPropagation();input.onpaste=e=>e.stopPropagation();input.onkeydown=e=>{e.stopPropagation();if(e.key==='Enter'){e.preventDefault();finish(false);}if(e.key==='Escape'){e.preventDefault();finish(true);}};input.onblur=()=>finish(false);input.focus();if(initial===undefined)input.select();
  }
  key(e,i){if(e.ctrlKey||e.metaKey){if(e.key==='c'){e.preventDefault();navigator.clipboard?.writeText(String(this.o.values[i])).catch(()=>TCU.notify('Clipboard unavailable on this connection',true));}return;}
    const move={ArrowLeft:-1,ArrowRight:1,ArrowUp:-this.o.columns.length,ArrowDown:this.o.columns.length};
    if(e.key in move){e.preventDefault();this.select(Math.max(0,Math.min(this.cells.length-1,i+move[e.key])));}
    else if(e.key==='Enter'||e.key==='F2'){e.preventDefault();this.edit(i);}else if(/^[0-9-]$/.test(e.key)){e.preventDefault();this.edit(i,e.key);}}
  paste(e,i){e.preventDefault();try{const lines=e.clipboardData.getData('text').trim().split(/\r?\n/).map(l=>l.split('\t'));
    const cols=this.o.columns.length,r0=Math.floor(i/cols),c0=i%cols,edits=[];
    lines.forEach((line,r)=>line.forEach((raw,c)=>{const v=Number(raw);if(r0+r>=this.o.rows.length||c0+c>=cols||raw.trim()===''||!Number.isInteger(v)||v<this.o.min||v>this.o.max)throw new Error('Paste exceeds the table or contains an invalid value');edits.push([(r0+r)*cols+c0+c,v]);}));
    edits.forEach(([idx,v])=>this.set(idx,v));}catch(err){TCU.notify(err.message,true);}}
  marker(row,col){this.cells.forEach((td,i)=>td.classList.toggle('operating',i===row*this.o.columns.length+col));}
};
