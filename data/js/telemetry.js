'use strict';
window.TCU = { events: new EventTarget(), latest: null, connected: false, lastSample: 0 };
(() => {
  let socket, retryTimer, retries = 0, requestId = 0, pending = null;
  const queue = [];
  const STALE_MS = 2000;
  let lastPacketAt = null, lastProgressAt = null, sampledMs = null, sourceAge = 0;
  let maxReceiveGapMs = 0, invalidPackets = 0, reconnects = 0, openedBefore = false;
  TCU.delivery = () => {
    const now = performance.now();
    const packetAgeMs = lastPacketAt === null ? null : now-lastPacketAt;
    const snapshotAgeMs = lastProgressAt === null ? null : sourceAge+now-lastProgressAt;
    const state = !TCU.connected ? 'disconnected' : snapshotAgeMs === null ? 'waiting'
      : snapshotAgeMs < STALE_MS ? 'live'
      : packetAgeMs < STALE_MS ? 'snapshot_stale' : 'delivery_stale';
    return {state,packetAgeMs,snapshotAgeMs,maxReceiveGapMs,invalidPackets,reconnects};
  };
  function invalidResponse() {
    if (++invalidPackets === 1) emit('error','Invalid response from controller; see delivery diagnostics.');
  }
  const emit = (type, detail) => TCU.events.dispatchEvent(new CustomEvent(type, {detail}));
  function rejectQueue(message) {
    if (pending) { clearTimeout(pending.timer); pending.reject(new Error(message)); pending = null; }
    while (queue.length) queue.shift().reject(new Error(message));
  }
  function pump() {
    if (pending || !queue.length || !TCU.connected) return;
    pending = queue.shift();
    const item = pending;
    item.id = ++requestId;
    item.timer = setTimeout(() => {
      rejectQueue('Acknowledgement timed out. Result is unknown; reload values before retrying.');
      socket.close();
    }, 6000);
    socket.send(JSON.stringify({...item.command, requestId:item.id}));
  }
  TCU.send = command => new Promise((resolve, reject) => {
    if (!TCU.connected || socket.readyState !== WebSocket.OPEN) return reject(new Error('Controller disconnected. No command sent.'));
    if (queue.length >= 100) return reject(new Error('Too many pending commands'));
    if (new TextEncoder().encode(JSON.stringify(command)).length > 2950) return reject(new Error('Command exceeds firmware size limit'));
    queue.push({command, resolve, reject}); pump();
  });
  function connect() {
    clearTimeout(retryTimer);
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    socket = new WebSocket(`${scheme}://${location.host}/ws`);
    const current = socket;
    const opening = setTimeout(() => { if (current.readyState === WebSocket.CONNECTING) current.close(); },10000);
    socket.onopen = () => {
      if (socket !== current) return;
      if (openedBefore) ++reconnects;
      openedBefore = true;
      lastPacketAt = lastProgressAt = sampledMs = null; sourceAge = 0;
      clearTimeout(opening); retries = 0; TCU.connected = true;
      TCU.lastSample = 0; emit('connection', true);
    };
    socket.onmessage = event => {
      if (socket !== current) return;
      let d; try { d = JSON.parse(event.data); } catch { invalidResponse(); return; }
      if (!d || typeof d !== 'object' || Array.isArray(d)) { invalidResponse(); return; }
      if (d.type === 'command_result') {
        if (!pending || d.requestId !== pending.id) return;
        const item = pending; clearTimeout(item.timer); pending = null;
        if (d.ok) item.resolve(d); else item.reject(new Error(d.message));
        pump(); return;
      }
      if (d.type === 'error') { rejectQueue(d.message || 'Controller rejected command'); return; }
      if (d.type === 'telemetry') {
        if (!Number.isInteger(d.sampledMs) || d.sampledMs < 0 || d.sampledMs > 0xffffffff
            || (d.snapshotAgeMs !== undefined && (!Number.isFinite(d.snapshotAgeMs) || d.snapshotAgeMs < 0))) {
          invalidResponse(); return;
        }
        const now = performance.now();
        if (lastPacketAt !== null) maxReceiveGapMs = Math.max(maxReceiveGapMs,now-lastPacketAt);
        lastPacketAt = now;
        // Repeated snapshots and freshly delivered OLD snapshots must not look live.
        if (sampledMs === null || d.sampledMs !== sampledMs) {
          sampledMs = d.sampledMs; lastProgressAt = now; sourceAge = d.snapshotAgeMs ?? 0;
          TCU.lastSample = now;
        }
        TCU.latest = d;
      }
      emit(d.type, d);
    };
    socket.onclose = () => {
      if (socket !== current) return;
      clearTimeout(opening); TCU.connected = false; TCU.lastSample = 0;
      rejectQueue('Connection lost. Unacknowledged actions may have applied; reload before retrying.');
      emit('connection',false);
      const delay = Math.min(15000,1000 * 2 ** Math.min(retries++,4)) + Math.random()*350;
      retryTimer = setTimeout(connect,delay);
    };
    socket.onerror = () => { if (socket === current) current.close(); };
  }
  TCU.start = connect;
  TCU.isFresh = () => TCU.delivery().state === 'live';
})();
