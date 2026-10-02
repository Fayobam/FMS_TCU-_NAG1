'use strict';
(() => {
  const $ = id => document.getElementById(id),
    phases = ['CRUISE', 'PREP', 'FILL', 'TORQUE', 'INERTIA', 'RELEASE', 'CATCH', 'LOCK', 'END'];
  const classes = ['POWER UPSHIFT', 'COAST UPSHIFT', 'POWER DOWNSHIFT', 'COAST DOWNSHIFT'];
  const pages = {
    overview: ['Overview', 'LIVE TELEMETRY', 'A clear view of every shift.',
      'Powertrain state, pressure commands and shaft speeds in one place.'
    ],
    analysis: ['Shift analysis', 'CAPTURE & REVIEW', 'Understand the transition.',
      'Observe shift phases, compare shaft speeds and export the completed capture.'
    ],
    maps: ['Calibration maps', 'TUNING WORKSPACE', 'Precision, cell by cell.',
      'Controller-defined maps and curves. Edit locally, then apply deliberately.'
    ],
    adaptation: ['Adaptation', 'LEARNED CORRECTIONS', 'See what the controller learns.',
      'Per-class, per-shift corrections across four torque bins.'
    ],
    profile: ['Engine & sensors', 'POWERTRAIN CONFIGURATION', 'Calibrate the source.',
      'Torque estimation, sensor transfer functions and clutch-model baselines.'
    ],
    diagnostics: ['Diagnostics', 'CONTROLLER HEALTH', 'Trace the signal. Find the fault.',
      'Sensor plausibility, controller state and persistent fault history.'
    ],
    network: ['Network', 'CONNECTION MANAGEMENT', 'Connected in the workshop.',
      'Remember your networks and keep fallback access within reach.'
    ],
    bench: ['Bench tools', 'CONTROLLED TESTING', 'Verify the hardware.',
      'Guarded circuit tests for a secured, stationary bench.'
    ]
  };
  let toastTimer, activePage = 'overview',
    lastPhase = -1,
    shiftStart = 0,
    lastTelemetryPhase = 0;
  let params = [],
    mapEditor = null,
    profile = null,
    profileDirty = false,
    torqueEditor = null,
    cells = null,
    cellsDirty = false,
    adaptEditor = null;
  let mapsDirty = new Set(),
    tuneUnpersisted = false,
    applying = false;
  let sensorReadouts = [];

  function updateSensorReadings() {
    const d = TCU.latest,
      fresh = TCU.isFresh();
    for (const {
        key,
        valid,
        value,
        status
      }
      of sensorReadouts) {
      const reading = d?.[key];
      value.textContent = Number.isFinite(reading) ? `${reading.toFixed(3)} V` : '—';
      status.textContent = !Number.isFinite(reading) ? 'Waiting for telemetry' : !fresh ?
        'STALE · last reading' : d[valid] === false ? 'OUT OF RANGE' : 'LIVE';
      status.classList.toggle('sensor-warning', !fresh || d?.[valid] === false);
    }
  }
  TCU.events.addEventListener('telemetry', updateSensorReadings);
  TCU.events.addEventListener('connection', updateSensorReadings);
  setInterval(updateSensorReadings, 500);
  const set = (id, text) => {
    const e = $(id);
    if (e.textContent !== String(text)) e.textContent = text;
  };
  TCU.notify = (text, error = false) => {
    clearTimeout(toastTimer);
    set('toast', text);
    $('toast').classList.toggle('error', error);
    $('toast').hidden = false;
    toastTimer = setTimeout(() => $('toast').hidden = true, 6500);
  };
  TCU.confirm = (title, text) => new Promise(resolve => {
    const d = $('confirm-dialog');
    if (d.open) {
      resolve(false);
      return;
    }
    set('confirm-title', title);
    set('confirm-text', text);
    d.returnValue = 'cancel';
    d.addEventListener('close', () => resolve(d.returnValue === 'confirm'), {
      once: true
    });
    d.showModal();
  });
  async function action(command, title, text) {
    try {
      if (title && !await TCU.confirm(title, text ||
          'This changes controller state. Confirm the intended action.')) return false;
      await TCU.send(command);
      TCU.notify('Controller acknowledged the action.');
      return true;
    } catch (e) {
      TCU.notify(e.message, true);
      return false;
    }
  }

  function bind(id, fn) {
    $(id).onclick = async () => {
      try {
        await fn();
      } catch (e) {
        TCU.notify(e.message, true);
      }
    };
  }
  const dirtyTag = (id, dirty, clean = 'Loaded from controller') => {
    set(id, dirty ? 'Unsaved changes' : clean);
    $(id).classList.toggle('changed', dirty);
  };

  function navigate() {
    const key = location.hash.slice(1);
    activePage = pages[key] ? key : 'overview';
    const info = pages[activePage];
    set('page-name', info[0]);
    set('page-kicker', info[1]);
    set('page-title', info[2]);
    set('page-subtitle', info[3]);
    document.querySelectorAll('.page').forEach(e => e.hidden = e.id !== activePage);
    document.querySelectorAll('nav a').forEach(e => {
      const active = e.hash === '#' + activePage;
      e.classList.toggle('active', active);
      if (active) e.setAttribute('aria-current', 'page');
      else e.removeAttribute('aria-current');
    });
    if (TCU.connected && activePage === 'network') TCU.send({
      cmd: 'network.get'
    }).catch(() => {});
  }
  addEventListener('hashchange', navigate);
  navigate();

  function metricGroup(id, list) {
    $(id).replaceChildren(...list.map(([key, label, digits = 0]) => {
      const div = document.createElement('div'),
        l = document.createElement('label'),
        b = document.createElement('b');
      l.textContent = label;
      b.dataset.field = key;
      b.dataset.digits = digits;
      b.textContent = '—';
      div.append(l, b);
      return div;
    }));
  }
  metricGroup('shift-metrics', [
    ['observedElapsed', 'Observed elapsed · ms'],
    ['turbRpm', 'Turbine · rpm'],
    ['targetRpm', 'Target shaft sync · rpm'],
    ['shiftTime', 'Last shift · ms'],
    ['onClutch', 'Oncoming slip · rpm'],
    ['offClutch', 'Offgoing slip · rpm'],
    ['mpc', 'MPC command · %'],
    ['spc', 'SPC command · %']
  ]);
  metricGroup('diagnostic-values', [
    ['n2', 'N2 · rpm'],
    ['n3', 'N3 · rpm'],
    ['ratio', 'Calculated ratio', 3],
    ['expectedRatio', 'Expected gear ratio', 3],
    ['tccActual', 'TCC slip · rpm'],
    ['tccTarget', 'TCC target · rpm'],
    ['tInput', 'Input torque estimate · Nm'],
    ['tEstNm', 'Engine torque estimate · Nm'],
    ['mpc', 'MPC command · %'],
    ['spc', 'SPC command · %'],
    ['tccPwm', 'TCC duty · %'],
    ['heap', 'Free heap · bytes']
  ]);
  metricGroup('bench-values', [
    ['tpsV', 'TPS input · V', 3],
    ['mapV', 'MAP input · V', 3],
    ['atfV', 'ATF input · V', 3],
    ['prnd', 'Selector']
  ]);
  $('pressure-meters').innerHTML = ['mpc', 'spc', 'tccPwm'].map((k, i) =>
    `<div class="pressure-meter"><div><label>${['MPC · Line','SPC · Shift','TCC · Lockup'][i]}</label><b data-field="${k}">—</b></div><div class="meter-track"><i id="meter-${k}"></i></div></div>`
  ).join('');
  const fields = [...document.querySelectorAll('[data-field]')];

  function health(id, items) {
    const host = $(id);
    if (host.children.length !== items.length) {
      host.replaceChildren(...items.map(([name]) => {
        const card = document.createElement('div');
        card.className = 'health-card';
        const l = document.createElement('label'),
          b = document.createElement('b');
        l.textContent = name;
        card.append(l, b);
        return card;
      }));
    }
    items.forEach(([, value, state], i) => {
      const c = host.children[i];
      c.className = 'health-card ' + (state || '');
      c.lastChild.textContent = value;
    });
  }

  function phaseTrack(d) {
    const path = d.shiftClass >= 2 ? [1, 5, 6, 7, 8] : [1, 2, 3, 4, 7, 8];
    for (const id of ['overview-phases', 'analysis-phases']) {
      $(id).replaceChildren(...path.map(p => {
        const e = document.createElement('span');
        e.textContent = phases[p];
        e.className = d.phase === p ? 'active' : d.phase > 0 && path.indexOf(d.phase) > path
          .indexOf(p) ? 'passed' : '';
        return e;
      }));
    }
  }
  const elementNames = ['K1', 'K2', 'K3', 'B1', 'B2', 'B3'];
  $('elements').replaceChildren(...elementNames.map(name => {
    const e = document.createElement('span');
    e.textContent = name;
    return e;
  }));
  ['SHIFT A', 'SHIFT B', 'SHIFT C', 'SHIFT D', 'PADDLE +', 'PADDLE −'].forEach(name => {
    const e = document.createElement('span');
    e.textContent = name;
    $('digital-inputs').append(e);
  });
  bind('selector-enable', () => action({
      cmd: 'selector.atf',
      on: true
    }, 'Enable ATF-only manual mode?',
    'Secure the vehicle in P/N, engine off. No automatic downshifts at a stop or electronic reverse-selection protection. The mode persists after restart. Bench verification is required.'
  ));
  bind('selector-disable', () => action({
      cmd: 'selector.atf',
      on: false
    }, 'Restore wired selector?',
    'Secure the vehicle in P/N, engine off. A valid wired selector is required for normal operation.'
  ));

  function live(d) {
    set('selector-status', d.atfOnly ? (d.forwardConfirmed ? 'ATF ONLY / FORWARD CONFIRMED' :
      'ATF ONLY / GEAR UNKNOWN') : 'WIRED SELECTOR');
    $('selector-enable').disabled = d.atfOnly === true;
    $('selector-disable').disabled = d.atfOnly !== true;
    if (d.phase > 0 && lastTelemetryPhase === 0) shiftStart = performance.now();
    lastTelemetryPhase = d.phase;
    const view = {
      ...d,
      phaseName: phases[d.phase] || 'UNKNOWN',
      className: classes[d.shiftClass] || 'UNKNOWN',
      observedElapsed: d.phase ? Math.round(performance.now() - shiftStart) : 0,
      targetRpm: Number.isFinite(d.targetRatio) ? d.outRpm * d.targetRatio : null
    };
    view.atfTemp = d.atfMeasuredC ?? (d.atfSignalOk === true ? d.atfTemp : null);
    fields.forEach(e => {
      const v = view[e.dataset.field],
        text = v == null ? '—' : typeof v === 'number' ? v.toFixed(Number(e.dataset.digits ||
          0)) : String(v);
      if (e.textContent !== text) e.textContent = text;
    });
    set('atf-reading-status', d.atfSignalOk ? 'Live sensor' : d.atfSource === 'held' ?
      'Unavailable · last value held for control' : 'Unavailable · control uses fallback');
    document.querySelector('.sample-rate').textContent = `${1000/(d.intervalMs||100)} Hz`;
    set('uptime', new Date((d.sampledMs || 0)).toISOString().slice(11, 19));
    $('rpm-bar').style.width = Math.min(100, d.engRpm / 70) + '%';
    ['mpc', 'spc', 'tccPwm'].forEach(k => $('meter-' + k).style.width = Math.max(0, d[k]) + '%');
    set('overview-state', d.testMode ? 'BENCH MODE' : d.limp ? 'LIMP PROTECTION' : d.atfOnly && !d
      .forwardConfirmed ? 'HYDRAULIC / GEAR UNKNOWN' : d.phase ? 'SHIFT IN PROGRESS' : d.dtcN ?
      'CHECK DIAGNOSTICS' : 'CONTROLLER ACTIVE');
    const phaseKey = d.phase + ':' + d.shiftClass;
    if (phaseKey !== lastPhase) {
      phaseTrack(d);
      lastPhase = phaseKey;
    }
    set('shift-pair', `${d.gear} → ${d.tgt}`);
    set('path-gear', `GEAR ${d.gear}`);
    // Preserve the existing gear-to-element schematic semantics; no 3D dependency.
    const gears = {
      1: ['K1', 'B2'],
      2: ['K1', 'B1'],
      3: ['K1', 'K2'],
      4: ['K2', 'K3'],
      5: ['K3', 'B1']
    };
    const engaged = d.prnd === 'P' || d.prnd === 'N' ? [] : d.prnd === 'R' ? ['K3', 'B3'] : (
        gears[d.gear] || []),
      incoming = d.phase ? (gears[d.tgt] || []) : [];
    [...$('elements').children].forEach((e, i) => {
      e.classList.toggle('on', engaged.includes(elementNames[i]));
      e.classList.toggle('incoming', incoming.includes(elementNames[i]) && !engaged.includes(
        elementNames[i]));
    });
    [...$('digital-inputs').children].forEach((e, i) => e.classList.toggle('on', !!(d.din & (1 <<
      i))));
    $('fault-banner').hidden = !d.limp && !d.testMode && !d.revAbuse;
    set('fault-banner', d.limp ? `LIMP PROTECTION · ${d.limpReason}` : d.revAbuse ?
      'REVERSE ABUSE PROTECTION ACTIVE' : d.testMode ?
      'BENCH MODE ACTIVE · Normal driving functions are altered.' : '');
    health('health-grid', [
      ['Speed acquisition', d.spdHwOk ? 'GOOD · hardware initialized' :
        'FAULT · hardware initialization', d.spdHwOk ? 'good' : 'fault'
      ],
      ['N2 / N3 plausibility', d.inTrust ? 'GOOD · input trusted' :
        'WARNING · input untrusted', d.inTrust ? 'good' : 'warning'
      ],
      ['TPS signal', d.tpsOk ? 'GOOD' : 'FAULT · voltage out of range', d.tpsOk ? 'good' :
        'fault'
      ],
      ['MAP signal', d.mapOk ? 'GOOD' : 'FAULT · voltage out of range', d.mapOk ? 'good' :
        'fault'
      ],
      ['ATF temperature', d.atfSignalOk ? 'LIVE' : d.atfSource === 'held' ?
        'UNAVAILABLE · last reading held' : 'UNAVAILABLE · no valid reading', d.atfSignalOk ?
        'good' : 'warning'
      ],
      ['ATF / P-N circuit', d.atfCircuit === 'temperature' ? 'Temperature path available' : d
        .atfCircuit === 'pn_or_open' ? 'P/N contact open or wiring open' : d.atfCircuit ===
        'low_or_short' ? 'Low voltage / possible short' : 'Not sampled', d.atfSignalOk ?
        'good' : 'warning'
      ],
      ['ATF voltage', Number.isFinite(d.atfV) ? `${d.atfV.toFixed(3)} V` : 'Unknown', ''],
      ['Temperature used by control', Number.isFinite(d.atfTemp) ?
        `${d.atfTemp.toFixed(1)} °C · ${d.atfSource||'source unknown'}` : 'Unknown', d
        .atfSignalOk ? 'good' : 'warning'
      ],
      ...[
        ['N2 pulses', 'n2Recent'],
        ['N3 pulses', 'n3Recent'],
        ['Output pulses', 'outRecent'],
        ['Engine pulses', 'engRecent']
      ].map(([label, key]) => [label, !d.spdHwOk ? 'ACQUISITION UNAVAILABLE' : d[key] ===
        true ? 'RECENT PULSES' : d[key] === false ?
        'NO RECENT PULSES · stopped or missing' : 'Not reported', d.spdHwOk && d[key] ===
        true ? 'good' : ''
      ]),
      ['Battery voltage', d.batterySupported && Number.isFinite(d.batteryV) ?
        `${d.batteryV.toFixed(2)} V` : 'Sensing not configured', ''
      ],
      ['Solenoid current', d.currentSupported && Number.isFinite(d.solenoidCurrentA) ?
        `${d.solenoidCurrentA.toFixed(2)} A` : 'Sensing not configured', ''
      ],
      ['Protection state', d.limp ? 'FAULT · limp active' : d.revAbuse ?
        'FAULT · reverse abuse' : d.testMode ? 'WARNING · bench mode' : d.dtcN ?
        'WARNING · active DTCs' : 'GOOD', d.limp || d.revAbuse ? 'fault' : d.testMode || d
        .dtcN ? 'warning' : 'good'
      ],
      ['Web assets', 'Embedded in firmware', 'good'],
      ['Optional SPIFFS', d.fsOk ? 'Mounted' : 'Unavailable · UI remains available', d.fsOk ?
        'good' : 'warning'
      ],
      ['Last safety event', d.safety || 'No event reported', ''],
      ['Gear ratio observation', d.outRpm > 200 ? 'Observable · compare ratios' :
        'Unavailable at low output speed', ''
      ],
      ['Flare / bind', d.flare || d.bind ? `${d.flare?'FLARE ':''}${d.bind?'BIND':''}` :
        'Not flagged', d.flare || d.bind ? 'warning' : 'good'
      ],
      ['WebSocket clients', String(d.clients), '']
    ]);
    $('bench-controls').hidden = !d.testMode;
    set('bench-status', d.testMode ? 'ACTIVE' : 'INACTIVE');
    $('bench-enable').disabled = !!d.testMode;
    document.querySelectorAll('[data-output-status]').forEach(e => e.textContent = d.tout & (1 <<
      Number(e.dataset.outputStatus)) ? 'LATCHED ON' : 'OFF');
    markOperatingPoint(d);
  }
  TCU.events.addEventListener('telemetry', e => live(e.detail));

  function freshness() {
    const link = TCU.delivery(),
      fresh = link.state === 'live',
      d = TCU.latest;
    const label = {
      live: 'Controller connected',
      waiting: 'Awaiting telemetry',
      snapshot_stale: 'Snapshot stale',
      delivery_stale: 'Telemetry delayed',
      disconnected: 'Reconnecting'
    };
    const message = {
      waiting: 'Connected. Waiting for the first telemetry sample.',
      snapshot_stale: 'Messages are arriving, but the control snapshot is stale. Values below are not live.',
      delivery_stale: 'Telemetry delivery has paused. Values below are the last received sample.',
      disconnected: 'Controller disconnected. Values below are stale. Pending commands are not replayed.'
    };
    document.body.classList.toggle('stale', !fresh);
    $('offline').hidden = fresh;
    set('connection-label', label[link.state]);
    $('connection-dot').classList.toggle('off', !fresh);
    if (!fresh) set('offline', message[link.state]);
    const ms = v => Number.isFinite(v) ? `${Math.round(v)} ms` : 'Unavailable';
    health('delivery-status', [
      ['Delivery state', label[link.state], fresh ? 'good' : 'warning'],
      ['Last telemetry arrival', ms(link.packetAgeMs), ''],
      ['Snapshot age (minimum)', ms(link.snapshotAgeMs), ''],
      ['Largest receive gap', ms(link.maxReceiveGapMs), ''],
      ['WebSocket reconnects', String(link.reconnects), ''],
      ['Invalid messages', String(link.invalidPackets), link.invalidPackets ? 'warning' : ''],
      ['Controller service gap (max)', ms(d?.serviceGapMaxMs), ''],
      ['Telemetry queue skips', d?.txQueueSkips ?? 'Unavailable', ''],
      ['Oversize telemetry drops', d?.txOversize ?? 'Unavailable', d?.txOversize ? 'warning' :
        ''
      ]
    ]);
  }
  // Clear the warning as soon as a fresh sample arrives, not at the next UI timer.
  TCU.events.addEventListener('telemetry', freshness);
  setInterval(freshness, 500);
  TCU.events.addEventListener('connection', async e => {
    freshness();
    if (!e.detail) return;
    lastTelemetryPhase = 0;
    for (const cmd of ['get_profile', 'param.list', 'get_cells', 'get_dtcs', 'network.get'])
      try {
        await TCU.send({
          cmd
        });
      } catch (err) {
        TCU.notify(err.message, true);
        break;
      }
  });
  TCU.events.addEventListener('error', e => TCU.notify(e.detail, true));
  TCU.events.addEventListener('dtc_data', ({
    detail: d
  }) => {
    $('dtc-body').replaceChildren(...d.dtcs.map(x => {
      const tr = document.createElement('tr');
      [String(x.code).padStart(2, '0'), x.name, x.active ? 'FAULT / ACTIVE' : x.count ?
        'HISTORY' : 'GOOD', x.count, x.lastMs ? `${x.lastMs} ms` : '—'
      ].forEach(v => {
        const td = document.createElement('td');
        td.textContent = v;
        tr.append(td);
      });
      return tr;
    }));
  });

  // Parameters are described by firmware. Preserve actual axes and units.
  function axes(p) {
    if (p.id === 'line.map') return {
      rows: Array.from({
        length: p.rows
      }, (_, i) => `Gear ${i+1}`),
      columns: Array.from({
        length: p.cols
      }, (_, i) => `${i*12.5}`),
      corner: 'Gear / load units'
    };
    if (p.id === 'auto.up' || p.id === 'auto.dn') return {
      rows: Array.from({
        length: p.rows
      }, (_, i) => p.id === 'auto.up' ? `${i+1} → ${i+2}` : `${i+2} → ${i+1}`),
      columns: Array.from({
        length: p.cols
      }, (_, i) => `${i*10}%`),
      corner: 'Shift / TPS'
    };
    if (p.id.startsWith('mode.')) return {
      rows: [p.name],
      columns: ['D', '4', '3', '2', '1'],
      corner: 'Lever position'
    };
    return {
      rows: Array.from({
        length: p.rows
      }, (_, i) => p.rows === 1 ? p.unit : `${i+1}`),
      columns: Array.from({
        length: p.cols
      }, (_, i) => p.cols === 11 ? `${i*10}%` : p.cols === 1 ? 'Value' : String(i)),
      corner: p.cols === 11 ? 'Load %' : ''
    };
  }

  function mapDirty() {
    dirtyTag('map-dirty', mapsDirty.size > 0, tuneUnpersisted ? 'Applied · not saved to NVS' :
      'Controller values');
  }

  function renderMap() {
    const p = params.find(x => x.idx === Number($('map-select').value));
    if (!p) return;
    set('map-name', p.name);
    set('map-unit', p.unit);
    set('map-help', p.help);
    mapEditor = new Heatmap($('map-grid'), {
      ...axes(p),
      label: p.name,
      values: p.v,
      min: p.min,
      max: p.max,
      onchange: (i) => {
        mapsDirty.add(p.idx + ':' + i);
        mapDirty();
      }
    });
    mapDirty();
  }
  TCU.events.addEventListener('param_list', ({
    detail: d
  }) => {
    if (mapsDirty.size || applying) return;
    params = d.params;
    $('map-select').replaceChildren(...params.map(p => {
      const o = document.createElement('option');
      o.value = p.idx;
      o.textContent = `${p.group} / ${p.name}`;
      return o;
    }));
    renderMap();
  });
  $('map-select').onchange = renderMap;
  bind('map-reload', async () => {
    if (mapsDirty.size && !await TCU.confirm('Discard staged map edits?',
        'Reload the current controller values.')) return;
    mapsDirty.clear();
    await TCU.send({
      cmd: 'param.list'
    });
  });
  bind('map-apply', async () => {
    if (applying || !mapsDirty.size) return;
    if (!await TCU.confirm('Apply calibration changes?',
        'Requires engine off, stopped shafts and P/N. Each changed cell is validated and applied; save separately to retain it after restart.'
      )) return;
    applying = true;
    $('map-grid').inert = true;
    ['map-select', 'map-reload', 'map-reset', 'map-apply', 'map-persist'].forEach(id => $(
      id).disabled = true);
    try {
      for (const key of [...mapsDirty]) {
        const [idx, i] = key.split(':').map(Number), p = params.find(x => x.idx === idx);
        await TCU.send({
          cmd: 'param.set',
          idx,
          row: Math.floor(i / p.cols),
          col: i % p.cols,
          val: p.v[i]
        });
        mapsDirty.delete(key);
        tuneUnpersisted = true;
        mapDirty();
      }
      TCU.notify('All edited cells applied. Save to retain after restart.');
    } finally {
      applying = false;
      $('map-grid').inert = false;
      ['map-select', 'map-reload', 'map-reset', 'map-apply', 'map-persist'].forEach(id => $(
        id).disabled = false);
      mapDirty();
    }
  });
  bind('map-persist', async () => {
    if (mapsDirty.size) throw new Error(
      'Apply or discard staged edits before saving controller values.');
    if (await action({
          cmd: 'param.persist'
        }, 'Save applied tuning?',
        'Write the current controller tuning to nonvolatile storage.')) {
      tuneUnpersisted = false;
      mapDirty();
    }
  });
  bind('map-reset', async () => {
    if (await action({
          cmd: 'param.reset'
        }, 'Restore all tuning defaults?',
        'This overwrites all tuning maps and mode settings and saves the defaults. Engine off and P/N required.'
      )) {
      mapsDirty.clear();
      tuneUnpersisted = false;
      await TCU.send({
        cmd: 'param.list'
      });
    }
  });

  function markOperatingPoint(d) {
    if (mapEditor) {
      const p = params.find(x => x.idx === Number($('map-select').value));
      let r = -100,
        c = 0;
      if (p?.id === 'line.map') {
        r = d.gear - 1;
        c = Math.min(15, Math.floor((d.tps * 1.25 + Math.max(0, d.map - 100) * .8) / 12.5));
      } else if (p?.id.startsWith('auto.')) {
        r = p.id === 'auto.up' ? d.gear - 1 : d.gear - 2;
        c = Math.min(10, Math.round(d.tps / 10));
      } else if (p?.id.startsWith('mode.')) {
        r = 0;
        c = d.mode;
      } else if (p?.cols === 11) {
        r = 0;
        c = Math.min(10, Math.round(d.loadPct / 10));
      }
      mapEditor.marker(r, c);
    }
    if (torqueEditor && profile) {
      const nearest = (a, v) => a.reduce((best, x, i) => Math.abs(x - v) < Math.abs(a[best] - v) ?
        i : best, 0);
      torqueEditor.marker(nearest(profile.rpm, d.engRpm), nearest(profile.map, d.map));
    }
  }

  function renderAdapt() {
    if (!cells) return;
    const cls = Number($('adapt-class').value),
      field = Number($('adapt-field').value);
    const values = Array.from({
      length: 16
    }, (_, i) => cells[(cls * 16 + i) * 3 + field]);
    adaptEditor = new Heatmap($('adapt-grid'), {
      rows: Array.from({
        length: 4
      }, (_, i) => cls < 2 ? `${i+1} → ${i+2}` : `${i+2} → ${i+1}`),
      columns: ['0–25%', '25–50%', '50–75%', '75–100%'],
      corner: 'Shift / torque',
      values,
      min: field === 0 ? -5 : -15,
      max: field === 0 ? 5 : 15,
      onchange: (i, v) => {
        cells[(cls * 16 + i) * 3 + field] = v;
        cellsDirty = true;
        dirtyTag('adapt-dirty', true);
      }
    });
    dirtyTag('adapt-dirty', cellsDirty);
  }
  TCU.events.addEventListener('cell_data', ({
    detail: d
  }) => {
    if (cellsDirty) return;
    if (d.data.length !== 192) {
      TCU.notify('Unsupported adaptation layout', true);
      return;
    }
    cells = [...d.data];
    renderAdapt();
  });
  $('adapt-class').onchange = renderAdapt;
  $('adapt-field').onchange = renderAdapt;
  bind('adapt-reload', async () => {
    if (cellsDirty && !await TCU.confirm('Discard correction edits?',
        'Reload learned values from the controller.')) return;
    cellsDirty = false;
    await TCU.send({
      cmd: 'get_cells'
    });
  });
  bind('adapt-save', async () => {
    if (!cells) throw new Error('Load adaptation first');
    if (await action({
          cmd: 'set_cells',
          data: cells
        }, 'Apply learned corrections?',
        'Replace all adaptation cells and schedule their NVS save. Engine off and P/N required.'
      )) {
      cellsDirty = false;
      dirtyTag('adapt-dirty', false, 'Applied · save scheduled');
    }
  });
  bind('adapt-reset', async () => {
    if (await action({
          cmd: 'set_cells',
          data: Array(192).fill(0)
        }, 'Reset all learned values?',
        'Zero all four shift classes and torque bins. Existing profile fill baselines are retained.'
      )) {
      cells = Array(192).fill(0);
      cellsDirty = false;
      renderAdapt();
    }
  });
  bind('nudge-soft', () => action({
    cmd: 'adapt_nudge',
    dir: -1
  }, 'Soften the last learned shift?'));
  bind('nudge-firm', () => action({
    cmd: 'adapt_nudge',
    dir: 1
  }, 'Firm the last learned shift?'));

  const schema = [
    ['Engine limits', [
      ['tmax', 'Reference torque · Nm', 100, 1200],
      ['overrev', 'Overrev threshold · rpm', 3000, 9000],
      ['lug', 'Lug threshold · rpm', 500, 2500],
      ['engPpr', 'Engine pulses / revolution', 1, 240],
      ['outPpr', 'Output pulses / revolution', 1, 240],
      ['kmhRpm', 'km/h per output rpm', .001, 1, .001]
    ]],
    ['Sensor calibration', [
      ['tpsC', 'TPS closed · V', 0, 3.3, .01],
      ['tpsW', 'TPS WOT · V', 0, 3.3, .01],
      ['map0', 'MAP at 0 V · kPa', -100, 260, .01],
      ['mapV', 'MAP slope · kPa / V', 1, 300, .01]
    ]],
    ['Converter & control', [
      ['tcStall', 'Stall multiplication · ×100', 100, 350],
      ['tcCoupSr', 'Coupling speed ratio · ×100', 50, 100],
      ['clEn', 'Ratio SPC loop · 0 off / 1 on', 0, 1],
      ['clKp', 'SPC proportional gain', 0, 1000],
      ['clSpeed', 'Clutch fill observer · 0 / 1', 0, 1],
      ['clPwr', 'Experimental pressure model · 0 / 1', 0, 1]
    ]],
    ['Clutch model', [
      ['pFull', 'Full-scale pressure · mbar', 4000, 25000],
      ['coefStat', 'Stationary coefficient', 50, 255],
      ['coefRel', 'Releasing coefficient', 50, 255],
      ['coefCold', 'Cold apply coefficient', 50, 255],
      ['coefHot', 'Hot apply coefficient', 50, 255]
    ]]
  ];
  const arrays = [
    ['fillp', 'Fill baseline · %', 0, 100],
    ['fillt', 'Fill baseline · ms', 0, 400],
    ['applyFric', 'Apply friction numerator', 500, 12000],
    ['relFric', 'Release friction numerator', 500, 12000],
    ['applySpring', 'Apply spring · mbar', 0, 5000],
    ['relSpring', 'Release spring · mbar', 0, 5000]
  ];

  function markProfile() {
    profileDirty = true;
    dirtyTag('profile-dirty', true);
  }

  function renderProfile() {
    if (!profile) return;
    torqueEditor = new Heatmap($('torque-grid'), {
      rows: profile.rpm,
      columns: profile.map,
      corner: 'RPM / MAP kPa',
      label: 'Engine torque in Nm',
      values: profile.torque,
      min: 0,
      max: 1200,
      onchange: markProfile
    });
    sensorReadouts = [];
    $('profile-fields').replaceChildren();
    const field = (grid, key, label, min, max, step = 1, index = null) => {
      const l = document.createElement('label');
      l.textContent = label;
      const input = document.createElement('input');
      input.type = 'number';
      input.min = min;
      input.max = max;
      input.step = step;
      input.value = index === null ? profile[key] : profile[key][index];
      input.dataset.key = key;
      if (index !== null) input.dataset.index = index;
      input.oninput = markProfile;
      l.append(input);
      grid.append(l);
    };
    const group = (name) => {
      const card = document.createElement('article');
      card.className = 'panel';
      const h = document.createElement('h2');
      h.textContent = name;
      const grid = document.createElement('div');
      grid.className = 'field-grid';
      card.append(h, grid);
      $('profile-fields').append(card);
      return grid;
    };
    schema.forEach(([name, entries]) => {
      const grid = group(name);
      if (name === 'Sensor calibration') {
        const readings = document.createElement('div');
        readings.className = 'sensor-readings';
        for (const [key, valid, label] of [
            ['tpsV', 'tpsOk', 'TPS · ADC input'],
            ['mapV', 'mapOk', 'MAP · ADC input']
          ]) {
          const card = document.createElement('div'),
            title = document.createElement('span'),
            value = document.createElement('output'),
            status = document.createElement('small');
          title.textContent = label;
          value.dataset.sensorReading = key;
          value.setAttribute('aria-label', label + ' volts');
          card.append(title, value, status);
          readings.append(card);
          sensorReadouts.push({
            key,
            valid,
            value,
            status
          });
        }
        const note = document.createElement('p');
        note.className = 'sensor-note';
        note.textContent =
          'Live ADC voltage before TPS / MAP conversion. Read-only; calibration edits are applied only when saved.';
        grid.before(readings, note);
      }
      entries.forEach(args => field(grid, ...args));
    });
    updateSensorReadings();
    arrays.forEach(([key, label, min, max]) => {
      const grid = group(label);
      for (let i = 0; i < 4; i++) field(grid, key, `${i+1} → ${i+2}`, min, max, 1, i);
    });
    $('variant').value = profile.transVariant;
    dirtyTag('profile-dirty', profileDirty);
  }
  TCU.events.addEventListener('profile_data', ({
    detail: d
  }) => {
    if (profileDirty) return;
    profile = JSON.parse(JSON.stringify(d));
    renderProfile();
  });
  bind('profile-reload', async () => {
    if (profileDirty && !await TCU.confirm('Discard profile edits?',
        'Reload profile values from the controller.')) return;
    profileDirty = false;
    await TCU.send({
      cmd: 'get_profile'
    });
  });
  bind('profile-save', async () => {
    if (!profile) throw new Error('Load the engine profile first');
    const next = {
      torque: [...profile.torque]
    };
    for (const input of $('profile-fields').querySelectorAll('input')) {
      if (!input.reportValidity() || input.value.trim() === '') throw new Error(
        'Correct the highlighted profile value');
      const key = input.dataset.key,
        v = Number(input.value);
      if (input.dataset.index !== undefined) {
        next[key] ??= [];
        next[key][Number(input.dataset.index)] = v;
      } else next[key] = v;
    }
    if (next.tpsW <= next.tpsC + .01) throw new Error('TPS WOT must exceed closed voltage');
    if (await action({
          cmd: 'set_profile',
          ...next
        }, 'Apply and save engine profile?',
        'Updates torque estimation, sensor calibration and clutch baselines. Requires engine off, stopped shafts and P/N.'
      )) {
      profileDirty = false;
      await TCU.send({
        cmd: 'get_profile'
      });
    }
  });
  bind('variant-save', async () => {
    if (await action({
          cmd: 'set_profile',
          transVariant: Number($('variant').value)
        }, 'Change transmission geometry?',
        'This changes gear ratios and seeds clutch-model defaults. Any staged profile edits will be discarded. Engine off and P/N required.'
      )) {
      profileDirty = false;
      await TCU.send({
        cmd: 'get_profile'
      });
    }
  });

  bind('dtc-reload', () => TCU.send({
    cmd: 'get_dtcs'
  }));
  bind('dtc-clear', async () => {
    if (await action({
          cmd: 'clear_dtcs'
        }, 'Clear diagnostic history?',
        'Clears stored occurrences. Active faults will be detected again by the controller.'
      )) await TCU.send({
      cmd: 'get_dtcs'
    });
  });
  bind('limp-reset', () => action({
      cmd: 'limp_reset'
    }, 'Request limp recovery?',
    'Stop the shafts, switch the engine off and select P/N. The controller retains its own recovery gates.'
  ));
  bind('bench-enable', () => action({
      cmd: 'test_mode',
      on: true
    }, 'Enter bench mode?',
    'Use a secured bench with engine off and selector in P/N. Automatic shifting and some protections are altered. Output latches persist if this browser disconnects.'
  ));
  bind('bench-disable', () => action({
    cmd: 'test_mode',
    on: false
  }));
  for (const v of 'PRND4321') {
    const b = document.createElement('button');
    b.textContent = v;
    b.onclick = () => action({
      cmd: 'test_prnd',
      v
    }, `Select virtual ${v}?`);
    $('bench-selector').append(b);
  }
  bind('paddle-up', () => action({
    cmd: 'test_paddle',
    dir: 1
  }, 'Request a bench upshift?'));
  bind('paddle-down', () => action({
    cmd: 'test_paddle',
    dir: -1
  }, 'Request a bench downshift?'));
  ['y3', 'y5', 'y4', 'mpc', 'spc', 'tcc', 'rp', 'tq'].forEach((id, i) => {
    const card = document.createElement('div');
    card.className = 'output-card';
    const label = document.createElement('label');
    label.textContent = id.toUpperCase();
    const state = document.createElement('small');
    state.dataset.outputStatus = i;
    state.textContent = 'UNKNOWN';
    const input = document.createElement('input');
    input.type = 'number';
    input.min = 0;
    input.max = 100;
    input.step = 1;
    input.value = 50;
    input.setAttribute('aria-label', id + ' command percent');
    if (i < 3 || i > 5) input.hidden = true;
    const on = document.createElement('button');
    on.className = 'danger';
    on.textContent = 'Activate…';
    on.onclick = () => {
      if (input.reportValidity()) action({
          cmd: 'test_io',
          id,
          on: true,
          v: Number(input.value)
        }, `Activate ${id.toUpperCase()}?`,
        'The output remains latched until released or bench mode is exited. Confirm wiring and the command polarity.'
      );
    };
    const off = document.createElement('button');
    off.textContent = 'Release';
    off.onclick = () => action({
      cmd: 'test_io',
      id,
      on: false
    });
    card.append(label, state, input, on, off);
    $('bench-outputs').append(card);
  });
  addEventListener('beforeunload', e => {
    if (mapsDirty.size || profileDirty || cellsDirty || tuneUnpersisted) {
      e.preventDefault();
      e.returnValue = '';
    }
  });
  setInterval(() => {
    if (TCU.connected && activePage === 'network') TCU.send({
      cmd: 'network.get'
    }).catch(() => {});
  }, 10000);
  TCU.start();
})();
