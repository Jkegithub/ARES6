/* ARES-6 — UI controller: control panel (declarative schema, two-way bound), telemetry, leg status, log, minimap, timeline */
(function () {
  'use strict';
  const A = window.ARES, S = A.State, D = A.DEG, U = A.U;
  const $ = (id) => document.getElementById(id);
  const el = (tag, cls, html) => { const e = document.createElement(tag); if (cls) e.className = cls; if (html !== undefined) e.innerHTML = html; return e; };

  // display texts for internal state keys (keys stay English for the logic)
  const MODE_DE = { MANUAL: 'MANUELL', AUTONOMOUS: 'AUTONOM', FAULT: 'FEHLER', SHUTDOWN: 'ABGESCHALTET', BOOTING: 'STARTET' };
  const LEG_ST_DE = { grounded: 'steht', lifting: 'hebt', moving: 'schwingt', error: 'Fehler', idle: 'Ruhe' };
  const DRONE_DE = { docked: 'ANGEDOCKT', launch: 'STEIGT', orbit: 'ORBIT', return: 'RÜCKFLUG', land: 'LANDET', emergency: 'NOTLANDUNG', grounded: 'GELANDET' };
  const LOCO_DE = { idle: 'STEHT', walk: 'LAUFEN', crawl: 'KRIECHEN', rotate: 'DREHEN', turnTo: 'WENDEN', nav: 'NAVIGATION', shift: 'RANGIEREN' };

  const UI = (A.UI = {
    sliders: [], toggles: [], btns: {}, acc: 0, fpsAcc: 0, fpsN: 0, alertT: 0,

    init() {
      const R = A.robot, M = A.Missions, Lo = A.Loco;
      const userLoco = () => { if (A.Tasks.running()) A.Tasks.cancel('operator override'); };
      const legSel = () => R.legById[S.selectedLeg];
      const arm = R.arm;
      const armSet = (k, v) => { if (A.Tasks.running()) A.Tasks.cancel('operator override'); arm.target[k] = v * D; };

      const schema = [
        { title: 'ROBOTERZUSTAND', id: 'robot-state', keep: true, items: [
          { t: 'btn', label: 'Simulation zurücksetzen', fn: () => A.Main.resetSim() },
          { t: 'btn', label: 'Nullpose', fn: () => M.zero() },
          { t: 'btn', label: 'Not-Aus', cls: 'danger', fn: () => M.shutdown() },
          { t: 'btn', label: 'Systeme neu starten', id: 'reboot', fn: () => M.reboot() },
          { t: 'btn', label: '⚕ Auto-Stabilisierung', id: 'stab', cls: 'full', fn: () => M.stabilize() },
          { t: 'tog', label: 'Autonom', key: 'autonomous', fn: (v) => { if (v) M.patrol(); else if (A.Tasks.cur && A.Tasks.cur.name === 'AUTONOME PATROUILLE') A.Tasks.cancel('off'); } },
          { t: 'tog', label: 'Debug-Rahmen', key: 'debug', fn: (v) => A.Main.setDebug(v) },
          { t: 'tog', label: 'Drahtgitter', key: 'wireframe', fn: (v) => R.setWireframe(v) },
          { t: 'tog', label: 'Sensorkegel', key: 'cones' },
        ] },
        { title: 'KÖRPERSTEUERUNG', id: 'body', items: [
          { t: 'sl', label: 'Körperhöhe', key: 'bodyHeight', min: 0.25, max: 1.1, step: 0.01, fmt: (v) => v.toFixed(2) + ' m' },
          { t: 'sl', label: 'Körper-Gier (Kurs)', min: -180, max: 180, step: 1, fmt: (v) => v.toFixed(0) + '°', get: () => U.wrap(R.pose.yaw) / D, set: (v) => { userLoco(); Lo.setMode('turnTo', { yaw: v * D }); } },
          { t: 'sl', label: 'Körper-Nicken', key: 'bodyPitch', min: -30, max: 30, step: 0.5, fmt: (v) => v.toFixed(1) + '°' },
          { t: 'sl', label: 'Körper-Rollen', key: 'bodyRoll', min: -30, max: 30, step: 0.5, fmt: (v) => v.toFixed(1) + '°' },
          { t: 'btn', label: 'Kriechpose', fn: () => M.setPose('crawl') },
          { t: 'btn', label: 'Hohe Bodenfreiheit', fn: () => M.setPose('high') },
          { t: 'btn', label: 'Stabile Rettungspose', cls: 'full', fn: () => M.setPose('rescue') },
        ] },
        { title: 'FORTBEWEGUNG', id: 'locomotion', items: [
          { t: 'btn', label: '▶ Laufzyklus starten', id: 'walk', fn: () => { userLoco(); if (Lo.setMode('walk')) A.Log.add('Laufzyklus gestartet — wechselnder Dreifußgang'); } },
          { t: 'btn', label: '■ Laufzyklus stoppen', fn: () => { userLoco(); Lo.stop('Laufzyklus gestoppt — Füße kommen zur Ruhe'); } },
          { t: 'btn', label: 'Vorwärts kriechen', fn: () => { userLoco(); M.setPose('crawl'); if (Lo.setMode('crawl')) A.Log.add('Vorwärts kriechen — flaches Profil'); } },
          { t: 'btn', label: 'Auf der Stelle drehen', fn: () => { userLoco(); if (Lo.setMode('rotate', { dir: 1 })) A.Log.add('Drehung auf der Stelle'); } },
          { t: 'btn', label: 'Kletter-Demo', cls: 'full', fn: () => M.climb() },
          { t: 'sl', label: 'Laufgeschwindigkeit', key: 'walkSpeed', min: 0, max: 100, step: 1, fmt: (v) => v.toFixed(0) + ' %' },
          { t: 'sl', label: 'Schritthöhe', key: 'stepHeight', min: 0, max: 100, step: 1, fmt: (v) => v.toFixed(0) + ' %' },
        ] },
        { title: 'EINZELBEINSTEUERUNG', id: 'leg', closed: true, items: [
          { t: 'legsel' },
          { t: 'sl', label: 'Hüftdrehung', min: -42, max: 42, step: 0.5, fmt: (v) => v.toFixed(1) + '°', get: () => legSel().angles.yaw / D, set: (v) => Lo.manualSet(S.selectedLeg, { yaw: v * D }) },
          { t: 'sl', label: 'Hüfthub', min: -55, max: 85, step: 0.5, fmt: (v) => v.toFixed(1) + '°', get: () => legSel().angles.pitch / D, set: (v) => Lo.manualSet(S.selectedLeg, { pitch: v * D }) },
          { t: 'sl', label: 'Kniebeugung', min: -165, max: -12, step: 0.5, fmt: (v) => v.toFixed(1) + '°', get: () => legSel().angles.knee / D, set: (v) => Lo.manualSet(S.selectedLeg, { knee: v * D }) },
          { t: 'sl', label: 'Knöchelwinkel', min: -80, max: 80, step: 0.5, fmt: (v) => v.toFixed(1) + '°', get: () => legSel().angles.ankle / D, set: (v) => Lo.manualSet(S.selectedLeg, { ankle: v * D }) },
          { t: 'btn', label: 'Fuß anheben', fn: () => Lo.liftLeg(S.selectedLeg) },
          { t: 'btn', label: 'Fuß aufsetzen', fn: () => Lo.plantLeg(S.selectedLeg) },
          { t: 'btn', label: 'Bein zurücksetzen', cls: 'full', fn: () => Lo.resetLeg(S.selectedLeg) },
        ] },
        { title: 'GREIFARM', id: 'robotic-arm', items: [
          { t: 'sl', label: 'Arm-Drehung', min: -100, max: 100, step: 1, fmt: (v) => v.toFixed(0) + '°', get: () => arm.target.yaw / D, set: (v) => armSet('yaw', v) },
          { t: 'sl', label: 'Schulterwinkel', min: -45, max: 115, step: 1, fmt: (v) => v.toFixed(0) + '°', get: () => arm.target.shoulder / D, set: (v) => armSet('shoulder', v) },
          { t: 'sl', label: 'Ellbogenwinkel', min: -165, max: 0, step: 1, fmt: (v) => v.toFixed(0) + '°', get: () => arm.target.elbow / D, set: (v) => armSet('elbow', v) },
          { t: 'sl', label: 'Handgelenk-Neigung', min: -125, max: 125, step: 1, fmt: (v) => v.toFixed(0) + '°', get: () => arm.target.wristPitch / D, set: (v) => armSet('wristPitch', v) },
          { t: 'sl', label: 'Handgelenk-Drehung', min: -180, max: 180, step: 1, fmt: (v) => v.toFixed(0) + '°', get: () => arm.target.wristRoll / D, set: (v) => armSet('wristRoll', v) },
          { t: 'btn', label: 'Greifer öffnen', fn: () => { if (arm.held) M.releasePayload(); arm.graspTarget = null; arm.target.gap = A.CFG.arm.maxGap; A.Log.add('Greifer öffnet'); } },
          { t: 'btn', label: 'Greifer schließen', fn: () => A.Main.manualClose() },
          { t: 'btn', label: '③ Greif-Demo', cls: 'warn', fn: () => M.grab() },
          { t: 'btn', label: 'Last loslassen', fn: () => M.release() },
          { t: 'btn', label: 'Träger räumen + stapeln', cls: 'full', fn: () => M.beams() },
          { t: 'btn', label: 'Regal: ein- und auslagern', cls: 'full', fn: () => M.rack() },
        ] },
        { title: 'SENSOREN', id: 'sensors', items: [
          { t: 'sl', label: 'Sensorkopf-Drehung', key: 'headYaw', min: -120, max: 120, step: 1, fmt: (v) => v.toFixed(0) + '°' },
          { t: 'sl', label: 'Sensorkopf-Neigung', key: 'headPitch', min: -30, max: 45, step: 1, fmt: (v) => v.toFixed(0) + '°' },
          { t: 'sl', label: 'Kameramast-Höhe', key: 'mastHeight', min: 0, max: 1.2, step: 0.01, fmt: (v) => v.toFixed(2) + ' m' },
          { t: 'tog', label: 'Lidar-Scan', key: 'lidar', fn: (v) => A.Log.add(v ? 'Lidar-Abtastung gestartet' : 'Lidar-Abtastung gestoppt') },
          { t: 'tog', label: 'Wärmebild', key: 'thermal', fn: (v) => A.Log.add(v ? 'Wärmebild aktiv' : 'Wärmebild aus') },
          { t: 'tog', label: 'Suchscheinwerfer', key: 'light', fn: (v) => A.Log.add(v ? 'Suchscheinwerfer an' : 'Suchscheinwerfer aus') },
          { t: 'btn', label: '', cls: 'hidden', fn: () => {} },
          { t: 'btn', label: 'Überlebenden-Scan', fn: () => M.scan() },
          { t: 'btn', label: 'Ziel markieren', fn: () => M.mark() },
        ] },
        { title: 'ERKUNDUNGSDROHNE', id: 'scout-drone', items: [
          { t: 'btn', label: '④ Drohne starten', fn: () => { A.DroneSys.launch(); } },
          { t: 'btn', label: 'Drohne zurückrufen', fn: () => A.DroneSys.recall() },
          { t: 'tog', label: 'Drohnenkamera', key: 'droneFeed' },
          { t: 'sl', label: 'Orbitradius', key: 'droneRadius', min: 2, max: 8, step: 0.1, fmt: (v) => v.toFixed(1) + ' m' },
        ] },
        { title: 'MISSIONEN', id: 'mission-presets', items: [
          { t: 'btn', label: 'Mission: Tunnelinspektion', cls: 'full', fn: () => M.run('tunnel') },
          { t: 'btn', label: 'Mission: Eingestürzte Lagerhalle', cls: 'full', fn: () => M.run('warehouse') },
          { t: 'btn', label: 'Mission: Überlebendensuche', cls: 'full', fn: () => M.run('survivor') },
          { t: 'btn', label: 'Mission: Trümmerräumung', cls: 'full', fn: () => M.run('debris') },
          { t: 'btn', label: '⑤ Mission: Systemfehler-Test', cls: 'full danger', fn: () => M.run('failure') },
          { t: 'btn', label: 'Aktive Aufgabe abbrechen', cls: 'full', fn: () => { A.Tasks.cancel('abort'); A.Log.add('Aufgabe vom Bediener abgebrochen', 'warn'); } },
        ] },
      ];
      this.build(schema);
      this.buildRight();
      this.bindViewport();
      S.on('log', (e) => this.addLog(e));
      S.on('logclear', () => { $('log').innerHTML = ''; });
      S.on('feedback', (f) => { const t = $('cmdtext'); t.textContent = f.text; t.className = f.level; });
      S.on('blocked', (r) => this.alert(`⛔ KOLLISION VERHINDERT — ${r.reason}`));
      S.on('oracle', (v) => this.alert(`ORAKEL: Durchdringung ${v.a} ↔ ${v.b}`));
      S.on('task', (c) => this.renderTimeline(c));
      S.on('power', () => this.syncPower());
    },

    // ------------------------------------------------------------------ left panel
    build(schema) {
      const root = $('controls');
      schema.forEach((sec, si) => {
        const s = el('section', 'sec' + (sec.closed ? ' closed' : '') + (sec.keep ? ' keep' : ''));
        s.id = 'sec-' + sec.id;
        const h = el('h4', '', `<span><span class="n">${String(si + 1).padStart(2, '0')}</span>${sec.title}</span><span>${sec.closed ? '+' : '−'}</span>`);
        h.onclick = () => { s.classList.toggle('closed'); h.lastChild.textContent = s.classList.contains('closed') ? '+' : '−'; };
        const b = el('div', 'body');
        s.append(h, b); root.appendChild(s);
        for (const it of sec.items) b.appendChild(this.item(it));
      });
    },
    item(it) {
      if (it.t === 'btn') {
        const b = el('button', 'btn ' + (it.cls || ''), it.label);
        b.onclick = () => {
          try { it.fn(); } catch (e) { console.error(e); }
          if (document.body.classList.contains('mobile')) this.drawer(null); // phone: show the effect in 3D
        };
        if (it.id) this.btns[it.id] = b;
        return b;
      }
      if (it.t === 'tog') {
        const t = el('div', 'tog', `<span>${it.label}</span><i></i>`);
        t.onclick = () => { S.toggles[it.key] = !S.toggles[it.key]; it.fn && it.fn(S.toggles[it.key]); this.syncToggles(); };
        this.toggles.push({ t, key: it.key });
        return t;
      }
      if (it.t === 'sl') {
        const w = el('div', 'sl'), lab = el('label', '', it.label), out = el('output'), inp = el('input');
        inp.type = 'range'; inp.min = it.min; inp.max = it.max; inp.step = it.step;
        const get = it.get || (() => S.cmd[it.key]);
        inp.value = get(); out.textContent = it.fmt(+inp.value);
        inp.oninput = () => {
          const v = +inp.value; out.textContent = it.fmt(v);
          if (it.set) it.set(v); else { S.cmd[it.key] = v; }
        };
        w.append(lab, out, inp);
        this.sliders.push({ inp, out, get, fmt: it.fmt });
        return w;
      }
      if (it.t === 'legsel') {
        const w = el('div', 'legsel');
        this.legBtns = A.CFG.legs.map((d) => {
          const b = el('button', 'btn', A.LN(d.id)); b.title = d.name; b.dataset.leg = d.id;
          b.onclick = () => { S.selectedLeg = d.id; this.syncLegSel(); };
          w.appendChild(b); return b;
        });
        this.syncLegSel();
        return w;
      }
      return el('div');
    },
    // phone layout: 'left' | 'right' | null
    drawer(which) {
      const b = document.body;
      b.classList.toggle('drawer-left', which === 'left');
      b.classList.toggle('drawer-right', which === 'right');
      document.querySelectorAll('#mobilenav button').forEach((x) => x.classList.toggle('on', x.dataset.drawer === (which || 'none')));
    },
    syncLegSel() { this.legBtns.forEach((b) => b.classList.toggle('on', b.dataset.leg === S.selectedLeg)); },
    syncToggles() { for (const { t, key } of this.toggles) t.classList.toggle('on', !!S.toggles[key]); },
    syncSliders() {
      for (const s of this.sliders) {
        if (document.activeElement === s.inp) continue;
        const v = s.get(); if (v === undefined || isNaN(v)) continue;
        s.inp.value = v; s.out.textContent = s.fmt(v);
      }
    },
    syncPower() { $('controls').classList.toggle('disabled-all', !S.powered); $('estop').classList.toggle('hidden', S.powered || S.booting); },

    // ------------------------------------------------------------------ right panel
    buildRight() {
      const rows = [['power', 'ENERGIE', '%'], ['temp', 'MOTORTEMPERATUR', '°C'], ['hyd', 'HYDRAULIKDRUCK', 'bar'], ['cpu', 'CPU-LAST', '%'], ['signal', 'SIGNALSTÄRKE', '%'], ['damage', 'SCHADEN', '%'], ['stab', 'STABILITÄTSRAND', 'cm']];
      const sys = $('sys'); sys.className = 'kv';
      this.sysEl = {};
      for (const [k, label] of rows) {
        const sp = el('span', '', label), bar = el('div', 'bar', '<i></i>'), b = el('b');
        sys.append(sp, bar, b); this.sysEl[k] = { bar, b };
      }
      sys.append(el('span', '', 'AKTUELLER MODUS'), el('div'), (this.modeEl = el('b')));
      const sens = $('sens'); sens.className = 'kv2'; this.sensEl = {};
      for (const [k, label] of [['lidar', 'LIDAR'], ['prob', 'ÜBERLEBENS-WAHRSCH.'], ['thermal', 'WÄRMEBILD'], ['dist', 'ZIELDISTANZ'], ['light', 'SCHEINWERFER'], ['dens', 'HINDERNISDICHTE'], ['drone', 'DROHNE'], ['inter', 'EINGRIFFE'], ['gait', 'GANGART'], ['checks', 'KOLL.-PRÜFUNGEN']]) {
        const b = el('b'); sens.append(el('span', '', label), b); this.sensEl[k] = b;
      }
      const legs = $('legs'); this.legEl = {};
      for (const d of A.CFG.legs) {
        const c = el('div', 'leg', `<span class="id">${A.LN(d.id)}</span><span class="s idle">Ruhe</span><div class="ang"></div><div class="bar"><i></i></div>`);
        legs.appendChild(c); this.legEl[d.id] = c;
        c.onclick = () => { S.selectedLeg = d.id; this.syncLegSel(); };
      }
      this.map = $('minimap').getContext('2d');
    },
    setBar(o, v, max, val, cls) { o.bar.querySelector('i').style.width = U.clamp((v / max) * 100, 0, 100) + '%'; o.bar.className = 'bar ' + (cls || ''); o.b.textContent = val; },
    addLog(e) {
      const box = $('log'), d = el('div', e.level, `<span class="t">${U.fmtClock(e.t)}</span>${e.msg}`);
      box.appendChild(d);
      while (box.childNodes.length > 160) box.removeChild(box.firstChild);
      box.scrollTop = box.scrollHeight;
      const tk = $('tickertext'); tk.textContent = e.msg; tk.className = e.level;
    },
    alert(text) { const a = $('alert'); a.textContent = text; a.className = ''; this.alertT = 2.4; },
    renderTimeline(c) {
      const box = $('tl-steps'); box.innerHTML = '';
      $('tl-name').textContent = c ? `AUFGABE · ${c.name}` : 'KEINE AKTIVE AUFGABE';
      if (!c) return;
      c.steps.forEach((s, i) => box.appendChild(el('span', i < c.step ? 'done' : i === c.step ? 'cur' : '', (i < c.step ? '✓ ' : '') + s)));
    },

    // ------------------------------------------------------------------ viewport widgets
    bindViewport() {
      const M = A.Missions, Lo = A.Loco;
      const demos = {
        walk: () => { if (Lo.mode === 'walk') Lo.stop('Laufzyklus gestoppt'); else { if (A.Tasks.running()) A.Tasks.cancel('demo'); if (Lo.setMode('walk')) A.Log.add('Laufzyklus gestartet — wechselnder Dreifußgang (VL·MR·HL / VR·ML·HR)'); } },
        scan: () => { Object.assign(S.toggles, { lidar: true, thermal: true }); S.toggles.cones = true; this.syncToggles(); A.Log.add('Lidar-Abtastung gestartet'); M.scan(); },
        grab: () => M.grab(),
        drone: () => { S.toggles.droneFeed = true; this.syncToggles(); if (A.DroneSys.state === 'docked') A.DroneSys.launch(); else A.DroneSys.recall(); },
        fail: () => { if (S.fault) M.stabilize(); else M.run('failure'); },
        climb: () => M.climb(),
        tunnel: () => M.run('tunnel'),
        beams: () => M.beams(),
        rack: () => M.rack(),
      };
      document.querySelectorAll('#demo-strip button').forEach((b) => (b.onclick = () => demos[b.dataset.demo]()));
      // narrow desktop: the demo strip wraps to two rows — view buttons and HUD readout move below it
      const strip = $('demo-strip'), fitStrip = () => $('viewport').style.setProperty('--strip-b', (strip.offsetTop + strip.offsetHeight + 8) + 'px');
      window.addEventListener('resize', fitStrip); fitStrip();
      document.querySelectorAll('#viewbtns button').forEach((b) => (b.onclick = () => A.Main.view(b.dataset.view)));
      document.querySelectorAll('#mobilenav button').forEach((b) => (b.onclick = () => {
        const d = b.dataset.drawer === 'none' ? null : b.dataset.drawer;
        this.drawer(document.body.classList.contains('drawer-' + d) ? null : d);
      }));
      $('drawer-shade').onclick = () => this.drawer(null);
      window.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT') return;
        const k = e.key.toLowerCase();
        if (k === 'w') demos.walk();
        else if (k === 's' || k === ' ') { Lo.stop('Stopp'); e.preventDefault(); }
        else if (k === 'a') { if (A.Tasks.running()) A.Tasks.cancel('key'); Lo.setMode('rotate', { dir: 1 }); }
        else if (k === 'd') { if (A.Tasks.running()) A.Tasks.cancel('key'); Lo.setMode('rotate', { dir: -1 }); }
        else if (k >= '1' && k <= '5') demos[['walk', 'scan', 'grab', 'drone', 'fail'][+k - 1]]();
      });
    },

    // ------------------------------------------------------------------ periodic update
    update(dt) {
      this.fpsAcc += dt; this.fpsN++;
      if (this.alertT > 0) { this.alertT -= dt; if (this.alertT <= 0) $('alert').className = 'hidden'; }
      this.acc += dt;
      if (this.acc < 0.1) return;
      this.acc = 0;
      const t = S.tele, R = A.robot, Lo = A.Loco;
      if (this.fpsAcc > 0.5) { $('tb-fps').textContent = Math.round(this.fpsN / this.fpsAcc); this.fpsAcc = 0; this.fpsN = 0; }
      $('tb-clock').textContent = U.fmtClock(A.Log.time());
      $('tb-mission').textContent = S.mission.name;
      const mode = $('tb-mode'); mode.textContent = MODE_DE[t.mode] || t.mode; mode.className = { FAULT: 'err', SHUTDOWN: 'err', BOOTING: 'warn', AUTONOMOUS: 'ok' }[t.mode] || '';
      $('tb-mode-chip').classList.toggle('alarm', t.mode === 'FAULT' || t.mode === 'SHUTDOWN');
      $('tb-guard').textContent = `AKTIV · ${S.stats.interventions} VERHINDERT`;
      const O = A.Oracle, ob = $('tb-oracle');
      ob.textContent = `${O.violations} VERSTÖSSE · ${(O.checks / 1000).toFixed(0)}k PROBEN`; ob.className = O.violations ? 'err' : 'ok';
      $('tb-oracle-chip').classList.toggle('alarm', O.violations > 0);
      // warn bar
      const wb = $('warnbar'), wt = $('warntext');
      let warn = null;
      if (!S.powered) warn = S.booting ? 'SYSTEMNEUSTART LÄUFT' : 'NOT-AUS — AKTUATOREN STROMLOS';
      else if (S.fault) warn = `AKTUATORFEHLER · BEIN ${A.LN(S.fault.leg)} · DREHMOMENTSPITZE · MOTOR ${t.temp.toFixed(0)} °C — AUTO-STABILISIERUNG VERFÜGBAR`;
      else if (Lo.haltedReason) warn = `FORTBEWEGUNG GESTOPPT — ${Lo.haltedReason}`;
      wb.classList.toggle('hidden', !warn); if (warn) wt.textContent = warn;
      if (Lo.mode !== 'idle') Lo.haltedReason = '';
      // buttons
      const sb = this.btns.stab; sb.disabled = !S.fault; sb.classList.toggle('hot', !!S.fault);
      this.btns.walk.classList.toggle('on', Lo.mode === 'walk');
      document.querySelector('#demo-strip [data-demo="walk"]').classList.toggle('on', Lo.mode === 'walk');
      document.querySelector('#demo-strip [data-demo="fail"]').textContent = S.fault ? '⑤ STABILISIEREN' : '⑤ FEHLERTEST';
      document.querySelector('#demo-strip [data-demo="drone"]').textContent = A.DroneSys.state === 'docked' ? '④ DROHNE STARTEN' : '④ DROHNE ZURÜCK';
      this.syncSliders(); this.syncToggles();
      // system status
      const E = this.sysEl;
      this.setBar(E.power, t.power, 100, t.power.toFixed(1) + ' %', t.power < 20 ? 'err' : t.power < 40 ? 'warn' : 'ok');
      this.setBar(E.temp, t.temp, 110, t.temp.toFixed(1) + ' °C', t.temp > 85 ? 'err' : t.temp > 65 ? 'warn' : '');
      this.setBar(E.hyd, t.hyd, 220, t.hyd.toFixed(0) + ' bar', t.hyd < 150 ? 'warn' : '');
      this.setBar(E.cpu, t.cpu, 100, t.cpu.toFixed(0) + ' %', t.cpu > 85 ? 'warn' : '');
      this.setBar(E.signal, t.signal, 100, t.signal.toFixed(0) + ' %', t.signal < 40 ? 'warn' : 'ok');
      this.setBar(E.damage, t.damage, 100, t.damage.toFixed(1) + ' %', t.damage > 15 ? 'err' : t.damage > 3 ? 'warn' : 'ok');
      const m = Math.max(0, Lo.margin * 100);
      this.setBar(E.stab, m, 60, (Lo.margin * 100).toFixed(0) + ' cm', m < 8 ? 'err' : m < 18 ? 'warn' : 'ok');
      this.modeEl.textContent = MODE_DE[t.mode] || t.mode; this.modeEl.className = t.mode === 'FAULT' || t.mode === 'SHUTDOWN' ? 'st-err' : 'st-on';
      // sensors
      const se = this.sensEl, onoff = (b, v) => { b.textContent = v ? 'AKTIV' : 'INAKTIV'; b.className = v ? 'st-on' : 'st-off'; };
      onoff(se.lidar, S.toggles.lidar && S.powered); onoff(se.thermal, S.toggles.thermal && S.powered); onoff(se.light, S.toggles.light && S.powered);
      se.prob.textContent = S.scan.prob ? (S.scan.prob * 100).toFixed(0) + ' %' : '—'; se.prob.className = S.scan.prob > 0.5 ? 'st-warn' : '';
      se.dist.textContent = S.scan.distance ? S.scan.distance.toFixed(2) + ' m' : '—';
      se.dens.textContent = (A.Sensors.density * 100).toFixed(0) + ' %';
      se.drone.textContent = DRONE_DE[A.DroneSys.state] || A.DroneSys.state; se.drone.className = A.DroneSys.state === 'docked' ? '' : 'st-on';
      se.inter.textContent = S.stats.interventions; se.inter.className = S.stats.interventions ? 'st-warn' : '';
      se.gait.textContent = Lo.gaitActive ? 'DREIFUSS' : 'STAND';
      se.checks.textContent = (S.stats.checks / 1000).toFixed(1) + 'k';
      const sf = $('scanfill'), st = $('scantext');
      sf.style.width = S.scan.progress + '%'; st.textContent = S.scan.active ? `ÜBERLEBENDEN-SCAN ${S.scan.progress.toFixed(0)} %` : S.scan.result === '—' ? 'ÜBERLEBENDEN-SCAN BEREIT' : S.scan.result;
      // legs
      for (const l of R.legs) {
        const c = this.legEl[l.id], s = c.querySelector('.s');
        s.textContent = LEG_ST_DE[l.status] || l.status; s.className = 's ' + l.status;
        c.classList.toggle('error', l.status === 'error');
        c.classList.toggle('sel', l.id === S.selectedLeg);
        c.querySelector('.ang').textContent = `D${(l.angles.yaw / D).toFixed(0)}° H${(l.angles.pitch / D).toFixed(0)}° K${(l.angles.knee / D).toFixed(0)}°`;
        const bi = c.querySelector('.bar'); bi.className = 'bar ' + (l.load > 0.45 ? 'err' : 'ok'); bi.querySelector('i').style.width = Math.min(100, l.load * 200) + '%';
      }
      // drone cam overlay
      const dc = $('dronecam'), dr = A.DroneSys;
      dc.classList.toggle('hidden', !(S.toggles.droneFeed && dr.state !== 'docked'));
      $('dc-state').textContent = DRONE_DE[dr.state] || dr.state;
      $('dc-foot').textContent = `HÖHE ${dr.pos.y.toFixed(1)} m · R ${S.cmd.droneRadius.toFixed(1)} m · FUNK ${t.signal.toFixed(0)}%`;
      // HUD readout
      const p = R.pose;
      $('hud-readout').innerHTML =
        `POS <b>${p.x.toFixed(2)} / ${p.z.toFixed(2)}</b> m &nbsp; KURS <b>${(U.wrap(p.yaw) / D).toFixed(0)}°</b><br>` +
        `KÖRPER H <b>${p.h.toFixed(2)}</b> m &nbsp; BODEN <b>${p.gy.toFixed(2)}</b> m<br>` +
        `NICKEN <b>${(p.pitch / D).toFixed(1)}°</b> &nbsp; ROLLEN <b>${(p.roll / D).toFixed(1)}°</b><br>` +
        `GANG <b>${LOCO_DE[Lo.mode] || Lo.mode}</b> &nbsp; V <b>${Lo.v.toFixed(2)}</b> m/s<br>` +
        `STÜTZE <b class="${Lo.margin < 0.08 ? 'r' : ''}">${R.legs.filter((l) => l.mode === 'stance').length}/6 · ${(Lo.margin * 100).toFixed(0)} cm</b>` +
        (S.stats.lastBlock ? `<br><span class="w">LETZTE SPERRE: ${S.stats.lastBlock}</span>` : '');
      this.drawMap();
      if (A.Tasks.cur) $('tl-time').textContent = 'T+' + A.Tasks.cur.t.toFixed(1) + ' s'; else $('tl-time').textContent = '';
    },

    // ------------------------------------------------------------------ minimap
    drawMap() {
      const g = this.map, W = g.canvas.width, H = g.canvas.height, R = A.robot;
      const sc = 7.4, cx = W / 2 - 1.5 * sc, cz = H / 2 - 1.2 * sc; // z −14 … +16.7 m (tunnel end to yard)
      const X = (x) => cx + x * sc, Z = (z) => cz + z * sc;
      g.fillStyle = '#070a0c'; g.fillRect(0, 0, W, H);
      g.strokeStyle = '#111a20'; g.lineWidth = 1;
      for (let i = -20; i <= 20; i += 2) { g.beginPath(); g.moveTo(X(i), 0); g.lineTo(X(i), H); g.stroke(); g.beginPath(); g.moveTo(0, Z(i)); g.lineTo(W, Z(i)); g.stroke(); }
      // zone highlight
      const zid = S.mission.zone && A.Env.zones[S.mission.zone];
      if (zid && zid.mesh.visible) { const z = zid.mesh.position, s = zid.def.s; g.strokeStyle = '#' + new THREE.Color(zid.def.color).getHexString(); g.setLineDash([4, 3]); g.strokeRect(X(z.x - s[0] / 2), Z(z.z - s[1] / 2), s[0] * sc, s[1] * sc); g.setLineDash([]); }
      // lidar points (sample)
      if (S.toggles.lidar) { g.fillStyle = 'rgba(41,211,255,0.55)'; const P = A.Sensors; for (let i = 0; i < P.maxPts; i += 3) if (P.age[i] < 2.5 && P.pos[i * 3 + 1] > 0.05) g.fillRect(X(P.pos[i * 3]) - 0.5, Z(P.pos[i * 3 + 2]) - 0.5, 1.2, 1.2); }
      // obstacles
      for (const o of A.World.obstacles) {
        if (o.held) continue;
        const c = o.obb.c, hx = o.obb.h.x, hz = o.obb.h.z, yaw = o.yaw();
        let col = o.ramp ? '#2a3b2a' : o.top < 0.35 ? '#2b3540' : '#4a5864';
        if (o.kind === 'debris') col = '#f2b705';
        if (o.kind === 'survivor') { if (!A.Env.survivor.detected) continue; col = '#ff5a1f'; }
        if (o.bottom > 0.9) col = 'rgba(80,100,120,0.35)';
        g.save(); g.translate(X(c.x), Z(c.z)); g.rotate(-yaw); g.fillStyle = col; g.fillRect(-hx * sc, -hz * sc, hx * 2 * sc, hz * 2 * sc); g.restore();
      }
      // planned path
      const nav = A.Loco.nav;
      if (nav && nav.path) { g.strokeStyle = '#29d3ff'; g.setLineDash([3, 3]); g.beginPath(); nav.path.forEach((p, i) => (i ? g.lineTo(X(p.x), Z(p.z)) : g.moveTo(X(p.x), Z(p.z)))); g.stroke(); g.setLineDash([]); }
      // marker
      if (S.marker) { const t = performance.now() / 300; g.strokeStyle = '#ff3b3b'; g.lineWidth = 2; g.beginPath(); g.arc(X(S.marker.x), Z(S.marker.z), 4 + 2 * Math.sin(t), 0, 7); g.stroke(); g.lineWidth = 1; }
      // robot
      const p = R.pose;
      g.strokeStyle = 'rgba(242,183,5,0.25)'; g.beginPath(); g.arc(X(p.x), Z(p.z), A.Planner.last ? A.Planner.last.R * sc : 1.9 * sc, 0, 7); g.stroke();
      for (const l of R.legs) { g.fillStyle = l.mode === 'stance' ? '#22ff88' : '#29d3ff'; g.fillRect(X(l.foot.x) - 1.5, Z(l.foot.z) - 1.5, 3, 3); }
      g.save(); g.translate(X(p.x), Z(p.z)); g.rotate(-p.yaw);
      g.fillStyle = '#f2b705'; g.fillRect(-0.99 * sc, -0.41 * sc, 1.74 * sc, 0.82 * sc);
      g.fillStyle = '#000'; g.beginPath(); g.moveTo(0.7 * sc, 0); g.lineTo(0.1 * sc, -0.28 * sc); g.lineTo(0.1 * sc, 0.28 * sc); g.fill();
      g.restore();
      // drone
      const dr = A.DroneSys;
      if (dr.state !== 'docked') { g.fillStyle = '#29d3ff'; g.beginPath(); g.arc(X(dr.pos.x), Z(dr.pos.z), 3, 0, 7); g.fill(); g.strokeStyle = 'rgba(41,211,255,0.3)'; g.beginPath(); g.moveTo(X(p.x), Z(p.z)); g.lineTo(X(dr.pos.x), Z(dr.pos.z)); g.stroke(); }
      g.fillStyle = '#4f606c'; g.font = '9px JetBrains Mono, monospace'; g.fillText('TUNNEL', X(-0.9), Z(-12.9)); g.fillText('HALLE', X(7.6), Z(-5.2)); g.fillText('TRÜMMER', X(-9), Z(-2.6)); g.fillText('KLETTERN', X(-1.2), Z(11.2)); g.fillText('HOF', X(9.2), Z(8.6));
    },
  });
})();
