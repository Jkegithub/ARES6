/* ARES-6 — core: namespace, configuration, math helpers, state manager, log, task runner */
(function () {
  'use strict';
  const A = (window.ARES = {});
  const D = Math.PI / 180;

  // phone/tablet layout: narrow screen, or touch screen up to tablet width (perf profile is fixed at load)
  const isMobile = () => window.innerWidth <= 1100 || (matchMedia('(pointer: coarse)').matches && window.innerWidth <= 1400);
  A.MOBILE = isMobile();
  const markMobile = () => document.body && document.body.classList.toggle('mobile', isMobile());
  markMobile(); window.addEventListener('resize', markMobile);
  A.DEG = D;

  // ------------------------------------------------------------------ configuration
  A.CFG = {
    body: { L: 1.5, W: 0.82, H: 0.3 },
    // collision hull: chassis box + narrower rear battery box (body-local centre / half extents)
    hull: { hy: 0.16, parts: [{ c: [0, 0, 0], h: [0.765, 0.16, 0.42] }, { c: [-0.86, -0.01, 0], h: [0.14, 0.13, 0.31] }] },
    leg: {
      coxa: 0.16, upper: 0.72, lower: 0.86,
      ankleH: 0.1, padH: 0.03,          // ankle joint height above pad contact
      rUpper: 0.12, rLower: 0.075, rPad: 0.016,
      stepUp: 0.38,                      // max height a foot may climb per step
    },
    limits: { hipYaw: [-42 * D, 42 * D], hipPitch: [-55 * D, 85 * D], knee: [-165 * D, -12 * D], ankle: [-80 * D, 80 * D] },
    legs: [
      { id: 'FL', name: 'Vorne links', mount: [0.55, -0.02, -0.47], yaw: 55 * D, yawLim: [-26, 42], group: 0, front: 1, side: -1 },
      { id: 'ML', name: 'Mitte links', mount: [0.0, -0.02, -0.48], yaw: 90 * D, yawLim: [-40, 40], group: 1, front: 0, side: -1 },
      { id: 'RL', name: 'Hinten links', mount: [-0.55, -0.02, -0.47], yaw: 125 * D, yawLim: [-42, 26], group: 0, front: -1, side: -1 },
      { id: 'FR', name: 'Vorne rechts', mount: [0.55, -0.02, 0.47], yaw: -55 * D, yawLim: [-42, 26], group: 1, front: 1, side: 1 },
      { id: 'MR', name: 'Mitte rechts', mount: [0.0, -0.02, 0.48], yaw: -90 * D, yawLim: [-40, 40], group: 0, front: 0, side: 1 },
      { id: 'RR', name: 'Hinten rechts', mount: [-0.55, -0.02, 0.47], yaw: -125 * D, yawLim: [-26, 42], group: 1, front: -1, side: 1 },
    ],
    adjacent: [['FL', 'ML'], ['ML', 'RL'], ['FR', 'MR'], ['MR', 'RR']],
    arm: { mount: [0.92, -0.04, 0], L1: 0.62, L2: 0.55, palm: 0.07, finger: 0.18, maxGap: 0.42 },
    head: { pos: [0.48, 0.15, 0], r: 0.21 },
    mast: { pos: [0.05, 0.15, -0.26], seg: 0.25, max: 1.2 },
    dock: { pos: [-0.42, 0.15, 0.12] },
    height: { min: 0.25, max: 1.1, def: 0.6 },
    poses: {
      zero: { h: 0.6, pitch: 0, roll: 0, stance: 1.0 },
      crawl: { h: 0.28, pitch: 0, roll: 0, stance: 1.0 },
      high: { h: 1.0, pitch: 0, roll: 0, stance: 0.95 },
      rescue: { h: 0.5, pitch: 0, roll: 0, stance: 1.1 },
    },
  };

  // German display names of the legs (internal ids stay FL…RR)
  const LEG_DE = { FL: 'VL', ML: 'ML', RL: 'HL', FR: 'VR', MR: 'MR', RR: 'HR' };
  A.LN = (id) => LEG_DE[id] || id;

  // ------------------------------------------------------------------ math helpers
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const U = (A.U = {
    clamp,
    lerp: (a, b, t) => a + (b - a) * t,
    damp: (a, b, rate, dt) => a + (b - a) * (1 - Math.exp(-rate * dt)),
    approach: (a, b, step) => (Math.abs(b - a) <= step ? b : a + Math.sign(b - a) * step),
    wrap: (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; },
    smooth: (t) => t * t * (3 - 2 * t),
    rand: (a, b) => a + Math.random() * (b - a),
    // closest distance between segments p1-q1 and p2-q2 (Ericson, RTCD 5.1.9)
    segSeg(p1, q1, p2, q2) {
      const d1x = q1.x - p1.x, d1y = q1.y - p1.y, d1z = q1.z - p1.z;
      const d2x = q2.x - p2.x, d2y = q2.y - p2.y, d2z = q2.z - p2.z;
      const rx = p1.x - p2.x, ry = p1.y - p2.y, rz = p1.z - p2.z;
      const a = d1x * d1x + d1y * d1y + d1z * d1z, e = d2x * d2x + d2y * d2y + d2z * d2z;
      const f = d2x * rx + d2y * ry + d2z * rz;
      let s, t;
      if (a < 1e-9 && e < 1e-9) { s = 0; t = 0; }
      else if (a < 1e-9) { s = 0; t = clamp(f / e, 0, 1); }
      else {
        const c = d1x * rx + d1y * ry + d1z * rz;
        if (e < 1e-9) { t = 0; s = clamp(-c / a, 0, 1); }
        else {
          const b = d1x * d2x + d1y * d2y + d1z * d2z, den = a * e - b * b;
          s = den > 1e-9 ? clamp((b * f - c * e) / den, 0, 1) : 0;
          t = (b * s + f) / e;
          if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); } else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
        }
      }
      const x = p1.x + d1x * s - (p2.x + d2x * t), y = p1.y + d1y * s - (p2.y + d2y * t), z = p1.z + d1z * s - (p2.z + d2z * t);
      return Math.sqrt(x * x + y * y + z * z);
    },
    fmtClock(sec) {
      const m = Math.floor(sec / 60), s = Math.floor(sec % 60), ms = Math.floor((sec % 1) * 10);
      return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}.${ms}`;
    },
  });

  // ------------------------------------------------------------------ state manager + event bus
  const DEFAULT_CMD = () => ({
    bodyHeight: A.CFG.height.def, bodyYaw: 0, bodyPitch: 0, bodyRoll: 0, stance: 1.0,
    walkSpeed: 55, stepHeight: 50,
    headYaw: 0, headPitch: 0, mastHeight: 0,
    armYaw: 0, armShoulder: 80, armElbow: -150, armWristPitch: 60, armWrist: 0,
    droneRadius: 4.5,
  });
  const DEFAULT_TOGGLES = () => ({
    autonomous: false, debug: false, wireframe: false, cones: false,
    lidar: false, thermal: false, light: false, droneFeed: true, follow: true,
  });

  A.State = {
    cmd: DEFAULT_CMD(),
    toggles: DEFAULT_TOGGLES(),
    powered: true,
    booting: false,
    fault: null,                 // { leg, severity, t }
    mission: { id: null, name: 'BEREITSCHAFT', zone: null },
    scan: { active: false, progress: 0, prob: 0, distance: null, result: '—', located: false },
    marker: null,
    selectedLeg: 'FL',
    stats: { interventions: 0, lastBlock: '', checks: 0 },
    tele: {},
    _ls: {},
    on(evt, fn) { (this._ls[evt] = this._ls[evt] || []).push(fn); },
    emit(evt, data) { (this._ls[evt] || []).forEach((f) => f(data)); },
    reset() {
      this.cmd = DEFAULT_CMD(); this.toggles = DEFAULT_TOGGLES();
      this.powered = true; this.booting = false; this.fault = null;
      this.mission = { id: null, name: 'BEREITSCHAFT', zone: null };
      this.scan = { active: false, progress: 0, prob: 0, distance: null, result: '—', located: false };
      this.marker = null; this.stats.interventions = 0; this.stats.lastBlock = '';
      this.emit('reset');
    },
  };

  // ------------------------------------------------------------------ mission log
  A.Log = {
    entries: [], t0: performance.now(), _once: {},
    time() { return (performance.now() - this.t0) / 1000; },
    add(msg, level = 'info') {
      const e = { t: this.time(), msg, level };
      this.entries.push(e);
      if (this.entries.length > 300) this.entries.shift();
      A.State.emit('log', e);
      return e;
    },
    // rate-limited message (same key at most every `cool` seconds)
    once(key, msg, level = 'warn', cool = 3) {
      const now = this.time();
      if (this._once[key] && now - this._once[key] < cool) return null;
      this._once[key] = now;
      return this.add(msg, level);
    },
    clear() { this.entries = []; this._once = {}; this.t0 = performance.now(); A.State.emit('logclear'); },
  };

  A.feedback = (text, level = 'info') => A.State.emit('feedback', { text, level });

  // ------------------------------------------------------------------ task runner (generator coroutines)
  // A task is a generator. It may yield:
  //   undefined  -> resume next frame
  //   number     -> wait that many seconds
  //   function   -> wait until it returns truthy
  A.Tasks = {
    cur: null,
    start(name, gen, opts = {}) {
      this.cancel('superseded');
      this.cur = { name, it: gen, wait: null, steps: opts.steps || [], step: -1, onCancel: opts.onCancel, t: 0 };
      A.State.emit('task', this.cur);
    },
    step(label) {
      if (!this.cur) return;
      const i = this.cur.steps.indexOf(label);
      this.cur.step = i >= 0 ? i : this.cur.step;
      A.State.emit('task', this.cur);
      A.feedback(`${this.cur.name} ▸ ${label}`);
    },
    cancel(reason) {
      const c = this.cur;
      if (!c) return;
      this.cur = null;
      try { c.onCancel && c.onCancel(reason); } catch (e) { console.error(e); }
      try { c.it.return(); } catch (e) { /* ignore */ }
      A.State.emit('task', null);
    },
    running() { return !!this.cur; },
    update(dt) {
      const c = this.cur;
      if (!c) return;
      c.t += dt;
      if (c.wait !== null) {
        if (typeof c.wait === 'number') { c.wait -= dt; if (c.wait > 0) return; }
        else if (typeof c.wait === 'function') { if (!c.wait(dt)) return; }
        c.wait = null;
      }
      let r;
      try { r = c.it.next(dt); } catch (e) { console.error(e); A.Log.add(`Aufgabenfehler: ${e.message}`, 'err'); this.cur = null; A.State.emit('task', null); return; }
      if (this.cur !== c) return; // task replaced itself
      if (r.done) { this.cur = null; A.State.emit('task', null); return; }
      c.wait = r.value === undefined ? null : r.value;
      if (c.wait === null) c.wait = 0; // resume next frame
    },
  };
  // helper: wait until predicate or timeout; returns true if predicate became true
  A.Tasks.until = function* (pred, timeout = 10) {
    let t = 0, ok = false;
    yield (dt) => { t += dt; ok = !!pred(); return ok || t >= timeout; };
    return ok;
  };
})();
