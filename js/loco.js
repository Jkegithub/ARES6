/* ARES-6 — locomotion: tripod gait with planted feet, validated foothold selection, terrain adaptation,
   stability margin, path following and manual single-leg control */
(function () {
  'use strict';
  const A = window.ARES, C = A.CFG, U = A.U, W = A.World, LC = C.leg, D = A.DEG;
  const V = THREE.Vector3;

  const Loco = (A.Loco = {
    mode: 'idle', v: 0, w: 0, phase: 0, gaitActive: false, blockedT: 0, stallT: 0,
    nav: null, turnTarget: null, rotDir: 1, margin: 0, terrain: { pitch: 0, roll: 0 },
    lastReason: '', haltedReason: '',

    init(R) { this.R = R; },
    reset() {
      Object.assign(this, { mode: 'idle', v: 0, w: 0, phase: 0, gaitActive: false, blockedT: 0, stallT: 0, nav: null, turnTarget: null, haltedReason: '' });
      this.terrain = { pitch: 0, roll: 0 };
    },
    params() {
      const s = U.clamp(A.State.cmd.walkSpeed / 100, 0, 1);
      // turn rate is bounded by the hip-yaw sweep of the corner legs during one stance phase (~±9°)
      return { s, vmax: 0.42 * s, wmax: 0.1 + 0.26 * s, freq: 0.45 + 0.6 * s, H: 0.06 + 0.34 * (A.State.cmd.stepHeight / 100) };
    },

    // ------------------------------------------------------------------ high-level commands
    setMode(mode, opts = {}) {
      if (!A.State.powered) { A.feedback('Roboter abgeschaltet — zuerst neu starten', 'warn'); return false; }
      this.mode = mode; this.blockedT = 0; this.haltedReason = '';
      if (mode !== 'nav') this.nav = null;
      if (mode === 'rotate') this.rotDir = opts.dir || 1;
      if (mode === 'turnTo') this.turnTarget = opts.yaw;
      if (mode !== 'idle') for (const l of this.R.legs) if (l.mode === 'manual') this.plantLeg(l.id, true);
      A.State.emit('loco', mode);
      return true;
    },
    stop(msg) { this.mode = 'idle'; this.nav = null; this.turnTarget = null; if (msg) A.Log.add(msg); A.State.emit('loco', 'idle'); },
    navigate(path, finalYaw = null) { this.nav = { path, i: 1, finalYaw, status: 'active', replans: 0, goal: path[path.length - 1] }; this.setMode('nav'); this.nav.status = 'active'; },

    command() {
      const P = this.params(), p = this.R.pose;
      let v = 0, w = 0;
      if (!A.State.powered) return { v: 0, w: 0 };
      switch (this.mode) {
        case 'walk': v = P.vmax; break;
        case 'crawl': v = P.vmax * 0.55; break;
        case 'rotate': w = P.wmax * this.rotDir; break;
        case 'turnTo': {
          const e = U.wrap(this.turnTarget - p.yaw);
          if (Math.abs(e) < 1.2 * D) { this.mode = this.nav && this.nav.status === 'turning' ? 'idle' : 'idle'; if (this.nav && this.nav.status === 'turning') this.nav.status = 'done'; A.State.emit('loco', 'idle'); break; }
          w = U.clamp(e * 1.4, -P.wmax, P.wmax); if (Math.abs(w) < 0.08) w = Math.sign(e) * 0.08;
          break;
        }
        case 'nav': {
          const n = this.nav; if (!n) { this.mode = 'idle'; break; }
          const path = n.path, goal = path[path.length - 1];
          while (n.i < path.length - 1 && Math.hypot(path[n.i].x - p.x, path[n.i].z - p.z) < 0.5) n.i++;
          const tgt = path[n.i], dg = Math.hypot(goal.x - p.x, goal.z - p.z);
          if (dg < 0.22) {
            if (n.finalYaw !== null) { n.status = 'turning'; this.turnTarget = n.finalYaw; this.mode = 'turnTo'; }
            else { n.status = 'done'; this.mode = 'idle'; }
            A.State.emit('loco', this.mode); break;
          }
          const hd = Math.atan2(-(tgt.z - p.z), tgt.x - p.x), e = U.wrap(hd - p.yaw);
          if (Math.abs(e) > 28 * D) { w = Math.sign(e) * P.wmax; v = 0; }
          else { v = P.vmax * U.clamp(1 - Math.abs(e) / 0.7, 0.35, 1) * U.clamp(dg / 0.9, 0.3, 1); w = U.clamp(e * 1.6, -P.wmax, P.wmax); }
          break;
        }
      }
      if (A.State.fault && A.State.fault.severity > 0.1) { v *= 0.6; w *= 0.7; }
      return { v, w };
    },

    // ------------------------------------------------------------------ prediction + foothold search
    predict(pose, v, w, t) {
      const yaw = pose.yaw + w * t * 0.5;
      return { ...pose, x: pose.x + Math.cos(yaw) * v * t, z: pose.z - Math.sin(yaw) * v * t, yaw: pose.yaw + w * t };
    },
    arcPoint(sw, s, out) {
      const hs = U.smooth(U.clamp((s - 0.12) / 0.76, 0, 1));
      out.x = U.lerp(sw.from.x, sw.to.x, hs); out.z = U.lerp(sw.from.z, sw.to.z, hs);
      out.y = U.lerp(sw.from.y, sw.to.y, U.smooth(s)) + sw.H * Math.sin(Math.PI * s);
      return out;
    },
    arcClear(from, to, H0) {
      const p = new V(), q = new V(), sw = { from, to, H: H0 };
      for (const H of [H0, H0 + 0.12, H0 + 0.24, H0 + 0.36, H0 * 0.5, 0.04]) {
        sw.H = H; let ok = true;
        for (let i = 1; i < 12 && ok; i++) {
          this.arcPoint(sw, i / 12, p);
          q.set(p.x, p.y + 0.03, p.z); if (W.sphereHit(q, 0.1)) ok = false;
          q.set(p.x, p.y + LC.ankleH + LC.padH, p.z); if (ok && W.sphereHit(q, LC.rLower)) ok = false;
        }
        if (ok) return H;
      }
      return null;
    },
    // pick a validated foothold near the neutral position of the predicted pose
    planStep(l, pp, H0) {
      const R = this.R, n = R.neutralWorld(l, pp);
      const cands = [{ x: n.x, z: n.z, d: 0 }];
      for (const r of [0.07, 0.14, 0.21, 0.28, 0.36]) for (let k = 0; k < 10; k++) {
        const a = (k / 10) * Math.PI * 2 + r * 7;
        cands.push({ x: n.x + Math.cos(a) * r, z: n.z + Math.sin(a) * r, d: r });
      }
      const snap = R.snapshot(), from = l.foot.clone();
      let result = null;
      for (const c of cands) {
        const contact = R.contactAt(c.x, c.z, from.y);
        if (contact.y - from.y > LC.stepUp) continue;
        // footing: pad must sit on a surface without touching anything else
        let blocked = false;
        for (const [dx, dz] of [[0, 0], [0.1, 0], [-0.1, 0], [0, 0.1], [0, -0.1]]) if (W.sphereHit(new V(contact.x + dx, contact.y + 0.02, contact.z + dz), LC.rPad)) { blocked = true; break; }
        if (blocked) continue;
        // never step onto loose debris
        if (W.near(contact, 0.3).some((o) => o.kind === 'debris' && o.footprint(contact.x, contact.z, 0.14))) continue;
        // leg must reach + be collision free at the predicted touchdown pose
        l.foot.copy(contact);
        R.applyPose({ ...pp });
        const ok = R.singleLegCheck(l).ok;
        R.restore(snap);
        if (!ok) continue;
        let H = this.arcClear(from, contact, H0);
        if (H === null) continue;
        H = this.arcValid(l, from, contact, H, snap.pose, pp);
        if (H === null) continue;
        result = { to: contact, H }; break;
      }
      l.foot.copy(from);
      return result;
    },
    // full leg check (same as the validator) along the swing arc while the body moves pose0 → pp
    arcValid(l, from, to, H0, pose0, pp) {
      const R = this.R, snap = R.snapshot(), sw = { from, to, H: H0 }, p = new V();
      let found = null;
      // higher arcs clear obstacles, lower arcs keep the knees under low ceilings
      const Hs = [H0, H0 + 0.1, H0 + 0.2, H0 * 0.6, H0 * 0.35, 0.04].filter((h, i, a) => h >= 0.04 && a.indexOf(h) === i);
      for (const H of Hs) {
        if (found !== null) break;
        sw.H = H; let ok = true;
        for (let i = 1; i < 12 && ok; i++) {
          const t = i / 12;
          const pose = { ...pose0, x: U.lerp(pose0.x, pp.x, t), z: U.lerp(pose0.z, pp.z, t), yaw: pose0.yaw + U.wrap(pp.yaw - pose0.yaw) * t };
          l.foot.copy(this.arcPoint(sw, t, p));
          R.applyPose(pose);
          if (!R.singleLegCheck(l).ok) ok = false;
        }
        R.restore(snap);
        if (ok) found = H;
      }
      l.foot.copy(from);
      return found;
    },
    startSwing(l, to, H, dur) { l.swing = { from: l.foot.clone(), to: to.clone(), H, t: 0, dur }; l.mode = 'swing'; },

    // ------------------------------------------------------------------ stability
    supportMargin() {
      const R = this.R, pts = R.legs.filter((l) => l.mode === 'stance').map((l) => ({ x: l.foot.x, z: l.foot.z }));
      if (pts.length < 3) return -1;
      // convex hull (monotone chain)
      pts.sort((a, b) => a.x - b.x || a.z - b.z);
      const cr = (o, a, b) => (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
      const lo = [], up = [];
      for (const p of pts) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
      for (let i = pts.length - 1; i >= 0; i--) { const p = pts[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
      const hull = lo.slice(0, -1).concat(up.slice(0, -1));
      this.hull = hull;
      const com = R.body.localToWorld(new V(-0.06, 0, 0));
      this.com = com;
      let m = Infinity;
      for (let i = 0; i < hull.length; i++) {
        const a = hull[i], b = hull[(i + 1) % hull.length];
        const ex = b.x - a.x, ez = b.z - a.z, len = Math.hypot(ex, ez) || 1;
        m = Math.min(m, ((com.x - a.x) * ez - (com.z - a.z) * ex) / len * -1);
      }
      return m;
    },
    // least-squares support plane through the feet (body-local) → terrain pitch/roll
    terrainFit() {
      const R = this.R, P = R.pose;
      let n = 0, sx = 0, sz = 0, sy = 0, sxx = 0, szz = 0, sxz = 0, sxy = 0, szy = 0;
      for (const l of R.legs) {
        const f = l.mode === 'swing' ? l.swing.to : l.foot;
        const q = R.toLocalXZ(P, f.x, f.z);
        n++; sx += q.x; sz += q.z; sy += f.y; sxx += q.x * q.x; szz += q.z * q.z; sxz += q.x * q.z; sxy += q.x * f.y; szy += q.z * f.y;
      }
      const M = [[n, sx, sz], [sx, sxx, sxz], [sz, sxz, szz]], b = [sy, sxy, szy];
      const det = (m) => m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      const d0 = det(M); if (Math.abs(d0) < 1e-9) return { y: sy / n, pitch: 0, roll: 0 };
      const rep = (k) => M.map((row, i) => row.map((v, j) => (j === k ? b[i] : v)));
      const a0 = det(rep(0)) / d0, a1 = det(rep(1)) / d0, a2 = det(rep(2)) / d0;
      return { y: a0, pitch: Math.atan(a1), roll: Math.atan(-a2) };
    },

    // ------------------------------------------------------------------ main update
    update(dt) {
      const R = this.R, S = A.State, cmd = S.cmd, P = this.params();
      const c = this.command();
      const acc = 0.9 * dt;
      this.v = U.approach(this.v, c.v, acc); this.w = U.approach(this.w, c.w, acc * 1.6);
      const moving = Math.abs(this.v) > 0.004 || Math.abs(this.w) > 0.01;

      // settle: legs far from their neutral spot re-step when standing
      let needSettle = false;
      if (!moving && S.powered) for (const l of R.legs) {
        if (l.mode !== 'stance' || l.settleFail) continue;
        const n = R.neutralWorld(l, R.pose);
        if (Math.hypot(l.foot.x - n.x, l.foot.z - n.z) > 0.1) { needSettle = true; break; }
      }
      if (moving) for (const l of R.legs) l.settleFail = false;
      const swinging = R.legs.some((l) => l.mode === 'swing');
      const gaitOn = (moving || needSettle) && S.powered;
      if (gaitOn && !this.gaitActive) this.phase = 0.999;
      this.gaitActive = gaitOn || swinging;

      // ---- gait phase: swing starts at 0 (group 0) and 0.5 (group 1)
      const freq = moving ? P.freq : 0.9;
      const swingDur = 0.44 / freq;
      if (gaitOn && this.stallT === 0) {
        const prev = this.phase;
        this.phase = (this.phase + dt * freq) % 1;
        // never start a tripod while the other tripod is still in the air
        for (const [b, grp] of [[0, 0], [0.5, 1]]) {
          const crossing = (prev < b && this.phase >= b) || (b === 0 && this.phase < prev);
          if (crossing && R.legs.some((l) => l.mode === 'swing' && l.def.group !== grp)) this.phase = b === 0 ? 0.9999 : 0.4999;
        }
        const crossed = (b) => (prev < b && this.phase >= b) || (b === 0 && this.phase < prev);
        for (const [b, grp] of [[0, 0], [0.5, 1]]) if (crossed(b)) {
          const pp = this.predict(R.pose, c.v, c.w, swingDur + 0.25 / freq);
          for (const l of R.legs) {
            if (l.def.group !== grp || l.mode !== 'stance') continue;
            const n = R.neutralWorld(l, pp);
            if (!moving && Math.hypot(l.foot.x - n.x, l.foot.z - n.z) < 0.05) continue;
            let H = P.H; if (S.fault && S.fault.leg === l.id) H *= 1 - 0.6 * S.fault.severity;
            const st = this.planStep(l, pp, Math.max(0.05, H));
            if (!st) { l.errT = 0.6; A.Log.once('nofoot' + l.id, `${A.LN(l.id)}: kein sicherer Tritt — Fuß bleibt stehen`, 'warn', 5); l.settleFail = !moving; continue; }
            if (!moving && st.to.distanceTo(l.foot) < 0.04) { l.settleFail = true; continue; }
            this.startSwing(l, st.to, st.H, swingDur);
          }
        }
      }

      // ---- reflex step: a stance leg close to its hip-yaw limit steps out of sequence while the other five stand
      if (gaitOn && !R.legs.some((l) => l.mode === 'swing')) {
        for (const l of R.legs) {
          if (l.mode !== 'stance') continue;
          const m = Math.min(l.angles.yaw - l.yawLim[0], l.yawLim[1] - l.angles.yaw);
          if (m > 7 * D) continue;
          const pp = this.predict(R.pose, c.v, c.w, swingDur + 0.25 / freq);
          const st = this.planStep(l, pp, Math.max(0.05, P.H));
          if (st) { this.startSwing(l, st.to, st.H, swingDur * 0.8); A.Log.once('reflex' + l.id, `${A.LN(l.id)}: Reflexschritt — Hüftdrehung nahe Grenze`, 'info', 4); break; }
        }
      }

      // ---- advance swing legs (validated; a blocked swing stalls the gait)
      if (R.legs.some((l) => l.mode === 'swing')) {
        const snap = R.snapshot(), p = new V();
        for (const l of R.legs) if (l.mode === 'swing') { const sw = l.swing; sw.nt = Math.min(1, sw.t + dt / sw.dur); this.arcPoint(sw, sw.nt, p); l.foot.copy(p); }
        R.applyPose(R.pose);
        const r = R.validate(['legs', 'arm']);
        if (r.ok) {
          this.stallT = 0;
          for (const l of R.legs) if (l.mode === 'swing') {
            l.swing.t = l.swing.nt;
            if (l.swing.t >= 1) { l.foot.copy(l.swing.to); l.mode = 'stance'; l.swing = null; if (l.plantCb) { l.plantCb(); l.plantCb = null; } }
          }
        } else {
          R.restore(snap);
          this.stallT += dt;
          const leg = r.part && r.part.startsWith('leg:') ? R.legById[r.part.slice(4)] : null;
          if (leg) leg.errT = 0.4;
          R.blocked('swing', r);
          if (this.stallT > 0.7) { // retract: reverse every stuck swing back to its lift-off point
            for (const l of R.legs) if (l.mode === 'swing') { const sw = l.swing; const back = sw.from.clone(); sw.from = l.foot.clone(); sw.to = back; sw.t = 0; sw.H = 0.04; }
            this.stallT = 0;
          }
        }
      }

      // ---- body pose: translation/rotation + posture + terrain, validated
      const T = this.terrainFit();
      this.terrain.pitch = T.pitch; this.terrain.roll = T.roll;
      const pose = R.pose, fault = S.fault;
      let sag = 0, wob = 0;
      if (fault && fault.severity > 0) { sag = -6 * D * fault.severity; wob = Math.sin(performance.now() / 160) * 2.2 * D * fault.severity + (Math.random() - 0.5) * 1.2 * D * fault.severity; }
      const hT = S.powered ? cmd.bodyHeight : C.height.min;
      const pitchT = U.clamp(cmd.bodyPitch * D + T.pitch * 0.85, -32 * D, 32 * D);
      const rollT = U.clamp(cmd.bodyRoll * D + T.roll * 0.85 + sag + wob, -32 * D, 32 * D);
      const posture = {
        gy: U.damp(pose.gy, T.y, 5, dt),
        h: U.approach(pose.h, hT, (S.powered ? 0.32 : 0.12) * dt),
        pitch: U.approach(pose.pitch, pitchT, 0.7 * dt),
        roll: U.approach(pose.roll, rollT, (fault ? 1.6 : 0.7) * dt),
      };
      this.margin = this.supportMargin();
      const stable = this.margin > 0.06;
      let motion = null;
      if (moving && stable) {
        const yaw = pose.yaw + this.w * dt;
        motion = { x: pose.x + Math.cos(yaw) * this.v * dt, z: pose.z - Math.sin(yaw) * this.v * dt, yaw };
      }
      if (moving && !stable) A.Log.once('stab', `Stabilitätsrand ${(this.margin * 100).toFixed(0)} cm — Bewegung pausiert zum Ausbalancieren`, 'warn', 4);
      let res = { ok: true }, moved = false;
      if (motion) {
        res = R.tryPose({ ...pose, ...posture, ...motion });
        if (res.ok) moved = true;
        else {
          const r2 = R.tryPose({ ...pose, ...motion });           // motion with old posture
          if (r2.ok) { moved = true; const r3 = R.tryPose({ ...R.pose, ...posture }); if (!r3.ok) R.blocked('posture', r3); }
          else { const r3 = R.tryPose({ ...pose, ...posture }); if (!r3.ok) R.blocked('posture', r3); res = r2; }
        }
      } else {
        const r3 = R.tryPose({ ...pose, ...posture });
        if (!r3.ok) R.blocked('posture', r3);
      }

      // ---- blocked locomotion handling
      if (motion && !moved) {
        this.blockedT += dt; this.lastReason = res.reason;
        if (this.blockedT > 0.08 && this.blockedT - dt <= 0.08) R.blocked('motion', res);
        // reach/limit blocks resolve by re-stepping — give the gait time before halting
        const legLimited = /grenze|reichweite|zu nah/i.test(res.reason || '');
        if (this.blockedT > (legLimited ? 2.5 : 0.6)) this.onBlocked(res);
      } else this.blockedT = Math.max(0, this.blockedT - dt * 2);

      // ---- statuses + loads
      const stance = R.legs.filter((l) => l.mode === 'stance').length || 1;
      for (const l of R.legs) {
        let s = 'idle';
        if (l.mode === 'swing') s = l.swing.t < 0.3 ? 'lifting' : 'moving';
        else if (l.mode === 'manual') s = l.pts.pad.y - W.surfaceBelow(l.pts.pad.x, l.pts.pad.z, l.pts.pad.y).y > 0.03 ? 'lifting' : 'idle';
        else s = this.gaitActive ? 'grounded' : 'idle';
        if (S.fault && S.fault.leg === l.id && S.fault.severity > 0.15) l.errT = Math.max(l.errT, 0.1);
        l.setStatus(s);
        const base = l.mode === 'stance' ? 1 / stance : 0;
        l.load = U.damp(l.load, base * (S.fault && S.fault.leg === l.id ? 1 + 1.8 * S.fault.severity : 1), 6, dt);
      }
    },

    onBlocked(res) {
      const S = A.State;
      this.blockedT = 0;
      if (this.mode === 'nav' && this.nav) {
        const n = this.nav;
        if (n.replans < 3) {
          n.replans++;
          A.Log.add(`Weg blockiert (${res.reason}) — neue Route ${n.replans}/3`, 'warn');
          const path = A.Planner.plan({ x: this.R.pose.x, z: this.R.pose.z }, n.goal, A.Planner.profile(this.R));
          if (path) { n.path = path; n.i = 1; return; }
        }
        n.status = 'failed'; this.mode = 'idle';
        A.Log.add(`Navigation abgebrochen — ${res.reason}`, 'err'); S.emit('loco', 'idle'); return;
      }
      if (this.mode === 'walk' || this.mode === 'crawl' || this.mode === 'rotate') {
        this.haltedReason = res.reason;
        A.Log.add(`Fortbewegung gestoppt — ${res.reason}`, 'warn');
        A.feedback(`GESTOPPT: ${res.reason}`, 'warn');
        const prev = this.mode;
        this.mode = 'idle'; S.emit('loco', 'idle');
        if (S.toggles.autonomous && prev !== 'rotate') {
          A.Log.add('Autonome Umleitung: Drehung weg vom Hindernis', 'info');
          this.rerouteDir = -(this.rerouteDir || 1);
          this.setMode('turnTo', { yaw: U.wrap(this.R.pose.yaw + this.rerouteDir * 70 * D) });
          this.resumeAfterTurn = prev;
        }
      }
      if (this.mode === 'turnTo') { this.mode = 'idle'; A.Log.add(`Drehung gestoppt — ${res.reason}`, 'warn'); S.emit('loco', 'idle'); }
    },
    postUpdate() {
      if (this.mode === 'idle' && this.resumeAfterTurn) { const m = this.resumeAfterTurn; this.resumeAfterTurn = null; if (A.State.toggles.autonomous) this.setMode(m); }
    },

    // ------------------------------------------------------------------ manual single-leg control
    canManual() {
      if (this.gaitActive || this.mode !== 'idle') { A.feedback('Erst den Laufzyklus stoppen, dann Einzelbeinsteuerung', 'warn'); return false; }
      if (!A.State.powered) { A.feedback('Roboter abgeschaltet', 'warn'); return false; }
      return true;
    },
    // set joint angles (radians, partial) on one leg; bisects back to the last collision-free configuration
    manualSet(id, partial) {
      if (!this.canManual()) return;
      const R = this.R, l = R.legById[id];
      for (const o of R.legs) if (o !== l && o.mode === 'manual') this.plantLeg(o.id, true);
      if (l.mode === 'swing') return;
      if (l.mode !== 'manual') {
        const others = R.legs.filter((o) => o !== l && o.mode === 'stance').length;
        if (others < 5) { A.feedback('Einzelbeinsteuerung braucht fünf stehende Beine', 'warn'); return; }
        l.mode = 'manual';
      }
      const LIM = C.limits, from = { ...l.angles };
      const to = { ...from, ...partial };
      to.yaw = U.clamp(to.yaw, l.yawLim[0], l.yawLim[1]); to.pitch = U.clamp(to.pitch, LIM.hipPitch[0], LIM.hipPitch[1]);
      to.knee = U.clamp(to.knee, LIM.knee[0], LIM.knee[1]); to.ankle = U.clamp(to.ankle, -Math.PI * 0.9, Math.PI * 0.9);
      const test = (t) => {
        const a = {}; for (const k of ['yaw', 'pitch', 'knee', 'ankle']) a[k] = U.lerp(from[k], to[k], t);
        l.setAngles(a); l.footG.quaternion.identity(); R.root.updateMatrixWorld(true); l.points();
        return R.singleLegCheck(l);
      };
      let r = test(1);
      if (!r.ok) {
        let lo = 0, hi = 1;
        for (let i = 0; i < 7; i++) { const m = (lo + hi) / 2; if (test(m).ok) lo = m; else hi = m; }
        test(lo);
        R.blocked('leg', r);
        A.feedback(`${A.LN(id)}: ${r.reason} — Gelenk an Kontaktstelle gestoppt`, 'warn');
      }
      l.foot.copy(l.points().pad);
    },
    liftLeg(id) {
      if (!this.canManual()) return;
      const l = this.R.legById[id];
      if (l.mode !== 'manual') this.manualSet(id, {});
      if (l.mode !== 'manual') return;
      this.manualSet(id, { pitch: l.angles.pitch + 0.45, knee: l.angles.knee - 0.15, ankle: l.angles.ankle - 0.3 });
      A.Log.add(`Fuß ${A.LN(id)} angehoben (manuell)`);
    },
    // plant a manual/raised leg: swing the pad down to a validated foothold
    plantLeg(id, quiet, cb) {
      const R = this.R, l = R.legById[id];
      if (l.mode === 'swing') return;
      R.root.updateMatrixWorld(true);
      const pad = l.points().pad.clone();
      const wasManual = l.mode === 'manual';
      l.mode = 'stance'; l.foot.copy(pad);
      const st = this.planStepAround(l, pad.x, pad.z);
      if (!st) { if (wasManual) l.mode = 'manual'; A.Log.add(`${A.LN(id)}: kein sicherer Tritt unter dem Fuß`, 'warn'); return; }
      l.swing = { from: pad, to: st.to, H: Math.max(0.02, Math.min(0.1, pad.y - st.to.y)), t: 0, dur: 0.55 };
      l.mode = 'swing'; l.plantCb = cb || null;
      if (!quiet) A.Log.add(`Fuß ${A.LN(id)} setzt auf — Kontakt bei ${st.to.y.toFixed(2)} m`);
    },
    planStepAround(l, x, z) {
      const R = this.R, snap = R.snapshot(), from = l.foot.clone();
      const n = R.neutralWorld(l, R.pose);
      const cands = [{ x, z }];
      for (const r of [0.08, 0.16, 0.24]) for (let k = 0; k < 8; k++) cands.push({ x: x + Math.cos(k * 0.785) * r, z: z + Math.sin(k * 0.785) * r });
      cands.push(n);
      let out = null;
      for (const c of cands) {
        const contact = R.contactAt(c.x, c.z, Math.max(0, from.y - 0.2));
        l.foot.copy(contact); R.applyPose(R.pose);
        const ok = R.singleLegCheck(l).ok; R.restore(snap);
        if (ok) { out = { to: contact }; break; }
      }
      l.foot.copy(from);
      return out;
    },
    resetLeg(id) {
      const R = this.R, l = R.legById[id];
      if (!this.canManual() && l.mode !== 'manual') return;
      const n = R.neutralWorld(l, R.pose);
      R.root.updateMatrixWorld(true);
      const pad = l.points().pad.clone();
      l.mode = 'stance'; l.foot.copy(pad);
      const st = this.planStepAround(l, n.x, n.z);
      if (!st) { A.Log.add(`${A.LN(id)}: Neutralstellung nicht erreichbar`, 'warn'); return; }
      l.swing = { from: pad, to: st.to, H: 0.12, t: 0, dur: 0.6 }; l.mode = 'swing';
      A.Log.add(`Bein ${A.LN(id)} in Neutralstellung zurückgesetzt`);
    },
  });
})();
