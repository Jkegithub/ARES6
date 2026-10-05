/* ARES-6 — mission system: presets, demo sequences (grab, scan, climb, failure), system actions */
(function () {
  'use strict';
  const A = window.ARES, C = A.CFG, U = A.U, W = A.World, D = A.DEG;
  const V = THREE.Vector3;
  const T = A.Tasks, S = A.State, L = A.Log;
  let R; // robot

  // ------------------------------------------------------------------ building blocks (generators)
  function* settle(timeout = 4) {
    yield* T.until(() => Math.abs(R.pose.h - S.cmd.bodyHeight) < 0.01 && !A.Loco.gaitActive, timeout);
  }
  function setPose(name) {
    const p = C.poses[name];
    Object.assign(S.cmd, { bodyHeight: p.h, bodyPitch: p.pitch, bodyRoll: p.roll, stance: p.stance });
    S.emit('cmd');
  }
  function* goTo(goal, finalYaw = null, label = 'Ziel') {
    const prof = A.Planner.profile(R);
    const path = A.Planner.plan({ x: R.pose.x, z: R.pose.z }, goal, prof);
    if (!path) { L.add(`Keine kollisionsfreie Route zu ${label} in dieser Pose`, 'warn'); return false; }
    L.add(`Route zu ${label}: ${path.length - 1} Abschnitte, ${pathLen(path).toFixed(1)} m (Freiraum r=${prof.R.toFixed(2)} m)`);
    A.Loco.navigate(path, finalYaw);
    const ok = yield* T.until(() => !A.Loco.nav || A.Loco.nav.status === 'done' || A.Loco.nav.status === 'failed' || A.Loco.mode === 'idle', 120);
    const st = A.Loco.nav ? A.Loco.nav.status : 'aborted';
    A.Loco.nav = null;
    yield* settle(3);
    return ok && st === 'done';
  }
  const pathLen = (p) => p.reduce((s, q, i) => (i ? s + Math.hypot(q.x - p[i - 1].x, q.z - p[i - 1].z) : 0), 0);
  function* turnTo(yaw) {
    A.Loco.setMode('turnTo', { yaw });
    yield* T.until(() => A.Loco.mode !== 'turnTo', 20);
    yield* settle(3);
  }
  function lookAt(p) {
    const hc = R.headCenter(new V()), d = p.clone().sub(hc);
    const yaw = U.wrap(Math.atan2(-d.z, d.x) - R.pose.yaw) / D;
    S.cmd.headYaw = U.clamp(yaw, -120, 120);
    S.cmd.headPitch = U.clamp(Math.atan2(d.y, Math.hypot(d.x, d.z)) / D, -30, 45);
    S.emit('cmd');
  }
  // cartesian claw move (claw pointing down) along a straight line; stopFn may end it early
  function* clawLine(to, speed = 0.3, stopFn = null) {
    const arm = R.arm;
    const from = arm.points().grasp.clone(), dist = from.distanceTo(to);
    const n = Math.max(1, Math.ceil(dist / 0.02));
    for (let i = 1; i <= n; i++) {
      const p = from.clone().lerp(to, i / n);
      R.root.updateMatrixWorld(true);
      const ik = arm.solveDown(p, arm.target.wristRoll);
      if (!ik.ok) { L.add(`Arm-IK: ${ik.why}`, 'warn'); return false; }
      Object.assign(arm.target, ik.a);
      const ok = yield* T.until(() => arm.reached(0.01) || arm.blocked, 0.02 / speed + 1.2);
      if (arm.blocked) { L.add(`Arm gestoppt — ${arm.blockReason}`, 'warn'); return false; }
      if (!ok) return false;
      if (stopFn && stopFn()) return true;
    }
    return true;
  }
  function stowArm() { Object.assign(R.arm.target, { yaw: 0, shoulder: 80 * D, elbow: -150 * D, wristPitch: 60 * D, wristRoll: 0 }); syncArmCmd(); }
  function syncArmCmd() {
    const t = R.arm.target;
    Object.assign(S.cmd, { armYaw: t.yaw / D, armShoulder: t.shoulder / D, armElbow: t.elbow / D, armWristPitch: t.wristPitch / D, armWrist: t.wristRoll / D });
    S.emit('cmd');
  }

  // ------------------------------------------------------------------ grab debris demo
  function* grabDemo() {
    const arm = R.arm;
    if (arm.held) { L.add('Greifer hält bereits eine Last — erst loslassen', 'warn'); return; }
    const d = A.Env.nearestDebris(R.pose, 12);
    if (!d) { L.add('Keine losen Trümmer im Umkreis von 12 m', 'warn'); return; }
    T.step('Orten');
    A.Env.highlight('debris', d.ob.obb.c);
    L.add(`Trümmerräumung gestartet — Ziel ${d.ob.name} (${d.mass} kg)`, 'info');
    lookAt(d.ob.obb.c);
    arm.target.gap = C.arm.maxGap;
    // approach: stand so that the debris is ~1.0 m in front of the shoulder
    T.step('Anfahrt');
    const dc = d.ob.obb.c, dx = dc.x - R.pose.x, dz = dc.z - R.pose.z, dd = Math.hypot(dx, dz);
    const yaw = Math.atan2(-dz, dx);
    const shoulderDist = () => { const s = R.arm.root.localToWorld(new V()); return Math.hypot(dc.x - s.x, dc.z - s.z); };
    if (dd > 2.3 || Math.abs(U.wrap(yaw - R.pose.yaw)) > 0.25) {
      const stand = { x: dc.x - (dx / dd) * 1.85, z: dc.z - (dz / dd) * 1.85 };
      const ok = yield* goTo(stand, yaw, d.ob.name);
      if (!ok && shoulderDist() > 1.25) { L.add('Anfahrt fehlgeschlagen — Trümmer außer Reichweite', 'err'); return; }
    }
    // fine approach / back-off
    if (shoulderDist() > 1.05) { A.Loco.setMode('crawl'); yield* T.until(() => shoulderDist() < 1.0 || A.Loco.mode === 'idle', 12); A.Loco.stop(); yield* settle(3); }
    if (shoulderDist() > 1.2) { L.add('Trümmer weiter außer Reichweite — Ablauf abgebrochen', 'err'); return; }
    lookAt(d.ob.obb.c);
    // align jaws with the debris box (jaw axis across the shorter horizontal side)
    T.step('Vorgreifen');
    const debYaw = d.ob.yaw(), across = d.ob.obb.h.x < d.ob.obb.h.z ? debYaw : debYaw + Math.PI / 2;
    R.root.updateMatrixWorld(true);
    const pre = dc.clone(); pre.y = dc.y + d.ob.obb.h.y + 0.35;
    let ik = arm.solveDown(pre);
    if (!ik.ok) { L.add(`Vorgriff-Position unerreichbar (${ik.why})`, 'err'); return; }
    const armWorldYaw = R.pose.yaw + ik.a.yaw;
    // claw pointing down: jaw axis world angle = armWorldYaw − 90° − roll  →  roll = armWorldYaw − 90° − across
    ik.a.wristRoll = U.wrap(armWorldYaw - Math.PI / 2 - across);
    if (ik.a.wristRoll > Math.PI / 2) ik.a.wristRoll -= Math.PI; if (ik.a.wristRoll < -Math.PI / 2) ik.a.wristRoll += Math.PI;
    Object.assign(arm.target, ik.a);
    yield* T.until(() => arm.reached(0.01) || arm.blocked, 6);
    if (arm.blocked) { L.add(`Vorgreifen blockiert — ${arm.blockReason}`, 'err'); return; }
    // descend around the debris; only the fingers may touch it
    T.step('Absenken');
    arm.graspTarget = d.ob;
    // palm stays just above the top face; fingers straddle the upper part of the block
    const grasp = dc.clone(); grasp.y = dc.y + d.ob.obb.h.y - 0.02;
    if (!(yield* clawLine(grasp, 0.22))) { arm.graspTarget = null; return; }
    // close until contact
    T.step('Greifen');
    arm.target.gap = 0;
    yield* T.until(() => arm.contact, 4);
    if (!arm.contact) { L.add('Griff fehlgeschlagen — kein Fingerkontakt', 'err'); arm.graspTarget = null; arm.target.gap = C.arm.maxGap; return; }
    L.add(`Fingerkontakt — Greifkraft normal, Backenabstand ${(arm.angles.gap * 100).toFixed(1)} cm`, 'ok');
    arm.wristR.attach(d.mesh); arm.held = d; d.ob.held = true; arm.graspTarget = null; arm.target.gap = arm.angles.gap;
    R.syncHeld();
    // lift
    T.step('Anheben');
    const lift = R.arm.points().grasp.clone(); lift.y += 0.4;
    if (!(yield* clawLine(lift, 0.25))) { L.add('Anheben unterbrochen — Last wird gehalten', 'warn'); }
    // move aside: find a free placement spot
    T.step('Umsetzen');
    const sh = R.arm.root.localToWorld(new V());
    let place = null;
    for (const a of [-40, 40, -28, 28, -55, 55]) {
      const wy = R.pose.yaw + a * D, rr = 0.98;
      const p = new V(sh.x + Math.cos(wy) * rr, 0, sh.z - Math.sin(wy) * rr);
      const surf = W.surfaceBelow(p.x, p.z, 1).y;
      p.y = surf + d.ob.obb.h.y + 0.003;
      const probe = new A.OBB(p, d.ob.obb.h.clone().addScalar(0.03), new THREE.Quaternion().setFromAxisAngle(new V(0, 1, 0), d.ob.yaw()));
      if (W.obbHit(probe, probe.samples(0.1), d.ob, 0)) continue;
      if (R.legs.some((l) => Math.hypot(l.foot.x - p.x, l.foot.z - p.z) < 0.45)) continue;
      place = p; break;
    }
    if (!place) { L.add('Kein freier Ablageplatz — Last wird gehalten', 'warn'); return; }
    const above = place.clone(); above.y = Math.max(lift.y, place.y + 0.45);
    // arc over in a few waypoints at constant height
    const cur = R.arm.points().grasp.clone();
    for (let k = 1; k <= 4; k++) {
      const wp = cur.clone().lerp(above, k / 4); wp.y = above.y;
      if (!(yield* clawLine(wp, 0.35))) { L.add('Umsetzen blockiert — Last wird gehalten', 'warn'); return; }
    }
    // lower until the payload touches its support
    T.step('Absetzen');
    const touch = () => { const o = d.ob; const b = o.obb.c.y - o.obb.h.y; return b - A.Env.debrisSupport(d) < 0.012; };
    const down = place.clone(); down.y = place.y - d.ob.obb.h.y + 0.0;
    const tgt = R.arm.points().grasp.clone(); tgt.y -= (d.ob.obb.c.y - d.ob.obb.h.y) - W.surfaceBelow(place.x, place.z, 1).y;
    yield* clawLine(tgt, 0.18, touch);
    L.add(`${d.ob.name} abgesetzt — Bodenkontakt bestätigt`, 'ok');
    // release
    T.step('Loslassen');
    releasePayload();
    arm.graspTarget = d.ob;            // fingers may slide along the released block while opening
    arm.target.gap = C.arm.maxGap;
    yield 0.6;
    const up = R.arm.points().grasp.clone(); up.y += 0.35;
    yield* clawLine(up, 0.3);
    arm.graspTarget = null;
    T.step('Verstauen');
    stowArm(); arm.target.gap = 0.06;
    yield* T.until(() => arm.reached(0.02) || arm.blocked, 6);
    L.add('Trümmerräumung abgeschlossen — Arm verstaut', 'ok');
  }
  function releasePayload() {
    const arm = R.arm, d = arm.held;
    if (!d) return false;
    A.Env.group.attach(d.mesh);
    // keep only the yaw of the payload (rests flat), physics settles it
    const e = new THREE.Euler().setFromQuaternion(d.mesh.quaternion, 'YXZ');
    d.mesh.rotation.set(0, e.y, 0);
    d.ob.obb.set(d.mesh.position, d.mesh.quaternion); d.ob.held = false; d.ob.refresh();
    arm.held = null;
    L.add(`Greifer hat ${d.ob.name} losgelassen`, 'info');
    return true;
  }

  // ------------------------------------------------------------------ load handling building blocks (beams, parcels)
  // claw move along a straight line, wrist roll interpolated; mode 'down' = claw vertical, 'front' = claw level
  function* clawPath(to, roll, mode = 'down', speed = 0.3, stopFn = null, step = 0.02) {
    const arm = R.arm, from = arm.points().grasp.clone(), r0 = arm.target.wristRoll;
    const n = Math.max(1, Math.ceil(Math.max(from.distanceTo(to) / step, Math.abs(roll - r0) / (2 * D))));
    for (let i = 1; i <= n; i++) {
      const p = from.clone().lerp(to, i / n), rr = r0 + ((roll - r0) * i) / n;
      R.root.updateMatrixWorld(true);
      const ik = mode === 'front' ? arm.solveFront(p, rr) : arm.solveDown(p, rr);
      if (!ik.ok) { L.add(`Arm-IK: ${ik.why}`, 'warn'); return false; }
      Object.assign(arm.target, ik.a);
      const ok = yield* T.until(() => arm.reached(0.01) || arm.blocked, 0.02 / speed + 1.2);
      if (arm.blocked) { L.add(`Arm gestoppt — ${arm.blockReason}`, 'warn'); return false; }
      if (!ok) { L.add('Arm erreicht Bahnpunkt nicht (Zeitüberschreitung)', 'warn'); return false; }
      if (stopFn && stopFn()) return true;
    }
    return true;
  }
  // joint-space move to a claw pose (e.g. out of the stowed arm, where a straight claw line is not defined)
  function* armTo(p, roll, mode = 'down', timeout = 8) {
    const arm = R.arm;
    R.root.updateMatrixWorld(true);
    const ik = mode === 'front' ? arm.solveFront(p, roll) : arm.solveDown(p, roll);
    if (!ik.ok) { L.add(`Arm-IK: ${ik.why}`, 'warn'); return false; }
    Object.assign(arm.target, ik.a);
    yield* T.until(() => arm.reached(0.01) || arm.blocked, timeout);
    if (arm.blocked) { L.add(`Arm gestoppt — ${arm.blockReason}`, 'warn'); return false; }
    return arm.reached(0.01);
  }
  // wrist roll that turns a gripped long load to world angle `axis` (claw down: load axis = arm yaw − roll), nearest to now
  function rollFor(p, axis) {
    R.root.updateMatrixWorld(true);
    const ik = R.arm.solveDown(p), aw = R.pose.yaw + ik.a.yaw, cur = R.arm.target.wristRoll;
    let r = U.wrap(aw - axis);
    for (const c of [r - Math.PI, r + Math.PI]) if (Math.abs(c) <= Math.PI && Math.abs(c - cur) < Math.abs(r - cur)) r = c;
    return r;
  }
  const fwd = (yaw) => new V(Math.cos(yaw), 0, -Math.sin(yaw));
  const shoulder = () => { R.root.updateMatrixWorld(true); return R.arm.root.localToWorld(new V()); };
  function attachLoad(d) {
    const arm = R.arm;
    arm.wristR.attach(d.mesh); arm.held = d; d.ob.held = true; arm.graspTarget = null; arm.target.gap = arm.angles.gap;
    R.syncHeld();
  }
  function* closeOn(d) {
    const arm = R.arm;
    arm.target.gap = 0;
    yield* T.until(() => arm.contact, 4);
    if (!arm.contact) { L.add(`Griff an ${d.ob.name} fehlgeschlagen — kein Fingerkontakt`, 'err'); arm.graspTarget = null; arm.target.gap = C.arm.maxGap; return false; }
    L.add(`Fingerkontakt an ${d.ob.name} — Backenabstand ${(arm.angles.gap * 100).toFixed(1)} cm, Last ${d.mass} kg von ${C.arm.payload} kg`, 'ok');
    attachLoad(d);
    return true;
  }
  function payloadOk(d) {
    if (d.mass <= C.arm.payload) return true;
    L.add(`${d.ob.name}: ${d.mass} kg über der Traglast (${C.arm.payload} kg) — Griff verweigert`, 'err');
    return false;
  }
  // stand at `p` facing `yaw`; the along-track error is removed by a straight manoeuvre (lateral error stays, logged)
  // a navigation that ends with a leg wound up at its hip-yaw limit (tight curves in the high pose) is resumed:
  // stop, let the legs re-step to neutral, plan again
  function* goToRetry(p, yaw, label) {
    for (let k = 0; k < 3; k++) {
      if (yield* goTo(p, yaw, label)) return true;
      if (k < 2) {
        L.add(`${label}: Anfahrt wird neu aufgenommen — Beine neu aufsetzen (${k + 1}/2)`, 'info');
        A.Loco.stop();
        for (const l of R.legs) l.settleFail = false;
        yield* settle(5);
      }
    }
    return false;
  }
  function* turnToRetry(yaw, tol = 2 * D) {
    for (let k = 0; k < 3; k++) {
      yield* turnTo(yaw);
      if (Math.abs(U.wrap(R.pose.yaw - yaw)) <= tol) return true;
      A.Loco.stop(); for (const l of R.legs) l.settleFail = false;
      yield* settle(5);
    }
    L.add(`Kurs ${(U.wrap(yaw) / D).toFixed(0)}° nicht erreicht`, 'warn');
    return false;
  }
  // travel in the high pose (narrower footprint: the planner finds the yard access), work in the zero pose
  function* standAt(p, yaw, label) {
    if (Math.hypot(R.pose.x - p.x, R.pose.z - p.z) > 0.12 || Math.abs(U.wrap(R.pose.yaw - yaw)) > 2 * D) {
      if (Math.hypot(R.pose.x - p.x, R.pose.z - p.z) > 0.6) { setPose('high'); yield* settle(5); }
      // into the yard through the gap between ramp and rack AS, keeping off the rack corner
      const G = A.Env.YARD_GATE;
      if (p.z > G.z && R.pose.z < G.z - 0.5 && R.pose.x > -3) { if (!(yield* goToRetry(G, null, 'Hofzufahrt'))) return false; }
      if (!(yield* goToRetry(p, yaw, label))) return false;
    }
    if (S.cmd.bodyHeight !== C.poses.zero.h) { setPose('zero'); yield* settle(5); }
    // lateral error (the planner may stop short of a goal that lies close to obstacles): face the point, move, face back
    const f0 = fwd(yaw), lat0 = (p.x - R.pose.x) * -f0.z + (p.z - R.pose.z) * f0.x;
    if (Math.abs(lat0) > 0.05) {
      const dx = p.x - R.pose.x, dz = p.z - R.pose.z, dist = Math.hypot(dx, dz);
      let hd = Math.atan2(-dz, dx), dir = 1;
      if (Math.abs(U.wrap(hd - R.pose.yaw)) > Math.PI / 2) { hd = U.wrap(hd + Math.PI); dir = -1; } // back up rather than turn round
      if (!(yield* turnToRetry(hd))) return false;
      A.Loco.setMode('shift', { dist: dir * dist });
      yield* T.until(() => A.Loco.mode !== 'shift', 15); if (A.Loco.mode === 'shift') A.Loco.stop('Rangieren: Zeitüberschreitung');
      yield* settle(3);
    }
    if (Math.abs(U.wrap(R.pose.yaw - yaw)) > 1.5 * D && !(yield* turnToRetry(yaw))) return false;
    const f = fwd(yaw), e = (p.x - R.pose.x) * f.x + (p.z - R.pose.z) * f.z;
    if (Math.abs(e) > 0.012) {
      A.Loco.setMode('shift', { dist: e });
      yield* T.until(() => A.Loco.mode !== 'shift', 12); if (A.Loco.mode === 'shift') A.Loco.stop('Rangieren: Zeitüberschreitung');
      yield* settle(3);
    }
    const lat = (p.x - R.pose.x) * -f.z + (p.z - R.pose.z) * f.x;
    L.add(`${label}: Standpunkt erreicht — Längsfehler ${(Math.abs((p.x - R.pose.x) * f.x + (p.z - R.pose.z) * f.z) * 100).toFixed(1)} cm, Querfehler ${(Math.abs(lat) * 100).toFixed(1)} cm`);
    return true;
  }
  function* stow() {
    stowArm(); R.arm.target.gap = 0.06;
    yield* T.until(() => R.arm.reached(0.02) || R.arm.blocked, 6);
  }
  // release onto the support and back the claw off along `away` (fingers may slide along the load meanwhile)
  function* letGo(d, away, mode, roll) {
    releasePayload();
    R.arm.graspTarget = d.ob; R.arm.target.gap = C.arm.maxGap;
    yield 0.6;
    // as far as reachable (on a high stack the full lift is out of reach), at least until the fingers are clear
    const g0 = R.arm.points().grasp.clone();
    let back = null;
    for (const k of [1, 0.66, 0.4]) {
      const b = g0.clone().addScaledVector(away, k);
      R.root.updateMatrixWorld(true);
      if ((mode === 'front' ? R.arm.solveFront(b, roll) : R.arm.solveDown(b, roll)).ok) { back = b; break; }
    }
    if (back) yield* clawPath(back, roll, mode, 0.3);
    R.arm.graspTarget = null;
    // pull the claw in towards the body at this height before the joint-space stow
    if (mode === 'down') {
      const sh = shoulder(), cur = R.arm.points().grasp, dir = cur.clone().sub(sh); dir.y = 0; dir.normalize();
      const inner = sh.clone().addScaledVector(dir, 0.6); inner.y = cur.y;
      yield* clawPath(inner, roll, mode, 0.3);
    }
  }
  const bottomOf = (d) => d.ob.obb.c.y - d.ob.obb.h.y;
  // lowest corner (a carried beam hangs slightly tilted within the joint tolerance) close to its support
  const touching = (d) => () => d.ob.bottom - A.Env.debrisSupport(d) < 0.008;
  // the lowering path may end early (stop on contact, or validator block just above the support): release only
  // when the measured gap is small, otherwise keep holding and report
  function* confirmRest(d, label) {
    yield 0.3; // let the arm finish the last waypoint
    const gap = d.ob.bottom - A.Env.debrisSupport(d);
    if (gap > 0.012) { L.add(`${d.ob.name}: Absetzen ${label} unterbrochen — ${(gap * 100).toFixed(1)} cm über der Auflage, Last wird gehalten`, 'err'); return false; }
    L.add(`${d.ob.name} ${label} abgesetzt — Auflage bestätigt (Spalt ${(Math.max(gap, 0) * 1000).toFixed(0)} mm)`, 'ok');
    return true;
  }

  // ------------------------------------------------------------------ demo: clear the beams and stack them on the timbers
  // The 2.4 m beam cannot be swung past the front legs by the arm alone (measured), so ARES-6 grips it in front of
  // the body and turns on the spot (no walking with load); the stack lies behind the stand point.
  function stackState() {
    const ST = A.Env.STACK, on = A.Env.debris.filter((d) => d.type === 'beam' && !d.ob.held && d.ob.footprint(ST.x, ST.z, 0.2) && bottomOf(d) > 0.05);
    on.sort((a, b) => a.ob.obb.c.y - b.ob.obb.c.y);
    return on;
  }
  function stackReport() {
    const ST = A.Env.STACK, on = stackState();
    let off = 0, ang = 0, gap = 0;
    on.forEach((d, i) => {
      off = Math.max(off, Math.hypot(d.ob.obb.c.x - ST.x, d.ob.obb.c.z - ST.z));
      let a = Math.abs(U.wrap(d.ob.yaw() - Math.PI / 2)); if (a > Math.PI / 2) a = Math.PI - a;
      ang = Math.max(ang, a);
      const below = i === 0 ? ST.timber : on[i - 1].ob.top;
      gap = Math.max(gap, Math.abs(bottomOf(d) - below));
    });
    return { n: on.length, off, ang, gap };
  }
  function* beamOne(d) {
    const arm = R.arm, ST = A.Env.STACK, { P, th, face } = d.stand;
    if (!payloadOk(d)) return false;
    T.step('Anfahrt');
    A.Env.highlight('beams');
    lookAt(d.ob.obb.c);
    if (!(yield* standAt(P, th, d.ob.name))) return false;
    // grip from above at the centre of gravity, jaws across the flange
    T.step('Greifen');
    const c = d.ob.obb.c.clone(), top = d.ob.top;
    lookAt(c);
    arm.target.gap = C.arm.maxGap;
    const pre = c.clone(); pre.y = Math.max(top + 0.35, 0.75);
    const roll = rollFor(pre, d.ob.yaw());
    if (!(yield* armTo(pre, roll))) return false;
    arm.graspTarget = d.ob;
    const g = c.clone(); g.y = top - 0.02;
    if (!(yield* clawPath(g, roll, 'down', 0.22))) { arm.graspTarget = null; return false; }
    if (!(yield* closeOn(d))) return false;
    // lift above everything on the ground and the stack, turn the beam across the body for carrying
    T.step('Anheben');
    const carryY = Math.max(0.75, (stackState().reduce((m, e) => Math.max(m, e.ob.top), ST.timber)) + 0.28);
    const up = arm.points().grasp.clone(); up.y = carryY;
    if (!(yield* clawPath(up, arm.target.wristRoll, 'down', 0.25))) return false;
    const sh = shoulder(), carry = sh.clone().addScaledVector(fwd(R.pose.yaw), 0.95); carry.y = carryY;
    if (!(yield* clawPath(carry, rollFor(carry, R.pose.yaw + Math.PI / 2), 'down', 0.25))) return false;
    // turn on the spot to face the stack
    T.step('Wenden');
    L.add(`${d.ob.name} angehoben (${d.mass} kg) — Drehung auf der Stelle zum Stapelplatz`);
    yield* turnToRetry(face, 3 * D);
    if (Math.abs(U.wrap(R.pose.yaw - face)) > 3 * D) { L.add('Wendung nicht abgeschlossen — Last wird gehalten', 'warn'); return false; }
    // set down on the stack: parallel to the timbers' normal (beam along z), centred on the stack
    T.step('Absetzen');
    const sup = stackState().reduce((m, e) => Math.max(m, e.ob.top), ST.timber);
    const off = arm.points().grasp.clone().sub(d.ob.obb.c); // grasp point relative to the load centre
    const slot = new V(ST.x, sup + d.ob.obb.h.y, ST.z);
    const above = slot.clone().add(off); above.y = Math.max(carryY, slot.y + off.y + 0.15);
    const pr = rollFor(above, Math.PI / 2);
    if (!(yield* clawPath(above, pr, 'down', 0.25))) return false;
    const down = slot.clone().add(off); down.y += 0.004;
    yield* clawPath(down, pr, 'down', 0.15, touching(d), 0.005);
    if (!(yield* confirmRest(d, 'auf dem Stapel'))) return false;
    T.step('Loslassen');
    yield* letGo(d, new V(0, 0.3, 0), 'down', pr);
    d.ob.avoid = true; // planner walks round the stack (a lying beam may be stepped over, the stack not)
    yield* stow();
    return true;
  }
  function* beamDemo() {
    const arm = R.arm;
    if (arm.held) { L.add('Greifer hält bereits eine Last — erst loslassen', 'warn'); return; }
    L.add('Trägerräumung — drei Stahlträger kreuz und quer im Hof, Ziel: Stapel auf den Kanthölzern', 'info');
    for (const d of A.Env.debris.filter((x) => x.type === 'beam')) {
      if (stackState().includes(d)) continue;
      if (!(yield* beamOne(d))) { L.add(`Trägerräumung angehalten bei ${d.ob.name}`, 'err'); return; }
    }
    T.step('Prüfen');
    const s = stackReport();
    S.stackCheck = s;
    L.add(`Stapel geprüft — ${s.n} Träger, Versatz max ${(s.off * 100).toFixed(1)} cm, Winkelabweichung max ${(s.ang / D).toFixed(1)}°, Auflagespalt max ${(s.gap * 1000).toFixed(0)} mm`, s.n === 3 && s.off < 0.05 && s.ang < 3 * D ? 'ok' : 'warn');
    L.add('Trägerräumung abgeschlossen — Zufahrt frei', 'ok');
  }

  // ------------------------------------------------------------------ demo: rack BS — put a parcel away and take one out
  // Front grip with a level claw. Stand point in front of the west compartment; the pallet lies to the right.
  function rackStand() { const RK = A.Env.RACK, P = A.Env.PALLET; return new V(RK.x - 0.725, 0, P.z); }
  // parcel centre → claw grasp point for a front grip from direction `dir` (fingers 3 cm behind the near face)
  const frontGrasp = (c, half, dir) => c.clone().addScaledVector(dir, -(half - 0.03));
  function* frontPick(d, dir) {
    const arm = R.arm, g = frontGrasp(d.ob.obb.c, frontDepth(d, dir), dir);
    arm.target.gap = C.arm.maxGap;
    const pre = g.clone().addScaledVector(dir, -0.28);
    lookAt(d.ob.obb.c);
    if (!(yield* armTo(pre, 0, 'front'))) return false;
    arm.graspTarget = d.ob;
    if (!(yield* clawPath(g, 0, 'front', 0.15))) { arm.graspTarget = null; return false; }
    return yield* closeOn(d);
  }
  function* frontPlace(d, centre, dir, label) {
    const arm = R.arm;
    const off = arm.points().grasp.clone().sub(d.ob.obb.c);
    const outside = centre.clone().addScaledVector(dir, -0.45).add(off); outside.y += 0.03;
    const inside = centre.clone().add(off); inside.y += 0.03;
    if (!(yield* clawPath(outside, 0, 'front', 0.25))) return false;
    if (!(yield* clawPath(inside, 0, 'front', 0.15))) return false;
    const down = inside.clone(); down.y -= 0.03 - 0.004;
    yield* clawPath(down, 0, 'front', 0.1, touching(d), 0.005);
    if (!(yield* confirmRest(d, label))) return false;
    yield* letGo(d, dir.clone().multiplyScalar(-0.3), 'front', 0);
    return true;
  }
  function* rackDemo() {
    const arm = R.arm, RK = A.Env.RACK, P = A.Env.PALLET;
    if (arm.held) { L.add('Greifer hält bereits eine Last — erst loslassen', 'warn'); return; }
    const p2 = A.Env.load('Paket P2'), p1 = A.Env.load('Paket P1');
    const home = p1 && p2 && Math.abs(p2.ob.bottom - P.h) < 0.01 && Math.hypot(p2.ob.obb.c.x - P.x, p2.ob.obb.c.z - P.z) < 0.1
      && Math.abs(p1.ob.bottom - (RK.levels[0] + 0.025)) < 0.01;
    if (!home) { L.add('Pakete P1/P2 nicht am Ausgangsort (P2 auf der Palette, P1 im unteren Fach) — Simulation zurücksetzen', 'warn'); return; }
    A.Env.highlight('rack');
    L.add('Regal-Demo — P2 von der Palette einlagern, P1 aus dem Regal auslagern und seitlich ablegen', 'info');
    const east = 0, north = Math.PI / 2, st = rackStand();
    // 1 · take P2 from the pallet (robot faces east)
    T.step('Anfahrt');
    if (!(yield* standAt(st, east, 'Regal BS'))) return;
    T.step('Palette greifen');
    if (!payloadOk(p2) || !(yield* frontPick(p2, fwd(east)))) return;
    const lift = arm.points().grasp.clone(); lift.y = 0.65;
    if (!(yield* clawPath(lift, 0, 'front', 0.2))) return;
    // carried close to the body while turning: the parcel must not sweep into the rack
    const near = shoulder().addScaledVector(fwd(east), 0.55); near.y = 0.65;
    if (!(yield* clawPath(near, 0, 'front', 0.25))) return;
    // 2 · turn to the rack, put P2 into the free middle slot of the west compartment
    T.step('Zum Regal');
    if (!(yield* turnToRetry(north))) return;
    T.step('Einlagern');
    const fz = RK.z + 0.4, top1 = RK.levels[1] + 0.025;
    const slot = new V(RK.x - 0.725, top1 + p2.ob.obb.h.y, fz - frontDepth(p2, fwd(north)));
    lookAt(slot);
    if (!(yield* frontPlace(p2, slot, fwd(north), 'im Regal (Mitte)'))) return;
    // 3 · take P1 out of the lower level
    T.step('Auslagern');
    if (!payloadOk(p1) || !(yield* frontPick(p1, fwd(north)))) return;
    const up1 = arm.points().grasp.clone(); up1.y += 0.015;
    if (!(yield* clawPath(up1, 0, 'front', 0.1))) return;
    const out1 = up1.clone().addScaledVector(fwd(north), -0.5);
    if (!(yield* clawPath(out1, 0, 'front', 0.15))) return;
    const hi1 = out1.clone(); hi1.y = 0.65;
    if (!(yield* clawPath(hi1, 0, 'front', 0.2))) return;
    // 4 · turn to the pallet and set P1 down beside the rack
    T.step('Zur Palette');
    if (!(yield* turnToRetry(east))) return;
    T.step('Ablegen');
    const pc = new V(P.x, P.h + p1.ob.obb.h.y, P.z);
    if (!(yield* frontPlace(p1, pc, fwd(east), 'auf der Palette'))) return;
    yield* stow();
    T.step('Prüfen');
    const inRack = Math.abs(p2.ob.obb.c.z - (fz - frontDepth(p2, fwd(north)))) < 0.05 && Math.abs(bottomOf(p2) - top1) < 0.01 && Math.abs(p2.ob.obb.c.x - (RK.x - 0.725)) < 0.06;
    const onPallet = Math.abs(bottomOf(p1) - P.h) < 0.01 && Math.abs(p1.ob.obb.c.x - P.x) < 0.25 && Math.abs(p1.ob.obb.c.z - P.z) < 0.35;
    S.rackCheck = { inRack, onPallet };
    L.add(`Regal-Demo abgeschlossen — P2 im Fach: ${inRack ? 'ja' : 'NEIN'}, P1 auf der Palette: ${onPallet ? 'ja' : 'NEIN'}`, inRack && onPallet ? 'ok' : 'warn');
  }
  // half depth of a parcel along the approach direction
  function frontDepth(d, dir) {
    const ax = d.ob.obb.axes();
    return Math.abs(ax[0].dot(dir)) * d.ob.obb.h.x + Math.abs(ax[2].dot(dir)) * d.ob.obb.h.z;
  }

  // ------------------------------------------------------------------ survivor scan
  function* survivorScan() {
    T.step('Schwenk');
    if (!S.toggles.thermal || !S.toggles.lidar) { S.toggles.thermal = S.toggles.lidar = true; S.emit('toggles'); L.add('Wärmebild + Lidar für den Überlebenden-Scan eingeschaltet'); }
    Object.assign(S.scan, { active: true, progress: 0, result: 'SCANNE…', located: false });
    L.add('Überlebenden-Scan gestartet — Wärmeschwenk ±70°', 'info');
    let best = { p: 0 }, bestYaw = 0;
    const dur = 4.5;
    for (let t = 0; t < dur; ) {
      const dt = yield;
      t += dt || 0.016;
      S.scan.progress = Math.min(100, (t / dur) * 100);
      S.cmd.headYaw = Math.sin((t / dur) * Math.PI * 2) * 70; S.cmd.headPitch = 2; S.emit('cmd');
      const e = A.Sensors.evaluateSurvivor();
      if (e.p > best.p) { best = e; bestYaw = S.cmd.headYaw; }
    }
    T.step('Auswertung');
    const sv = A.Env.survivor;
    if (best.p > 0.5) {
      lookAt(sv.pos);
      Object.assign(S.scan, { active: false, progress: 100, prob: best.p, located: true, result: `ÜBERLEBENDER WAHRSCHEINLICH — ${(best.p * 100).toFixed(0)} %` });
      L.add(`Möglicher Überlebender geortet — ${(best.p * 100).toFixed(0)} % Konfidenz, ${best.dist.toFixed(1)} m, ${best.occl} verdeckende Schicht(en)`, 'alert');
      A.Env.highlight('survivor');
    } else {
      S.cmd.headYaw = bestYaw; S.emit('cmd');
      Object.assign(S.scan, { active: false, progress: 100, prob: best.p, located: false, result: `KEINE BESTÄTIGTE SIGNATUR (${(best.p * 100).toFixed(0)} %)` });
      L.add(`Überlebenden-Scan beendet — keine bestätigte Signatur (${(best.p * 100).toFixed(0)} %). Näher heran / Richtung Tunnel ausrichten.`, 'warn');
    }
  }
  function markTarget() {
    let p;
    if (S.scan.located) p = A.Env.survivor.pos.clone();
    else {
      const hc = R.headCenter(new V()), dir = A.Sensors.headDir(new V());
      const h = W.raycast(hc, dir, 20);
      if (!h) { L.add('Ziel markieren: keine Fläche in Sensorrichtung', 'warn'); return; }
      p = h.point;
    }
    S.marker = { x: p.x, y: p.y, z: p.z, t: L.time() };
    A.Main.placeMarker(p);
    L.add(`Ziel markiert bei (${p.x.toFixed(1)}, ${p.z.toFixed(1)}) — Bake an Einsatzleitung gemeldet`, 'ok');
  }

  // ------------------------------------------------------------------ failure test + auto stabilize
  function* failureTest() {
    T.step('Fehler einspeisen');
    A.Env.highlight('start');
    S.fault = { leg: 'ML', severity: 0, t: 0 };
    L.add('SYSTEMFEHLER-TEST — Aktuatorfehler wird eingespeist', 'alert');
    yield 0.4;
    L.add('Warnung: Drehmomentspitze Bein Mitte links (ML-Knie 182 % Nennwert)', 'err');
    T.step('Verschlechterung');
    for (let t = 0; t < 2.0; ) { const dt = yield; t += dt || 0.016; S.fault.severity = Math.min(1, t / 1.6); }
    L.add('Motortemperatur steigt — Antrieb ML 78 °C', 'warn');
    L.add('Druckabfall im linken Hydraulikkreis erkannt', 'warn');
    T.step('Instabile Bewegung');
    A.Loco.setMode('rotate', { dir: 1 });
    yield 3.2;
    A.Loco.stop();
    L.add('Stabilitätsrand verringert — Gang auf 60 % begrenzt', 'warn');
    T.step('Warten auf Stabilisierung');
    L.add('AUTO-STABILISIERUNG verfügbar — Bediener muss auslösen', 'alert');
    S.emit('fault');
  }
  function* autoStabilize() {
    if (!S.fault) { L.add('Kein aktiver Fehler — Systeme normal'); return; }
    T.step('Isolieren');
    L.add('Auto-Stabilisierung aktiv — Aktuator ML isoliert, Last wird umverteilt', 'info');
    A.Loco.stop();
    T.step('Ausbalancieren');
    setPose('rescue');
    const f = S.fault;
    for (let t = 0; t < 2.6; ) { const dt = yield; t += dt || 0.016; f.severity = Math.max(0, 1 - t / 2.2); }
    T.step('Neu aufsetzen');
    yield* settle(4);
    const ml = R.legById[f.leg];
    A.Loco.resetLeg(f.leg);
    yield* T.until(() => ml.mode === 'stance', 3);
    S.fault = null; S.emit('fault');
    for (const l of R.legs) l.errT = 0;
    T.step('Bestätigen');
    L.add('Stabilisierung bestätigt — alle sechs Beine normal, Körper waagerecht', 'ok');
  }

  // ------------------------------------------------------------------ climb demo
  function* climbDemo() {
    T.step('Hinfahren');
    A.Env.highlight('climb');
    setPose('zero');
    yield* settle(3);
    L.add('Kletter-Demo — Rampe 0,30 m, Plateau, Stufe abwärts, Schuttplatten');
    const ok = yield* goTo({ x: 0, z: 1.6 }, -90 * D, 'Parcoursstart');
    if (!ok) { L.add('Kletterparcours von hier aus nicht erreichbar', 'warn'); return; }
    T.step('Klettern');
    S.toggles.lidar = true; S.emit('toggles');
    const ok2 = yield* goTo({ x: 0, z: 11.6 }, -90 * D, 'Parcoursende');
    T.step('Bericht');
    L.add(ok2 ? 'Kletterparcours bewältigt — Füße ans Gelände angepasst, Körper an der Stützebene ausgerichtet' : 'Klettern von Sicherheitsprüfung gestoppt', ok2 ? 'ok' : 'warn');
  }

  // task aborted (operator, or superseded by a new task): stop walking; an empty claw opens and keeps its finger
  // exception for the object between the fingers until it is fully open (so it can slide off instead of jamming)
  function abortCleanup() {
    A.Loco.stop();
    const arm = R.arm;
    if (!arm.held && arm.graspTarget) { arm.target.gap = C.arm.maxGap; arm.clearGraspOnOpen = arm.graspTarget; }
  }

  // ------------------------------------------------------------------ missions
  const MISSIONS = {
    tunnel: {
      name: 'TUNNELINSPEKTION', steps: ['Konfigurieren', 'Kriechpose', 'Anfahrt', 'Einfahrt', 'Inspektion'],
      *gen() {
        T.step('Konfigurieren'); A.Env.highlight('tunnel');
        Object.assign(S.toggles, { lidar: true, light: true, thermal: false }); S.cmd.mastHeight = 0; S.cmd.headPitch = 0; S.cmd.headYaw = 0; S.emit('toggles'); S.emit('cmd');
        L.add('Mission: Tunnelinspektion — Durchfahrt 1,0 m, Kriechpose erforderlich');
        T.step('Kriechpose'); setPose('crawl'); yield* settle(5);
        T.step('Anfahrt');
        if (!(yield* goTo({ x: 0, z: -3.6 }, 90 * D, 'Tunnelportal'))) return;
        T.step('Einfahrt');
        L.add('Einfahrt in Tunnel B-2 — Deckenabstand wird überwacht');
        if (!(yield* goTo({ x: 0, z: -8.0 }, 90 * D, 'Tunnelinneres'))) return;
        T.step('Inspektion');
        for (const y of [-50, 50, 0]) { S.cmd.headYaw = y; S.emit('cmd'); yield 1.4; }
        L.add('Tunnelinspektion abgeschlossen — Strukturscan protokolliert', 'ok');
      },
    },
    warehouse: {
      name: 'EINGESTÜRZTE LAGERHALLE', steps: ['Konfigurieren', 'Hohe Bodenfreiheit', 'Durchqueren', 'Erkundung'],
      *gen() {
        T.step('Konfigurieren'); A.Env.highlight('warehouse');
        Object.assign(S.toggles, { lidar: true, light: false, thermal: false }); S.emit('toggles');
        L.add('Mission: Eingestürzte Lagerhalle — umgestürzter und schräger Träger im Gang');
        T.step('Hohe Bodenfreiheit'); setPose('high'); S.cmd.mastHeight = 0.0; S.emit('cmd'); yield* settle(5);
        T.step('Durchqueren');
        if (!(yield* goTo({ x: 8.0, z: -0.6 }, 0, 'Hallengang'))) return;
        T.step('Erkundung');
        S.cmd.mastHeight = 0.9; S.emit('cmd');
        for (const y of [-70, 70, 0]) { S.cmd.headYaw = y; S.emit('cmd'); yield 1.5; }
        L.add('Hallenerkundung abgeschlossen — Einsturzrisiko der Regale: HOCH', 'ok');
      },
    },
    survivor: {
      name: 'ÜBERLEBENDENSUCHE', steps: ['Konfigurieren', 'Anfahrt', 'Schwenk', 'Auswertung', 'Markieren'],
      *gen() {
        T.step('Konfigurieren'); A.Env.highlight('tunnel');
        Object.assign(S.toggles, { thermal: true, lidar: true, light: false }); S.emit('toggles');
        setPose('rescue'); S.cmd.mastHeight = 0.6; S.emit('cmd');
        L.add('Mission: Überlebendensuche — Fusion aus Wärmebild und Lidar');
        yield* settle(4);
        T.step('Anfahrt');
        const d = Math.hypot(R.pose.x, R.pose.z + 3.2);
        if (d > 0.6 && !(R.pose.z < -5)) { if (!(yield* goTo({ x: 0, z: -3.2 }, 90 * D, 'Tunnelportal'))) return; }
        yield* survivorScan();
        if (S.scan.located) { T.step('Markieren'); markTarget(); }
      },
    },
    debris: { name: 'TRÜMMERRÄUMUNG', steps: ['Orten', 'Anfahrt', 'Vorgreifen', 'Absenken', 'Greifen', 'Anheben', 'Umsetzen', 'Absetzen', 'Loslassen', 'Verstauen'], *gen() { setPose('rescue'); yield* settle(3); yield* grabDemo(); } },
    failure: { name: 'SYSTEMFEHLER-TEST', steps: ['Fehler einspeisen', 'Verschlechterung', 'Instabile Bewegung', 'Warten auf Stabilisierung'], *gen() { yield* failureTest(); } },
  };

  A.Missions = {
    init(robot) { R = robot; },
    defs: MISSIONS,
    run(id) {
      const m = MISSIONS[id];
      if (!S.powered) return A.feedback('Roboter abgeschaltet — zuerst neu starten', 'warn');
      S.mission = { id, name: m.name, zone: S.mission.zone };
      S.emit('mission');
      T.start(m.name, m.gen(), { steps: m.steps, onCancel: abortCleanup });
    },
    task(name, gen, steps) {
      if (!S.powered && name !== 'NEUSTART') return A.feedback('Roboter abgeschaltet — zuerst neu starten', 'warn');
      T.start(name, gen, { steps, onCancel: abortCleanup });
    },
    grab() { this.task('GREIF-DEMO', grabDemo(), MISSIONS.debris.steps); },
    beams() { this.task('TRÄGERRÄUMUNG', beamDemo(), ['Anfahrt', 'Greifen', 'Anheben', 'Wenden', 'Absetzen', 'Loslassen', 'Prüfen']); },
    rack() { this.task('REGAL-DEMO', rackDemo(), ['Anfahrt', 'Palette greifen', 'Zum Regal', 'Einlagern', 'Auslagern', 'Zur Palette', 'Ablegen', 'Prüfen']); },
    stackReport,
    release() { if (releasePayload()) { R.arm.target.gap = C.arm.maxGap; } else A.feedback('Greifer ist leer'); },
    scan() { this.task('ÜBERLEBENDEN-SCAN', survivorScan(), ['Schwenk', 'Auswertung']); },
    mark: markTarget,
    climb() { this.task('KLETTER-DEMO', climbDemo(), ['Hinfahren', 'Klettern', 'Bericht']); },
    stabilize() { this.task('AUTO-STABILISIERUNG', autoStabilize(), ['Isolieren', 'Ausbalancieren', 'Neu aufsetzen', 'Bestätigen']); },
    setPose(name) {
      setPose(name);
      L.add(`Pose: ${{ crawl: 'Kriechpose', high: 'Hohe Bodenfreiheit', rescue: 'Stabile Rettungspose', zero: 'Nullpose' }[name]} (h=${C.poses[name].h.toFixed(2)} m)`);
    },
    zero() {
      T.cancel('zero');
      setPose('zero');
      Object.assign(S.cmd, { headYaw: 0, headPitch: 0, mastHeight: 0 });
      if (!R.arm.held) R.arm.target.gap = 0.06;
      stowArm();
      for (const l of R.legs) if (l.mode === 'manual') A.Loco.plantLeg(l.id, true);
      L.add('Nullpose befohlen');
    },
    patrol() {
      const self = this;
      function* gen() {
        S.toggles.lidar = true; S.emit('toggles');
        L.add('Autonome Patrouille aktiv — Planer wählt kollisionsfreie Wegpunkte');
        const pts = [{ x: -3.4, z: 2.6 }, { x: 3.2, z: -2.4 }, { x: 0.5, z: 1.5 }, { x: -2.8, z: -3.0 }, { x: 2.8, z: 2.8 }];
        for (let i = 0; S.toggles.autonomous; i = (i + 1) % pts.length) {
          T.step('Patrouille');
          const p = pts[i];
          lookAt(new V(p.x, 0.5, p.z));
          yield* goTo(p, null, `Wegpunkt ${i + 1}`);
          T.step('Scan');
          yield 1.2;
        }
      }
      self.task('AUTONOME PATROUILLE', gen(), ['Patrouille', 'Scan']);
    },
    shutdown() {
      T.cancel('shutdown'); A.Loco.stop();
      S.powered = false;
      Object.assign(S.toggles, { lidar: false, thermal: false, light: false }); S.emit('toggles');
      A.DroneSys.emergencyLand();
      L.add('NOT-AUS — Aktuatoren stromlos, Körper senkt sich ab', 'err');
      S.mission = { id: null, name: 'NOT-AUS', zone: null }; S.emit('mission'); S.emit('power');
    },
    reboot() {
      if (S.powered && !S.booting) return A.feedback('Systeme laufen bereits');
      function* boot() {
        S.booting = true; S.emit('power');
        T.step('Bootloader'); L.add('Neustart: Bootloader v6.2 — Speichertest OK'); yield 0.6;
        T.step('IMU'); L.add('IMU-Kalibrierung … Kreiseldrift 0,02°/s'); yield 0.6;
        T.step('Beinsteuerungen');
        for (const l of R.legs) { L.add(`Beinsteuerung ${A.LN(l.id)} bereit`); l.errT = 0; yield 0.18; }
        T.step('Sensoren'); L.add('Sensorbus bereit — Kopf, Mast, Lidar, Wärmebild'); yield 0.5;
        S.powered = true; S.booting = false; S.cmd.bodyHeight = C.height.def; S.emit('cmd'); S.emit('power');
        S.mission = { id: null, name: 'BEREITSCHAFT', zone: null }; S.emit('mission');
        L.add('ARES-6 Systeme bereit — Körper hebt sich', 'ok');
        yield* settle(4);
      }
      T.start('NEUSTART', boot(), { steps: ['Bootloader', 'IMU', 'Beinsteuerungen', 'Sensoren'] });
    },
    releasePayload,
  };
})();
