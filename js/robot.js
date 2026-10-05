/* ARES-6 — robot builder, pose application and the collision validator (try → validate → commit/revert) */
(function () {
  'use strict';
  const A = window.ARES, C = A.CFG, U = A.U, W = A.World, LC = C.leg, D = A.DEG;
  const V = THREE.Vector3;
  const _a = new V(), _b = new V(), _p = new V();

  function materials() {
    return {
      armor: new THREE.MeshStandardMaterial({ color: 0x2b3036, roughness: 0.42, metalness: 0.65 }),
      dark: new THREE.MeshStandardMaterial({ color: 0x15181b, roughness: 0.6, metalness: 0.5 }),
      joint: new THREE.MeshStandardMaterial({ color: 0x8b939c, roughness: 0.28, metalness: 0.92 }),
      chrome: new THREE.MeshStandardMaterial({ color: 0xd8dde2, roughness: 0.12, metalness: 1 }),
      yellow: new THREE.MeshStandardMaterial({ color: 0xf2b705, roughness: 0.45, metalness: 0.25 }),
      hazard: new THREE.MeshStandardMaterial({ map: A.hazardTex(1), roughness: 0.5, metalness: 0.2 }),
      rubber: new THREE.MeshStandardMaterial({ color: 0x0c0c0c, roughness: 0.95 }),
      lens: new THREE.MeshStandardMaterial({ color: 0x0a1a24, roughness: 0.05, metalness: 0.9, emissive: 0x0b3a52, emissiveIntensity: 0.8 }),
      glowCyan: new THREE.MeshBasicMaterial({ color: 0x29d3ff }),
      glowRed: new THREE.MeshBasicMaterial({ color: 0xff3030 }),
    };
  }

  class Robot {
    constructor(scene) {
      this.scene = scene;
      this.M = materials();
      this.pose = { x: 0, z: 0, yaw: 0, gy: 0, h: C.height.def, pitch: 0, roll: 0 };
      this.head = { yaw: 0, pitch: 0 };
      this.mastExt = 0;
      this.t = 0;
      this.build();
      this.reset();
    }

    // ------------------------------------------------------------------ construction
    build() {
      const M = this.M;
      const add = (p, geo, mat, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = true; m.receiveShadow = true; p.add(m); return m; };
      const root = (this.root = new THREE.Group()); root.name = 'robotRoot'; this.scene.add(root);
      const bp = (this.bodyPivot = new THREE.Group()); bp.name = 'bodyPivot'; root.add(bp);
      const body = (this.body = new THREE.Group()); body.name = 'body'; bp.add(body);
      const B = C.body;
      // chassis: layered armor
      this.chassis = add(body, new THREE.BoxGeometry(B.L, B.H * 0.7, B.W), M.armor);
      add(body, new THREE.BoxGeometry(B.L - 0.12, B.H * 0.3, B.W - 0.1), M.dark, 0, B.H * 0.35 - 0.001, 0);
      add(body, new THREE.BoxGeometry(B.L - 0.3, 0.02, B.W - 0.22), M.armor, 0, B.H / 2 - 0.01, 0);
      add(body, new THREE.BoxGeometry(B.L - 0.1, 0.06, B.W - 0.06), M.dark, 0, -B.H / 2 + 0.03, 0);
      for (const s of [-1, 1]) {
        add(body, new THREE.BoxGeometry(B.L * 0.82, 0.07, 0.012), M.hazard, 0, 0.02, s * (B.W / 2 + 0.002));
        add(body, new THREE.BoxGeometry(0.06, 0.12, 0.012), M.yellow, 0.62, 0.0, s * (B.W / 2 + 0.003));
      }
      add(body, new THREE.BoxGeometry(0.03, 0.16, B.W - 0.1), M.yellow, B.L / 2 + 0.005, -0.02, 0);   // front bumper
      add(body, new THREE.BoxGeometry(0.22, 0.012, 0.14), M.hazard, 0.25, B.H / 2 + 0.006, 0.25);
      // rear battery / power unit
      const bat = add(body, new THREE.BoxGeometry(0.26, 0.24, 0.6), M.dark, -0.86, -0.01, 0);
      add(bat, new THREE.BoxGeometry(0.02, 0.18, 0.5), M.yellow, -0.135, 0, 0);
      for (let i = 0; i < 4; i++) add(bat, new THREE.BoxGeometry(0.2, 0.012, 0.56), M.joint, 0, -0.08 + i * 0.05, 0);
      // status LEDs (front + rear), warning beacon on deck
      this.leds = [];
      for (const [x, z] of [[0.755, -0.3], [0.755, 0.3], [-0.995, -0.22], [-0.995, 0.22], [0.2, -0.413], [0.2, 0.413], [-0.4, -0.413], [-0.4, 0.413]]) {
        const m = new THREE.MeshBasicMaterial({ color: 0x29d3ff });
        const l = add(body, new THREE.SphereGeometry(0.022, 10, 8), m, x, 0.06, z); l.castShadow = false; this.leds.push(l);
      }
      this.beaconMat = new THREE.MeshStandardMaterial({ color: 0x331a00, emissive: 0xff7a00, emissiveIntensity: 0 });
      this.beacon = add(body, new THREE.CylinderGeometry(0.045, 0.05, 0.07, 14), this.beaconMat, -0.2, B.H / 2 + 0.035, -0.28);
      this.beaconLight = new THREE.PointLight(0xff5a00, 0, 3.5, 2); this.beaconLight.position.set(-0.2, B.H / 2 + 0.15, -0.28); body.add(this.beaconLight);

      // sensor head
      const hp = (this.headYawJ = new THREE.Group()); hp.name = 'sensorHead'; hp.position.fromArray(C.head.pos); body.add(hp);
      add(hp, new THREE.CylinderGeometry(0.1, 0.12, 0.06, 18), M.joint, 0, 0.03, 0);
      const hpp = (this.headPitchJ = new THREE.Group()); hpp.position.y = 0.13; hp.add(hpp);
      add(hpp, new THREE.BoxGeometry(0.3, 0.17, 0.36), M.armor);
      add(hpp, new THREE.BoxGeometry(0.302, 0.03, 0.362), M.yellow, 0, 0.055, 0);
      const lens = add(hpp, new THREE.CylinderGeometry(0.055, 0.06, 0.04, 20), M.lens, 0.16, 0, 0); lens.rotation.z = Math.PI / 2;
      add(hpp, new THREE.TorusGeometry(0.062, 0.008, 6, 20), M.joint, 0.18, 0, 0).rotation.y = Math.PI / 2;
      // thermal sensor (left), search light (right), lidar puck (top)
      this.thermalMat = new THREE.MeshStandardMaterial({ color: 0x220800, emissive: 0xff4a00, emissiveIntensity: 0.15 });
      const th = add(hpp, new THREE.BoxGeometry(0.04, 0.06, 0.07), this.thermalMat, 0.15, -0.02, -0.12);
      th.name = 'thermalSensor';
      this.lightMat = new THREE.MeshStandardMaterial({ color: 0x333333, emissive: 0xfff2d0, emissiveIntensity: 0.0 });
      const sl = add(hpp, new THREE.CylinderGeometry(0.04, 0.035, 0.05, 16), this.lightMat, 0.15, -0.02, 0.12); sl.rotation.z = Math.PI / 2;
      const lp = (this.lidarPuck = new THREE.Group()); lp.position.y = 0.1; hpp.add(lp);
      add(lp, new THREE.CylinderGeometry(0.05, 0.055, 0.05, 18), M.dark);
      this.lidarRing = add(lp, new THREE.CylinderGeometry(0.052, 0.052, 0.012, 18), new THREE.MeshBasicMaterial({ color: 0x0d4a5c }));
      this.camAnchor = new THREE.Object3D(); this.camAnchor.position.set(0.19, 0, 0); hpp.add(this.camAnchor);
      this.thermalAnchor = new THREE.Object3D(); this.thermalAnchor.position.set(0.18, -0.02, -0.12); hpp.add(this.thermalAnchor);
      this.lightAnchor = new THREE.Object3D(); this.lightAnchor.position.set(0.18, -0.02, 0.12); hpp.add(this.lightAnchor);
      this.lidarAnchor = new THREE.Object3D(); this.lidarAnchor.position.set(0, 0.12, 0); hpp.add(this.lidarAnchor);

      // telescopic camera mast
      const mp = (this.mast = new THREE.Group()); mp.name = 'mast'; mp.position.fromArray(C.mast.pos); body.add(mp);
      add(mp, new THREE.CylinderGeometry(0.065, 0.075, 0.08, 16), M.joint, 0, 0.04, 0);
      this.mastSegs = [0.05, 0.04, 0.03].map((r, i) => add(mp, new THREE.CylinderGeometry(r, r, C.mast.seg, 14), i === 1 ? M.yellow : M.chrome, 0, 0, 0));
      const mc = (this.mastCam = new THREE.Group()); mp.add(mc);
      add(mc, new THREE.BoxGeometry(0.12, 0.08, 0.1), M.armor, 0, 0.04, 0);
      const ml = add(mc, new THREE.CylinderGeometry(0.025, 0.025, 0.03, 12), M.lens, 0.07, 0.04, 0); ml.rotation.z = Math.PI / 2;
      this.mastLed = add(mc, new THREE.SphereGeometry(0.012, 8, 6), new THREE.MeshBasicMaterial({ color: 0xff3030 }), 0, 0.09, 0);

      // drone dock
      const dk = (this.dock = new THREE.Group()); dk.name = 'droneDock'; dk.position.fromArray(C.dock.pos); body.add(dk);
      add(dk, new THREE.CylinderGeometry(0.2, 0.22, 0.03, 24), M.dark, 0, 0.015, 0);
      add(dk, new THREE.TorusGeometry(0.19, 0.008, 6, 28), M.yellow, 0, 0.032, 0).rotation.x = Math.PI / 2;
      this.dockAnchor = new THREE.Object3D(); this.dockAnchor.position.y = 0.035; dk.add(this.dockAnchor);

      // legs + arm
      this.legs = C.legs.map((d) => new A.Leg(d, M, body));
      this.legById = Object.fromEntries(this.legs.map((l) => [l.id, l]));
      this.arm = new A.Arm(M, body);

      // collision hull of the chassis (body-local)
      this.hull = {
        parts: C.hull.parts.map((p) => {
          const obb = new A.OBB(new V(), new V().fromArray(p.h)), samples = obb.samples(0.24);
          return { obb, local: new V().fromArray(p.c), samples, bottom: samples.filter((s) => Math.abs(s.y + p.h[1]) < 1e-6) };
        }),
        distance(p) { let d = Infinity; for (const x of this.parts) d = Math.min(d, x.obb.distance(p)); return d; },
      };
      this.bodyAxes = new THREE.AxesHelper(0.6); this.bodyAxes.visible = false; body.add(this.bodyAxes);
    }

    reset() {
      Object.assign(this.pose, { x: 0, z: 0, yaw: 0, gy: 0, h: C.height.def, pitch: 0, roll: 0 });
      this.head.yaw = this.head.pitch = 0; this.mastExt = 0;
      if (this.arm.held) this.arm.held = null;
      this.arm.apply({ yaw: 0, shoulder: 80 * D, elbow: -150 * D, wristPitch: 60 * D, wristRoll: 0, gap: 0.06 });
      Object.assign(this.arm.target, this.arm.angles);
      this.applyHeadMast();
      this.applyTransform();
      for (const l of this.legs) {
        l.mode = 'stance'; l.swing = null; l.errT = 0;
        const n = this.neutralWorld(l, this.pose, 1);
        l.foot.copy(this.contactAt(n.x, n.z, 1));
      }
      this.applyPose(this.pose);
    }

    // ------------------------------------------------------------------ kinematic helpers
    // neutral pad position (body-local xz) for the current height and stance scale
    neutralLocal(l, h, stance) {
      const d0 = U.clamp(0.85 + (0.6 - h) * 0.5, 0.62, 1.04) * stance;
      const m = l.def.mount, ya = l.def.yaw, R = LC.coxa + d0;
      return { x: m[0] + Math.cos(ya) * R, z: m[2] - Math.sin(ya) * R };
    }
    toWorldXZ(pose, lx, lz) {
      const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
      return { x: pose.x + lx * c + lz * s, z: pose.z - lx * s + lz * c };
    }
    toLocalXZ(pose, wx, wz) {
      const dx = wx - pose.x, dz = wz - pose.z, c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
      return { x: dx * c - dz * s, z: dx * s + dz * c };
    }
    neutralWorld(l, pose, stance) {
      const n = this.neutralLocal(l, A.State.cmd.bodyHeight, stance || A.State.cmd.stance);
      return this.toWorldXZ(pose, n.x, n.z);
    }
    // contact point for a flat pad at (x,z): rests on the highest surface under its rim
    contactAt(x, z, yRef, ex) {
      const yMax = (yRef === undefined ? 0 : yRef) + LC.stepUp;
      let y = 0, ob = null;
      for (const [dx, dz] of [[0, 0], [0.09, 0], [-0.09, 0], [0, 0.09], [0, -0.09]]) {
        const s = W.surfaceBelow(x + dx, z + dz, yMax, ex);
        if (s.y > y) { y = s.y; ob = s.ob; }
      }
      const p = new V(x, y, z); p.support = ob; return p;
    }

    applyTransform() {
      const p = this.pose;
      this.root.position.set(p.x, p.gy, p.z); this.root.rotation.y = p.yaw;
      this.bodyPivot.position.y = p.h; this.bodyPivot.rotation.set(p.roll, 0, p.pitch);
      this.root.updateMatrixWorld(true);
    }
    // set pose, solve IK of all non-manual legs
    applyPose(p) {
      if (p !== this.pose) Object.assign(this.pose, p);
      this.applyTransform();
      for (const l of this.legs) {
        if (l.mode === 'manual') { l.ikFail = null; continue; }
        const r = l.solveIK(A.Leg.ankleFor(l.foot, _a));
        l.ikFail = r.ok ? null : r.why;
        l.setAngles(r.a);
      }
      this.root.updateMatrixWorld(true);
      this.levelFeet();
    }
    levelFeet() {
      for (const l of this.legs) l.levelFoot(this.root.quaternion);
      this.root.updateMatrixWorld(true);
    }
    applyHeadMast() {
      this.headYawJ.rotation.y = this.head.yaw; this.headPitchJ.rotation.z = this.head.pitch;
      const e = this.mastExt, s = C.mast.seg;
      this.mastSegs.forEach((m, i) => { m.position.y = 0.08 + s / 2 + (e * (i + 1)) / 3 - i * 0.0; });
      this.mastCam.position.y = 0.08 + s + e;
    }

    snapshot() {
      return {
        pose: { ...this.pose }, head: { ...this.head }, mast: this.mastExt, arm: { ...this.arm.angles },
        legs: this.legs.map((l) => ({ a: { ...l.angles }, foot: l.foot.clone(), ik: l.ikFail })),
      };
    }
    restore(s) {
      Object.assign(this.pose, s.pose); Object.assign(this.head, s.head); this.mastExt = s.mast;
      this.arm.apply(s.arm); this.applyHeadMast();
      this.legs.forEach((l, i) => { l.setAngles(s.legs[i].a); l.foot.copy(s.legs[i].foot); l.ikFail = s.legs[i].ik; });
      this.applyTransform();
      this.levelFeet();
    }

    hullOBB() {
      const q = new THREE.Quaternion(); this.body.getWorldQuaternion(q);
      for (const x of this.hull.parts) x.obb.set(this.body.localToWorld(_p.copy(x.local)), q);
      return this.hull;
    }
    headCenter(out) { return this.headPitchJ.getWorldPosition(out); }
    mastPoints() {
      const base = this.mast.localToWorld(new V(0, 0.08, 0)), top = this.mastCam.localToWorld(new V(0, 0.06, 0));
      return { base, top };
    }
    dockCenter(out) { return this.dockAnchor.localToWorld(out.set(0, 0.07, 0)); }

    // ------------------------------------------------------------------ the validator
    // parts: subset of ['body','legs','arm','head','mast','dock'] (default: everything)
    validate(parts) {
      const P = parts ? new Set(parts) : null, has = (k) => !P || P.has(k);
      A.State.stats.checks++;
      const fail = (reason, ob, point, part) => ({ ok: false, reason, ob, point, part });
      const held = this.arm.held ? this.arm.held.ob : null;
      if (held) { this.arm.wristR.updateWorldMatrix(true, false); this.syncHeld(); } // the load moves with every body/arm pose
      const hull = this.hullOBB();
      const droneDocked = A.DroneSys && A.DroneSys.state === 'docked';

      if (has('legs')) for (const l of this.legs) if (l.ikFail) return fail(`Bein ${A.LN(l.id)}: ${l.ikFail}`, null, null, 'leg:' + l.id);

      if (has('body')) {
        for (const part of hull.parts) {
          const h = W.obbHit(part.obb, part.samples, held, 0.0);
          if (h) return fail(`Chassis-Kontakt: ${h.ob.name}`, h.ob, h.point, 'body');
          for (const s of part.bottom) {
            part.obb.toWorld(s, _p);
            const g = W.surfaceBelow(_p.x, _p.z, _p.y + 0.05);
            if (_p.y < g.y + 0.03) return fail('Chassis setzt auf', g.ob || 'floor', _p.clone(), 'body');
          }
        }
      }
      if (has('head')) {
        const hc = this.headCenter(_a);
        const o = W.sphereHit(hc, 0.25, held);
        if (o) return fail(`Sensorkopf-Kontakt: ${o.name}`, o, hc.clone(), 'head');
      }
      if (has('mast')) {
        const mp = this.mastPoints();
        const h = W.capsuleHit(mp.base, mp.top, 0.07, held);
        if (h) return fail(`Kameramast-Kontakt: ${h.ob.name}`, h.ob, h.point, 'mast');
      }
      if (has('dock') && droneDocked) {
        const dc = this.dockCenter(_a), o = W.sphereHit(dc, 0.3, held);
        if (o) return fail(`Angedockte Drohne berührt: ${o.name}`, o, dc.clone(), 'dock');
      }

      // legs: environment, floor, chassis, neighbours
      if (has('legs') || has('arm')) for (const l of this.legs) l.points();
      if (has('legs')) {
        for (const l of this.legs) { const r = this.legCheck(l, hull, held); if (r) return r; }
        for (const [ia, ib] of C.adjacent) { const r = this.pairCheck(ia, ib); if (r) return r; }
      }

      if (has('arm')) {
        const r = this.validateArm(hull, held);
        if (!r.ok) return r;
      } else if (held) {
        const s = this.loadSelfCheck(held, hull);
        if (s) return fail(s.reason, null, s.point, 'load');
      }
      return { ok: true };
    }

    // one leg vs environment, floor and chassis (points must be current)
    legCheck(l, hull, held) {
      const fail = (reason, ob, point) => ({ ok: false, reason, ob, point, part: 'leg:' + l.id });
      const p = l.pts;
      const h = W.capsuleHit(p.hip, p.knee, LC.rUpper, held) || W.capsuleHit(p.knee, p.ankle, LC.rLower, held);
      if (h) return fail(`Bein ${A.LN(l.id)} berührt ${h.ob.name}`, h.ob, h.point);
      // pad: centre + rim points just above the contact plane
      for (const [dx, dz] of [[0, 0], [0.1, 0], [-0.1, 0], [0, 0.1], [0, -0.1]]) {
        _b.set(p.pad.x + dx, p.pad.y + 0.02, p.pad.z + dz);
        const o = W.sphereHit(_b, LC.rPad, held);
        if (o) return fail(`Fuß ${A.LN(l.id)} berührt ${o.name}`, o, _b.clone());
      }
      const ph = W.padHit(p.pad, held); // whole pad disk incl. rim (the 5 points above miss diagonal edges)
      if (ph) return fail(`Fuß ${A.LN(l.id)} berührt ${ph.ob.name}`, ph.ob, ph.point);
      if (p.pad.y < -0.005) return fail(`Fuß ${A.LN(l.id)} unter dem Boden`, 'floor', p.pad.clone());
      // tilted pad (manual ankle): no rim point may sink into the supporting surface
      for (const q of p.rim) {
        const g = W.surfaceBelow(q.x, q.z, q.y + 0.05);
        if (q.y < g.y - 0.004) return fail(`Fußsohle ${A.LN(l.id)} kantet in ${g.ob ? g.ob.name : 'Boden'}`, g.ob || 'floor', q.clone());
      }
      if (p.knee.y < LC.rUpper || p.ankle.y < LC.rLower) return fail(`Bein ${A.LN(l.id)} schlägt am Boden an`, 'floor', p.knee.clone());
      // leg vs chassis (skip the part of the upper segment next to the hip)
      // near the hip the segment is slimmer (joint drum, bare beam; the piston starts at 30 %)
      for (let i = 0; i <= 10; i++) {
        _b.copy(p.hip).lerp(p.knee, i / 10);
        const r = i === 0 ? 0.105 : i < 3 ? 0.088 : LC.rUpper;
        if (hull.distance(_b) < r + 0.01) return fail(`Bein ${A.LN(l.id)} kollidiert mit Chassis`, null, _b.clone());
      }
      for (let i = 0; i <= 10; i++) { _b.copy(p.knee).lerp(p.ankle, i / 10); if (hull.distance(_b) < LC.rLower + 0.01) return fail(`Bein ${A.LN(l.id)} kollidiert mit Chassis`, null, _b.clone()); }
      return null;
    }
    pairCheck(ia, ib) {
      const a = this.legById[ia].pts, b = this.legById[ib].pts;
      const segs = (x) => [[x.hip, x.knee, LC.rUpper], [x.knee, x.ankle, LC.rLower]];
      for (const [a1, a2, ra] of segs(a)) for (const [b1, b2, rb] of segs(b)) {
        if (U.segSeg(a1, a2, b1, b2) < ra + rb + 0.02) return { ok: false, reason: `Beine ${A.LN(ia)}/${A.LN(ib)} kollidieren`, ob: null, point: a2.clone(), part: 'leg:' + ia };
      }
      if (a.pad.distanceTo(b.pad) < 0.24) return { ok: false, reason: `Füße ${A.LN(ia)}/${A.LN(ib)} zu nah`, ob: null, point: a.pad.clone(), part: 'leg:' + ia };
      return null;
    }
    // full check of a single leg incl. reach and neighbours (matrices must be current)
    singleLegCheck(l) {
      if (l.ikFail) return { ok: false, reason: `Bein ${A.LN(l.id)}: ${l.ikFail}`, part: 'leg:' + l.id };
      const hull = this.hullOBB(), held = this.arm.held ? this.arm.held.ob : null;
      if (held) { this.arm.wristR.updateWorldMatrix(true, false); this.syncHeld(); } // predicted pose (foothold planning)
      for (const x of this.legs) x.points();
      const r = this.legCheck(l, hull, held);
      if (r) return r;
      for (const [ia, ib] of C.adjacent) if (ia === l.id || ib === l.id) { const q = this.pairCheck(ia, ib); if (q) return q; }
      if (l.def.front === 1) { const q = this.validateArm(hull, held); if (!q.ok) return q; }
      else if (held) { const s = this.loadSelfCheck(held, hull); if (s) return { ok: false, reason: s.reason, point: s.point, part: 'leg:' + l.id }; }
      return { ok: true };
    }

    validateArm(hull, held) {
      const fail = (reason, ob, point) => ({ ok: false, reason, ob, point, part: 'arm' });
      const P = this.arm.points();
      const ex = new Set(); if (held) ex.add(held);
      const fex = new Set(ex); if (this.arm.graspTarget) fex.add(this.arm.graspTarget);
      const segs = [
        [P.shoulder, P.elbow, 0.07, ex, 0.18], [P.elbow, P.wrist, 0.058, ex, 0],
        [P.wrist, P.palm, 0.075, ex, 0], [P.baseL, P.tipL, 0.03, fex, 0], [P.baseR, P.tipR, 0.03, fex, 0],
      ];
      for (const [a, b, r, x, skip] of segs) {
        const h = W.capsuleHit(a, b, r, x);
        if (h) return fail(`Arm berührt ${h.ob.name}`, h.ob, h.point);
        for (let i = 0; i <= 8; i++) {
          _b.copy(a).lerp(b, i / 8);
          if (i / 8 * a.distanceTo(b) < skip) continue;
          const g = W.surfaceBelow(_b.x, _b.z, _b.y);
          if (_b.y - r < g.y - 0.002) return fail('Arm schlägt am Boden an', g.ob || 'floor', _b.clone());
          if (hull.distance(_b) < r + 0.008) return fail('Arm kollidiert mit Chassis', null, _b.clone());
        }
      }
      // shoulder drum swings with the arm yaw — must stay clear of the chassis front
      for (let i = 0; i <= 4; i++) { _b.copy(P.drumA).lerp(P.drumB, i / 4); if (hull.distance(_b) < 0.075 + 0.008) return fail('Armschulter kollidiert mit Chassis', null, _b.clone()); }
      const hc = this.headCenter(_a);
      for (const [a, b, r] of segs) if (U.segSeg(a, b, hc, hc) < r + 0.25) return fail('Arm kollidiert mit Sensorkopf', null, hc.clone());
      for (const id of ['FL', 'FR']) {
        const lp = this.legById[id].pts;
        for (const [a, b, r] of segs) {
          if (U.segSeg(a, b, lp.hip, lp.knee) < r + LC.rUpper + 0.02 || U.segSeg(a, b, lp.knee, lp.ankle) < r + LC.rLower + 0.02)
            return fail(`Arm kollidiert mit Bein ${A.LN(id)}`, null, b.clone());
        }
      }
      if (held) {
        // exact box test; 3 mm overlap tolerated so a load can be set down on its support
        const o = W.boxHit(held.obb, held, -0.003);
        if (o) return fail(`Last berührt ${o.name}`, o, held.obb.c.clone());
        for (const c of held.obb.corners()) {
          const g = W.surfaceBelow(c.x, c.z, c.y + 0.01, held);
          if (c.y < g.y - 0.004) return fail('Last schlägt am Boden an', g.ob || 'floor', c);
        }
        const s = this.loadSelfCheck(held, hull);
        if (s) return fail(s.reason, null, s.point);
      }
      return { ok: true };
    }
    // carried load vs the robot's own links: every link sampled along its length (a long beam can cross a leg
    // between its corners), chassis parts by exact box test
    loadSelfCheck(held, hull) {
      const o = held.obb, P = this.arm.points(), segs = [];
      for (const l of this.legs) {
        segs.push([l.pts.hip, l.pts.knee, LC.rUpper, `Bein ${A.LN(l.id)}`], [l.pts.knee, l.pts.ankle, LC.rLower, `Bein ${A.LN(l.id)}`]);
      }
      segs.push([P.shoulder, P.elbow, 0.07, 'Oberarm'], [P.elbow, P.wrist, 0.058, 'Unterarm']);
      const mp = this.mastPoints(); segs.push([mp.base, mp.top, 0.07, 'Kameramast']);
      const hc = this.headCenter(new V()); segs.push([hc, hc, 0.25, 'Sensorkopf']);
      if (A.DroneSys && A.DroneSys.state === 'docked') { const dc = this.dockCenter(new V()); segs.push([dc, dc, 0.3, 'Drohne']); }
      const R0 = o.h.length();
      for (const [a, b, r, name] of segs) {
        const len = a.distanceTo(b), n = Math.max(1, Math.ceil(len / (r * 0.6)));
        if (U.segSeg(a, b, o.c, o.c) > R0 + r + 0.02) continue; // far from the load
        for (let i = 0; i <= n; i++) {
          _b.copy(a).lerp(b, i / n);
          if (o.distance(_b) < r + 0.01) return { reason: `Last kollidiert mit ${name}`, point: _b.clone() };
        }
      }
      for (const part of hull.parts) if (o.intersects(part.obb, 0.015)) return { reason: 'Last kollidiert mit Chassis', point: o.c.clone() };
      return null;
    }

    // try a full pose; commit when valid, otherwise revert
    tryPose(p, parts) {
      const s = this.snapshot();
      this.applyPose(p);
      const r = this.validate(parts);
      if (!r.ok) this.restore(s);
      return r;
    }

    // ------------------------------------------------------------------ appendages (head, mast, arm) with validation
    updateAppendages(dt) {
      const S = A.State, cmd = S.cmd, pw = S.powered;
      // sensor head
      const hy = U.clamp(cmd.headYaw, -120, 120) * D, hpi = U.clamp(cmd.headPitch, -30, 45) * D;
      const ny = U.approach(this.head.yaw, pw ? hy : this.head.yaw, 2.2 * dt), np = U.approach(this.head.pitch, pw ? hpi : -25 * D, 1.5 * dt);
      if (ny !== this.head.yaw || np !== this.head.pitch) {
        const s = this.snapshot();
        this.head.yaw = ny; this.head.pitch = np; this.applyHeadMast(); this.root.updateMatrixWorld(true);
        const r = this.validate(['head', 'arm']);
        if (!r.ok) { this.restore(s); this.blocked('head', r); }
      }
      // mast
      const mt = pw ? U.clamp(cmd.mastHeight, 0, C.mast.max) : 0;
      const ne = U.approach(this.mastExt, mt, 0.6 * dt);
      if (ne !== this.mastExt) {
        const s = this.snapshot();
        this.mastExt = ne; this.applyHeadMast(); this.root.updateMatrixWorld(true);
        const r = this.validate(['mast']);
        if (!r.ok) { this.restore(s); this.blocked('mast', r); if (ne > s.mast) A.State.cmd.mastHeight = Math.max(0, s.mast - 0.01); }
      }
      // arm joints → target
      const arm = this.arm, a = arm.angles, t = arm.target;
      // synchronised joint interpolation: all joints arrive together (straight line in joint space)
      const step = arm.speed * dt * (pw ? 1 : 0);
      const J = ['yaw', 'shoulder', 'elbow', 'wristPitch', 'wristRoll'];
      const maxD = Math.max(...J.map((k) => Math.abs(t[k] - a[k])));
      const f = maxD > 1e-9 ? Math.min(1, step / maxD) : 1;
      const next = { gap: a.gap };
      for (const k of J) next[k] = a[k] + (t[k] - a[k]) * f;
      // claw: closing stops at contact with the payload between the fingers
      let gt = t.gap;
      if (arm.graspTarget && gt < a.gap) {
        const ext = this.jawExtent(arm.graspTarget);
        if (ext !== null && gt < ext) { gt = ext; }
      }
      next.gap = pw ? U.approach(a.gap, gt, arm.gapSpeed * dt) : a.gap;
      arm.contact = !!(arm.graspTarget && Math.abs(next.gap - gt) < 1e-4 && gt > t.gap + 1e-4);
      // after an abort: drop the finger exception of the old object once the claw is fully open (never a new target)
      if (arm.clearGraspOnOpen && (arm.held || a.gap >= C.arm.maxGap - 1e-3)) {
        if (!arm.held && arm.graspTarget === arm.clearGraspOnOpen) arm.graspTarget = null;
        arm.clearGraspOnOpen = null;
      }
      const moved = Object.keys(next).some((k) => Math.abs(next[k] - a[k]) > 1e-7);
      if (moved) {
        const s = this.snapshot();
        arm.apply(next); this.root.updateMatrixWorld(true);
        this.syncHeld();
        const r = this.validate(['arm']);
        if (!r.ok) { this.restore(s); this.syncHeld(); arm.blocked = true; arm.blockReason = r.reason; this.blocked('arm', r); }
        else arm.blocked = false;
      }
      this.syncHeld();
    }
    // jaw opening that just touches an object between the fingers (null when not between them)
    jawExtent(ob) {
      const P = this.arm.points(), ax = this.arm.jawAxis(new V());
      const mid = P.baseL.clone().add(P.tipR).multiplyScalar(0.5);
      const rel = ob.obb.c.clone().sub(mid);
      if (Math.abs(rel.dot(ax)) > 0.12) return null;
      // extent of the box along the jaw axis
      const e = ['x', 'y', 'z'].reduce((acc, k, i) => {
        const u = new V(i === 0 ? 1 : 0, i === 1 ? 1 : 0, i === 2 ? 1 : 0).applyQuaternion(ob.obb.q);
        return acc + Math.abs(u.dot(ax)) * ob.obb.h[k];
      }, 0);
      return 2 * e + 0.004;
    }
    syncHeld() {
      const h = this.arm.held;
      if (!h) return;
      h.mesh.updateMatrixWorld(true);
      const p = new V(), q = new THREE.Quaternion(); h.mesh.getWorldPosition(p); h.mesh.getWorldQuaternion(q);
      h.ob.obb.set(p, q); h.ob.refresh();
    }
    blocked(part, r) {
      A.State.stats.lastBlock = r.reason;
      const key = 'blk-' + part + r.reason;
      if (A.Log.once(key, `Bewegung verhindert — ${r.reason}`, 'warn', 4)) {
        A.State.stats.interventions++;
        A.Env.flash(r.ob, r.point);
        A.State.emit('blocked', r);
      }
    }

    // ------------------------------------------------------------------ visuals
    updateVisuals(dt) {
      this.t += dt;
      const S = A.State, fault = S.fault && S.fault.severity > 0.05, off = !S.powered;
      const blink = Math.sin(this.t * 12) > 0;
      this.leds.forEach((l, i) => {
        let c = 0x29d3ff;
        if (off) c = i < 2 ? (Math.sin(this.t * 2) > 0 ? 0x550000 : 0x110000) : 0x050505;
        else if (fault) c = blink ? (i % 2 ? 0xff2020 : 0xffa000) : 0x220000;
        else if (A.Loco && A.Loco.gaitActive) c = 0x22ff88;
        l.material.color.setHex(c);
      });
      const warn = fault || off || (A.Loco && A.Loco.blockedT > 0);
      this.beaconMat.emissiveIntensity = warn ? 1.5 + Math.sin(this.t * 14) * 1.5 : 0;
      this.beaconLight.intensity = warn ? 1.2 + Math.sin(this.t * 14) : 0;
      this.beacon.rotation.y += dt * 8;
      this.lidarPuck.rotation.y += dt * (S.toggles.lidar && !off ? 18 : 0);
      this.lidarRing.material.color.setHex(S.toggles.lidar && !off ? 0x29d3ff : 0x0d4a5c);
      this.thermalMat.emissiveIntensity = S.toggles.thermal && !off ? 1.6 + Math.sin(this.t * 5) * 0.4 : 0.1;
      this.lightMat.emissiveIntensity = S.toggles.light && !off ? 3 : 0;
      this.mastLed.material.color.setHex(Math.sin(this.t * 4) > 0.6 && !off ? 0xff3030 : 0x330000);
      this.arm.clawLed.material.color.setHex(this.arm.held ? 0x22ff88 : this.arm.blocked ? 0xff3030 : 0x29d3ff);
      for (const l of this.legs) { if (l.errT > 0) l.errT -= dt; }
      // fault leg glows
      for (const l of this.legs) {
        const bad = S.fault && S.fault.leg === l.id && S.fault.severity > 0.05;
        l.segMat.emissive.setHex(bad ? 0xff1a00 : 0x000000);
        l.segMat.emissiveIntensity = bad ? (0.4 + 0.4 * Math.sin(this.t * 10)) * S.fault.severity : 0;
      }
    }
    setWireframe(on) {
      this.root.traverse((o) => { if (o.isMesh && o.material && 'wireframe' in o.material) o.material.wireframe = on; });
    }
    setDebug(on) {
      for (const l of this.legs) l.axes.forEach((a) => (a.visible = on));
      this.arm.axes.forEach((a) => (a.visible = on));
      this.bodyAxes.visible = on;
    }
  }
  A.Robot = Robot;
})();
