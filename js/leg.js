/* ARES-6 — leg module: joint hierarchy, analytic IK with joint limits, forward kinematics points */
(function () {
  'use strict';
  const A = window.ARES, C = A.CFG, U = A.U, LC = C.leg, LIM = C.limits;
  const V = THREE.Vector3;
  const _t = new V();

  const STATUS_COL = { grounded: 0x22ff88, lifting: 0x29d3ff, moving: 0x3d7bff, error: 0xff2a2a, idle: 0x8aa0b0 };

  class Leg {
    constructor(def, M, parent) {
      this.def = def; this.id = def.id;
      this.angles = { yaw: 0, pitch: 0.5, knee: -1.6, ankle: 0 };
      this.manual = { yaw: 0, pitch: 0.5, knee: -1.6, ankle: 0 };
      this.mode = 'stance';          // stance | swing | manual
      this.foot = new V();           // world contact point of the pad
      this.swing = null;             // {from, to, t, dur, H}
      this.status = 'idle';
      this.errT = 0;                 // seconds of error indication left
      this.ikFail = null;
      this.load = 1 / 3;
      this.pts = { hip: new V(), knee: new V(), ankle: new V(), pad: new V(), rim: [new V(), new V(), new V(), new V()] };
      // hip yaw range is asymmetric: the corner legs cannot swing their hip drum into the chassis
      this.yawLim = def.yawLim.map((v) => v * A.DEG);
      this.build(M, parent);
    }

    build(M, parent) {
      const d = this.def, L1 = LC.upper, L2 = LC.lower;
      const r = (this.root = new THREE.Group()); r.name = 'leg_' + d.id;
      r.position.fromArray(d.mount); r.rotation.y = d.yaw; parent.add(r);
      this.ledMat = new THREE.MeshStandardMaterial({ color: 0x050505, emissive: STATUS_COL.idle, emissiveIntensity: 2 });
      this.segMat = M.armor.clone();
      const add = (p, geo, mat, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = true; m.receiveShadow = true; p.add(m); return m; };

      // hip yaw actuator
      const hip = (this.hip = new THREE.Group()); hip.name = 'hipJoint'; r.add(hip);
      add(hip, new THREE.CylinderGeometry(0.095, 0.1, 0.2, 18), M.joint);
      const led = add(hip, new THREE.TorusGeometry(0.098, 0.012, 6, 24), this.ledMat, 0, 0.1, 0); led.rotation.x = Math.PI / 2;
      add(hip, new THREE.BoxGeometry(LC.coxa, 0.11, 0.13), M.armor, LC.coxa / 2, 0, 0);

      // hip pitch + upper leg
      const pj = (this.pitchJ = new THREE.Group()); pj.name = 'upperLegPivot'; pj.position.x = LC.coxa; hip.add(pj);
      add(pj, new THREE.CylinderGeometry(0.085, 0.085, 0.2, 18), M.joint).rotation.x = Math.PI / 2;
      const up = (this.upperMesh = add(pj, new THREE.BoxGeometry(L1 - 0.1, 0.12, 0.11), this.segMat, L1 / 2, 0, 0)); up.name = 'upperLeg';
      add(pj, new THREE.BoxGeometry((L1 - 0.1) * 0.62, 0.012, 0.112), M.hazard, L1 / 2, 0.066, 0);
      add(pj, new THREE.BoxGeometry(L1 - 0.2, 0.05, 0.13), M.dark, L1 / 2, -0.06, 0);

      // knee + lower leg
      const kn = (this.knee = new THREE.Group()); kn.name = 'kneeJoint'; kn.position.x = L1; pj.add(kn);
      add(kn, new THREE.CylinderGeometry(0.075, 0.075, 0.17, 18), M.joint).rotation.x = Math.PI / 2;
      const kl = add(kn, new THREE.CylinderGeometry(0.03, 0.03, 0.172, 10), this.ledMat); kl.rotation.x = Math.PI / 2;
      const lg = new THREE.CylinderGeometry(0.045, 0.06, L2 - 0.08, 14); lg.rotateZ(Math.PI / 2);
      const lo = (this.lowerMesh = add(kn, lg, this.segMat, L2 / 2, 0, 0)); lo.name = 'lowerLeg';
      add(kn, new THREE.BoxGeometry(0.26, 0.09, 0.1), M.yellow, 0.2, 0.0, 0);

      // ankle + foot
      const an = (this.ankle = new THREE.Group()); an.name = 'ankleJoint'; an.position.x = L2; kn.add(an);
      add(an, new THREE.SphereGeometry(0.055, 14, 10), M.joint);
      const ft = (this.footG = new THREE.Group()); ft.name = 'foot'; an.add(ft);
      add(ft, new THREE.CylinderGeometry(0.03, 0.035, LC.ankleH, 10), M.joint, 0, -LC.ankleH / 2, 0);
      this.pad = add(ft, new THREE.CylinderGeometry(0.1, 0.1, LC.padH, 20), M.rubber, 0, -LC.ankleH - LC.padH / 2, 0);
      this.pad.name = 'footPad';
      const rim = add(ft, new THREE.TorusGeometry(0.1, 0.01, 6, 20), M.yellow, 0, -LC.ankleH - 0.004, 0); rim.rotation.x = Math.PI / 2;
      this.padContact = new THREE.Object3D(); this.padContact.position.y = -LC.ankleH - LC.padH; ft.add(this.padContact);

      // hydraulic knee piston (barrel on the upper leg, rod tracks the lower leg anchor)
      const ps = (this.piston = new THREE.Group()); pj.add(ps);
      const barrel = new THREE.CylinderGeometry(0.028, 0.028, 0.24, 10); barrel.rotateZ(Math.PI / 2); barrel.translate(0.12, 0, 0);
      add(ps, barrel, M.yellow);
      const rod = new THREE.CylinderGeometry(0.014, 0.014, 1, 8); rod.rotateZ(Math.PI / 2); rod.translate(0.5, 0, 0);
      this.rod = add(ps, rod, M.chrome);
      // debug axes
      this.axes = [hip, pj, kn, an].map((g) => { const ax = new THREE.AxesHelper(0.22); ax.visible = false; g.add(ax); return ax; });
    }

    setAngles(a) {
      this.angles.yaw = a.yaw; this.angles.pitch = a.pitch; this.angles.knee = a.knee; this.angles.ankle = a.ankle;
      this.hip.rotation.y = a.yaw; this.pitchJ.rotation.z = a.pitch; this.knee.rotation.z = a.knee; this.ankle.rotation.z = a.ankle;
      // piston geometry in the upper-leg frame
      const ax = 0.2, ay = 0.1, bx = 0.24, by = 0.06, k = a.knee, L1 = LC.upper;
      const Bx = L1 + bx * Math.cos(k) - by * Math.sin(k), By = bx * Math.sin(k) + by * Math.cos(k);
      const dx = Bx - ax, dy = By - ay;
      this.piston.position.set(ax, ay, 0.0); this.piston.rotation.z = Math.atan2(dy, dx);
      this.rod.scale.x = Math.hypot(dx, dy);
    }

    // world-space ankle target for a pad contact point
    static ankleFor(contact, out) { return out.copy(contact).setY(contact.y + LC.ankleH + LC.padH); }

    // analytic IK — root.matrixWorld must be current
    solveIK(ankleW) {
      const p = this.root.worldToLocal(_t.copy(ankleW));
      const yaw = Math.atan2(-p.z, p.x);
      const r = Math.hypot(p.x, p.z) - LC.coxa, y = p.y;
      const L1 = LC.upper, L2 = LC.lower, D = Math.hypot(r, y);
      let why = null;
      if (D > L1 + L2 - 0.004) why = 'außer Reichweite';
      else if (D < 0.32) why = 'zu nah an der Hüfte';
      const Dc = U.clamp(D, 0.32, L1 + L2 - 0.004);
      const knee = -(Math.PI - Math.acos(U.clamp((L1 * L1 + L2 * L2 - Dc * Dc) / (2 * L1 * L2), -1, 1)));
      const pitch = Math.atan2(y, r) + Math.acos(U.clamp((L1 * L1 + Dc * Dc - L2 * L2) / (2 * L1 * Dc), -1, 1));
      if (!why) {
        if (yaw < this.yawLim[0] || yaw > this.yawLim[1]) why = 'Hüftdreh-Grenze';
        else if (pitch < LIM.hipPitch[0] || pitch > LIM.hipPitch[1]) why = 'Hüfthub-Grenze';
        else if (knee < LIM.knee[0] || knee > LIM.knee[1]) why = 'Kniegrenze';
      }
      const a = {
        yaw: U.clamp(yaw, this.yawLim[0], this.yawLim[1]),
        pitch: U.clamp(pitch, LIM.hipPitch[0], LIM.hipPitch[1]),
        knee: U.clamp(knee, LIM.knee[0], LIM.knee[1]), ankle: 0,
      };
      a.ankle = U.clamp(-(a.pitch + a.knee), LIM.ankle[0], LIM.ankle[1]);
      return { ok: !why, why, a };
    }

    // passive (compliant) ankle: in auto modes the pad aligns level with the world, like a gimballed foot
    levelFoot(rootQ) {
      if (this.mode === 'manual') { this.footG.quaternion.identity(); return; }
      const pq = new THREE.Quaternion(); this.ankle.getWorldQuaternion(pq);
      this.footG.quaternion.copy(pq.invert().multiply(rootQ));
    }
    // forward kinematics: world positions (matrices must be current)
    points() {
      this.pitchJ.getWorldPosition(this.pts.hip);
      this.knee.getWorldPosition(this.pts.knee);
      this.ankle.getWorldPosition(this.pts.ankle);
      this.padContact.getWorldPosition(this.pts.pad);
      const y = -LC.ankleH - LC.padH;
      [[0.1, 0], [-0.1, 0], [0, 0.1], [0, -0.1]].forEach(([x, z], i) => this.pts.rim[i].copy(this.footG.localToWorld(_t.set(x, y, z))));
      return this.pts;
    }

    setStatus(s) {
      if (this.errT > 0) s = 'error';
      if (s !== this.status) { this.status = s; this.ledMat.emissive.setHex(STATUS_COL[s] || 0xffffff); }
    }
  }
  Leg.STATUS_COL = STATUS_COL;
  A.Leg = Leg;
})();
