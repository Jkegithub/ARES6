/* ARES-6 — robotic arm module: yaw/shoulder/elbow/wrist chain, two-finger claw with contact stop, IK */
(function () {
  'use strict';
  const A = window.ARES, C = A.CFG, U = A.U, AC = C.arm, D = A.DEG;
  const V = THREE.Vector3;
  const _t = new V();

  const LIM = { yaw: [-100 * D, 100 * D], shoulder: [-45 * D, 115 * D], elbow: [-165 * D, 0], wristPitch: [-125 * D, 125 * D], wristRoll: [-180 * D, 180 * D] };

  class Arm {
    constructor(M, parent) {
      this.angles = { yaw: 0, shoulder: 80 * D, elbow: -150 * D, wristPitch: 60 * D, wristRoll: 0, gap: 0.06 };
      this.target = { ...this.angles };
      this.speed = 1.0;          // rad/s joint speed
      this.gapSpeed = 0.35;      // m/s finger speed
      this.blocked = false; this.blockReason = '';
      this.held = null;          // debris record
      this.contact = false;      // fingers stopped on an object
      this.graspTarget = null;   // obstacle the fingers may touch
      this.pts = { shoulder: new V(), elbow: new V(), wrist: new V(), palm: new V(), tipL: new V(), tipR: new V(), baseL: new V(), baseR: new V(), grasp: new V() };
      this.build(M, parent);
      this.apply(this.angles);
    }

    build(M, parent) {
      const add = (p, geo, mat, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = true; p.add(m); return m; };
      const r = (this.root = new THREE.Group()); r.name = 'armRoot'; r.position.fromArray(AC.mount); parent.add(r);
      add(r, new THREE.BoxGeometry(0.2, 0.14, 0.26), M.armor, -0.08, 0, 0);
      const yj = (this.yawJ = new THREE.Group()); yj.name = 'armYaw'; r.add(yj);
      add(yj, new THREE.CylinderGeometry(0.085, 0.09, 0.1, 18), M.joint);
      const sj = (this.shoulderJ = new THREE.Group()); sj.name = 'shoulderJoint'; yj.add(sj);
      add(sj, new THREE.CylinderGeometry(0.075, 0.075, 0.16, 18), M.yellow).rotation.x = Math.PI / 2;
      this.drumA = new THREE.Object3D(); this.drumA.position.z = -0.08; yj.add(this.drumA);
      this.drumB = new THREE.Object3D(); this.drumB.position.z = 0.08; yj.add(this.drumB);
      add(sj, new THREE.BoxGeometry(AC.L1 - 0.1, 0.09, 0.09), M.armor, AC.L1 / 2, 0, 0);
      add(sj, new THREE.BoxGeometry((AC.L1 - 0.1) * 0.6, 0.01, 0.092), M.hazard, AC.L1 / 2, 0.05, 0);
      const ej = (this.elbowJ = new THREE.Group()); ej.name = 'elbowJoint'; ej.position.x = AC.L1; sj.add(ej);
      add(ej, new THREE.CylinderGeometry(0.065, 0.065, 0.16, 16), M.joint).rotation.x = Math.PI / 2;
      const fg = new THREE.CylinderGeometry(0.04, 0.05, AC.L2 - 0.08, 14); fg.rotateZ(Math.PI / 2);
      add(ej, fg, M.armor, AC.L2 / 2, 0, 0);
      const wp = (this.wristP = new THREE.Group()); wp.name = 'wristJoint'; wp.position.x = AC.L2; ej.add(wp);
      add(wp, new THREE.SphereGeometry(0.05, 14, 10), M.joint);
      const wr = (this.wristR = new THREE.Group()); wr.name = 'wristRoll'; wp.add(wr);
      add(wr, new THREE.BoxGeometry(AC.palm, 0.1, 0.13), M.armor, AC.palm / 2, 0, 0);
      this.clawLed = add(wr, new THREE.SphereGeometry(0.016, 8, 6), new THREE.MeshBasicMaterial({ color: 0x29d3ff }), AC.palm, 0.04, 0);
      this.fingers = [-1, 1].map((s) => {
        const f = new THREE.Group(); f.position.x = AC.palm - 0.005; wr.add(f);
        add(f, new THREE.BoxGeometry(AC.finger, 0.05, 0.022), M.dark, AC.finger / 2, 0, 0);
        add(f, new THREE.BoxGeometry(0.05, 0.052, 0.024), M.yellow, AC.finger - 0.025, 0, 0);
        f.userData.s = s;
        f.tip = new THREE.Object3D(); f.tip.position.x = AC.finger; f.add(f.tip);
        return f;
      });
      this.graspPt = new THREE.Object3D(); this.graspPt.position.x = AC.palm + 0.11; wr.add(this.graspPt);
      this.axes = [yj, sj, ej, wp].map((g) => { const ax = new THREE.AxesHelper(0.18); ax.visible = false; g.add(ax); return ax; });
    }

    apply(a) {
      Object.assign(this.angles, a);
      this.yawJ.rotation.y = this.angles.yaw;
      this.shoulderJ.rotation.z = this.angles.shoulder;
      this.elbowJ.rotation.z = this.angles.elbow;
      this.wristP.rotation.z = this.angles.wristPitch;
      this.wristR.rotation.x = this.angles.wristRoll;
      for (const f of this.fingers) f.position.z = f.userData.s * (this.angles.gap / 2 + 0.011);
    }

    clampAngles(a) {
      for (const k of Object.keys(LIM)) if (a[k] !== undefined) a[k] = U.clamp(a[k], LIM[k][0], LIM[k][1]);
      if (a.gap !== undefined) a.gap = U.clamp(a.gap, 0, AC.maxGap);
      return a;
    }

    // IK for a claw pointing straight down with its grasp point at graspW; root.matrixWorld must be current
    solveDown(graspW, worldYawOfJaw = null) {
      const p = this.root.worldToLocal(_t.copy(graspW));
      const yaw = Math.atan2(-p.z, p.x);
      const rr = Math.hypot(p.x, p.z), wy = p.y + (AC.palm + 0.11); // wrist sits above the grasp point
      const L1 = AC.L1, L2 = AC.L2, Dd = Math.hypot(rr, wy);
      let why = null;
      if (Dd > L1 + L2 - 0.005) why = 'außer Reichweite'; else if (Dd < 0.25) why = 'zu nah';
      const Dc = U.clamp(Dd, 0.25, L1 + L2 - 0.005);
      const elbow = -(Math.PI - Math.acos(U.clamp((L1 * L1 + L2 * L2 - Dc * Dc) / (2 * L1 * L2), -1, 1)));
      const shoulder = Math.atan2(wy, rr) + Math.acos(U.clamp((L1 * L1 + Dc * Dc - L2 * L2) / (2 * L1 * Dc), -1, 1));
      const wristPitch = -Math.PI / 2 - shoulder - elbow;
      const a = { yaw, shoulder, elbow, wristPitch, wristRoll: this.target.wristRoll };
      if (worldYawOfJaw !== null) a.wristRoll = worldYawOfJaw;
      for (const k of ['yaw', 'shoulder', 'elbow', 'wristPitch']) if (!why && (a[k] < LIM[k][0] || a[k] > LIM[k][1])) why = `Grenze ${{ yaw: 'Arm-Drehung', shoulder: 'Schulter', elbow: 'Ellbogen', wristPitch: 'Handgelenk' }[k]}`;
      return { ok: !why, why, a: this.clampAngles(a) };
    }

    points() {
      const P = this.pts;
      this.shoulderJ.getWorldPosition(P.shoulder); this.elbowJ.getWorldPosition(P.elbow); this.wristP.getWorldPosition(P.wrist);
      this.graspPt.getWorldPosition(P.grasp);
      _t.set(AC.palm, 0, 0); P.palm.copy(this.wristR.localToWorld(_t));
      this.fingers[0].tip.getWorldPosition(P.tipL); this.fingers[1].tip.getWorldPosition(P.tipR);
      this.fingers[0].getWorldPosition(P.baseL); this.fingers[1].getWorldPosition(P.baseR);
      P.drumA = this.drumA.getWorldPosition(P.drumA || new V()); P.drumB = this.drumB.getWorldPosition(P.drumB || new V());
      return P;
    }
    jawAxis(out) { // world direction across the jaws (finger travel axis)
      const q = new THREE.Quaternion(); this.wristR.getWorldQuaternion(q);
      return out.set(0, 0, 1).applyQuaternion(q);
    }
    reached(tol = 0.02) {
      for (const k of ['yaw', 'shoulder', 'elbow', 'wristPitch', 'wristRoll']) if (Math.abs(this.angles[k] - this.target[k]) > tol) return false;
      return true;
    }
  }
  Arm.LIM = LIM;
  A.Arm = Arm;
})();
