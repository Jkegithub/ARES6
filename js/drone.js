/* ARES-6 — scout drone: dock / launch / orbit / return / land with obstacle-aware altitude and contact checks */
(function () {
  'use strict';
  const A = window.ARES, U = A.U, W = A.World;
  const V = THREE.Vector3;
  const RAD = 0.3; // collision sphere of the drone

  const Drone = (A.DroneSys = {
    state: 'docked', pos: new V(), vel: new V(), ang: 0, alt: 2.4, t: 0, holdT: 0,

    build(scene, R) {
      this.R = R; this.scene = scene;
      const g = (this.mesh = new THREE.Group()); g.name = 'scoutDrone'; scene.add(g);
      const M = R.M;
      const add = (geo, mat, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = true; g.add(m); return m; };
      add(new THREE.BoxGeometry(0.2, 0.06, 0.14), M.armor);
      add(new THREE.BoxGeometry(0.12, 0.03, 0.1), M.yellow, 0, 0.04, 0);
      this.rotors = [];
      for (const [sx, sz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
        const arm = add(new THREE.BoxGeometry(0.2, 0.02, 0.025), M.dark, sx * 0.09, 0, sz * 0.07);
        arm.rotation.y = Math.atan2(-sz * 0.07, sx * 0.09);
        add(new THREE.CylinderGeometry(0.018, 0.018, 0.04, 8), M.joint, sx * 0.17, 0.02, sz * 0.13);
        const r = add(new THREE.CylinderGeometry(0.075, 0.075, 0.004, 20), new THREE.MeshBasicMaterial({ color: 0x9fb4c4, transparent: true, opacity: 0.35, depthWrite: false }), sx * 0.17, 0.045, sz * 0.13);
        r.castShadow = false; this.rotors.push(r);
        const bl = add(new THREE.BoxGeometry(0.15, 0.003, 0.012), M.dark, sx * 0.17, 0.047, sz * 0.13); this.rotors.push(bl);
      }
      for (const [sx, sz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) add(new THREE.CylinderGeometry(0.006, 0.006, 0.05, 6), M.dark, sx * 0.06, -0.05, sz * 0.05);
      this.led = add(new THREE.SphereGeometry(0.014, 8, 6), new THREE.MeshBasicMaterial({ color: 0x22ff88 }), 0.1, 0, 0);
      const gim = add(new THREE.SphereGeometry(0.03, 12, 8), M.lens, 0.05, -0.045, 0);
      gim.castShadow = false;
      this.light = new THREE.PointLight(0x29d3ff, 0, 2.5, 2); g.add(this.light);
      this.cam = new THREE.PerspectiveCamera(62, 16 / 9, 0.05, 90);
      this.camTrail = new V();
      this.reset();
    },
    reset() { this.state = 'docked'; this.vel.set(0, 0, 0); this.syncDock(); },
    dockPos(out) { return this.R.dockCenter(out); },
    syncDock() {
      this.dockPos(this.pos);
      this.mesh.position.copy(this.pos);
      const q = new THREE.Quaternion(); this.R.dock.getWorldQuaternion(q); this.mesh.quaternion.copy(q);
    },

    // contact test against world + robot parts (dock excluded while landing)
    collides(p, landing) {
      if (W.sphereHit(p, RAD)) return 'Umgebung';
      if (p.y < RAD * 0.6 + W.surfaceBelow(p.x, p.z, p.y).y) return 'Boden';
      const R = this.R;
      // the dock column (straight above the pad) is the only approach allowed close to the chassis
      const dp = this.dockPos(new V());
      const inColumn = landing && Math.hypot(p.x - dp.x, p.z - dp.z) < 0.2 && p.y >= dp.y - 0.01;
      if (!inColumn && R.hullOBB().distance(p) < RAD + 0.05) return 'Roboter-Chassis';
      if (R.headCenter(new V()).distanceTo(p) < RAD + 0.25) return 'Sensorkopf';
      const m = R.mastPoints();
      if (U.segSeg(m.base, m.top, p, p) < RAD + 0.08) return 'Kameramast';
      for (const l of R.legs) { l.points(); if (U.segSeg(l.pts.hip, l.pts.knee, p, p) < RAD + 0.12 || U.segSeg(l.pts.knee, l.pts.ankle, p, p) < RAD + 0.08) return `Bein ${A.LN(l.id)}`; }
      const ap = R.arm.points();
      if (U.segSeg(ap.shoulder, ap.elbow, p, p) < RAD + 0.07 || U.segSeg(ap.elbow, ap.grasp, p, p) < RAD + 0.07) return 'Arm';
      return null;
    },
    safeAltitude(cx, cz, r) {
      let alt = 2.2;
      for (const o of W.obstacles) {
        const d = Math.hypot(o.obb.c.x - cx, o.obb.c.z - cz);
        if (Math.abs(d - r) < o.radius + 1.0) alt = Math.max(alt, o.top + 0.8);
      }
      alt = Math.max(alt, this.R.mastPoints().top.y + 0.9);
      return Math.min(alt, 6);
    },

    launch() {
      const S = A.State;
      if (!S.powered) return A.feedback('Drohnenstart nicht möglich — Roboter abgeschaltet', 'warn');
      if (this.state !== 'docked') return A.feedback('Drohne ist bereits in der Luft');
      const p = this.dockPos(new V());
      const up = W.raycast(p, new V(0, 1, 0), 3, null, false);
      if (up && up.dist < 2.0) {
        A.Log.add(`Drohnenstart abgebrochen — Hindernis über dem Dock: ${up.ob.name} in ${up.dist.toFixed(2)} m`, 'warn');
        A.Env.flash(up.ob, up.point);
        S.stats.interventions++;
        return;
      }
      this.state = 'launch'; this.alt = this.safeAltitude(this.R.pose.x, this.R.pose.z, S.cmd.droneRadius);
      this.ang = Math.atan2(-(p.z - this.R.pose.z), p.x - this.R.pose.x);
      A.Log.add('Erkundungsdrohne gestartet — senkrechter Steigflug', 'info');
    },
    recall() {
      if (this.state === 'docked') return A.feedback('Drohne ist angedockt');
      this.state = 'return'; A.Log.add('Drohne zurückgerufen — Rückflug zum Dock');
    },
    emergencyLand() {
      if (this.state === 'docked') return;
      this.state = 'emergency'; A.Log.add('Drohne: Funkverbindung verloren — autonome Notlandung', 'warn');
    },

    // move toward target, climbing instead when the straight step would collide
    moveToward(target, maxV, dt, landing) {
      const d = target.clone().sub(this.pos), dist = d.length();
      if (dist < 1e-4) return true;
      const desired = d.multiplyScalar(Math.min(maxV, dist * 2.2) / dist);
      this.vel.lerp(desired, 1 - Math.exp(-4 * dt));
      const next = this.pos.clone().addScaledVector(this.vel, dt);
      const hit = this.collides(next, landing);
      if (!hit) { this.pos.copy(next); this.holdT = 0; }
      else {
        this.holdT += dt;
        const climb = this.pos.clone(); climb.y += 1.2 * dt;
        if (!this.collides(climb, landing)) this.pos.copy(climb);
        this.vel.multiplyScalar(0.5);
        A.Log.once('dronecol', `Drohne weicht aus: ${hit} voraus — steigt`, 'warn', 4);
      }
      return dist < 0.06;
    },

    update(dt) {
      this.t += dt;
      const S = A.State, R = this.R;
      const airborne = this.state !== 'docked';
      for (let i = 0; i < this.rotors.length; i++) this.rotors[i].rotation.y += dt * (airborne ? 60 : 0) * (i % 4 < 2 ? 1 : -1);
      this.led.material.color.setHex(airborne ? (Math.sin(this.t * 10) > 0 ? 0x22ff88 : 0x062010) : 0x29d3ff);
      this.light.intensity = airborne ? 0.6 : 0;
      const center = new V(R.pose.x, R.pose.gy + R.pose.h, R.pose.z);
      switch (this.state) {
        case 'docked': this.syncDock(); break;
        case 'launch': {
          const tgt = this.dockPos(new V()); tgt.y = Math.max(this.alt, tgt.y + 1.5);
          if (this.moveToward(tgt, 1.4, dt, true)) { this.state = 'orbit'; A.Log.add(`Drohne auf Position — Orbit r=${S.cmd.droneRadius.toFixed(1)} m, Höhe ${this.alt.toFixed(1)} m`); }
          break;
        }
        case 'orbit': {
          const r = S.cmd.droneRadius;
          if (Math.floor(this.t * 2) !== Math.floor((this.t - dt) * 2)) this.alt = this.safeAltitude(center.x, center.z, r);
          this.ang += (dt * 1.1) / Math.max(1.5, r) * 1.6;
          const tgt = new V(center.x + Math.cos(this.ang) * r, this.alt, center.z - Math.sin(this.ang) * r);
          this.moveToward(tgt, 3.2, dt);
          break;
        }
        case 'return': {
          const dp = this.dockPos(new V()), above = dp.clone(); above.y = Math.max(this.alt, dp.y + 1.6);
          const horiz = Math.hypot(this.pos.x - dp.x, this.pos.z - dp.z);
          if (horiz > 0.08) { const t = above.clone(); if (horiz > 1) t.y = Math.max(this.pos.y, above.y); this.moveToward(t, 3, dt); }
          else { this.state = 'land'; A.Log.add('Drohne über dem Dock ausgerichtet — Sinkflug'); }
          break;
        }
        case 'land': {
          const dp = this.dockPos(new V());
          const tgt = dp.clone(); this.pos.x = U.damp(this.pos.x, dp.x, 8, dt); this.pos.z = U.damp(this.pos.z, dp.z, 8, dt);
          this.moveToward(tgt, 0.8, dt, true);
          if (this.pos.distanceTo(dp) < 0.03) { this.state = 'docked'; A.Log.add('Drohne angedockt und verriegelt', 'ok'); }
          if (Math.hypot(this.pos.x - dp.x, this.pos.z - dp.z) > 0.3) this.state = 'return';
          break;
        }
        case 'emergency': {
          const g = W.surfaceBelow(this.pos.x, this.pos.z, this.pos.y).y + 0.06;
          const tgt = new V(this.pos.x, g, this.pos.z);
          this.moveToward(tgt, 0.7, dt, true);
          if (this.pos.y - g < 0.05) this.state = 'grounded';
          break;
        }
        case 'grounded': if (S.powered) { this.state = 'return'; A.Log.add('Drohnenverbindung wieder da — Rückflug zum Dock'); this.alt = this.safeAltitude(center.x, center.z, 2); } break;
      }
      if (this.state !== 'docked') {
        this.mesh.position.copy(this.pos);
        this.mesh.quaternion.slerp(new THREE.Quaternion().setFromEuler(new THREE.Euler(this.vel.z * 0.15, 0, -this.vel.x * 0.15)), 1 - Math.exp(-5 * dt));
      }
      // drone camera looks at the robot
      this.cam.position.copy(this.mesh.position).add(new V(0, -0.06, 0));
      this.camTrail.lerp(center, 1 - Math.exp(-6 * dt));
      this.cam.lookAt(this.camTrail);
    },
  });
})();
