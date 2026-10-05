/* ARES-6 — physics oracle: independent penetration audit.
   It does NOT reuse the validator's capsules/OBB math. It takes the actual rendered robot vertices and tests them
   for containment in the actual rendered obstacle meshes (ray-parity test with THREE.Raycaster), plus
   leg-vs-leg, limb-vs-chassis and floor penetration on render geometry. Every violation is counted. */
(function () {
  'use strict';
  const A = window.ARES;
  const V = THREE.Vector3;
  const TOL = 0.012; // 12 mm contact tolerance

  const Oracle = (A.Oracle = {
    frames: 0, checks: 0, violations: 0, worst: 0, last: null, log: [], acc: 0,

    init(R) {
      this.R = R;
      this.ray = new THREE.Raycaster();
      const sub = (mesh, n) => {
        const p = mesh.geometry.attributes.position, seen = new Set(), out = [];
        for (let i = 0; i < p.count; i++) {
          const k = `${p.getX(i).toFixed(3)},${p.getY(i).toFixed(3)},${p.getZ(i).toFixed(3)}`;
          if (seen.has(k)) continue; seen.add(k); out.push(new V(p.getX(i), p.getY(i), p.getZ(i)));
        }
        const step = Math.max(1, Math.floor(out.length / n));
        return out.filter((_, i) => i % step === 0);
      };
      // robot probe meshes with their owner tag
      this.probes = [];
      const tag = (obj, owner, n = 40) => obj.traverse((o) => { if (o.isMesh && o.geometry && o.material && o.material.visible !== false && !o.userData.fx) this.probes.push({ mesh: o, owner, pts: sub(o, n), pad: o.name === 'footPad' }); });
      R.legs.forEach((l) => tag(l.root, 'leg:' + l.id));
      tag(R.arm.root, 'arm', 30);
      // claw meshes (palm, fingers) hold the load by design; upper arm and forearm do not
      const clawSet = new Set(); R.arm.wristR.traverse((o) => clawSet.add(o));
      for (const pr of this.probes) pr.claw = clawSet.has(pr.mesh);
      tag(R.chassis, 'body', 60);
      tag(R.headPitchJ, 'head', 30);
      tag(R.mast, 'mast', 20);
      // solids for self-collision: segment meshes of legs, chassis box
      this.legSolids = R.legs.map((l) => ({ id: l.id, meshes: [l.upperMesh, l.lowerMesh] }));
      this.chassis = R.chassis;
      this.dblMat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide });
    },
    reset() { this.frames = this.checks = this.violations = 0; this.worst = 0; this.last = null; this.log = []; },

    // point inside closed mesh: odd number of ray crossings (double sided)
    inside(mesh, p) {
      const bb = mesh.userData._bb || (mesh.userData._bb = new THREE.Box3());
      if (mesh.userData._bbF !== this.frames) { bb.setFromObject(mesh); mesh.userData._bbF = this.frames; }
      if (!bb.containsPoint(p)) return false;
      const mat = mesh.material; mesh.material = this.dblMat;
      this.ray.set(p, new V(0.137, 0.981, 0.137).normalize()); this.ray.near = 0; this.ray.far = 50;
      const n = this.ray.intersectObject(mesh, false).length;
      mesh.material = mat;
      return n % 2 === 1;
    },
    // depth estimate: does the point stay inside when moved TOL in 6 directions? (filters touching contacts)
    deep(mesh, p) {
      if (!this.inside(mesh, p)) return false;
      for (const d of [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]) if (!this.inside(mesh, p.clone().add(new V(d[0], d[1], d[2]).multiplyScalar(TOL)))) return false;
      return true;
    },
    violation(kind, a, b, p) {
      this.violations++;
      this.last = { kind, a, b, t: A.Log.time(), p: p.clone() };
      if (this.log.length < 50) this.log.push(this.last);
      A.State.emit('oracle', this.last);
    },

    update(dt) {
      this.acc += dt;
      if (this.acc < (A.MOBILE ? 0.5 : 0.1)) return; // 10 Hz audit (2 Hz on phones)
      this.acc = 0; this.frames++;
      const R = this.R;
      R.root.updateMatrixWorld(true);
      const obstacles = A.World.obstacles.filter((o) => o.mesh && !o.held);
      const w = new V();
      let found = 0;
      for (const pr of this.probes) {
        pr.mesh.updateMatrixWorld();
        for (const lp of pr.pts) {
          w.copy(lp).applyMatrix4(pr.mesh.matrixWorld);
          this.checks++;
          if (w.y < -TOL) { if (found++ < 3) this.violation('floor', pr.owner, 'floor', w); continue; }
          for (const o of obstacles) {
            const dx = o.obb.c.x - w.x, dz = o.obb.c.z - w.z;
            if (dx * dx + dz * dz > (o.radius + 0.2) ** 2) continue;
            if (this.deep(o.mesh, w)) { if (found++ < 3) this.violation('environment', pr.owner, o.name, w); break; }
          }
          // self-collision: leg vertices inside other legs' segment solids / limbs inside chassis
          if (pr.owner.startsWith('leg:') || pr.owner === 'arm') {
            // mounting hardware (hip actuators, arm base/turret) sits in the chassis by design
            if (!pr.mesh.parent || ['hipJoint', 'armRoot', 'armYaw'].includes(pr.mesh.parent.name)) continue;
            if (this.deep(this.chassis, w)) { if (found++ < 3) this.violation('self', pr.owner, 'chassis', w); continue; }
            for (const s of this.legSolids) {
              if (pr.owner === 'leg:' + s.id) continue;
              for (const m of s.meshes) if (this.deep(m, w)) { if (found++ < 3) this.violation('self', pr.owner, 'leg:' + s.id, w); break; }
            }
          }
        }
      }
      // carried payload inside environment
      const held = R.arm.held;
      if (held) {
        held.mesh.updateMatrixWorld();
        const p = held.mesh.geometry.attributes.position, hc = held.ob.obb.c, hr = held.ob.obb.h.length();
        const near = obstacles.filter((o) => o.obb.c.distanceTo(hc) < o.radius + hr + 0.1);
        const legMeshes = this.legSolids.flatMap((s) => s.meshes.map((m) => ({ id: s.id, m })));
        let hit = false;
        for (let i = 0; i < p.count && !hit; i += 3) {
          w.fromBufferAttribute(p, i).applyMatrix4(held.mesh.matrixWorld);
          this.checks++;
          if (w.y < -TOL) { this.violation('floor', 'payload', 'floor', w); hit = true; break; }
          if (this.deep(this.chassis, w)) { this.violation('self', 'payload', 'chassis', w); hit = true; break; }
          for (const s of legMeshes) if (this.deep(s.m, w)) { this.violation('self', 'payload', 'leg:' + s.id, w); hit = true; break; }
          if (!hit) for (const o of near) if (this.deep(o.mesh, w)) { this.violation('environment', 'payload', o.name, w); hit = true; break; }
        }
        // robot vertices inside the carried load (the claw holds it by design and is skipped)
        if (!hit) for (const pr of this.probes) {
          if (pr.claw) continue;
          for (const lp of pr.pts) {
            w.copy(lp).applyMatrix4(pr.mesh.matrixWorld);
            if (w.distanceTo(hc) > hr + 0.3) continue;
            this.checks++;
            if (this.deep(held.mesh, w)) { this.violation('self', pr.owner, 'payload', w); hit = true; break; }
          }
          if (hit) break;
        }
      }
      // drone vs environment
      const dr = A.DroneSys;
      if (dr.state !== 'docked') for (const o of obstacles) if (o.distance(dr.mesh.position) < 0.05 && this.deep(o.mesh, dr.mesh.position)) this.violation('environment', 'drone', o.name, dr.mesh.position);
    },
  });
})();
