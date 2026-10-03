/* ARES-6 — sensor module: lidar sweep with raycast point cloud, thermal detection, search light, sensor cones */
(function () {
  'use strict';
  const A = window.ARES, U = A.U, W = A.World, D = A.DEG;
  const V = THREE.Vector3;

  const Sensors = (A.Sensors = {
    sweep: 0, hits: [], density: 0, maxPts: 5000,

    build(scene, R) {
      this.R = R; this.scene = scene;
      // lidar point cloud (ring buffer, fading)
      const g = new THREE.BufferGeometry();
      this.pos = new Float32Array(this.maxPts * 3); this.col = new Float32Array(this.maxPts * 3); this.age = new Float32Array(this.maxPts).fill(99);
      g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(this.col, 3));
      this.cloud = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.055, vertexColors: true, transparent: true, opacity: 0.95, depthWrite: false, blending: THREE.AdditiveBlending }));
      this.cloud.frustumCulled = false; scene.add(this.cloud); this.ptr = 0;
      // sweep fan
      const fan = new THREE.Mesh(new THREE.CircleGeometry(9, 32, -0.09, 0.18), new THREE.MeshBasicMaterial({ color: 0x29d3ff, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false, blending: THREE.AdditiveBlending }));
      fan.rotation.x = -Math.PI / 2; this.fanPivot = new THREE.Group(); this.fanPivot.add(fan); scene.add(this.fanPivot);
      const ln = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new V(0, 0, 0), new V(9, 0, 0)]), new THREE.LineBasicMaterial({ color: 0x7fe9ff, transparent: true, opacity: 0.9 }));
      this.fanPivot.add(ln);
      const ring = new THREE.Mesh(new THREE.RingGeometry(0.0, 1, 64, 1), new THREE.MeshBasicMaterial({ color: 0x29d3ff, transparent: true, opacity: 0.0, side: THREE.DoubleSide, depthWrite: false, blending: THREE.AdditiveBlending }));
      ring.rotation.x = -Math.PI / 2; scene.add(ring); this.pulse = ring; this.pulseT = 0;

      // search light: spot + volumetric cone
      const spot = (this.spot = new THREE.SpotLight(0xfff1d6, 0, 22, 0.32, 0.45, 1.4));
      spot.castShadow = true; spot.shadow.mapSize.set(1024, 1024); spot.shadow.camera.near = 0.2;
      R.lightAnchor.add(spot); spot.position.set(0, 0, 0);
      const tgt = new THREE.Object3D(); tgt.position.set(5, 0, 0); R.lightAnchor.add(tgt); spot.target = tgt;
      const coneGeo = new THREE.ConeGeometry(Math.tan(0.32) * 9, 9, 32, 1, true); coneGeo.translate(0, -4.5, 0); coneGeo.rotateZ(Math.PI / 2);
      this.beam = new THREE.Mesh(coneGeo, new THREE.ShaderMaterial({
        transparent: true, depthWrite: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending,
        uniforms: { a: { value: 0 } },
        vertexShader: 'varying vec3 vP; void main(){ vP=position; gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0); }',
        fragmentShader: 'uniform float a; varying vec3 vP; void main(){ float f=1.0-clamp(vP.x/9.0,0.0,1.0); gl_FragColor=vec4(1.0,0.95,0.82, a*0.16*f*f); }',
      }));
      R.lightAnchor.add(this.beam);

      // sensor cones (camera frustum + thermal cone)
      const fr = new THREE.Group(), L = 4, hw = Math.tan(30 * D) * L, hh = Math.tan(20 * D) * L;
      const c = [new V(0, 0, 0), new V(L, hh, hw), new V(L, hh, -hw), new V(L, -hh, -hw), new V(L, -hh, hw)];
      const e = [0, 1, 0, 2, 0, 3, 0, 4, 1, 2, 2, 3, 3, 4, 4, 1].map((i) => c[i]);
      fr.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(e), new THREE.LineBasicMaterial({ color: 0x29d3ff, transparent: true, opacity: 0.8 })));
      R.camAnchor.add(fr); this.camCone = fr;
      const tg = new THREE.ConeGeometry(Math.tan(32 * D) * 7, 7, 28, 1, true); tg.translate(0, -3.5, 0); tg.rotateZ(Math.PI / 2);
      this.thermCone = new THREE.Mesh(tg, new THREE.MeshBasicMaterial({ color: 0xff5a1f, transparent: true, opacity: 0.07, side: THREE.DoubleSide, depthWrite: false, wireframe: false }));
      const tw = new THREE.Mesh(tg, new THREE.MeshBasicMaterial({ color: 0xff7a3a, transparent: true, opacity: 0.18, wireframe: true }));
      this.thermCone.add(tw); R.thermalAnchor.add(this.thermCone);
      // visual effects are not physical robot parts
      for (const m of [this.beam, this.thermCone, tw]) m.userData.fx = true;
    },

    // head forward direction (world)
    headDir(out) {
      const q = new THREE.Quaternion(); this.R.headPitchJ.getWorldQuaternion(q);
      return out.set(1, 0, 0).applyQuaternion(q);
    },

    update(dt) {
      const S = A.State, T = S.toggles, R = this.R, on = S.powered;
      const lidar = T.lidar && on;
      // ---- lidar sweep + raycasts
      const origin = R.lidarAnchor.getWorldPosition(new V());
      this.fanPivot.visible = lidar; this.fanPivot.position.copy(origin);
      if (lidar) {
        const prev = this.sweep; this.sweep += dt * Math.PI * 2 * 0.9;
        this.fanPivot.rotation.y = this.sweep;
        const elev = [-24, -15, -9, -4, 0, 5, 11];
        const steps = Math.max(1, Math.round(((this.sweep - prev) / (2 * Math.PI)) * 180));
        let near = 0, tot = 0;
        for (let s = 0; s < steps; s++) {
          const az = prev + ((this.sweep - prev) * (s + 1)) / steps;
          for (const el of elev) {
            const e = el * D, d = new V(Math.cos(az) * Math.cos(e), Math.sin(e), -Math.sin(az) * Math.cos(e));
            const h = W.raycast(origin, d, 12, null, true);
            tot++;
            if (h) {
              if (h.ob !== 'floor' && h.dist < 3.5) near++;
              this.addPoint(h.point, h.dist, h.ob === 'floor');
            }
          }
        }
        this.density = U.damp(this.density, tot ? near / tot : 0, 2, dt);
      } else this.density = U.damp(this.density, 0, 1, dt);
      // fade cloud
      for (let i = 0; i < this.maxPts; i++) {
        if (this.age[i] > 6) continue;
        this.age[i] += dt;
        const k = Math.max(0, 1 - this.age[i] / 6) * (lidar ? 1 : 0.6);
        const b = this.age[i] < 0.05 ? 1.6 : 1;
        this.col[i * 3] = this.cr[i] * k * b; this.col[i * 3 + 1] = this.cg[i] * k * b; this.col[i * 3 + 2] = this.cb[i] * k * b;
      }
      this.cloud.geometry.attributes.color.needsUpdate = true;
      // lidar pulse ring
      if (lidar) { this.pulseT = (this.pulseT + dt / 1.6) % 1; }
      this.pulse.visible = lidar; this.pulse.position.set(origin.x, 0.02 + R.pose.gy, origin.z);
      this.pulse.scale.setScalar(0.3 + this.pulseT * 10); this.pulse.material.opacity = 0.18 * (1 - this.pulseT);

      // ---- search light
      const light = T.light && on;
      this.spot.intensity = U.damp(this.spot.intensity, light ? 9 : 0, 6, dt);
      this.beam.material.uniforms.a.value = U.damp(this.beam.material.uniforms.a.value, light ? 1 : 0, 6, dt);
      this.beam.visible = this.beam.material.uniforms.a.value > 0.01;

      // ---- cones
      this.camCone.visible = T.cones && on;
      this.thermCone.visible = T.cones && on && T.thermal;

      // ---- thermal detection of the survivor signature
      const sv = A.Env.survivor;
      const hc = R.headCenter(new V());
      const toT = sv.pos.clone().sub(hc), dist = toT.length();
      S.scan.distance = dist;
      const dir = this.headDir(new V());
      const ang = Math.acos(U.clamp(dir.dot(toT.clone().normalize()), -1, 1));
      const inFov = ang < 55 * D && dist < 16;
      this.thermalVisible = T.thermal && on && inFov;
      if (this.thermalVisible && !sv.detected) {
        sv.detected = true;
        A.Log.add(`Wärmeanomalie erkannt — Peilung ${(Math.atan2(-toT.z, toT.x) / D).toFixed(0)}°, ${dist.toFixed(1)} m`, 'alert');
      }
    },
    addPoint(p, dist, floor) {
      const i = this.ptr; this.ptr = (this.ptr + 1) % this.maxPts;
      this.pos[i * 3] = p.x; this.pos[i * 3 + 1] = p.y + 0.01; this.pos[i * 3 + 2] = p.z;
      this.age[i] = 0;
      if (!this.cr) { this.cr = new Float32Array(this.maxPts); this.cg = new Float32Array(this.maxPts); this.cb = new Float32Array(this.maxPts); }
      const t = U.clamp(dist / 10, 0, 1);
      if (floor) { this.cr[i] = 0.05; this.cg[i] = 0.35; this.cb[i] = 0.45; }
      else { this.cr[i] = 0.2 + 0.8 * (1 - t); this.cg[i] = 0.85 - 0.3 * t; this.cb[i] = 1.0; if (dist < 2.2) { this.cr[i] = 1; this.cg[i] = 0.45; this.cb[i] = 0.2; } }
      this.cloud.geometry.attributes.position.needsUpdate = true;
    },
    clearCloud() { this.age.fill(99); this.col.fill(0); this.cloud.geometry.attributes.color.needsUpdate = true; },

    // survivor probability from distance, line of sight and sensor state
    evaluateSurvivor() {
      const sv = A.Env.survivor, R = this.R;
      const hc = R.headCenter(new V()), mc = R.mastPoints().top;
      const dist = hc.distanceTo(sv.pos);
      const occl = Math.min(W.occluders(hc, sv.pos, sv.ob), W.occluders(mc, sv.pos, sv.ob));
      let p = 0.04 + Math.random() * 0.05;
      if (this.thermalVisible) p = U.clamp(0.98 - dist * 0.018 - occl * 0.08, 0.35, 0.97);
      else if (dist < 18 && A.State.toggles.thermal) p = U.clamp(0.4 - dist * 0.01, 0.1, 0.4);
      return { p, dist, occl };
    },
  });
})();
