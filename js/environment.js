/* ARES-6 — procedural disaster test environment, debris dynamics, survivor signature, zone highlights */
(function () {
  'use strict';
  const A = window.ARES, U = A.U, W = A.World;
  const V = THREE.Vector3;

  // ------------------------------------------------------------------ procedural textures
  function canvasTex(size, draw, repeat = 1) {
    const c = document.createElement('canvas'); c.width = c.height = size;
    draw(c.getContext('2d'), size);
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(repeat, repeat);
    t.anisotropy = 4;
    return t;
  }
  function concreteTex() {
    return canvasTex(256, (g, s) => {
      g.fillStyle = '#6b6d6f'; g.fillRect(0, 0, s, s);
      const img = g.getImageData(0, 0, s, s);
      for (let i = 0; i < img.data.length; i += 4) {
        const n = (Math.random() - 0.5) * 38;
        img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n;
      }
      g.putImageData(img, 0, 0);
      g.strokeStyle = 'rgba(20,20,20,0.55)'; g.lineWidth = 1.2;
      for (let k = 0; k < 7; k++) {
        g.beginPath(); let x = Math.random() * s, y = Math.random() * s; g.moveTo(x, y);
        for (let j = 0; j < 8; j++) { x += (Math.random() - 0.5) * 50; y += (Math.random() - 0.5) * 50; g.lineTo(x, y); }
        g.stroke();
      }
      for (let k = 0; k < 40; k++) { g.fillStyle = `rgba(30,30,30,${Math.random() * 0.4})`; g.beginPath(); g.arc(Math.random() * s, Math.random() * s, Math.random() * 3, 0, 7); g.fill(); }
    });
  }
  A.hazardTex = function (rep = 1) {
    return canvasTex(128, (g, s) => {
      g.fillStyle = '#f2b705'; g.fillRect(0, 0, s, s);
      g.fillStyle = '#111'; g.beginPath();
      for (let i = -s; i < s * 2; i += 32) { g.moveTo(i, 0); g.lineTo(i + 16, 0); g.lineTo(i + 16 - s, s); g.lineTo(i - s, s); }
      g.fill();
    }, rep);
  };
  function floorTex() {
    return canvasTex(512, (g, s) => {
      g.fillStyle = '#16191c'; g.fillRect(0, 0, s, s);
      const img = g.getImageData(0, 0, s, s);
      for (let i = 0; i < img.data.length; i += 4) { const n = (Math.random() - 0.5) * 14; img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n; }
      g.putImageData(img, 0, 0);
      for (let k = 0; k < 30; k++) { g.fillStyle = `rgba(0,0,0,${Math.random() * 0.25})`; g.beginPath(); g.ellipse(Math.random() * s, Math.random() * s, Math.random() * 40, Math.random() * 20, Math.random() * 3, 0, 7); g.fill(); }
    }, 10);
  }
  function labelTex(text, color = '#f2b705', w = 512, h = 128, bg = 'rgba(0,0,0,0)') {
    const c = document.createElement('canvas'); c.width = w; c.height = h;
    const g = c.getContext('2d'); g.fillStyle = bg; g.fillRect(0, 0, w, h);
    // shrink the font until the (often longer German) text fits the label
    let fs = Math.floor(h * 0.5);
    do { g.font = `700 ${fs}px "JetBrains Mono", Consolas, monospace`; fs -= 2; } while (g.measureText(text).width > w * 0.94 && fs > 10);
    g.fillStyle = color; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, w / 2, h / 2);
    const t = new THREE.CanvasTexture(c); t.anisotropy = 4; return t;
  }
  A.labelTex = labelTex;

  // ------------------------------------------------------------------ environment builder
  const Env = (A.Env = {
    group: null, debris: [], zones: {}, survivor: null, flashes: [], t: 0,

    build(scene) {
      this.scene = scene;
      this.group = new THREE.Group(); this.group.name = 'environment'; scene.add(this.group);
      const M = (this.mats = {
        concrete: new THREE.MeshStandardMaterial({ map: concreteTex(), color: 0x9a9c9e, roughness: 0.95, metalness: 0.02 }),
        concreteDark: new THREE.MeshStandardMaterial({ map: concreteTex(), color: 0x6c6e70, roughness: 1, metalness: 0 }),
        steel: new THREE.MeshStandardMaterial({ color: 0x5a6068, roughness: 0.55, metalness: 0.75 }),
        rust: new THREE.MeshStandardMaterial({ color: 0x6b4630, roughness: 0.85, metalness: 0.4 }),
        hazard: new THREE.MeshStandardMaterial({ map: A.hazardTex(2), roughness: 0.6, metalness: 0.2 }),
        crate: new THREE.MeshStandardMaterial({ color: 0x4a5a3a, roughness: 0.9 }),
        rock: new THREE.MeshStandardMaterial({ map: concreteTex(), color: 0x7d7468, roughness: 1, flatShading: true }),
        debris: new THREE.MeshStandardMaterial({ map: concreteTex(), color: 0xa0948a, roughness: 0.9, flatShading: true }),
      });

      // floor + grids
      const floor = new THREE.Mesh(new THREE.PlaneGeometry(90, 90), new THREE.MeshStandardMaterial({ map: floorTex(), roughness: 0.92, metalness: 0.1 }));
      floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; this.group.add(floor);
      const g1 = new THREE.GridHelper(60, 60, 0x24323c, 0x1a242c); g1.position.y = 0.003; this.group.add(g1);
      const g2 = new THREE.GridHelper(60, 12, 0x2f6f86, 0x2f6f86); g2.position.y = 0.004; g2.material.opacity = 0.35; g2.material.transparent = true; this.group.add(g2);
      this.paintFloor();

      this.buildTunnel(M);
      this.buildWarehouse(M);
      this.buildDebrisField(M);
      this.buildClimbCourse(M);
      this.buildSurvivor();
      this.spawnDebris();
      this.buildZones();
      this.buildAtmosphere();
    },

    // --- helpers --------------------------------------------------------
    box(name, x, y0, z, sx, sy, sz, yaw, mat, opts = {}) {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat);
      mesh.position.set(x, y0 + sy / 2, z); mesh.rotation.y = yaw || 0;
      mesh.castShadow = mesh.receiveShadow = true; this.group.add(mesh);
      const ob = new A.Obstacle({ name, kind: opts.kind, center: mesh.position.clone(), size: new V(sx, sy, sz), yaw, standable: opts.standable, mesh });
      if (!opts.noRegister) W.add(ob);
      return ob;
    },
    // mesh whose collision box is derived from its own (yaw-only) geometry bounds → no visual overhang
    fitted(name, geo, mat, x, z, yaw, opts = {}) {
      geo.computeBoundingBox();
      const bb = geo.boundingBox, size = bb.getSize(new V()), ctr = bb.getCenter(new V());
      geo.translate(-ctr.x, -bb.min.y - size.y / 2, -ctr.z);
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(x, (opts.y0 || 0) + size.y / 2, z); mesh.rotation.y = yaw || 0;
      mesh.castShadow = mesh.receiveShadow = true; this.group.add(mesh);
      const ob = new A.Obstacle({ name, kind: opts.kind, center: mesh.position.clone(), size, yaw, standable: opts.standable, mesh });
      if (!opts.noRegister) W.add(ob);
      return ob;
    },
    rock(name, x, z, r, yaw) {
      const geo = new THREE.IcosahedronGeometry(r, 1);
      const p = geo.attributes.position;
      for (let i = 0; i < p.count; i++) {
        const v = new V().fromBufferAttribute(p, i), k = 0.75 + Math.random() * 0.4;
        p.setXYZ(i, v.x * k * 1.25, Math.max(v.y * k * 0.75, -r * 0.55), v.z * k);
      }
      geo.computeVertexNormals();
      return this.fitted(name, geo, this.mats.rock, x, z, yaw, { standable: false });
    },
    ibeam(len, w = 0.22, h = 0.28) {
      const s = new THREE.Shape(), t = 0.035, fw = w / 2, hh = h / 2;
      s.moveTo(-fw, -hh); s.lineTo(fw, -hh); s.lineTo(fw, -hh + t); s.lineTo(t / 2, -hh + t); s.lineTo(t / 2, hh - t);
      s.lineTo(fw, hh - t); s.lineTo(fw, hh); s.lineTo(-fw, hh); s.lineTo(-fw, hh - t); s.lineTo(-t / 2, hh - t);
      s.lineTo(-t / 2, -hh + t); s.lineTo(-fw, -hh + t); s.closePath();
      const g = new THREE.ExtrudeGeometry(s, { depth: len, bevelEnabled: false });
      g.translate(0, 0, -len / 2); g.rotateY(Math.PI / 2); // length along local X
      return g;
    },
    // beam between two world points (arbitrary orientation)
    beamBetween(name, p1, p2, mat, w = 0.22, h = 0.28) {
      const dir = p2.clone().sub(p1), len = dir.length(); dir.normalize();
      const q = new THREE.Quaternion().setFromUnitVectors(new V(1, 0, 0), dir);
      const mesh = new THREE.Mesh(this.ibeam(len, w, h), mat);
      mesh.position.copy(p1).add(p2).multiplyScalar(0.5); mesh.quaternion.copy(q);
      mesh.castShadow = mesh.receiveShadow = true; this.group.add(mesh);
      return W.add(new A.Obstacle({ name, center: mesh.position.clone(), size: new V(len, h, w), quat: q, standable: false, mesh }));
    },
    sign(text, x, y, z, rotY, w = 2.2, color) {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, w / 4), new THREE.MeshBasicMaterial({ map: labelTex(text, color), transparent: true, depthWrite: false }));
      m.position.set(x, y, z); m.rotation.y = rotY; this.group.add(m); return m;
    },
    floorText(text, x, z, rot, w = 3, color = 'rgba(242,183,5,0.55)') {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, w / 4), new THREE.MeshBasicMaterial({ map: labelTex(text, color), transparent: true, depthWrite: false }));
      m.rotation.x = -Math.PI / 2; m.rotation.z = rot || 0; m.position.set(x, 0.006, z); this.group.add(m); return m;
    },
    paintFloor() {
      const mat = new THREE.LineBasicMaterial({ color: 0xf2b705, transparent: true, opacity: 0.5 });
      const rect = (x, z, w, d) => {
        const pts = [[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2], [-w / 2, -d / 2]].map((p) => new V(x + p[0], 0.007, z + p[1]));
        this.group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mat));
      };
      rect(0, 0, 6.4, 6.4); rect(0, 0, 6.6, 6.6);
      this.floorText('ARES-6 TESTFELD 06', 0, 2.75, 0, 3.2);
      this.floorText('◀ TRÜMMERFELD', -4.3, -0.2, Math.PI / 2, 2.6);
      this.floorText('LAGERHALLE ▶', 4.4, 0, -Math.PI / 2, 2.6);
    },

    // --- zones ------------------------------------------------------------
    buildTunnel(M) {
      // collapsed façade
      const hl = [2.4, 1.7, 2.6, 1.1, 2.0], hr = [2.0, 2.6, 1.4, 2.3, 1.8];
      hl.forEach((h, i) => this.box(`Einsturzwand W${i + 1}`, -3.4 - i * 1.2, 0, -6.0 + U.rand(-0.08, 0.08), 1.18, h, 0.5, U.rand(-0.06, 0.06), M.concrete, { standable: false }));
      hr.forEach((h, i) => this.box(`Einsturzwand O${i + 1}`, 3.4 + i * 1.2, 0, -6.0 + U.rand(-0.08, 0.08), 1.18, h, 0.5, U.rand(-0.06, 0.06), M.concrete, { standable: false }));
      // tunnel shell
      this.box('Tunnelwand West', -2.55, 0, -9.25, 0.5, 1.3, 6.5, 0, M.concreteDark, { standable: false });
      this.box('Tunnelwand Ost', 2.55, 0, -9.25, 0.5, 1.3, 6.5, 0, M.concreteDark, { standable: false });
      this.box('Tunneldecke', 0, 1.0, -9.25, 5.6, 0.3, 6.5, 0, M.concreteDark, { standable: false });
      this.box('Tunnelsturz', 0, 1.3, -6.05, 5.8, 0.3, 0.6, 0, M.hazard, { standable: false });
      this.box('Verschüttetes Tunnelende', 0, 0, -12.75, 5.6, 1.6, 0.5, 0, M.concrete, { standable: false });
      this.box('Schuttblock T1', 0.2, 0, -10.5, 1.8, 0.6, 0.7, 0.15, M.concrete, { standable: false });
      this.rock('Schutt T2', -1.5, -11.6, 0.35, 0.4);
      // rubble at façade base
      this.rock('Fels N1', -4.4, -4.95, 0.5, 0.3);
      this.rock('Fels N2', 5.2, -5.0, 0.6, 1.1);
      this.rock('Fels N3', -6.6, -4.8, 0.42, 2.0);
      this.rock('Fels N4', 7.4, -4.9, 0.38, 0.7);
      this.sign('TUNNEL B-2  ·  DURCHFAHRT 1,0 m', 0, 1.82, -5.74, 0, 3.4);
      // portal beacon
      const bc = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.16, 12), new THREE.MeshStandardMaterial({ color: 0xff8a00, emissive: 0xff6a00, emissiveIntensity: 2 }));
      bc.position.set(2.55, 1.68, -6.05); this.group.add(bc);
      this.beacon = new THREE.PointLight(0xff7a00, 2.2, 7, 2); this.beacon.position.set(2.55, 1.9, -5.7); this.group.add(this.beacon);
      this.beaconMesh = bc;
      // tunnel interior lights
      for (let z = -7; z >= -12; z -= 2.5) {
        const l = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.04, 0.12), new THREE.MeshBasicMaterial({ color: 0x9fd8ff }));
        l.position.set(0, 0.98, z); this.group.add(l);
        const pl = new THREE.PointLight(0x7fc8ff, 0.5, 3.5, 2); pl.position.set(0, 0.8, z); this.group.add(pl);
      }
    },
    buildWarehouse(M) {
      const shelf = (x, z) => {
        const ob = this.box(`Regal ${x > 9 ? 'B' : 'A'}${z > 0 ? 'S' : 'N'}`, x, 0, z, 3.0, 2.6, 0.9, 0, new THREE.MeshBasicMaterial({ visible: false }), { standable: false });
        ob.mesh.castShadow = false;
        const g = new THREE.Group(); g.position.set(x, 0, z); this.group.add(g);
        for (const sx of [-1.45, 0, 1.45]) for (const sz of [-0.4, 0.4]) {
          const u = new THREE.Mesh(new THREE.BoxGeometry(0.08, 2.6, 0.08), M.rust); u.position.set(sx, 1.3, sz); u.castShadow = true; g.add(u);
        }
        for (const y of [0.15, 0.95, 1.75, 2.55]) {
          const p = new THREE.Mesh(new THREE.BoxGeometry(3.0, 0.05, 0.9), M.steel); p.position.y = y; p.castShadow = p.receiveShadow = true; g.add(p);
          if (y < 2.5) for (let i = 0; i < 4; i++) {
            if (Math.random() < 0.3) continue;
            const s = U.rand(0.35, 0.6), c = new THREE.Mesh(new THREE.BoxGeometry(s, s * 0.8, U.rand(0.4, 0.75)), M.crate);
            c.position.set(-1.1 + i * 0.73, y + 0.025 + s * 0.4, U.rand(-0.05, 0.05)); c.rotation.y = U.rand(-0.2, 0.2); c.castShadow = true; g.add(c);
          }
        }
      };
      shelf(7.5, -4.0); shelf(11.0, -4.0); shelf(7.5, 4.0); shelf(11.0, 4.0);
      for (const [x, z] of [[5.6, -2.9], [5.6, 2.9], [9.3, -2.9], [9.3, 2.9]]) {
        this.box(`Säule ${x}/${z}`, x, 0, z, 0.45, 3.2, 0.45, 0, M.steel, { standable: false });
        const band = new THREE.Mesh(new THREE.BoxGeometry(0.47, 0.6, 0.47), M.hazard); band.position.set(x, 0.3, z); this.group.add(band);
      }
      // warehouse back wall with breach
      this.box('Hallenwand N', 13.8, 0, -3.4, 0.4, 3.0, 5.2, 0, M.concrete, { standable: false });
      this.box('Hallenwand S', 13.8, 0, 4.3, 0.4, 3.0, 3.4, 0, M.concrete, { standable: false });
      this.box('Schutt im Wanddurchbruch', 13.7, 0, 1.7, 0.9, 0.9, 1.6, 0.2, M.concrete, { standable: false });
      // fallen beam on the floor (step-over obstacle) and a leaning beam
      const fb = this.fitted('Umgestürzter I-Träger', this.ibeam(3.2), M.rust, 8.3, 0.4, 0.45, { standable: false });
      fb.mesh.receiveShadow = true;
      this.beamBetween('Schräger Träger', new V(10.9, 2.55, -3.5), new V(9.85, 0.16, -1.7), M.rust);
      this.box('Kiste C1', 5.0, 0, -4.4, 0.8, 0.8, 0.8, 0.3, M.crate, { standable: false });
      this.box('Kiste C2', 12.5, 0, -1.4, 0.9, 0.9, 0.9, -0.2, M.crate, { standable: false });
      this.box('Kiste C3', 12.5, 0.9, -1.4, 0.65, 0.6, 0.65, 0.25, M.crate, { standable: false });
      this.sign('HALLE 3  ·  STATIKSCHADEN', 13.57, 2.4, 0.0, -Math.PI / 2, 3.6, '#ff5b3a');
    },
    buildDebrisField(M) {
      this.box('Betonblock D1', -6.2, 0, 1.2, 1.4, 0.9, 1.0, 0.4, M.concrete, { standable: false });
      this.box('Betonblock D2', -8.0, 0, -1.5, 1.1, 1.2, 1.3, -0.3, M.concrete, { standable: false });
      this.box('Betonblock D3', -9.5, 0, 2.5, 1.6, 0.7, 0.9, 0.9, M.concrete, { standable: false });
      this.box('Bodenplatte S1', -6.8, 0, -0.9, 1.6, 0.16, 1.2, 0.2, M.concreteDark, { standable: true });
      this.box('Bodenplatte S2', -8.6, 0, 0.7, 1.2, 0.12, 1.0, -0.4, M.concreteDark, { standable: true });
      this.rock('Fels W1', -5.4, -2.3, 0.45, 0.2);
      this.rock('Fels W2', -7.4, 3.3, 0.55, 1.3);
      this.rock('Fels W3', -10.3, -0.6, 0.6, 2.2);
      this.beamBetween('Eingestürzter Träger', new V(-9.4, 0.84, 2.45), new V(-6.3, 1.04, 1.25), M.steel);
    },
    buildClimbCourse(M) {
      // ramp: wedge rising along +Z from z=3.5 (h 0) to z=6.0 (h 0.3)
      const H = 0.3, len = 2.5, wid = 4.6;
      const sh = new THREE.Shape(); sh.moveTo(-len / 2, -H / 2); sh.lineTo(len / 2, -H / 2); sh.lineTo(len / 2, H / 2); sh.closePath();
      const geo = new THREE.ExtrudeGeometry(sh, { depth: wid, bevelEnabled: false }); geo.translate(0, 0, -wid / 2);
      const ramp = new THREE.Mesh(geo, M.concreteDark); ramp.position.set(0, H / 2, 4.75); ramp.rotation.y = -Math.PI / 2;
      ramp.castShadow = ramp.receiveShadow = true; this.group.add(ramp);
      W.add(new A.Obstacle({ name: 'Kletterrampe', center: ramp.position.clone(), size: new V(len, H, wid), yaw: -Math.PI / 2, ramp: { H }, mesh: ramp }));
      this.box('Kletterplateau', 0, 0, 7.0, 4.6, 0.3, 2.0, 0, M.concrete, { standable: true });
      const edge = new THREE.Mesh(new THREE.BoxGeometry(4.6, 0.06, 0.12), M.hazard); edge.position.set(0, 0.27, 7.94); this.group.add(edge);
      this.box('Schuttplatte R1', -0.9, 0, 9.0, 1.5, 0.14, 1.0, 0.15, M.concreteDark, { standable: true });
      this.box('Schuttplatte R2', 1.1, 0, 9.6, 1.3, 0.12, 0.9, -0.2, M.concreteDark, { standable: true });
      for (const z of [4.0, 7.0, 9.8]) for (const x of [-2.95, 2.95]) {
        const ob = this.box('Poller', x, 0, z, 0.24, 0.9, 0.24, 0, M.hazard, { standable: false });
        ob.mesh.geometry = new THREE.CylinderGeometry(0.11, 0.11, 0.9, 14);
      }
      this.floorText('KLETTERPARCOURS  ▼', 0, 3.0, Math.PI, 2.8);
    },
    buildSurvivor() {
      // abstract heat signature: invisible collision volume + glowing blobs (no human depiction)
      const pos = new V(0.4, 0, -11.5);
      const ob = this.box('Überlebenden-Zone', pos.x, 0, pos.z, 1.4, 0.44, 0.62, 0.1, new THREE.MeshBasicMaterial({ visible: false }), { kind: 'survivor', standable: false });
      ob.mesh.castShadow = false;
      const g = new THREE.Group(); g.position.set(pos.x, 0.22, pos.z); g.rotation.y = 0.1; this.group.add(g);
      const blobMat = new THREE.MeshBasicMaterial({ color: 0xff5a1f, transparent: true, opacity: 0.0, blending: THREE.AdditiveBlending, depthWrite: false });
      const coreMat = new THREE.MeshBasicMaterial({ color: 0xffd27a, transparent: true, opacity: 0.0, blending: THREE.AdditiveBlending, depthWrite: false });
      const blobs = [];
      [[-0.45, 0.16], [-0.12, 0.2], [0.2, 0.17], [0.48, 0.13]].forEach(([x, r]) => {
        const b = new THREE.Mesh(new THREE.SphereGeometry(r * 1.6, 20, 14), blobMat); b.position.x = x; b.scale.y = 0.7; g.add(b); blobs.push(b);
        const c = new THREE.Mesh(new THREE.SphereGeometry(r * 0.8, 16, 10), coreMat); c.position.x = x; g.add(c); blobs.push(c);
      });
      const wire = new THREE.Mesh(new THREE.IcosahedronGeometry(0.42, 1), new THREE.MeshBasicMaterial({ color: 0x4a5560, wireframe: true, transparent: true, opacity: 0.35 }));
      wire.scale.set(2.0, 0.6, 1.0); g.add(wire);
      const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: labelTex('WÄRMESIGNATUR 01', '#ff7a3a'), transparent: true, depthTest: false, opacity: 0 }));
      label.scale.set(1.6, 0.4, 1); label.position.y = 0.75; g.add(label);
      this.survivor = { ob, group: g, blobMat, coreMat, wire, label, pos: new V(pos.x, 0.22, pos.z), detected: false };
    },
    spawnDebris() {
      for (const d of this.debris) { this.group.remove(d.mesh); W.remove(d.ob); }
      this.debris = [];
      const spec = [
        ['Trümmerstück A', 2.6, 0.25, 0.30, 0.22, 0.26, 0.35],
        ['Trümmerstück B', -2.8, -1.9, 0.34, 0.20, 0.28, -0.5],
        ['Trümmerstück C', 3.6, -2.6, 0.28, 0.24, 0.30, 1.0],
      ];
      for (const [name, x, z, sx, sy, sz, yaw] of spec) {
        const geo = new THREE.BoxGeometry(sx, sy, sz, 2, 2, 2);
        const p = geo.attributes.position; // chip the corners slightly inward (stays inside the collision box)
        for (let i = 0; i < p.count; i++) { const k = 0.9 + Math.random() * 0.1; p.setXYZ(i, p.getX(i) * k, p.getY(i) * (0.92 + Math.random() * 0.08), p.getZ(i) * k); }
        geo.computeVertexNormals();
        const mesh = new THREE.Mesh(geo, this.mats.debris); mesh.castShadow = mesh.receiveShadow = true;
        mesh.position.set(x, sy / 2, z); mesh.rotation.y = yaw; this.group.add(mesh);
        const ob = W.add(new A.Obstacle({ name, kind: 'debris', center: mesh.position.clone(), size: new V(sx, sy, sz), yaw, standable: false, mesh }));
        this.debris.push({ ob, mesh, vy: 0, mass: Math.round(sx * sy * sz * 2300) });
      }
    },
    nearestDebris(p, maxD = 9) {
      let best = null, bd = maxD;
      for (const d of this.debris) { if (d.ob.held) continue; const dd = Math.hypot(d.ob.obb.c.x - p.x, d.ob.obb.c.z - p.z); if (dd < bd) { bd = dd; best = d; } }
      return best;
    },
    // support height under a debris box: floor, standable surfaces and other debris tops
    debrisSupport(d) {
      const o = d.ob, bottom = o.obb.c.y - o.obb.h.y;
      let best = 0;
      const pts = [[0, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]].map(([a, b]) => o.obb.toWorld(new V(a * o.obb.h.x * 0.95, 0, b * o.obb.h.z * 0.95), new V()));
      for (const p of pts) {
        best = Math.max(best, W.surfaceBelow(p.x, p.z, bottom + 0.05, o).y);
        for (const e of this.debris) if (e !== d && !e.ob.held && e.ob.footprint(p.x, p.z) && e.ob.top <= bottom + 0.05) best = Math.max(best, e.ob.top);
      }
      return best;
    },
    updateDebris(dt) {
      for (const d of this.debris) {
        if (d.ob.held) continue;
        const o = d.ob, bottom = o.obb.c.y - o.obb.h.y, sup = this.debrisSupport(d);
        if (bottom > sup + 1e-4) {
          d.vy -= 9.81 * dt;
          const nb = Math.max(sup, bottom + d.vy * dt);
          o.obb.c.y = nb + o.obb.h.y;
          if (nb <= sup + 1e-4) { if (d.vy < -1) A.Log.add(`${o.name} aufgeschlagen — liegt auf`, 'info'); d.vy = 0; }
          o.refresh(); d.mesh.position.copy(o.obb.c);
        } else d.vy = 0;
      }
    },

    // --- zone highlights --------------------------------------------------
    buildZones() {
      const defs = {
        tunnel: { c: [0, -9.25], s: [4.6, 6.5], color: 0x29d3ff, label: 'TUNNEL B-2' },
        warehouse: { c: [9.4, 0], s: [9.4, 9.6], color: 0x29d3ff, label: 'HALLE 3' },
        survivor: { c: [0.4, -11.5], s: [2.2, 1.6], color: 0xff5a1f, label: 'WÄRMESIGNATUR 01' },
        debris: { c: [2.6, 0.25], s: [1.6, 1.6], color: 0xf2b705, label: 'TRÜMMERZIEL' },
        climb: { c: [0, 6.6], s: [5.2, 7.2], color: 0x7cff6b, label: 'KLETTERPARCOURS' },
        start: { c: [0, 0], s: [6.4, 6.4], color: 0xff3b3b, label: 'FEHLERISOLATION' },
      };
      for (const [id, z] of Object.entries(defs)) {
        const mat = new THREE.ShaderMaterial({
          transparent: true, depthWrite: false, side: THREE.DoubleSide,
          uniforms: { t: { value: 0 }, col: { value: new THREE.Color(z.color) }, size: { value: new THREE.Vector2(z.s[0], z.s[1]) }, a: { value: 0 } },
          vertexShader: 'varying vec2 vUv; void main(){ vUv=uv; gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0); }',
          fragmentShader: `uniform float t; uniform vec3 col; uniform vec2 size; uniform float a; varying vec2 vUv;
            void main(){ vec2 p=vUv*size; float edge=min(min(p.x,size.x-p.x),min(p.y,size.y-p.y));
              float border=smoothstep(0.09,0.0,edge); float stripe=step(0.5,fract((p.x+p.y)*1.4-t*0.6))*0.10;
              float pulse=0.55+0.45*sin(t*3.0); gl_FragColor=vec4(col,(border*0.9+stripe)*a*pulse); }`,
        });
        const m = new THREE.Mesh(new THREE.PlaneGeometry(z.s[0], z.s[1]), mat);
        m.rotation.x = -Math.PI / 2; m.position.set(z.c[0], 0.012, z.c[1]); m.visible = false; m.renderOrder = 2; this.group.add(m);
        const lbl = new THREE.Sprite(new THREE.SpriteMaterial({ map: labelTex(z.label, '#' + new THREE.Color(z.color).getHexString()), transparent: true, depthTest: false }));
        lbl.scale.set(2.4, 0.6, 1); lbl.position.set(z.c[0], 2.2, z.c[1]); lbl.visible = false; lbl.renderOrder = 3; this.group.add(lbl);
        this.zones[id] = { mesh: m, mat, lbl, def: z };
      }
    },
    highlight(id, at) {
      for (const [k, z] of Object.entries(this.zones)) {
        const on = k === id;
        z.mesh.visible = z.lbl.visible = on;
        if (on && at) { z.mesh.position.x = at.x; z.mesh.position.z = at.z; z.lbl.position.x = at.x; z.lbl.position.z = at.z; }
      }
      A.State.mission.zone = id;
    },
    flash(ob, point) {
      if (!ob || ob === 'floor') return;
      const h = ob.obb.h;
      const m = new THREE.Mesh(new THREE.BoxGeometry(h.x * 2 + 0.04, h.y * 2 + 0.04, h.z * 2 + 0.04), new THREE.MeshBasicMaterial({ color: 0xff2a2a, transparent: true, opacity: 0.35, depthWrite: false }));
      m.position.copy(ob.obb.c); m.quaternion.copy(ob.obb.q);
      const e = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0xff4040, transparent: true }));
      m.add(e); this.scene.add(m);
      let spark = null;
      if (point) {
        spark = new THREE.Mesh(new THREE.SphereGeometry(0.07, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, depthTest: false }));
        spark.position.copy(point); this.scene.add(spark);
      }
      this.flashes.push({ m, e, spark, t: 0 });
    },

    buildAtmosphere() {
      const n = 900, pos = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) { pos[i * 3] = U.rand(-18, 18); pos[i * 3 + 1] = U.rand(0, 5); pos[i * 3 + 2] = U.rand(-16, 14); }
      const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      this.dust = new THREE.Points(g, new THREE.PointsMaterial({ color: 0xb8c4cc, size: 0.035, transparent: true, opacity: 0.35, depthWrite: false }));
      this.group.add(this.dust);
    },

    update(dt) {
      this.t += dt;
      this.updateDebris(dt);
      // dust drift
      const p = this.dust.geometry.attributes.position;
      for (let i = 0; i < p.count; i++) {
        let y = p.getY(i) + dt * 0.05 * Math.sin(i + this.t * 0.3), x = p.getX(i) + dt * 0.08;
        if (x > 18) x = -18; if (y > 5) y = 0; if (y < 0) y = 5;
        p.setX(i, x); p.setY(i, y);
      }
      p.needsUpdate = true;
      // beacon
      const b = 0.5 + 0.5 * Math.sin(this.t * 6);
      this.beacon.intensity = 0.8 + 2.2 * b; this.beaconMesh.material.emissiveIntensity = 0.6 + 2.4 * b;
      // zones
      for (const z of Object.values(this.zones)) { z.mat.uniforms.t.value = this.t; z.mat.uniforms.a.value = z.mesh.visible ? Math.min(1, z.mat.uniforms.a.value + dt * 2) : 0; }
      // collision flashes
      this.flashes = this.flashes.filter((f) => {
        f.t += dt; const a = Math.max(0, 1 - f.t / 0.9);
        f.m.material.opacity = 0.35 * a; f.e.material.opacity = a;
        if (f.spark) { f.spark.material.opacity = a; f.spark.scale.setScalar(1 + f.t * 3); }
        if (a <= 0) { this.scene.remove(f.m); if (f.spark) this.scene.remove(f.spark); return false; }
        return true;
      });
      // survivor signature (visible through walls in thermal mode)
      const s = this.survivor, th = A.State.toggles.thermal && A.State.powered;
      const pulse = 0.75 + 0.25 * Math.sin(this.t * 2.2);
      const target = th ? 1 : 0.06;
      s.blobMat.opacity = U.damp(s.blobMat.opacity, target * 0.32 * pulse, 4, dt);
      s.coreMat.opacity = U.damp(s.coreMat.opacity, target * 0.75 * pulse, 4, dt);
      s.blobMat.depthTest = s.coreMat.depthTest = !th;
      s.label.material.opacity = U.damp(s.label.material.opacity, th && s.detected ? 1 : 0, 4, dt);
      s.wire.rotation.y += dt * 0.3;
    },
  });
})();
