/* ARES-6 — scene setup, camera controls, debug overlays, picture-in-picture drone feed, animation loop */
(function () {
  'use strict';
  const A = window.ARES;
  if (!window.THREE) return;
  const S = A.State, U = A.U, D = A.DEG;
  const V = THREE.Vector3;

  // ------------------------------------------------------------------ renderer / scene
  const canvas = document.getElementById('c');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
  // phones/tablets: lower render resolution and shadow map (decided once at load)
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, A.MOBILE ? 1.5 : 2));
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.15;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x07090b);
  scene.fog = new THREE.FogExp2(0x07090b, 0.034);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 200);

  scene.add(new THREE.HemisphereLight(0x8fb2cc, 0x0b0d10, 0.55));
  const key = new THREE.DirectionalLight(0xfff1e0, 1.35);
  key.castShadow = true; key.shadow.mapSize.setScalar(A.MOBILE ? 1024 : 2048);
  Object.assign(key.shadow.camera, { left: -14, right: 14, top: 14, bottom: -14, near: 1, far: 60 });
  key.shadow.bias = -0.0004; key.shadow.normalBias = 0.02;
  scene.add(key, key.target);
  const rim = new THREE.DirectionalLight(0x3fa9ff, 0.55); rim.position.set(-8, 6, 10); scene.add(rim);
  const fill = new THREE.PointLight(0xff9a3a, 0.5, 18, 2); fill.position.set(6, 3, 6); scene.add(fill);

  // ------------------------------------------------------------------ world + robot
  A.Env.build(scene);
  const robot = (A.robot = new A.Robot(scene));
  A.Loco.init(robot);
  A.Missions.init(robot);
  A.Sensors.build(scene, robot);
  A.DroneSys.build(scene, robot);
  A.Telemetry.reset();
  A.Oracle.init(robot);
  A.UI.init();

  // ------------------------------------------------------------------ orbit camera (LMB orbit, RMB/Shift pan, wheel zoom)
  const Cam = {
    target: new V(0, 0.5, 0), theta: -2.35, phi: 1.08, dist: 9.5, follow: true, mode: 'follow', drag: null,
    pts: new Map(), pinch: null,
    pan(dx, dy) {
      const right = new V().setFromMatrixColumn(camera.matrix, 0), fwd = new V().crossVectors(camera.up, right).normalize();
      const k = this.dist * 0.0016;
      this.target.addScaledVector(right, -dx * k).addScaledVector(fwd, dy * k);
      if (this.follow) this.setMode('free');
    },
    // two touch points: distance → zoom, midpoint → pan
    pinchState() {
      const [a, b] = [...this.pts.values()];
      return { d: Math.hypot(a.x - b.x, a.y - b.y), x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    },
    attach() {
      canvas.addEventListener('contextmenu', (e) => e.preventDefault());
      canvas.addEventListener('pointerdown', (e) => {
        try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* synthetic pointer */ }
        this.pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
        if (this.pts.size === 2) { this.pinch = this.pinchState(); this.drag = null; return; }
        this.drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.button === 1 || e.shiftKey };
        if (this.mode === 'chase' || this.mode === 'side') this.setMode('free');
      });
      canvas.addEventListener('pointermove', (e) => {
        if (this.pts.has(e.pointerId)) this.pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
        if (this.pinch && this.pts.size >= 2) {
          const p = this.pinchState();
          if (p.d > 10 && this.pinch.d > 10) this.dist = U.clamp(this.dist * (this.pinch.d / p.d), 2.2, 45);
          this.pan(p.x - this.pinch.x, p.y - this.pinch.y);
          this.pinch = p;
          return;
        }
        if (!this.drag) return;
        const dx = e.clientX - this.drag.x, dy = e.clientY - this.drag.y; this.drag.x = e.clientX; this.drag.y = e.clientY;
        if (this.drag.pan) this.pan(dx, dy);
        else { this.theta += dx * 0.006; this.phi = U.clamp(this.phi - dy * 0.005, 0.08, 1.52); }
      });
      const end = (e) => {
        this.pts.delete(e.pointerId);
        if (this.pts.size < 2) this.pinch = null;
        // one finger left after a pinch: continue orbiting from its position without a jump
        if (this.pts.size === 1) { const [p] = [...this.pts.values()]; this.drag = { x: p.x, y: p.y, pan: false }; }
        else this.drag = null;
      };
      canvas.addEventListener('pointerup', end); canvas.addEventListener('pointercancel', end);
      canvas.addEventListener('wheel', (e) => { e.preventDefault(); this.dist = U.clamp(this.dist * (1 + Math.sign(e.deltaY) * 0.09), 2.2, 45); }, { passive: false });
    },
    setMode(m) {
      this.mode = m; this.follow = m !== 'free';
      document.querySelectorAll('#viewbtns button').forEach((b) => b.classList.toggle('on', b.dataset.view === m || (b.dataset.view === 'follow' && this.follow && m !== 'chase')));
    },
    view(v) {
      const yaw = robot.pose.yaw;
      if (v === 'follow') { this.setMode(this.follow && this.mode === 'follow' ? 'free' : 'follow'); return; }
      if (v === 'iso') { this.theta = -yaw - 2.35; this.phi = 1.0; this.dist = 9.5; this.setMode('follow'); }
      if (v === 'top') { this.phi = 0.09; this.dist = 18; this.setMode('follow'); }
      if (v === 'side') { this.theta = -yaw - Math.PI / 2; this.phi = 1.38; this.dist = 7; this.setMode('side'); }
      if (v === 'chase') { this.phi = 1.12; this.dist = 6.5; this.setMode('chase'); }
    },
    update(dt) {
      const p = robot.pose;
      if (this.follow) this.target.lerp(new V(p.x, p.gy + p.h * 0.6, p.z), 1 - Math.exp(-3 * dt));
      if (this.mode === 'chase') this.theta = U.damp(this.theta, -p.yaw + Math.PI, 3, dt);
      const sp = Math.sin(this.phi);
      camera.position.set(this.target.x + this.dist * sp * Math.cos(this.theta), this.target.y + this.dist * Math.cos(this.phi), this.target.z + this.dist * sp * Math.sin(this.theta));
      camera.lookAt(this.target);
    },
  };
  Cam.attach();

  // ------------------------------------------------------------------ debug overlay (collision volumes, footholds, support polygon, path)
  const dbg = new THREE.Group(); dbg.visible = false; scene.add(dbg);
  const dbgObs = new Map();
  const footMk = robot.legs.map(() => { const m = new THREE.Mesh(new THREE.RingGeometry(0.1, 0.13, 20), new THREE.MeshBasicMaterial({ color: 0x22ff88, side: THREE.DoubleSide, depthTest: false })); m.rotation.x = -Math.PI / 2; m.renderOrder = 5; dbg.add(m); return m; });
  const swingMk = robot.legs.map(() => { const m = new THREE.Mesh(new THREE.RingGeometry(0.05, 0.08, 16), new THREE.MeshBasicMaterial({ color: 0x29d3ff, side: THREE.DoubleSide, depthTest: false })); m.rotation.x = -Math.PI / 2; m.renderOrder = 5; dbg.add(m); return m; });
  const supLine = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x22ff88, depthTest: false })); supLine.renderOrder = 5; dbg.add(supLine);
  const comMk = new THREE.Mesh(new THREE.SphereGeometry(0.05, 10, 8), new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false })); comMk.renderOrder = 6; dbg.add(comMk);
  const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0x29d3ff, dashSize: 0.2, gapSize: 0.12 })); dbg.add(pathLine);
  const hullBoxes = A.CFG.hull.parts.map((p) => { const b = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(p.h[0] * 2, p.h[1] * 2, p.h[2] * 2)), new THREE.LineBasicMaterial({ color: 0xf2b705 })); dbg.add(b); return b; });
  function updateDebug() {
    if (!dbg.visible) return;
    for (const o of A.World.obstacles) {
      let m = dbgObs.get(o);
      if (!m) {
        m = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(o.obb.h.x * 2, o.obb.h.y * 2, o.obb.h.z * 2)), new THREE.LineBasicMaterial({ color: o.kind === 'debris' ? 0xf2b705 : o.kind === 'survivor' ? 0xff5a1f : 0xff3b9a, transparent: true, opacity: 0.6 }));
        dbg.add(m); dbgObs.set(o, m);
      }
      m.position.copy(o.obb.c); m.quaternion.copy(o.obb.q); m.visible = !o.held;
    }
    for (const [o, m] of dbgObs) if (!A.World.obstacles.includes(o)) { dbg.remove(m); dbgObs.delete(o); }
    robot.legs.forEach((l, i) => {
      footMk[i].position.set(l.foot.x, l.foot.y + 0.01, l.foot.z);
      footMk[i].material.color.setHex(l.mode === 'stance' ? 0x22ff88 : l.mode === 'manual' ? 0xf2b705 : 0x29d3ff);
      swingMk[i].visible = l.mode === 'swing';
      if (l.swing) swingMk[i].position.set(l.swing.to.x, l.swing.to.y + 0.012, l.swing.to.z);
    });
    const hl = A.Loco.hull || [];
    supLine.geometry.setFromPoints(hl.map((p) => new V(p.x, robot.pose.gy + 0.02, p.z)));
    supLine.material.color.setHex(A.Loco.margin > 0.08 ? 0x22ff88 : 0xff3b3b);
    if (A.Loco.com) comMk.position.set(A.Loco.com.x, robot.pose.gy + 0.03, A.Loco.com.z);
    const nav = A.Loco.nav;
    pathLine.visible = !!(nav && nav.path);
    if (pathLine.visible) { pathLine.geometry.setFromPoints(nav.path.map((p) => new V(p.x, 0.05, p.z))); pathLine.computeLineDistances(); }
    robot.hullOBB().parts.forEach((p, i) => { hullBoxes[i].position.copy(p.obb.c); hullBoxes[i].quaternion.copy(p.obb.q); });
  }

  // ------------------------------------------------------------------ target marker
  let marker = null;
  function placeMarker(p) {
    if (marker) scene.remove(marker);
    marker = new THREE.Group(); marker.position.copy(p);
    const beam = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 6, 8, 1, true), new THREE.MeshBasicMaterial({ color: 0xff3b3b, transparent: true, opacity: 0.45, blending: THREE.AdditiveBlending, depthWrite: false }));
    beam.position.y = 3; marker.add(beam);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.35, 0.42, 32), new THREE.MeshBasicMaterial({ color: 0xff3b3b, side: THREE.DoubleSide, transparent: true, depthTest: false }));
    ring.rotation.x = -Math.PI / 2; ring.position.y = 0.02; marker.add(ring); marker.userData.ring = ring;
    const lbl = new THREE.Sprite(new THREE.SpriteMaterial({ map: A.labelTex('ZIEL MARKIERT', '#ff3b3b'), transparent: true, depthTest: false }));
    lbl.scale.set(1.8, 0.45, 1); lbl.position.y = 1.4; marker.add(lbl);
    scene.add(marker);
  }

  // ------------------------------------------------------------------ public API
  A.Main = {
    view: (v) => Cam.view(v),
    camState: () => ({ theta: Cam.theta, phi: Cam.phi, dist: Cam.dist }),
    placeMarker,
    setDebug(on) { dbg.visible = on; robot.setDebug(on); },
    manualClose() {
      const arm = robot.arm;
      if (arm.held) return A.feedback('Greifer hält bereits eine Last');
      const gp = arm.points().grasp;
      const d = A.Env.debris.find((x) => !x.ob.held && x.ob.obb.c.distanceTo(gp) < 0.2);
      arm.target.gap = 0;
      if (!d) { A.Log.add('Greifer schließt (keine Last zwischen den Fingern)'); return; }
      arm.graspTarget = d.ob;
      A.Missions.task('MANUELLER GRIFF', (function* () {
        const ok = yield* A.Tasks.until(() => arm.contact, 3);
        if (!ok) { arm.graspTarget = null; A.Log.add('Griff fehlgeschlagen — kein Kontakt', 'warn'); return; }
        arm.wristR.attach(d.mesh); arm.held = d; d.ob.held = true; arm.graspTarget = null; arm.target.gap = arm.angles.gap; robot.syncHeld();
        A.Log.add(`Manueller Griff: ${d.ob.name} gesichert`, 'ok');
      })(), ['Greifen']);
    },
    resetSim() {
      A.Tasks.cancel('reset');
      const arm = robot.arm;
      if (arm.held) { arm.held.mesh.parent && arm.held.mesh.parent.remove(arm.held.mesh); arm.held = null; }
      arm.graspTarget = null;
      S.reset(); A.Log.clear(); A.Loco.reset();
      A.Env.spawnDebris(); A.Env.survivor.detected = false; A.Env.highlight(null);
      robot.reset(); A.DroneSys.reset(); A.Sensors.clearCloud(); A.Telemetry.reset(); A.Oracle.reset();
      robot.setWireframe(false); this.setDebug(false);
      if (marker) { scene.remove(marker); marker = null; }
      Cam.setMode('follow');
      S.emit('power'); A.UI.syncToggles(); A.UI.renderTimeline(null);
      boot();
    },
  };

  function boot() {
    A.Log.add('ARES-6 initialisiert — Firmware 2.1 (kollisionssichere Fortbewegung)', 'ok');
    A.Log.add(`Kollisionswelt aktiv — ${A.World.obstacles.length} Körper registriert`);
    const r = robot.validate();
    A.Log.add(r.ok ? 'Selbsttest bestanden — 6/6 Beine am Boden, keine Kollision' : `Selbsttest-Warnung: ${r.reason}`, r.ok ? 'ok' : 'warn');
    A.feedback('ARES-6 bereit. Demo-Leiste, Steuerpanel oder ? TOUR nutzen.');
  }

  // ------------------------------------------------------------------ resize + loop
  const vp = document.getElementById('viewport');
  function resize() {
    const w = vp.clientWidth, h = vp.clientHeight;
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize); resize();

  function simulate(dt) {
    A.Tasks.update(dt);
    A.Loco.update(dt); A.Loco.postUpdate();
    robot.updateAppendages(dt);
    robot.updateVisuals(dt);
    A.DroneSys.update(dt);
    A.Sensors.update(dt);
    A.Env.update(dt);
    A.Oracle.update(dt);
    A.Telemetry.update(dt);
  }
  // fixed-step simulation without rendering (test harness; also works in a hidden tab)
  A.Main.simulate = function (seconds, dt = 1 / 60) {
    const n = Math.round(seconds / dt);
    for (let i = 0; i < n; i++) simulate(dt);
    A.UI.update(1); updateDebug(); Cam.update(1);
    renderer.render(scene, camera);
  };
  let last = performance.now();
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    try {
      simulate(dt);
      A.UI.update(dt);
      updateDebug();
      if (marker) { const r = marker.userData.ring; r.scale.setScalar(1 + 0.3 * Math.sin(now / 200)); }
    } catch (e) { console.error(e); A.Log.once('loop', 'Simulationsfehler: ' + e.message, 'err', 5); }
    Cam.update(dt);
    // shadow frustum follows the robot
    key.position.set(robot.pose.x + 7, 12, robot.pose.z + 5); key.target.position.set(robot.pose.x, 0, robot.pose.z);
    const w = vp.clientWidth, h = vp.clientHeight;
    renderer.setScissorTest(false); renderer.setViewport(0, 0, w, h);
    renderer.render(scene, camera);
    const dr = A.DroneSys;
    if (S.toggles.droneFeed && dr.state !== 'docked') {
      const pw = Math.round(Math.min(384, w * 0.42)), ph = Math.round(pw * 9 / 16), m = A.MOBILE ? 8 : 16;
      const x = w - pw - m, y = A.MOBILE ? 64 : m; // WebGL viewport origin is bottom-left (matches CSS bottom)
      dr.mesh.visible = false;
      renderer.setScissorTest(true); renderer.setScissor(x, y, pw, ph); renderer.setViewport(x, y, pw, ph);
      dr.cam.aspect = pw / ph; dr.cam.updateProjectionMatrix();
      renderer.render(scene, dr.cam);
      renderer.setScissorTest(false);
      dr.mesh.visible = true;
    }
    requestAnimationFrame(frame);
  }
  boot();
  requestAnimationFrame(frame);
})();
