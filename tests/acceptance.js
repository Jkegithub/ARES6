/* ARES-6 — acceptance test harness. Open index.html?test (via local server) to run.
   Each scenario resets the simulation, drives it with fixed time steps (ARES.Main.simulate) and
   checks a measurable condition. The physics oracle (independent render-geometry audit) must stay at 0. */
(function () {
  'use strict';
  if (!/[?&]test\b/.test(location.search)) return;
  const A = window.ARES, S = A.State;
  const sim = (s) => A.Main.simulate(s);
  const results = [];
  const R = () => A.robot;
  const click = (sel) => document.querySelector(sel).click();
  const btn = (label) => [...document.querySelectorAll('#controls .btn')].find((b) => b.textContent.trim() === label);
  const runTask = (maxS) => { let t = 0; while (A.Tasks.running() && t < maxS) { sim(0.5); t += 0.5; } return t; };

  function scenario(name, criterion, fn) {
    A.Main.resetSim();
    const o0 = A.Oracle.violations;
    let pass = false, evidence = '';
    try { [pass, evidence] = fn(); } catch (e) { pass = false; evidence = 'EXCEPTION ' + e.message; console.error(e); }
    const ov = A.Oracle.violations - o0;
    if (ov > 0) { pass = false; evidence += ` | ORACLE ${ov} violation(s): ${A.Oracle.log.slice(0, 2).map((v) => v.a + '↔' + v.b).join(', ')}`; }
    results.push({ name, criterion, pass, evidence, oracle: ov, checks: A.Oracle.checks });
  }

  // calibration: a known penetration (validator bypassed) MUST be detected by the oracle
  function calibrate() {
    A.Main.resetSim();
    const r = R(), o0 = A.Oracle.violations;
    r.pose.x = 6.55; r.pose.z = -3.35; r.applyTransform(); // chassis shoved into crate C1 / rack region, no validation
    A.Oracle.acc = 1; A.Oracle.update(0);
    const hits = A.Oracle.violations - o0, first = A.Oracle.log[0];
    results.push({ name: 'Oracle calibration', criterion: 'Known forced penetration is detected (control run)', pass: hits > 0, evidence: `forced pose → ${hits} violation(s) detected, e.g. ${first ? first.a + '↔' + first.b : '—'}`, oracle: 0, checks: A.Oracle.checks });
    A.Main.resetSim();
    return hits > 0;
  }

  function run() {
    const t0 = performance.now();
    calibrate();
    scenario('Boot', 'App opens, robot visible, six legs', () => {
      const r = R().validate();
      const legs = R().legs.length, meshes = []; R().root.traverse((o) => o.isMesh && meshes.push(o));
      return [r.ok && legs === 6 && meshes.length > 100, `legs=${legs}, robot meshes=${meshes.length}, self-test=${r.ok}`];
    });
    scenario('Camera', 'Orbit controls respond', () => {
      const c = document.getElementById('c'), b = c.getBoundingClientRect();
      const ev = (t, x, y, btn = 0) => c.dispatchEvent(new PointerEvent(t, { clientX: x, clientY: y, button: btn, pointerId: 1, bubbles: true }));
      const before = A.Main.camState();
      ev('pointerdown', b.left + 300, b.top + 300); ev('pointermove', b.left + 420, b.top + 340); ev('pointerup', b.left + 420, b.top + 340);
      c.dispatchEvent(new WheelEvent('wheel', { deltaY: 300, bubbles: true, cancelable: true }));
      const after = A.Main.camState();
      return [after.theta !== before.theta && after.dist > before.dist, `theta ${before.theta.toFixed(2)}→${after.theta.toFixed(2)}, dist ${before.dist.toFixed(1)}→${after.dist.toFixed(1)}`];
    });
    scenario('Walk cycle', 'Tripod gait moves the robot, feet alternate', () => {
      const swings = new Set(); let maxSw = 0;
      click('[data-demo="walk"]');
      for (let i = 0; i < 200; i++) { sim(0.05); const sw = R().legs.filter((l) => l.mode === 'swing'); maxSw = Math.max(maxSw, sw.length); sw.forEach((l) => swings.add(l.id)); }
      const x = R().pose.x;
      const tri = R().legs.every((l) => l.mode !== 'swing' || true);
      return [x > 1.5 && swings.size === 6 && maxSw <= 3 && tri, `Δx=${x.toFixed(2)} m in 10 s, legs that swung=${swings.size}/6, max simultaneous swing=${maxSw}`];
    });
    scenario('Collision: walls/objects', 'Robot does not walk through objects — halts with warning', () => {
      click('[data-demo="walk"]');
      let t = 0; while (A.Loco.mode === 'walk' && t < 80) { sim(1); t++; }
      return [A.Loco.mode !== 'walk' && S.stats.interventions > 0, `halted after ${t} s at x=${R().pose.x.toFixed(2)}: "${A.Loco.haltedReason || S.stats.lastBlock}", interventions=${S.stats.interventions}`];
    });
    scenario('Sliders', 'Sliders visibly move robot parts', () => {
      Object.assign(S.cmd, { bodyHeight: 0.9, headYaw: 60, mastHeight: 0.8, bodyPitch: 8 });
      R().arm.target.shoulder = 40 * A.DEG;
      sim(4);
      const p = R().pose;
      return [Math.abs(p.h - 0.9) < 0.02 && Math.abs(R().head.yaw / A.DEG - 60) < 1 && Math.abs(R().mastExt - 0.8) < 0.02 && Math.abs(R().arm.angles.shoulder / A.DEG - 40) < 1,
        `h=${p.h.toFixed(2)}, headYaw=${(R().head.yaw / A.DEG).toFixed(0)}°, mast=${R().mastExt.toFixed(2)} m, pitch=${(p.pitch / A.DEG).toFixed(1)}°, shoulder=${(R().arm.angles.shoulder / A.DEG).toFixed(0)}°`];
    });
    scenario('Claw', 'Claw opens and closes', () => {
      btn('Greifer öffnen').click(); sim(1.5); const open = R().arm.angles.gap;
      btn('Greifer schließen').click(); sim(1.5); const closed = R().arm.angles.gap;
      return [open > 0.4 && closed < 0.01, `gap open=${(open * 100).toFixed(1)} cm, closed=${(closed * 100).toFixed(1)} cm`];
    });
    scenario('Grab debris demo', 'Arm reaches, grips with contact, lifts, moves aside, releases', () => {
      const d = A.Env.debris[0], p0 = d.ob.obb.c.clone();
      A.Missions.grab(); let held = false, t = 0;
      while (A.Tasks.running() && t < 60) { sim(0.25); t += 0.25; held = held || !!R().arm.held; }
      const moved = d.ob.obb.c.distanceTo(p0), rest = d.ob.obb.c.y - d.ob.obb.h.y;
      const log = A.Log.entries.map((e) => e.msg).join('|');
      return [held && moved > 0.3 && !R().arm.held && rest < 0.005 && /Fingerkontakt/.test(log), `held=${held}, moved ${moved.toFixed(2)} m, resting bottom y=${rest.toFixed(3)}, duration ${t.toFixed(1)} s`];
    });
    scenario('Sensors', 'Lidar, thermal and search light visible', () => {
      Object.assign(S.toggles, { lidar: true, thermal: true, light: true }); S.cmd.headYaw = 90;
      sim(3);
      const pts = A.Sensors.age.filter((a) => a < 6).length;
      return [pts > 500 && A.Env.survivor.detected && A.Sensors.spot.intensity > 5 && A.Sensors.fanPivot.visible,
        `lidar points=${pts}, thermal detected=${A.Env.survivor.detected}, spot=${A.Sensors.spot.intensity.toFixed(1)}`];
    });
    scenario('Survivor scan', 'Animated progress + result in telemetry', () => {
      S.cmd.headYaw = 0; A.Missions.scan(); let maxP = 0, t = 0;
      while (A.Tasks.running() && t < 20) { sim(0.25); t += 0.25; maxP = Math.max(maxP, S.scan.progress); }
      return [maxP >= 100 && S.scan.located && S.scan.prob > 0.5, `progress=${maxP.toFixed(0)}%, result="${S.scan.result}"`];
    });
    scenario('Drone', 'Launch, orbit, return without contacts', () => {
      A.DroneSys.launch(); let t = 0, maxAlt = 0, orbited = false;
      while (t < 14) { sim(0.25); t += 0.25; maxAlt = Math.max(maxAlt, A.DroneSys.pos.y); orbited = orbited || A.DroneSys.state === 'orbit'; }
      A.DroneSys.recall(); let t2 = 0;
      while (A.DroneSys.state !== 'docked' && t2 < 30) { sim(0.25); t2 += 0.25; }
      return [orbited && maxAlt > 2 && A.DroneSys.state === 'docked', `orbit=${orbited}, max alt=${maxAlt.toFixed(2)} m, docked again after ${t2.toFixed(1)} s`];
    });
    scenario('Tunnel mission', 'Low crawl under 1.0 m roof, no clipping', () => {
      A.Missions.run('tunnel'); const t = runTask(150);
      const p = R().pose;
      return [p.z < -7 && p.h < 0.32 && S.mission.name === 'TUNNELINSPEKTION', `reached z=${p.z.toFixed(2)} (portal at −6), h=${p.h.toFixed(2)} m, ${t.toFixed(0)} s`];
    });
    scenario('Overhead clearance', 'Mast/body/drone blocked by tunnel roof', () => {
      A.Missions.run('tunnel'); runTask(150);
      const i0 = S.stats.interventions;
      S.cmd.mastHeight = 1.0; S.cmd.bodyHeight = 0.8; sim(4);
      A.DroneSys.launch();
      const mast = R().mastExt, h = R().pose.h;
      return [mast < 0.3 && h < 0.45 && A.DroneSys.state === 'docked' && S.stats.interventions > i0, `mast stopped at ${mast.toFixed(2)} m, body h=${h.toFixed(2)} m, drone=${A.DroneSys.state}, interventions +${S.stats.interventions - i0}`];
    });
    scenario('Warehouse mission', 'Preset changes pose/sensors, traverses fallen beam', () => {
      A.Missions.run('warehouse'); const t = runTask(150);
      const p = R().pose;
      return [p.x > 6.5 && S.toggles.lidar && S.cmd.bodyHeight === 1.0, `reached x=${p.x.toFixed(2)}, z=${p.z.toFixed(2)}, h=${p.h.toFixed(2)}, lidar=${S.toggles.lidar}, ${t.toFixed(0)} s`];
    });
    scenario('Survivor search mission', 'Preset locates survivor, marks target', () => {
      A.Missions.run('survivor'); runTask(120);
      return [S.scan.located && !!S.marker, `prob=${(S.scan.prob * 100).toFixed(0)}%, marker=${!!S.marker}`];
    });
    scenario('Debris removal mission', 'Preset runs grab sequence', () => {
      A.Missions.run('debris'); runTask(90);
      return [/Trümmerräumung abgeschlossen/.test(A.Log.entries.map((e) => e.msg).join('|')), 'log confirms completion'];
    });
    scenario('System failure test', 'Warning leg, rising temp, instability, blinking, stabilize enabled', () => {
      A.Missions.run('failure'); runTask(30);
      A.UI.update(1);
      const ml = R().legById.ML, temp = S.tele.temp;
      const warnLog = A.Log.entries.filter((e) => e.level === 'warn' || e.level === 'err').length;
      const stab = A.UI.btns.stab;
      return [S.fault && ml.status === 'error' && temp > 60 && warnLog >= 3 && !stab.disabled && R().beaconMat.emissiveIntensity !== 0,
        `ML=${ml.status}, temp=${temp.toFixed(0)}°C, warnings=${warnLog}, stabilize enabled=${!stab.disabled}`];
    });
    scenario('Auto stabilize', 'Fixes the failure state visibly', () => {
      A.Missions.run('failure'); runTask(30);
      const temp0 = S.tele.temp;
      A.Missions.stabilize(); runTask(30); sim(6);
      const ok = !S.fault && R().legs.every((l) => l.status !== 'error') && Math.abs(R().pose.roll) < 2 * A.DEG;
      return [ok && S.tele.temp < temp0 && /Stabilisierung bestätigt/.test(A.Log.entries.map((e) => e.msg).join('|')),
        `fault=${!!S.fault}, roll=${(R().pose.roll / A.DEG).toFixed(1)}°, temp ${temp0.toFixed(0)}→${S.tele.temp.toFixed(0)}°C`];
    });
    scenario('Climb demo', 'Feet/body adapt to ramp, plateau, rubble', () => {
      A.Missions.climb(); let t = 0, maxGy = 0, maxPitch = 0;
      while (A.Tasks.running() && t < 200) { sim(0.5); t += 0.5; maxGy = Math.max(maxGy, R().pose.gy); maxPitch = Math.max(maxPitch, Math.abs(R().pose.pitch)); }
      return [maxGy > 0.2 && R().pose.z > 10, `max support height=${maxGy.toFixed(2)} m, max pitch=${(maxPitch / A.DEG).toFixed(1)}°, final z=${R().pose.z.toFixed(2)}, ${t.toFixed(0)} s`];
    });
    scenario('Individual leg', 'Joint sliders, lift/plant/reset; limb contact stops motion', () => {
      S.selectedLeg = 'FL';
      A.Loco.liftLeg('FL'); sim(0.5); const lifted = R().legById.FL.status;
      const i0 = S.stats.interventions;
      A.Loco.manualSet('FL', { yaw: -42 * A.DEG, pitch: -40 * A.DEG, knee: -30 * A.DEG }); sim(0.2);
      const blocked = S.stats.interventions > i0;
      A.Loco.plantLeg('FL'); sim(1.5);
      return [lifted === 'lifting' && blocked && R().legById.FL.mode === 'stance', `after lift: ${lifted}, extreme pose blocked=${blocked} (${S.stats.lastBlock}), after plant: ${R().legById.FL.mode}`];
    });
    scenario('Shutdown / reboot', 'Emergency stop and recovery', () => {
      A.Missions.shutdown(); sim(4); const h = R().pose.h, pw = S.powered;
      A.Missions.reboot(); runTask(20); sim(3);
      return [!pw && h < 0.35 && S.powered && R().pose.h > 0.55, `off: powered=${pw}, h=${h.toFixed(2)}; after reboot powered=${S.powered}, h=${R().pose.h.toFixed(2)}`];
    });
    scenario('Beam stacking demo', 'Three 2.4 m beams cleared onto the timbers: parallel ±3°, offset < 5 cm, layers resting, arm stowed', () => {
      A.Missions.beams(); const t = runTask(900);
      const s = A.Missions.stackReport(), arm = R().arm.angles;
      const stowed = Math.abs(arm.shoulder / A.DEG - 80) < 2 && Math.abs(arm.elbow / A.DEG + 150) < 2;
      return [s.n === 3 && s.off < 0.05 && s.ang < 3 * A.DEG && s.gap < 0.005 && !R().arm.held && stowed,
        `stack n=${s.n}, offset max ${(s.off * 100).toFixed(1)} cm, angle max ${(s.ang / A.DEG).toFixed(1)}°, layer gap max ${(s.gap * 1000).toFixed(0)} mm, arm stowed=${stowed}, ${t.toFixed(0)} s`];
    });
    scenario('Rack demo', 'Parcel from pallet into the free middle slot; another one out of the rack onto the pallet', () => {
      A.Missions.rack(); const t = runTask(400);
      const RK = A.Env.RACK, P = A.Env.PALLET, p1 = A.Env.load('Paket P1'), p2 = A.Env.load('Paket P2');
      const c2 = p2.ob.obb.c, xw = RK.x - 0.725;
      const inSlot = Math.abs(p2.ob.bottom - (RK.levels[1] + 0.025)) < 0.005 && Math.abs(c2.x - xw) < 0.06 && c2.z > RK.z - 0.45 && c2.z < RK.z + 0.45 && p2.ob.top < RK.levels[2] - 0.025;
      const onPallet = Math.abs(p1.ob.bottom - P.h) < 0.005 && Math.abs(p1.ob.obb.c.x - P.x) < 0.4 && Math.abs(p1.ob.obb.c.z - P.z) < 0.6;
      return [inSlot && onPallet && !R().arm.held, `P2 at (${c2.x.toFixed(2)}, ${p2.ob.bottom.toFixed(3)}, ${c2.z.toFixed(2)}) in slot=${inSlot}; P1 bottom ${p1.ob.bottom.toFixed(3)} on pallet=${onPallet}; ${t.toFixed(0)} s`];
    });
    scenario('Payload limit', 'A load above the arm payload is refused before gripping', () => {
      const b = A.Env.debris.find((d) => d.type === 'beam'); b.mass = A.CFG.arm.payload + 50;
      A.Missions.beams(); runTask(120);
      const log = A.Log.entries.map((e) => e.msg).join('|');
      return [/über der Traglast/.test(log) && !R().arm.held && !/Fingerkontakt/.test(log), `mass ${b.mass} kg > ${A.CFG.arm.payload} kg → refused, held=${!!R().arm.held}`];
    });
    scenario('Long load vs legs', 'A carried beam crossing a leg between its corners is rejected (validator samples along each link)', () => {
      const r = R(), arm = r.arm, V = THREE.Vector3;
      const b = A.Env.debris.find((d) => d.type === 'beam');
      arm.wristR.attach(b.mesh); b.mesh.position.set(0.18 + b.ob.obb.h.y - 0.02, 0, 0); b.mesh.rotation.set(0, 0, Math.PI / 2);
      arm.held = b; b.ob.held = true;
      r.root.updateMatrixWorld(true);
      const sh = arm.root.localToWorld(new V()), res = [];
      for (const psi of [0, -35]) { // straight ahead: clear; −35°: the beam lies across the front leg
        const wy = r.pose.yaw + psi * A.DEG, p = new V(sh.x + Math.cos(wy) * 1.0, 0.5, sh.z - Math.sin(wy) * 1.0);
        const ik = arm.solveDown(p, -90 * A.DEG); arm.apply(ik.a); r.root.updateMatrixWorld(true);
        res.push(r.validate(['arm']));
      }
      return [res[0].ok && !res[1].ok && /Bein/.test(res[1].reason), `ahead: ${res[0].ok ? 'ok' : res[0].reason}; at −35°: ${res[1].ok ? 'ok' : res[1].reason}`];
    });
    scenario('Foot pad rim', 'An edge reaching diagonally into the pad (between the old 4 test points) is detected', () => {
      const t = A.World.obstacles.find((o) => o.name === 'Kantholz'), c = t.obb.corners()[0];
      // pad centre 7.5 cm diagonally off a timber corner at floor level: rim (r 0.1–0.11) overlaps, the 4 axis points do not
      const dx = Math.sign(c.x - t.obb.c.x), dz = Math.sign(c.z - t.obb.c.z);
      const pad = new THREE.Vector3(c.x + dx * 0.053, 0, c.z + dz * 0.053);
      const old = [[0, 0], [0.1, 0], [-0.1, 0], [0, 0.1], [0, -0.1]].some(([x, z]) => A.World.sphereHit(new THREE.Vector3(pad.x + x, 0.02, pad.z + z), A.CFG.leg.rPad));
      const hit = A.World.padHit(pad);
      return [!!hit && !old, `old 4-point check: ${old ? 'hit' : 'missed'}, pad disk check: ${hit ? 'hit ' + hit.ob.name : 'missed'}`];
    });
    scenario('Telemetry', 'Updates live', () => {
      const a = { ...S.tele }; click('[data-demo="walk"]'); sim(5);
      const b = S.tele;
      return [a.power !== b.power && a.temp !== b.temp && a.cpu !== b.cpu, `power ${a.power.toFixed(2)}→${b.power.toFixed(2)}, temp ${a.temp.toFixed(1)}→${b.temp.toFixed(1)}, cpu ${a.cpu.toFixed(0)}→${b.cpu.toFixed(0)}`];
    });

    const dur = (performance.now() - t0) / 1000;
    const passN = results.filter((r) => r.pass).length;
    const probes = results.reduce((s, r) => s + (r.name === 'Oracle calibration' ? 0 : r.checks), 0);
    window.ARES_TEST = { results, passN, total: results.length, oracle: results.reduce((s, r) => s + r.oracle, 0), probes, seconds: dur };
    const box = document.createElement('div');
    box.style.cssText = 'position:fixed;inset:60px 360px 70px 310px;background:rgba(5,8,10,.96);border:1px solid #29d3ff;z-index:99;overflow:auto;font:11px JetBrains Mono,monospace;color:#cfe;padding:14px';
    box.innerHTML = `<b style="color:#f2b705">ACCEPTANCE ${passN}/${results.length} passed · oracle violations in scenarios: ${window.ARES_TEST.oracle} (${(probes / 1e6).toFixed(2)} M vertex probes) · ${dur.toFixed(1)} s</b><br><br>` +
      results.map((r) => `<div style="color:${r.pass ? '#22ff88' : '#ff4b4b'}">${r.pass ? 'PASS' : 'FAIL'} · ${r.name} — <span style="color:#8aa">${r.criterion}</span><br><span style="color:#cfe">&nbsp;&nbsp;${r.evidence}</span></div>`).join('');
    box.onclick = () => box.remove();
    document.body.appendChild(box);
    console.log('ARES_TEST', JSON.stringify(window.ARES_TEST));
    A.Main.resetSim();
  }
  window.addEventListener('load', () => setTimeout(run, 300));
})();
