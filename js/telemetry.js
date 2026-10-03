/* ARES-6 — telemetry simulator: power, thermal, hydraulics, compute, link, damage */
(function () {
  'use strict';
  const A = window.ARES, U = A.U;

  A.Telemetry = {
    reset() {
      A.State.tele = { power: 96.4, temp: 36, hyd: 182, cpu: 18, signal: 97, damage: 0, mode: 'MANUAL', stability: 0, torqueML: 0 };
    },
    update(dt) {
      const S = A.State, t = S.tele, T = S.toggles, R = A.robot, Lo = A.Loco;
      const on = S.powered, f = S.fault ? S.fault.severity : 0;
      const gait = Lo.gaitActive ? 1 : 0, air = A.DroneSys.state !== 'docked';
      // power
      const draw = on ? 0.006 + gait * 0.03 + (T.lidar ? 0.006 : 0) + (T.thermal ? 0.004 : 0) + (T.light ? 0.01 : 0) + (air ? 0.012 : 0) + (R.arm.held ? 0.01 : 0) : 0.001;
      t.power = Math.max(3, t.power - draw * dt * 6);
      // motor temperature
      const load = Math.abs(R.pose.h - 0.6) * 10 + (R.arm.held ? 4 : 0);
      const tgtTemp = on ? 36 + gait * 17 + load + f * 58 : 24;
      t.temp = U.damp(t.temp, tgtTemp, f > 0 ? 0.35 : 0.15, dt);
      // hydraulics
      const tgtHyd = on ? 182 + gait * 8 * Math.sin(performance.now() / 180) - f * 46 + (R.arm.held ? -6 : 0) : 0;
      t.hyd = U.damp(t.hyd, tgtHyd, 2, dt);
      // compute
      const tgtCpu = on ? 16 + (T.lidar ? 21 : 0) + (T.thermal ? 13 : 0) + (Lo.nav ? 12 : 0) + (air && T.droneFeed ? 14 : 0) + gait * 9 + (A.Tasks.running() ? 6 : 0) : 2;
      t.cpu = U.clamp(U.damp(t.cpu, tgtCpu + (Math.random() - 0.5) * 6, 3, dt), 0, 100);
      // link quality: distance + tunnel shielding, drone relays
      const dist = Math.hypot(R.pose.x, R.pose.z), inTunnel = R.pose.z < -6.2 && Math.abs(R.pose.x) < 2.4;
      const tgtSig = on ? U.clamp(99 - dist * 1.6 - (inTunnel ? 38 : 0) + (air ? 14 : 0), 5, 99) : 0;
      t.signal = U.damp(t.signal, tgtSig + (Math.random() - 0.5) * 3, 1.5, dt);
      // damage from fault + prevented collisions (minor contact stress)
      t.damage = U.clamp(Math.max(t.damage, f * 22) + 0, 0, 100);
      if (!S.fault) t.damage = U.damp(t.damage, Math.min(t.damage, S.stats.interventions * 0.4), 0.5, dt);
      t.mode = !on ? (S.booting ? 'BOOTING' : 'SHUTDOWN') : f > 0.05 ? 'FAULT' : T.autonomous ? 'AUTONOMOUS' : 'MANUAL';
      t.stability = Lo.margin;
      t.torqueML = R.legById.ML.load;
      if (f > 0.5) A.Log.once('temp', `Motortemperatur ${t.temp.toFixed(0)} °C — Antrieb ML gedrosselt`, 'warn', 6);
    },
  };
})();
