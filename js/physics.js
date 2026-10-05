/* ARES-6 — collision world: oriented boxes, ramps, capsule/sphere/box queries, analytic raycasting */
(function () {
  'use strict';
  const A = window.ARES, U = A.U;
  const V = THREE.Vector3;
  const _l = new V(), _l2 = new V(), _w = new V();

  // ------------------------------------------------------------------ oriented bounding box
  class OBB {
    constructor(c, h, q) {
      this.c = c.clone(); this.h = h.clone();
      this.q = q ? q.clone() : new THREE.Quaternion();
      this.qi = this.q.clone().invert();
    }
    set(c, q) { this.c.copy(c); this.q.copy(q); this.qi.copy(q).invert(); return this; }
    toLocal(p, out) { return out.copy(p).sub(this.c).applyQuaternion(this.qi); }
    toWorld(l, out) { return out.copy(l).applyQuaternion(this.q).add(this.c); }
    distance(p) {
      const l = this.toLocal(p, _l);
      const dx = Math.max(Math.abs(l.x) - this.h.x, 0), dy = Math.max(Math.abs(l.y) - this.h.y, 0), dz = Math.max(Math.abs(l.z) - this.h.z, 0);
      return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    contains(p, m = 0) {
      const l = this.toLocal(p, _l);
      return Math.abs(l.x) <= this.h.x + m && Math.abs(l.y) <= this.h.y + m && Math.abs(l.z) <= this.h.z + m;
    }
    closest(p, out) {
      const l = this.toLocal(p, _l2);
      l.x = U.clamp(l.x, -this.h.x, this.h.x); l.y = U.clamp(l.y, -this.h.y, this.h.y); l.z = U.clamp(l.z, -this.h.z, this.h.z);
      return this.toWorld(l, out);
    }
    // surface sample points in local space, roughly `spacing` apart (corners always included)
    samples(spacing) {
      const n = (h) => Math.max(1, Math.ceil((2 * h) / spacing));
      const nx = n(this.h.x), ny = n(this.h.y), nz = n(this.h.z), out = [];
      for (let i = 0; i <= nx; i++) for (let j = 0; j <= ny; j++) for (let k = 0; k <= nz; k++) {
        if (i !== 0 && i !== nx && j !== 0 && j !== ny && k !== 0 && k !== nz) continue;
        out.push(new V(-this.h.x + (2 * this.h.x * i) / nx, -this.h.y + (2 * this.h.y * j) / ny, -this.h.z + (2 * this.h.z * k) / nz));
      }
      return out;
    }
    corners() {
      const o = [];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) o.push(this.toWorld(new V(sx * this.h.x, sy * this.h.y, sz * this.h.z), new V()));
      return o;
    }
    axes() { return [new V(1, 0, 0).applyQuaternion(this.q), new V(0, 1, 0).applyQuaternion(this.q), new V(0, 0, 1).applyQuaternion(this.q)]; }
    // exact box/box overlap (separating axis test, Ericson RTCD 4.4.1); m grows this box (negative m shrinks it)
    intersects(b, m = 0) {
      const ua = this.axes(), ub = b.axes();
      const ea = [this.h.x + m, this.h.y + m, this.h.z + m], eb = [b.h.x, b.h.y, b.h.z];
      const R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], AR = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { R[i][j] = ua[i].dot(ub[j]); AR[i][j] = Math.abs(R[i][j]) + 1e-7; }
      const d = b.c.clone().sub(this.c), t = [d.dot(ua[0]), d.dot(ua[1]), d.dot(ua[2])];
      for (let i = 0; i < 3; i++) if (Math.abs(t[i]) > ea[i] + eb[0] * AR[i][0] + eb[1] * AR[i][1] + eb[2] * AR[i][2]) return false;
      for (let j = 0; j < 3; j++) if (Math.abs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]) > ea[0] * AR[0][j] + ea[1] * AR[1][j] + ea[2] * AR[2][j] + eb[j]) return false;
      for (let i = 0; i < 3; i++) {
        const i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (let j = 0; j < 3; j++) {
          const j1 = (j + 1) % 3, j2 = (j + 2) % 3;
          const ra = ea[i1] * AR[i2][j] + ea[i2] * AR[i1][j], rb = eb[j1] * AR[i][j2] + eb[j2] * AR[i][j1];
          if (Math.abs(t[i2] * R[i1][j] - t[i1] * R[i2][j]) > ra + rb) return false;
        }
      }
      return true;
    }
    // ray/box slab test → [tEnter, tExit] or null
    slab(o, d) {
      const lo = this.toLocal(o, _l).clone();
      const ld = d.clone().applyQuaternion(this.qi);
      let t0 = -Infinity, t1 = Infinity;
      for (const ax of ['x', 'y', 'z']) {
        if (Math.abs(ld[ax]) < 1e-9) { if (Math.abs(lo[ax]) > this.h[ax]) return null; continue; }
        let a = (-this.h[ax] - lo[ax]) / ld[ax], b = (this.h[ax] - lo[ax]) / ld[ax];
        if (a > b) { const t = a; a = b; b = t; }
        t0 = Math.max(t0, a); t1 = Math.min(t1, b);
        if (t0 > t1) return null;
      }
      return [t0, t1];
    }
  }
  A.OBB = OBB;

  // ------------------------------------------------------------------ obstacle (static, debris, survivor)
  let _id = 0;
  class Obstacle {
    // opts: name, kind ('static'|'debris'|'survivor'), center, size, quat | yaw, standable, ramp:{H}
    constructor(o) {
      this.id = ++_id;
      this.name = o.name || `Obstacle-${this.id}`;
      this.kind = o.kind || 'static';
      this.standable = o.standable !== false;
      this.ramp = o.ramp || null;
      const q = o.quat || new THREE.Quaternion().setFromAxisAngle(new V(0, 1, 0), o.yaw || 0);
      this.obb = new OBB(o.center, new V(o.size.x / 2, o.size.y / 2, o.size.z / 2), q);
      this.flat = !o.quat; // yaw-only box → flat top usable as a surface
      this.mesh = o.mesh || null;
      this.held = false;
      this.refresh();
    }
    refresh() {
      const cs = this.obb.corners();
      this.top = Math.max(...cs.map((c) => c.y));
      this.bottom = Math.min(...cs.map((c) => c.y));
      this.radius = this.obb.h.length();
      this._samples = this.obb.samples(0.32);
    }
    yaw() { return new THREE.Euler().setFromQuaternion(this.obb.q, 'YXZ').y; }
    footprint(x, z, m = 0) {
      _w.set(x, this.obb.c.y, z);
      const l = this.obb.toLocal(_w, _l);
      return Math.abs(l.x) <= this.obb.h.x + m && Math.abs(l.z) <= this.obb.h.z + m;
    }
    rampY(lx) { // local-space surface height of a ramp at local x
      const h = this.obb.h, t = (U.clamp(lx, -h.x, h.x) + h.x) / (2 * h.x);
      return -h.y + this.ramp.H * t;
    }
    // world-space height of the walkable surface at (x,z) or null
    surfaceAt(x, z) {
      if (!this.standable || !this.flat || this.held) return null;
      if (!this.footprint(x, z)) return null;
      if (this.ramp) { _w.set(x, 0, z); const l = this.obb.toLocal(_w, _l); return this.obb.c.y + this.rampY(l.x); }
      return this.top;
    }
    distance(p) {
      if (!this.ramp) return this.obb.distance(p);
      const l = this.obb.toLocal(p, _l), h = this.obb.h;
      const dx = Math.max(Math.abs(l.x) - h.x, 0), dz = Math.max(Math.abs(l.z) - h.z, 0);
      const s = this.rampY(l.x);
      const dy = Math.max(l.y - s, 0) + Math.max(-h.y - l.y, 0);
      return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    contains(p, m = 0) { return this.distance(p) <= m; }
    rayHit(o, d, maxD) {
      const s = this.obb.slab(o, d);
      if (!s) return null;
      let [t0, t1] = s;
      if (t1 < 0 || t0 > maxD) return null;
      t0 = Math.max(t0, 0); t1 = Math.min(t1, maxD);
      if (!this.ramp) return t0;
      // ramp: march through the box to find the slope surface
      const inside = (t) => { _w.copy(o).addScaledVector(d, t); const l = this.obb.toLocal(_w, _l); return l.y <= this.rampY(l.x) + 1e-4; };
      if (inside(t0)) return t0;
      const N = 20; let prev = t0;
      for (let i = 1; i <= N; i++) {
        const t = t0 + ((t1 - t0) * i) / N;
        if (inside(t)) {
          let a = prev, b = t;
          for (let k = 0; k < 8; k++) { const m = (a + b) / 2; if (inside(m)) b = m; else a = m; }
          return b;
        }
        prev = t;
      }
      return null;
    }
    worldSamples() { return this._samples.map((s) => this.obb.toWorld(s, new V())); }
  }
  A.Obstacle = Obstacle;

  // ------------------------------------------------------------------ world queries
  const skipFn = (ex) => {
    if (!ex) return () => false;
    if (typeof ex === 'function') return ex;
    if (ex instanceof Set) return (o) => ex.has(o);
    return (o) => o === ex;
  };

  A.World = {
    obstacles: [],
    add(o) { this.obstacles.push(o); return o; },
    remove(o) { this.obstacles = this.obstacles.filter((x) => x !== o); },
    clearKind(kind) { this.obstacles = this.obstacles.filter((x) => x.kind !== kind); },
    near(p, r, ex) {
      const sk = skipFn(ex), out = [];
      for (const o of this.obstacles) {
        if (o.held || sk(o)) continue;
        const dx = o.obb.c.x - p.x, dy = o.obb.c.y - p.y, dz = o.obb.c.z - p.z, R = r + o.radius;
        if (dx * dx + dy * dy + dz * dz <= R * R) out.push(o);
      }
      return out;
    },
    // highest walkable surface at (x,z) not above yMax; floor (y=0) otherwise
    surfaceBelow(x, z, yMax = Infinity, ex = null) {
      const sk = skipFn(ex);
      let best = 0, ob = null;
      for (const o of this.obstacles) {
        if (sk(o)) continue;
        const s = o.surfaceAt(x, z);
        if (s !== null && s <= yMax + 1e-6 && s > best) { best = s; ob = o; }
      }
      return { y: best, ob };
    },
    sphereHit(p, r, ex) {
      for (const o of this.near(p, r, ex)) if (o.distance(p) < r) return o;
      return null;
    },
    capsuleHit(a, b, r, ex, list) {
      const len = a.distanceTo(b), n = Math.min(24, Math.max(2, Math.ceil(len / (r * 0.7))));
      const mid = _w.copy(a).add(b).multiplyScalar(0.5).clone();
      const cand = list || this.near(mid, len / 2 + r, ex);
      if (!cand.length) return null;
      const p = new V();
      for (let i = 0; i <= n; i++) {
        p.copy(a).lerp(b, i / n);
        for (const o of cand) if (o.distance(p) < r) return { ob: o, point: p.clone() };
      }
      return null;
    },
    // box vs obstacles: robot-box samples inside obstacles + obstacle samples inside the box
    obbHit(obb, samplesLocal, ex, margin = 0) {
      const cand = this.near(obb.c, obb.h.length() + margin, ex);
      if (!cand.length) return null;
      const p = new V();
      for (const s of samplesLocal) {
        obb.toWorld(s, p);
        for (const o of cand) if (o.distance(p) <= margin) return { ob: o, point: p.clone() };
      }
      for (const o of cand) {
        for (const s of o._samples) {
          o.obb.toWorld(s, p);
          if (obb.distance(p) <= margin) return { ob: o, point: p.clone() };
        }
      }
      return null;
    },
    // foot pad (disk r 0.1 + rim 0.01, 36 mm high) standing at contact point c: any obstacle reaching into it?
    // Yaw-only boxes exactly (footprint distance + height overlap above the contact plane), ramps by the surface
    // check of the caller, tilted boxes by a dense rim sampling.
    padHit(c, ex, R = 0.11, H = 0.036) {
      const p = new V();
      for (const o of this.near(c, R + 0.05, ex)) {
        if (o.ramp) continue;
        if (o.flat) {
          if (o.bottom >= c.y + H || o.top <= c.y + 0.004) continue;
          const l = o.obb.toLocal(p.set(c.x, o.obb.c.y, c.z), _l);
          const dx = Math.max(Math.abs(l.x) - o.obb.h.x, 0), dz = Math.max(Math.abs(l.z) - o.obb.h.z, 0);
          if (dx * dx + dz * dz < R * R) return { ob: o, point: o.obb.closest(p.set(c.x, c.y + H / 2, c.z), new V()) };
        } else {
          if (o.distance(p.set(c.x, c.y + 0.02, c.z)) < 0.016) return { ob: o, point: p.clone() };
          for (let i = 0; i < 24; i++) for (const rr of [0.06, R - 0.006]) for (const dy of [0.012, 0.028]) {
            const a = (i / 24) * Math.PI * 2;
            p.set(c.x + Math.cos(a) * rr, c.y + dy, c.z + Math.sin(a) * rr);
            if (o.distance(p) < 0.008) return { ob: o, point: p.clone() };
          }
        }
      }
      return null;
    },
    // exact box test against the world (carried loads); m < 0 tolerates resting contact
    boxHit(obb, ex, m = 0) {
      for (const o of this.near(obb.c, obb.h.length() + Math.max(m, 0), ex)) if (obb.intersects(o.obb, m)) return o;
      return null;
    },
    raycast(o, d, maxD = 30, ex = null, floor = true) {
      const sk = skipFn(ex);
      let best = maxD, hit = null;
      for (const ob of this.obstacles) {
        if (ob.held || sk(ob)) continue;
        // cheap reject: distance from ray to bounding sphere
        const cx = ob.obb.c.x - o.x, cy = ob.obb.c.y - o.y, cz = ob.obb.c.z - o.z;
        const t = cx * d.x + cy * d.y + cz * d.z;
        if (t < -ob.radius || t > best + ob.radius) continue;
        const px = cx - d.x * t, py = cy - d.y * t, pz = cz - d.z * t;
        if (px * px + py * py + pz * pz > ob.radius * ob.radius) continue;
        const th = ob.rayHit(o, d, best);
        if (th !== null && th < best) { best = th; hit = ob; }
      }
      if (floor && d.y < -1e-6) {
        const tf = -o.y / d.y;
        if (tf >= 0 && tf < best) { best = tf; hit = 'floor'; }
      }
      if (!hit) return null;
      return { dist: best, point: o.clone().addScaledVector(d, best), ob: hit };
    },
    // number of obstacles intersected along a segment (line-of-sight attenuation)
    occluders(a, b, ex) {
      const d = b.clone().sub(a), len = d.length(); d.normalize();
      const sk = skipFn(ex); let n = 0;
      for (const ob of this.obstacles) { if (ob.held || sk(ob)) continue; const t = ob.rayHit(a, d, len); if (t !== null) n++; }
      return n;
    },
  };
})();
