/* ARES-6 — A* path planner on a clearance grid built from the collision world and the robot's current pose */
(function () {
  'use strict';
  const A = window.ARES, W = A.World;
  const V = THREE.Vector3;

  class Heap {
    constructor() { this.a = []; }
    push(n, f) { const a = this.a; a.push([f, n]); let i = a.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (a[p][0] <= a[i][0]) break; [a[p], a[i]] = [a[i], a[p]]; i = p; } }
    pop() {
      const a = this.a, top = a[0], last = a.pop();
      if (a.length) { a[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < a.length && a[l][0] < a[m][0]) m = l; if (r < a.length && a[r][0] < a[m][0]) m = r; if (m === i) break; [a[m], a[i]] = [a[i], a[m]]; i = m; } }
      return top[1];
    }
    get size() { return this.a.length; }
  }

  A.Planner = {
    cell: 0.2, min: -16, n: 160, last: null,
    idx(x, z) { const i = Math.floor((x - this.min) / this.cell), j = Math.floor((z - this.min) / this.cell); return i < 0 || j < 0 || i >= this.n || j >= this.n ? -1 : j * this.n + i; },
    ctr(k) { const i = k % this.n, j = (k / this.n) | 0; return { x: this.min + (i + 0.5) * this.cell, z: this.min + (j + 0.5) * this.cell }; },

    // robot footprint profile for planning (relative to the support level)
    profile(R) {
      const p = R.pose;
      let reach = 0;
      for (const l of R.legs) { const n = R.neutralLocal(l, A.State.cmd.bodyHeight, A.State.cmd.stance); reach = Math.max(reach, Math.hypot(n.x, n.z)); }
      R.root.updateMatrixWorld(true);
      let top = R.headCenter(new V()).y + 0.25;
      top = Math.max(top, R.mastPoints().top.y + 0.1);
      for (const l of R.legs) { l.points(); top = Math.max(top, l.pts.knee.y + A.CFG.leg.rUpper); }
      const hb = A.CFG.hull.hy;
      return { R: reach + 0.24, bodyBottom: p.h - hb - 0.02 - Math.abs(p.pitch) * 0.9, top: top - p.gy + 0.06, climb: 0.33 };
    },

    buildGrid(prof, ignore) {
      const N = this.n * this.n, clr = new Float32Array(N).fill(99);
      const lo = 0.05, hi = prof.top;
      for (const ob of W.obstacles) {
        if (ob.held || (ignore && ignore.has(ob))) continue;
        if (ob.bottom >= hi) continue;                                   // overhead structure clears the robot
        if (ob.top <= prof.climb && ob.top < prof.bodyBottom - 0.06) continue; // legs step on/over it, chassis clears it
        const y0 = Math.max(ob.bottom, lo), y1 = Math.min(ob.top, hi);
        const ys = [y0, (y0 + y1) / 2, y1];
        const reach = ob.radius + prof.R + 0.6;
        const cx = ob.obb.c.x, cz = ob.obb.c.z;
        const i0 = Math.max(0, Math.floor((cx - reach - this.min) / this.cell)), i1 = Math.min(this.n - 1, Math.floor((cx + reach - this.min) / this.cell));
        const j0 = Math.max(0, Math.floor((cz - reach - this.min) / this.cell)), j1 = Math.min(this.n - 1, Math.floor((cz + reach - this.min) / this.cell));
        const p = new V(), q = new V();
        for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
          const x = this.min + (i + 0.5) * this.cell, z = this.min + (j + 0.5) * this.cell;
          let d = 99;
          for (const y of ys) { p.set(x, y, z); ob.obb.closest(p, q); d = Math.min(d, Math.hypot(q.x - x, q.z - z)); }
          const k = j * this.n + i;
          if (d < clr[k]) clr[k] = d;
        }
      }
      return clr;
    },

    // returns array of {x,z} or null
    plan(start, goal, prof, ignore) {
      const clr = this.buildGrid(prof, ignore), R = prof.R, n = this.n;
      const free = (k) => k >= 0 && clr[k] >= R;
      const nearestFree = (k, maxR) => {
        if (free(k)) return k;
        const c = this.ctr(k); let best = -1, bd = Infinity;
        const r = Math.ceil(maxR / this.cell), i0 = k % n, j0 = (k / n) | 0;
        for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
          const i = i0 + di, j = j0 + dj; if (i < 0 || j < 0 || i >= n || j >= n) continue;
          const kk = j * n + i; if (!free(kk)) continue;
          const cc = this.ctr(kk), d = Math.hypot(cc.x - c.x, cc.z - c.z); if (d < bd) { bd = d; best = kk; }
        }
        return best;
      };
      let s = this.idx(start.x, start.z), g = this.idx(goal.x, goal.z);
      if (s < 0 || g < 0) return null;
      const sFree = nearestFree(s, 1.2); g = nearestFree(g, 2.5);
      if (g < 0 || sFree < 0) { this.last = { clr, R, path: null }; return null; }
      const gC = this.ctr(g);
      const gs = new Float32Array(n * n).fill(Infinity), from = new Int32Array(n * n).fill(-1), closed = new Uint8Array(n * n);
      const h = (k) => { const c = this.ctr(k), dx = Math.abs(c.x - gC.x), dz = Math.abs(c.z - gC.z); return Math.max(dx, dz) + 0.414 * Math.min(dx, dz); };
      const heap = new Heap(); gs[s] = 0; heap.push(s, h(s));
      const nb = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, 1.414], [1, -1, 1.414], [-1, 1, 1.414], [-1, -1, 1.414]];
      let found = false, it = 0;
      while (heap.size && it++ < 60000) {
        const k = heap.pop(); if (closed[k]) continue; closed[k] = 1;
        if (k === g) { found = true; break; }
        const i = k % n, j = (k / n) | 0;
        for (const [di, dj, c] of nb) {
          const ii = i + di, jj = j + dj; if (ii < 0 || jj < 0 || ii >= n || jj >= n) continue;
          const kk = jj * n + ii;
          // leaving a blocked start region is allowed (robot may begin close to an obstacle)
          if (!free(kk) && !(gs[k] < 1.2 && clr[kk] >= clr[k])) continue;
          const pen = clr[kk] < R + 0.5 ? 0.8 : 0;
          const ng = gs[k] + c * this.cell + pen * this.cell;
          if (ng < gs[kk]) { gs[kk] = ng; from[kk] = k; heap.push(kk, ng + h(kk)); }
        }
      }
      if (!found) { this.last = { clr, R, path: null }; return null; }
      const cells = []; for (let k = g; k !== -1; k = from[k]) cells.push(k);
      cells.reverse();
      let pts = cells.map((k) => this.ctr(k));
      pts[0] = { x: start.x, z: start.z };
      // line-of-sight smoothing
      const los = (a, b) => {
        const d = Math.hypot(b.x - a.x, b.z - a.z), m = Math.ceil(d / 0.1);
        for (let t = 1; t < m; t++) { const k = this.idx(a.x + ((b.x - a.x) * t) / m, a.z + ((b.z - a.z) * t) / m); if (!free(k)) return false; }
        return true;
      };
      const out = [pts[0]]; let i = 0;
      while (i < pts.length - 1) {
        let j = pts.length - 1;
        while (j > i + 1 && !los(pts[i], pts[j])) j--;
        out.push(pts[j]); i = j;
      }
      this.last = { clr, R, path: out };
      return out;
    },
  };
})();
