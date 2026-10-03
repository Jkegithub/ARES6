/* ARES-6 — video recorder: whole tab incl. panels (getDisplayMedia) or 3D viewport only (canvas.captureStream).
   Result is downloaded as .webm (or .mp4 where only that is supported). Nothing leaves the machine. */
(function () {
  'use strict';
  const A = window.ARES;
  const $ = (id) => document.getElementById(id);

  const Rec = (A.Recorder = {
    rec: null, chunks: [], stream: null, t0: 0, kind: '', timer: null,

    canTab() { return !!(navigator.mediaDevices && navigator.mediaDevices.getDisplayMedia) && window.isSecureContext; },
    canCanvas() { return typeof MediaRecorder !== 'undefined' && !!HTMLCanvasElement.prototype.captureStream; },
    mime() {
      for (const m of ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm', 'video/mp4']) if (MediaRecorder.isTypeSupported(m)) return m;
      return '';
    },

    init() {
      const btn = $('rec-btn'), menu = $('rec-menu');
      btn.onclick = (e) => { e.stopPropagation(); if (this.rec) this.stop(); else menu.classList.toggle('hidden'); };
      document.addEventListener('click', () => menu.classList.add('hidden'));
      const tab = $('rec-tab'), cnv = $('rec-canvas');
      if (!this.canTab()) { tab.disabled = true; tab.title = 'Von diesem Browser/Gerät nicht unterstützt (z. B. Handy) — „Nur 3D-Fenster“ oder die Bildschirmaufnahme des Handys nutzen'; }
      if (!this.canCanvas()) { cnv.disabled = true; btn.disabled = true; btn.title = 'Videoaufnahme wird nicht unterstützt'; }
      tab.onclick = () => this.start('tab');
      cnv.onclick = () => this.start('canvas');
    },

    async start(kind) {
      $('rec-menu').classList.add('hidden');
      try {
        if (kind === 'tab') {
          this.stream = await navigator.mediaDevices.getDisplayMedia({ video: { frameRate: 30 }, audio: false, preferCurrentTab: true, selfBrowserSurface: 'include' });
        } else {
          this.stream = $('c').captureStream(30);
        }
      } catch (e) {
        A.Log.add(`Aufnahme abgebrochen (${e.name || e.message})`, 'warn');
        return;
      }
      const mime = this.mime();
      try { this.rec = new MediaRecorder(this.stream, mime ? { mimeType: mime, videoBitsPerSecond: 8e6 } : undefined); }
      catch (e) { A.Log.add('Aufnahme konnte nicht starten: ' + e.message, 'err'); this.release(); return; }
      this.chunks = []; this.kind = kind; this.t0 = performance.now();
      this.rec.ondataavailable = (ev) => ev.data && ev.data.size && this.chunks.push(ev.data);
      this.rec.onstop = () => this.save();
      // user ended the share via the browser UI
      this.stream.getVideoTracks().forEach((t) => (t.onended = () => this.stop()));
      this.rec.start(1000);
      $('rec-btn').classList.add('live');
      this.timer = setInterval(() => this.tick(), 250);
      A.Log.add(`Aufnahme läuft — ${kind === 'tab' ? 'ganzer Tab mit Panels' : 'nur 3D-Fenster'}`, 'ok');
    },
    tick() {
      const s = (performance.now() - this.t0) / 1000;
      $('rec-label').textContent = `AUFNAHME ${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')} · STOPP`;
    },
    stop() {
      if (!this.rec) return;
      if (this.rec.state !== 'inactive') this.rec.stop();
      clearInterval(this.timer);
    },
    release() {
      if (this.stream) this.stream.getTracks().forEach((t) => t.stop());
      this.stream = null; this.rec = null;
      $('rec-btn').classList.remove('live'); $('rec-label').textContent = 'AUFNAHME';
    },
    save() {
      const type = (this.rec && this.rec.mimeType) || 'video/webm';
      const ext = type.includes('mp4') ? 'mp4' : 'webm';
      const blob = new Blob(this.chunks, { type });
      const secs = ((performance.now() - this.t0) / 1000).toFixed(0);
      const d = new Date(), pad = (n) => String(n).padStart(2, '0');
      const stamp = `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
      const mission = (A.State.mission.name || 'free').replace(/[^A-Z0-9]+/gi, '_');
      const name = `ARES6_${mission}_${stamp}.${ext}`;
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a'); a.href = url; a.download = name; document.body.appendChild(a); a.click(); a.remove();
      setTimeout(() => URL.revokeObjectURL(url), 60000);
      A.Log.add(`Aufnahme gespeichert: ${name} (${secs} s, ${(blob.size / 1e6).toFixed(1)} MB) → Download-Ordner`, 'ok');
      this.release();
    },
  });

  window.addEventListener('load', () => Rec.init());
})();
