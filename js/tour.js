/* ARES-6 — guided tour: spotlight on one UI region per step, short explanation (DE/EN), optional "try it" action */
(function () {
  'use strict';
  const A = window.ARES;
  const $ = (id) => document.getElementById(id);
  const sec = (n) => document.getElementById('sec-' + n);

  const STEPS = [
    { el: () => $('viewport'),
      de: ['3D-Ansicht', 'Linke Maustaste ziehen dreht die Kamera, Mausrad zoomt, rechte Taste oder Shift verschiebt. Rechts oben: feste Ansichten — FOLGEN hält den Roboter im Bild.'],
      en: ['3D viewport', 'Left-drag orbits, wheel zooms, right-drag or Shift pans. Top right: fixed views — FOLLOW tracks the robot.'] },
    { el: () => $('demo-strip'),
      de: ['Demo-Leiste', 'Die fünf Höhepunkte mit einem Klick (auch Tasten 1–5). Ideal für Vorführungen.'],
      en: ['Demo strip', 'The five highlights in one click (keys 1–5 too). Made for presentations.'] },
    { el: () => sec('locomotion'), open: 'locomotion',
      de: ['Laufen', 'Dreifußgang: drei Beine stehen immer. Jeder Fußtritt wird vorher auf Reichweite und Kollision geprüft — der Roboter läuft nie durch Hindernisse.'],
      en: ['Walking', 'Tripod gait: three legs always stand. Every foothold is checked for reach and collision first — the robot never walks through obstacles.'],
      try: ['Losgehen', 'Start walking'], act: () => document.querySelector('[data-demo="walk"]').click() },
    { el: () => sec('sensors'), open: 'sensors',
      de: ['Sensoren', 'Lidar tastet die Umgebung mit echten Strahlen ab, das Wärmebild sieht eine Signatur im Tunnel — auch durch Wände. Der Scan schätzt die Überlebenswahrscheinlichkeit.'],
      en: ['Sensors', 'Lidar samples the scene with real rays, thermal sees a signature in the tunnel — even through walls. The scan estimates survivor probability.'],
      try: ['Scan starten', 'Run scan'], act: () => document.querySelector('[data-demo="scan"]').click() },
    { el: () => sec('robotic-arm'), open: 'robotic-arm',
      de: ['Greifarm', 'Die Greif-Demo fährt an, richtet die Finger aus, stoppt sie beim Kontakt, hebt den Block und setzt ihn seitlich ab.'],
      en: ['Robotic arm', 'The grab demo approaches, aligns the jaws, stops the fingers on contact, lifts the block and places it aside.'],
      try: ['Greifen', 'Grab'], act: () => A.Missions.grab() },
    { el: () => $('demo-strip'),
      de: ['Lasten handhaben', 'TRÄGER räumt drei 2,4-m-Stahlträger (je 99 kg) aus dem Hof auf einen Stapel — mit der Last vor dem Körper dreht sich der Roboter auf der Stelle. REGAL lagert ein Paket von der Palette ins Regal ein, holt ein anderes heraus und legt es seitlich ab.'],
      en: ['Load handling', 'BEAMS clears three 2.4 m steel beams (99 kg each) from the yard onto a stack — holding the load in front, the robot turns on the spot. RACK puts a parcel from the pallet into the rack, takes another one out and sets it down aside.'],
      try: ['Regal-Demo', 'Rack demo'], act: () => A.Missions.rack() },
    { el: () => sec('scout-drone'), open: 'scout-drone',
      de: ['Drohne', 'Startet senkrecht aus dem Dock, kreist in sicherer Höhe und liefert ein Kamerabild unten rechts. Unter einer Decke verweigert sie den Start.'],
      en: ['Scout drone', 'Launches vertically, orbits at a safe altitude and streams a camera feed bottom-right. Under a ceiling it refuses to launch.'],
      try: ['Drohne starten', 'Launch'], act: () => A.DroneSys.launch() },
    { el: () => sec('mission-presets'), open: 'mission-presets',
      de: ['Missionen', 'Fertige Abläufe: Tunnel im Kriechgang, Lagerhalle, Suche, Trümmer. Der Systemfehler-Test erzeugt Warnungen — danach „Auto Stabilize“.'],
      en: ['Missions', 'Scripted runs: crawl through the tunnel, warehouse, search, debris. The failure test raises warnings — then press “Auto Stabilize”.'],
      try: ['Fehlertest', 'Failure test'], act: () => A.Missions.run('failure') },
    { el: () => $('right'),
      de: ['Telemetrie', 'Live-Werte, Beinstatus, Minikarte und Missionslog. Oben in der Kopfzeile: KOLLISIONSSCHUTZ zählt verhinderte Kollisionen, PHYSIK-ORAKEL prüft unabhängig auf Durchdringungen.'],
      en: ['Telemetry', 'Live values, leg status, minimap and mission log. In the top bar: COLLISION GUARD counts prevented collisions, PHYSICS ORACLE independently audits for penetrations.'] },
    { el: () => $('rec-btn'),
      de: ['Aufnahme', 'AUFNAHME zeichnet den ganzen Tab mit Panels auf (Browser fragt nach — „Dieser Tab“ wählen) oder nur das 3D-Fenster. Das Video landet im Download-Ordner.'],
      en: ['Recording', 'REC captures the whole tab with panels (the browser asks — pick “this tab”) or the 3D view only. The video goes to your downloads.'] },
  ];

  const Tour = (A.Tour = {
    i: 0, lang: 'de',
    init() {
      $('tour-btn').onclick = () => this.open(0);
      $('tour-next').onclick = () => (this.i < STEPS.length - 1 ? this.show(this.i + 1) : this.close());
      $('tour-prev').onclick = () => this.i > 0 && this.show(this.i - 1);
      $('tour-close').onclick = () => this.close();
      $('tour-lang').onclick = () => { this.lang = this.lang === 'de' ? 'en' : 'de'; this.show(this.i); };
      $('tour-try').onclick = () => { const s = STEPS[this.i]; if (s.act) { try { s.act(); } catch (e) { console.error(e); } } };
      window.addEventListener('resize', () => !$('tour').classList.contains('hidden') && this.place());
      // phone drawers slide in: re-measure once the slide has finished
      for (const id of ['left', 'right']) $(id).addEventListener('transitionend', () => !$('tour').classList.contains('hidden') && this.place());
      window.addEventListener('keydown', (e) => { if (e.key === 'Escape') this.close(); });
      try { if (!localStorage.getItem('ares6-tour-seen')) setTimeout(() => this.hint(), 1500); } catch (e) { /* storage blocked */ }
    },
    hint() { $('tour-btn').classList.add('pulse'); },
    open(i) {
      $('tour').classList.remove('hidden'); $('tour-btn').classList.remove('pulse');
      try { localStorage.setItem('ares6-tour-seen', '1'); } catch (e) { /* ignore */ }
      this.show(i);
    },
    close() { $('tour').classList.add('hidden'); if (document.body.classList.contains('mobile')) A.UI.drawer(null); },
    show(i) {
      this.i = i;
      const s = STEPS[i], t = s[this.lang];
      // phone layout: slide in the drawer that holds the target
      if (document.body.classList.contains('mobile')) {
        const el = s.el();
        A.UI.drawer(el.closest('#left') ? 'left' : el.closest('#right') ? 'right' : null);
        setTimeout(() => this.place(), 320); // fallback if no transition runs (drawer already open)
      }
      if (s.open) { const el = sec(s.open); el.classList.remove('closed'); el.scrollIntoView({ block: 'nearest' }); }
      $('tour-title').textContent = t[0];
      $('tour-text').textContent = t[1];
      $('tour-step').textContent = `${i + 1} / ${STEPS.length}`;
      $('tour-lang').textContent = this.lang === 'de' ? 'EN' : 'DE';
      $('tour-prev').disabled = i === 0;
      $('tour-next').textContent = i === STEPS.length - 1 ? (this.lang === 'de' ? 'Fertig' : 'Done') : (this.lang === 'de' ? 'Weiter ›' : 'Next ›');
      $('tour-prev').textContent = this.lang === 'de' ? '‹ Zurück' : '‹ Back';
      const tr = $('tour-try');
      tr.classList.toggle('hidden', !s.act);
      if (s.try) tr.textContent = '▶ ' + s.try[this.lang === 'de' ? 0 : 1];
      requestAnimationFrame(() => this.place());
      this.place();
    },
    // spotlight box over the target, card next to it (inside the window)
    place() {
      const el = STEPS[this.i].el(), r = el.getBoundingClientRect(), pad = 6;
      const spot = $('tour-spot'), card = $('tour-card');
      Object.assign(spot.style, { left: r.left - pad + 'px', top: r.top - pad + 'px', width: r.width + pad * 2 + 'px', height: r.height + pad * 2 + 'px' });
      const cw = card.offsetWidth, ch = card.offsetHeight, W = window.innerWidth, H = window.innerHeight;
      // prefer beside the target; only big regions without room beside get the card inside
      let x, y = r.top;
      if (r.right + 16 + cw <= W - 10) x = r.right + 16;
      else if (r.left - 16 - cw >= 10) x = r.left - cw - 16;
      else if (r.height > H * 0.6 && r.width > W * 0.4) { x = r.left + (r.width - cw) / 2; y = r.top + 70; }
      else { x = Math.min(Math.max(10, r.left), W - cw - 10); y = r.bottom + 16; if (y + ch > H - 10) y = Math.max(10, r.top - ch - 16); }
      card.style.left = Math.round(x) + 'px'; card.style.top = Math.round(Math.max(10, Math.min(y, H - ch - 10))) + 'px';
    },
  });

  window.addEventListener('load', () => Tour.init());
})();
