# ARES-6 — Sechsbeiniger Rettungsroboter (interaktive 3D-Simulation)

**Live:** https://jkegithub.github.io/ARES6/ — läuft direkt im Browser, auch auf Handy und Tablet.

Ein Katastrophen-Rettungsroboter mit sechs Beinen, Greifarm, Sensorkopf, Teleskopmast und
Erkundungsdrohne in einem prozeduralen Einsatzgelände (eingestürzter Tunnel, Lagerhalle,
Trümmerfeld, Kletterparcours). Alles wird im Code erzeugt — keine 3D-Modelle, kein Build-Schritt.

## Was die Simulation besonders macht

Jede Bewegung wird **vor** der Ausführung auf Kollisionen geprüft: Beine untereinander, Beine
und Arm gegen das Chassis, alles gegen Wände, Decken, Trümmer und Boden. Ungültige Bewegungen
werden sichtbar verhindert (Warnung, rot aufblitzendes Hindernis) statt durch Geometrie zu gleiten.

- Dreifußgang mit festen Standfüßen; jeder Tritt wird auf Reichweite, Gelenkgrenzen, Kollision und Schwungbahn geprüft
- Körper folgt dem Gelände (Rampe, Plateau, Stufen), Bewegung pausiert bei zu kleinem Stabilitätsrand
- Greifer schließt bis zum Fingerkontakt, Last wird bis zum Bodenkontakt abgesetzt
- Drohne startet nur bei freiem Luftraum und hält hindernisabhängige Höhe
- Kriechgang unter einer 1,0-m-Decke; Mast, Körper und Drohne werden dort gestoppt
- **Physik-Orakel:** eine zweite, unabhängige Prüfung zählt jede Durchdringung der gerenderten Geometrie (Kopfzeile)

## Bedienung

| | |
|---|---|
| **? TOUR** | geführte Tour in 9 Schritten (Deutsch/Englisch) |
| **Demo-Leiste** | ① Laufen · ② Lidar + Wärme · ③ Greifen · ④ Drohne · ⑤ Fehlertest/Stabilisieren · Klettern · Tunnel |
| **Missionen** | Tunnelinspektion · Eingestürzte Lagerhalle · Überlebendensuche · Trümmerräumung · Systemfehler-Test |
| **● AUFNAHME** | Video des ganzen Tabs oder nur des 3D-Fensters als `.webm` |
| Maus | links ziehen = drehen, rechts / Shift = verschieben, Rad = Zoom |
| Touch | ein Finger = drehen, zwei Finger = zoomen und verschieben |
| Tasten | `W` laufen · `S` stopp · `A`/`D` drehen · `1`–`5` Demos |

Auf dem Handy: unten **☰ STEUERUNG · 3D · TELEMETRIE**; Querformat empfohlen.

## Lokal starten

`index.html` im Browser öffnen genügt. Beim ersten Start wird Three.js (r128) von cdnjs geladen.
Optional ein cachefreier lokaler Server: `python tools/serve.py` → http://127.0.0.1:8642/

**Abnahmetest:** `index.html?test` (über einen Server) führt 22 automatische Prüfungen aus
(inkl. Kalibrierlauf des Orakels) und zählt dabei Orakel-Verstöße; Ergebnis als Overlay und in `window.ARES_TEST`.

## Aufbau

`js/core.js` Zustand, Log, Abläufe · `js/physics.js` Kollisionswelt · `js/environment.js` Gelände ·
`js/leg.js` `js/arm.js` Gelenkketten mit IK · `js/robot.js` Aufbau und Kollisionsprüfung ·
`js/loco.js` `js/planner.js` Gangart und A*-Planer · `js/sensors.js` `js/drone.js` ·
`js/missions.js` · `js/telemetry.js` · `js/oracle.js` · `js/ui.js` `js/tour.js` `js/recorder.js` · `js/main.js`

## Lizenz

MIT — siehe [LICENSE](LICENSE). Fremdkomponenten werden zur Laufzeit geladen und stehen unter
eigenen Lizenzen: Three.js r128 (MIT), Schriften Inter und JetBrains Mono über Google Fonts (SIL OFL 1.1).

## Grenzen

Kinematische, kollisionsgeprüfte Bewegung — keine Kräfte-Dynamik. Trümmer fallen nur senkrecht.
Felsen kollidieren als ihre Hüllbox. Das Orakel erkennt keine reinen Kante-Kante-Schnitte und prüft
in festen Abständen (100 ms, auf Handys 500 ms).
