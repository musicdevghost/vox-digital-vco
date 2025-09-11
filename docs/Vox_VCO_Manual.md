# Vox VCO — Performer’s Manual (Draft v0.9)

*Morphing stereo unison oscillator with one FM/AM/RM jack, auto sub, granular shimmer, and an AUX accent output.*

> Optional figure for the PDF (place the PNG next to this .md before exporting):  
> `VOX_overlay_v2.png`

## TL;DR
Morph sets the waveform family; Timbre sets PWM/drive and also turns the FM jack into AM then RM near max; Spread sets unison voices and stereo width, adding Sub at small values and Granular shimmer at large values; AUX gives you a handy accent/follower envelope.

---

## Quick Start
- Patch **1V/Oct**, take **Audio L** for mono or **L/R** for stereo.
- Set **Morph** to noon, **Timbre** to ~10 o’clock, **Spread** fully down for a solid mono tone.
- Turn **Spread** to ~2–3 o’clock for wide unison. Past ~3 o’clock adds granular shimmer.
- Raise **Timbre** to repurpose the **FM** jack into **AM** then **RM**; near max you also get **Noisefold**.
- Feed a drum loop into **FM** at high Timbre for instant RM textures.
- Clock **SYNC** from your sequencer; use **AUX** to ping a VCA or filter for accents.

---

## Callout Map (bind these letters to your panel sketch)
**Knobs**
- **[A] PITCH (Macro A)** — top
- **[B] MORPH (Macro B)** — right
- **[C] SPREAD (Macro C)** — center
- **[D] TIMBRE (Macro D)** — left

**Attenuverters**
- **[E]–[H] CV ATTENUVERTERS** — row of four under the SPREAD knob (bipolar; center detent)

**Jacks (top → bottom, left → right)**
- **[J] SYNC**
- **[K] AUX (DAC)**
- **[L] TIMBRE CV**
- **[M] 1V/Oct**
- **[N] SPREAD CV**
- **[O] MORPH CV**
- **[P] HSYNC** (same behavior as SYNC; alternate placement on some revisions)
- **[Q] FM** (single multi‑role jack)
- **[R] OUT L**
- **[S] OUT R**

> If your sketch uses a different arrangement, keep the letters bound to these names and place them closest to the physical parts.

---

## Panel Tour

### Knobs
| Callout | Control | Range | Default | Behavior Notes |
|---|---|---:|---:|---|
| [B] | **Morph** | 0..1 | 0.25 | Crossfades core through zones: Sine (PD/foldable) → Triangle (gentle shaping) → Saw (BLEP) → Square (PWM) → folded‑aggressive. Equal‑power morph keeps level steady. |
| [D] | **Timbre** | 0..1 | 0.50 | PWM duty (~5–95%), adds drive/fold/chaos. Above ~0.80 the **FM** jack crossfades **FM→AM→RM**; near max also adds **Noisefold**. |
| [C] | **Spread** | 0..1 | 0.00 | Unison & stereo width with smooth pair activation: 0–0.125=1v, 0.125–0.375=3v, 0.375–0.625=5v, 0.625–1.0=7v. Small Spread blends **Sub**; large Spread adds **Granular shimmer**. |
| [A] | **Pitch (macro)** | ±12 semitones | 0 | Transposes the core; **1V/Oct** still applies for precise tracking. |

### CV Attenuverters
| Callouts | Function | Range | Default | Notes |
|---|---|---:|---:|---|
| [E]–[H] | **Bipolar attenuverters (4×)** | −100%..0..+100% | 0 (center detent) | CCW = invert (−); CW = attenuate (+). Designed for ~±5 V CV. |

### Jacks
| Callout | Jack | Level (typ) | Normal Role | Special Behavior (high Timbre) |
|---|---|---|---|---|
| [M] | **1V/Oct** | ~±5 V | Pitch CV | — |
| [J]/[P] | **SYNC / HSYNC** | 0–5 V | Hard/Soft sync (see thresholds) | Triggers the **Accent** envelope used by AUX |
| [Q] | **FM (single jack)** | ±5 V (HPF ~5 Hz) | Linear **FM** | Crossfades to **AM**, then **RM**, as **Timbre** rises to max |
| [R]/[S] | **Audio L/R** | ±5 V (nom) | Main outs | Level‑managed, soft‑saturated, DC‑protected |
| [K] | **AUX (DAC)** | 0..1 normalized (hardware scales) | Accent + Follower mix | Ping VCF/VCA, drive folders, sidechain |

**Switches:** none.

---

## Core Behaviors

### Sub Oscillator (auto)
- Mono sub‑square at −1 octave blends in when **Spread < ~0.15**, fading up until ~0.40.  
- Sub follows **Sync**: hard resets; soft gently recenters.

### Granular Shimmer (auto)
- When **Spread ≥ ~0.90** (±0.05 fade), unison voices get short windowed grains and random offsets from **{−12, 0, +12} semitones**.  
- Effect fades in/out smoothly as you pass the threshold.

### FM / AM / RM (single jack: **[Q] FM**)
- Input is **high‑passed ~5 Hz**; expects **±5 V** audio/CV.
- **Timbre ≤ ~0.80**: Linear **FM** (depth scales with Timbre).
- **~0.80 → 1.00**: FM depth **fades out** while the same jack **crossfades** to **AM**, then **RM** at the top. Musical, level‑managed transition.
- Summary: **One jack, three roles** under **Timbre**.

### Noisefold (auto at high Timbre)
- From **~0.88 → 1.00 Timbre**, the core crossfades into a **noisy fold** variant (lightly band‑limited noise mixed pre‑folder) for aggressive harmonics.  
- Equal‑power morph avoids big loudness jumps.

### Sync (**[J]/[P] SYNC**)
- **Hard Sync**: rising edge **> ~+2 V** resets oscillator phase.  
- **Soft Sync**: **0 V** positive crossing gently recenters phase.  
- **Sub** and **Accent** respond to Sync as well.

### AUX Accent (to **[K]** DAC)
- Each Sync triggers a fast **AD Accent** (hard > soft).  
- A **level follower** tracks the pre‑stereo Mid signal.  
- **AUX = Slew( (1 − Mix)*Accent + Mix*Follower )**, default Mix ≈ 0.45.  
- Great for patch‑level dynamics without extra modulators.

### Output & Protection
- Audio outs centered around 0 V with soft saturation and slight coloration.  
- DC‑protection and auto‑limiting around **±5 V peak** (Rack build).  
- Safe for typical Eurorack inputs.

---

## Patch Recipes

**1) Poly‑Unison Pad (stereo + shimmer)**
- **Morph**: 11–1 o’clock (triangle→saw edge)  
- **Timbre**: 9–10 o’clock (clean)  
- **Spread**: ~3 o’clock (wide; shimmer active)  
- **FM**: empty  
Take **L/R** outs. Slow chords to **1V/Oct**. Optional: slow LFO to **Pitch** (±2–3 semis).

**2) Dirty AM Bass (focused mono + grit)**
- **Spread**: minimum (mono + sub)  
- **Morph**: ~3 o’clock (square/PWM)  
- **Timbre**: ~0.85 (AM active, Noisefold starting)  
Patch a square LFO or tom audio into **FM**. Take **L** out, low‑pass at 200–400 Hz.

**3) RM Cymbals (noise‑metal textures)**
- **Morph**: noon→3 o’clock (saw→square)  
- **Timbre**: full CW (RM + Noisefold)  
- **Spread**: noon (5‑voice, moderate width)  
Feed a bright drum loop into **FM**. Take **L/R**. Add **SYNC** bursts for accent transients.

**4) Granular Shimmer Lead (wide, twinkly)**
- **Spread**: max (7‑voice + shimmer)  
- **Morph**: 10–12 o’clock (sine/triangle mix)  
- **Timbre**: 10–11 o’clock (clean PWM)  
Mild delay/reverb post‑VOX. Use **AUX** to ping a VCA for rhythmic emphasis tied to **SYNC**.

---

## Ranges (summary tables)

### Knobs
| Control | Range | Default | Notes |
|---|---|---:|---:|---|
| Morph | 0..1 | 0.25 | Sine→Tri→Saw→Square→Fold (equal‑power) |
| Timbre | 0..1 | 0.50 | PWM, drive/fold/chaos; enables Noisefold + FM→AM→RM |
| Spread | 0..1 | 0.00 | 1/3/5/7 voices + stereo width; Sub at small; Shimmer at large |
| Pitch (macro) | ±12 semitones | 0 | Transpose; 1V/Oct still active |

### Jacks
| Jack | Level | Normal Role | Special (high Timbre) |
|---|---|---|---|
| 1V/Oct | ~±5 V | Pitch CV | — |
| SYNC | 0–5 V (typ) | Hard/Soft sync | Triggers Accent envelope |
| FM | ±5 V (HPF ~5 Hz) | Linear FM | Crossfades to AM then RM |
| Audio L/R | ±5 V (nom) | Main outs | Level‑managed, soft‑saturated |
| AUX | 0..1 (normalized) | Accent + Follower mix (slewed) | Ping VCF/VCA, drive folders, sidechain |

### AUX/DAC
| Output | Range | Source | Use Cases |
|---|---|---|---|
| AUX | 0..1 (normalized; hardware scales) | `AUX = Slew( (1 − Mix)*Accent + Mix*Follower )`, default Mix ≈ 0.45 | Ping VCF/VCA, drive folders, sidechain tricks |

---

## Tech Specs
- **Core**: Band‑limited oscillator with PD/folding, BLEP saw/square, PWM.  
- **Internal sample rate**: 48 kHz (assumed).  
- **1V/Oct**: calibrated for standard tracking.  
- **FM input**: high‑pass ~5 Hz; expects ~±5 V.  
- **PWM duty**: ~5–95%.  
- **Outputs**: nominal ±5 V; soft saturation and limiter near ±5 V peak (Rack build).  
- **Unison**: 1/3/5/7 voices with equal‑power stereo width.  
- **AUX**: normalized 0..1 (hardware scales to chosen volts).

---

## Safety & Compatibility
- Designed for **Eurorack ±12 V** systems.  
- Keep inputs within **±10 V**.  
- Outputs are centered at 0 V, DC‑protected, and level‑managed.  
- For hardware builds, choose the **AUX** voltage range you prefer (e.g., 0–5 V or 0–8 V).

---

## Notes for Builders / Implementation Detail
Approximate thresholds (~ values) reflect musical fade‑ins/outs in the firmware: Spread’s Sub and Shimmer regions, Timbre’s FM→AM→RM hand‑off, and Noisefold onset.

---

## Print Tips
- Put `VOX_overlay_v2.png` under the title page for a visual tour.  
- Use 10–11 pt body and keep the tables intact when exporting to PDF.

## Also Available
If you want, I can provide a **1‑page quick reference** (panel + three tables + quick start). See the separate file in this folder.
