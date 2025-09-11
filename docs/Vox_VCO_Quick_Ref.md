# Vox VCO — Quick Reference (1 page)

![Panel overlay](VOX_overlay_v2.png)

## Quick Start
- 1V/Oct in. L/R out.
- Morph noon, Timbre 10 o’clock, Spread min.
- Spread to 2–3 o’clock for wide unison; >3 o’clock adds shimmer.
- High Timbre repurposes FM → AM → RM; adds Noisefold near max.
- Feed drums to FM at high Timbre for RM textures.
- SYNC from sequencer; use AUX to ping a VCA.

## Knobs
| Control | Range | Default | Notes |
|---|---|---:|---:|---|
| Morph | 0..1 | 0.25 | Sine→Tri→Saw→Square→Fold |
| Timbre | 0..1 | 0.50 | PWM/drive/fold/chaos; FM→AM→RM; Noisefold near max |
| Spread | 0..1 | 0.00 | 1/3/5/7 voices; Sub small; Shimmer large |
| Pitch (macro) | ±12 semis | 0 | Transpose; 1V/Oct still active |

## Jacks
| Jack | Level | Role | Special |
|---|---|---|---|
| 1V/Oct | ~±5 V | Pitch CV | — |
| SYNC | 0–5 V | Hard/Soft sync | Triggers Accent |
| FM | ±5 V; HPF ~5 Hz | FM | AM then RM at high Timbre |
| Audio L/R | ±5 V nom | Main outs | Soft sat + limiter |
| AUX | 0..1 (norm) | Accent + Follower | Ping/sidechain |

## Patch Seeds
- **Poly pad**: Spread ~3 o’clock, Morph 11–1 o’clock, clean Timbre.  
- **AM bass**: Spread min (sub), Morph square, Timbre ~0.85, audio→FM.  
- **RM cymbals**: Timbre max, Morph saw→square, Spread noon, drums→FM.  
- **Shimmer lead**: Spread max, Morph sine/tri, Timbre clean, AUX→VCA.
