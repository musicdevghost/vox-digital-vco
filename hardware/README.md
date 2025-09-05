# Daisy firmware notes

- Audio: stereo in/out, 48kHz, block size 48 by default.
- Uses `VoxAudioCore` from `../src/dsp`.
- This target does **not** scan knobs by default. You can wire pots/CVs and map them to `VoxAudioCore::Params` in the callback.

## Build & Flash
```bash
export DAISY_DIR=~/daisy/libDaisy
export DAISYSP_DIR=~/daisy/DaisySP
make clean && make
make flash           # DFU via USB
```
