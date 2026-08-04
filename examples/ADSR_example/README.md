# ADSR_example

Minimal **single-envelope** sketch. Wiring matches the DCO firmware pattern (table init, setters at boot, `noteOn`/`noteOff`/`getWave` in the loop).

## Instance

| Setting | Value | Notes |
|---------|-------|-------|
| `vertical_resolution` | 4095 | DCO EnvVCA / EnvVCF (`ADSR_CV_CC`) |
| Curve types | 1 / 2 / 1 | Attack / decay / release |
| `adsrBezierInitTables` maxVal | **4000** | DCO `ADSR_1_CC` — shared table scale, not the instance max |

Table generation uses **4000**; the envelope outputs **0..4095**. That is the same split as in [`DCO/adsr.h`](../../../DCO/adsr.h).

## Build and upload (Pico 2)

From the library root:

```bash
cd /path/to/ADSR_Bezier
arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --library . \
  examples/ADSR_example
arduino-cli upload \
  --fqbn rp2040:rp2040:rpipico2 \
  -p /dev/ttyACM0 \
  examples/ADSR_example
```

Open Serial Monitor at **115200 baud**. The sketch waits for the port, then prints `level=… / 4095` every 10 ms while cycling note on/off.

## Float backend

```bash
arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --library . \
  --build-property build.extra_flags=-DADSR_BEZIER_USE_FLOAT=1 \
  examples/ADSR_example
```

Or set `#define ADSR_BEZIER_USE_FLOAT 1` before `#include <ADSR_Bezier.h>`.

## See also

- [`../ADSR_benchmark/`](../ADSR_benchmark/) — self-test, speed bench, three EnvDCO/VCA/VCF instances (DCO load model)
- [`../compare_fixed_float/`](../compare_fixed_float/) — host math regression (no hardware)
