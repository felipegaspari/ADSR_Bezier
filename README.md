## ADSR Bezier (millis/micros, dual math backend)

ADSR Bezier is a lightweight, digitally‑controlled envelope generator based on precomputed Bézier lookup tables.  
It is designed to be:

- **Fast at runtime** (fixed-point Q24/Q16 by default, RP2040‑friendly).
- **Portable** (optional optimized integer backend via `ADSR_BEZIER_USE_FLOAT` for RP2350).
- **Flexible in timing** (supports both `millis()` and `micros()` timebases).
- **Curve‑shaped** (attack/decay/release follow user‑defined Bézier curves).

You can use it as a drop‑in ADSR for any microcontroller synth; the example in this repo is a multi‑voice DCO synth on an RP2040.

For a conceptual introduction to ADSR and this style of lookup‑based envelopes, see the original author’s video:  
[YouTube – mo‑thunderz ADSR class](https://youtu.be/oMxui9rar9M)

---

## 1. Overview

- **Class**: `adsr` (defined in `ADSR_Bezier.h`).
- **Output**: integer envelope level from `0` to `vertical_resolution` (e.g. `0…4000`).
- **Time parameters**: `attack`, `decay`, `release` are set in **milliseconds**.
- **Timebase**: `ADSR_BEZIER_USE_MICROS` (micros vs millis).
- **Math backend**: `ADSR_BEZIER_USE_FLOAT` (0 = Q24/Q16 fast approximation, 1 = mul+shift index + round-nearest Q16 output).
- **Curves**:
  - Attack, decay and release each read from a Bézier‑generated lookup table.
  - 8 different curve types are supported (`0…7`), selected separately for A/D/R.

Internally, each call to `getWave()`:

1. Computes the elapsed time since `noteOn()` / `noteOff()` using the chosen timebase.
2. Converts elapsed time to a **table index** (Q24 fast path when `FLOAT=0`; mul+shift reciprocal when `FLOAT=1`).
3. Reads the appropriate table value for the current stage (attack uses pre-reversed tables).
4. Linearly maps that curve value to the requested output range (Q16 trunc when `FLOAT=0`; round-nearest Q16 when `FLOAT=1`).

Reciprocals and scales are precomputed in setters; the hot path has no runtime integer division.

---

## 1b. Math backend (`ADSR_BEZIER_USE_FLOAT`)

Select at compile time before including the header:

```cpp
#define ADSR_BEZIER_USE_FLOAT 0   // default: Q24/Q16 fast approximation (RP2040)
// #define ADSR_BEZIER_USE_FLOAT 1 // mul+shift index + round-nearest Q16 output (RP2350 / Pico 2)
#include <ADSR_Bezier.h>
```

| Value | Hot path | Best for |
|-------|----------|----------|
| `0` (default) | Q24 index + Q16 range | RP2040 / Cortex-M0+ — fastest integer path, ±1 step vs golden |
| `1` | Mul+shift index reciprocal (all phase lengths); round-nearest Q16 output | RP2350 / Pico 2; ±1 index/output step vs golden |

The optimized backend (`FLOAT=1`) precomputes index division reciprocals and per-phase Q16 output scales in setters/noteOn/noteOff. `getWave()` uses **integer-only** math: index via `(delta × precomputed_mul) >> shift` (uint64 divide only if reciprocal search fails); output via `(curve × range_q16_rn + 32768) >> 16`. No FPU in `getWave()`. Attack reads pre-reversed curve tables (no runtime index inversion).

**Branches:** `main` is canonical (dual backend). `fixed-point-version` and `float-version` are legacy aliases — use `main` with the define above.

---

## 1c. Examples and testing

| Layer | Example | What it validates |
|-------|---------|-------------------|
| **Host math** | [`examples/compare_fixed_float/`](examples/compare_fixed_float/) | Fixed vs reference index/output formulas (no hardware) |
| **Device integration** | [`examples/ADSR_benchmark/`](examples/ADSR_benchmark/) | Real `adsr` class, phase state machine, `getWave()` cycle timing |
| **Basic usage** | [`examples/ADSR_example/`](examples/ADSR_example/) ([README](examples/ADSR_example/README.md)) | Single envelope, DCO-style boot + loop |

**Host regression** (fixed vs reference math):

```bash
cd examples/compare_fixed_float
g++ -std=c++17 -O2 -o compare compare.cpp && ./compare
```

**On-device self-test + speed bench** (Pico 2):

```bash
arduino-cli compile \
  --fqbn rp2040:rp2040:rpipico2 \
  --library . \
  examples/ADSR_benchmark
```

Open Serial Monitor at **115200** after upload. The benchmark waits until the port is open, then prints PASS/FAIL self-tests and cycles per `getWave()` for attack/decay/sustain/release with **N instances** per iteration (default **1**; set `ADSR_BENCHMARK_INSTANCES` to **3** for DCO EnvDCO + EnvVCA + EnvVCF). At ~10 kHz voice updates with N=3 that is ~30k `getWave()` calls/s — multiply **µs per call** from the benchmark by `10000 × N` to estimate envelope CPU load.

Compare backends: rebuild with `--build-property build.extra_flags=-DADSR_BEZIER_USE_FLOAT=1` or uncomment the define in the sketch. See [`examples/ADSR_benchmark/README.md`](examples/ADSR_benchmark/README.md) for upload and float-toggle details.

---

## 2. Installation

The library is **header-only** (`library.properties` lists `includes=ADSR_Bezier.h` only).

### 2.1. As a generic Arduino library

1. Create a folder in your Arduino libraries directory, e.g. `ADSR_Bezier`.
2. Copy `ADSR_Bezier.h` and `library.properties` into that folder.
3. In your sketch, define `ARRAY_SIZE` **before** the include, then:

```cpp
#define ARRAY_SIZE 512
#include <ADSR_Bezier.h>
```

### 2.2. Inside the DCO monorepo

The DCO firmware pulls this repo in via a symlink:

- `DCO/_build_libs/ADSR_Bezier` → `../../ADSR_Bezier`
- [`DCO/adsr.h`](../DCO/adsr.h) sets `ARRAY_SIZE`, optional `ADSR_BEZIER_USE_FLOAT`, and `#include <ADSR_Bezier.h>`
- Boot calls `init_ADSR()` from the main sketch (table init + setters on all three envelope instances)

You do not need a separate `.cpp` or `adsrCreateTables` — call `adsrBezierInitTables()` once at startup (see §5.2).

### 2.3. Quick start (standalone sketch)

Minimal pattern (see [`examples/ADSR_example/`](examples/ADSR_example/)):

```cpp
#define ARRAY_SIZE 512
#ifndef ADSR_BEZIER_USE_FLOAT
#define ADSR_BEZIER_USE_FLOAT 0
#endif
#include <ADSR_Bezier.h>

static adsr env(4095, 0.9995f, 0.9995f, false, 1, 2, 1);

void setup() {
  adsrBezierInitTables(4000, ARRAY_SIZE, _curve_tables);
  env.setAttack(100);
  env.setDecay(200);
  env.setSustain(2000);   // 0 .. vertical_resolution
  env.setRelease(300);
  env.setResetAttack(true);
}

void loop() {
  env.noteOn();            // trigger as needed
  int level = env.getWave();
}
```

Table `maxVal` (4000 in DCO) can differ from instance `vertical_resolution` (4095 for EnvVCA/EnvVCF).

---

## 3. Public API

The main class lives in `ADSR_Bezier.h`:

```cpp
class adsr {
public:
    adsr(int vertical_resolution,
         float attack_alpha,
         float attack_decay_release,
         bool bezier,
         int bezier_attack_type,
         int bezier_decay_type,
         int bezier_release_type);

    void setAttack(unsigned long attack_ms);
    void setDecay(unsigned long decay_ms);
    void setSustain(int sustain_level);
    void setRelease(unsigned long release_ms);

    void setResetAttack(bool reset_attack);

    void adsrCurveAttack(uint8_t curveType);
    void adsrCurveDecay(uint8_t curveType);
    void adsrCurveRelease(uint8_t curveType);

    void noteOn();    // uses millis()/micros() internally
    void noteOff();   // uses millis()/micros() internally

    int getWave();    // returns current envelope level
};
```

### 3.1. Constructor

- **`vertical_resolution`**: maximum envelope value (e.g. `4000` for EnvDCO, `4095` for EnvVCA/EnvVCF in DCO).
- **`attack_alpha`, `attack_decay_release`**: legacy exponential-curve parameters; unused after `adsrBezierInitTables()` — DCO passes nominal values like `0.9995f`.
- **`bezier`**: legacy flag. Both paths use the shared global `_curve_tables` filled by `adsrBezierInitTables()`. DCO uses `false`.
- **Curve types** (`bezier_attack_type`, `bezier_decay_type`, `bezier_release_type`):  
  Indices into `_curve_tables[8]` (0–7). The constructor assigns internal `_bezier_*_type` fields from these args. Change curves at runtime with `adsrCurveAttack()`, `adsrCurveDecay()`, and `adsrCurveRelease()` (as in DCO curve helpers).

### 3.2. Time parameters (milliseconds)

All three functions take **milliseconds** and internally convert to the compiled timebase:

- **`void setAttack(unsigned long attack_ms)`**
- **`void setDecay(unsigned long decay_ms)`**
- **`void setRelease(unsigned long release_ms)`**

When `ADSR_BEZIER_USE_MICROS == 1`:

- 1 tick = 1 µs, so `attack_ms` is multiplied by 1000 internally.

When `ADSR_BEZIER_USE_MICROS == 0`:

- 1 tick = 1 ms, and `attack_ms` is used as‑is.

### 3.3. Sustain level

- **`void setSustain(int sustain_level)`**  
  Saturates into the range `[0, vertical_resolution]`.  
  This value is used as the target level for the decay stage and the hold level after decay.

### 3.4. Reset attack behavior

- **`void setResetAttack(bool reset)`**
  - `true`: every `noteOn()` starts from `0`.
  - `false`: `noteOn()` starts from the current envelope level (legato behavior).

### 3.5. Triggering

- **`void noteOn()`**:
  - Captures the current time (`micros()` or `millis()` depending on the flag).
  - Sets up internal state for the attack stage.
  - Precomputes the **attack range scale** for output mapping.

- **`void noteOff()`**:
  - Decrements the note counter.
  - When all notes are off, captures the current time and starts the release stage.
  - Precomputes the **release range scale** for output mapping.

### 3.6. Reading the envelope

- **`int getWave()`**:
  - Reads the current timestamp using the selected timebase.
  - Updates `_adsr_output` according to the ADSR state machine.
  - Returns the current envelope value (integer) in `[0, vertical_resolution]`.

You typically call `getWave()` at a fixed control rate (for example, every 100–200 µs on RP2040), and use the returned value to modulate oscillator amplitude, PWM, detune, etc.

---

## 4. Timebase selection (millis vs micros)

Timebase is selected at compile time with `ADSR_BEZIER_USE_MICROS`:

- **Micros mode (default)**:

```cpp
#define ARRAY_SIZE 512
#define ADSR_BEZIER_USE_MICROS 1
#include <ADSR_Bezier.h>
```

  - Internally uses `micros()` for timing.
  - Time resolution: 1 µs.
  - Parameter units remain **milliseconds**; conversion is done behind the scenes.

- **Millis mode (backwards‑compatible)**:

```cpp
#define ARRAY_SIZE 512
#define ADSR_BEZIER_USE_MICROS 0
#include <ADSR_Bezier.h>
```

  - Internally uses `millis()` for timing.
  - Coarser resolution, but closer to the original Arduino ADSR examples.

The rest of your code (parameter units, `noteOn()`, `getWave()`) does not change between modes.

---

## 5. Example: DCO synth ADSR integration (RP2040)

This section mirrors the live DCO firmware ([`DCO/adsr.h`](../DCO/adsr.h), [`DCO/adsr.ino`](../DCO/adsr.ino)).

### 5.1. Global ADSR configuration (`adsr.h`)

Each voice carries **three** envelope instances: EnvDCO (pitch/PW), EnvVCA, EnvVCF. Static prototypes are copied into `ADSRStruct`:

```cpp
#define ARRAY_SIZE 512
#ifndef ADSR_BEZIER_USE_FLOAT
#define ADSR_BEZIER_USE_FLOAT 0
#endif
#include <ADSR_Bezier.h>

static constexpr uint16_t ADSR_1_CC = 4000;
static constexpr uint16_t ADSR_CV_CC = 4095;

adsr adsr1_voice_0(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false, 7, 7, 7);
adsr adsr_vca_voice_0(ADSR_CV_CC, ADSR_VCA_curve1, ADSR_VCA_curve2, false, 1, 2, 1);
adsr adsr_vcf_voice_0(ADSR_CV_CC, ADSR_VCF_curve1, ADSR_VCF_curve2, false, 4, 6, 1);

struct ADSRStruct {
  adsr adsr1_voice;
  adsr adsr_vca_voice;
  adsr adsr_vcf_voice;
};

ADSRStruct ADSRVoices[] = {
  { adsr1_voice_0, adsr_vca_voice_0, adsr_vcf_voice_0 },
};
```

Notes:

- `adsrBezierInitTables` uses **`ADSR_1_CC` (4000)** for table generation; EnvVCA/EnvVCF still output **0..4095**.
- DCO uses `bezier=false`; curve shapes come from `_curve_tables` after init.

### 5.2. Initialization (`adsr.ino`)

Boot builds tables and applies setters to all three envs per voice — **no `noteOn()` at boot**:

```cpp
void init_ADSR() {
  adsrBezierInitTables(ADSR_1_CC, ARRAY_SIZE, _curve_tables);

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);
    ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);
    ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);
    ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);

    ADSRVoices[i].adsr_vca_voice.setAttack(ADSR_VCA_attack);
    // ... decay, sustain, release, setResetAttack for VCA and VCF ...
  }
}
```

### 5.3. Per‑voice update loop (~10 kHz)

Note edges only — **no setter spam on `noteOn`** (params stay current via `ADSR_set_parameters` / init / curve helpers). Three `getWave()` calls per voice:

```cpp
inline void ADSR_update() {
  tADSR = millis();
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    if (noteEnd[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff();
      ADSRVoices[i].adsr_vca_voice.noteOff();
      ADSRVoices[i].adsr_vcf_voice.noteOff();
      noteEnd[i] = 0;
    } else if (noteStart[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff();
      ADSRVoices[i].adsr1_voice.noteOn();
      ADSRVoices[i].adsr_vca_voice.noteOff();
      ADSRVoices[i].adsr_vca_voice.noteOn();
      ADSRVoices[i].adsr_vcf_voice.noteOff();
      ADSRVoices[i].adsr_vcf_voice.noteOn();
      noteStart[i] = 0;
    }
    ADSR1Level[i] = ADSRVoices[i].adsr1_voice.getWave();
    ADSR_VCA_Level[i] = ADSRVoices[i].adsr_vca_voice.getWave();
    ADSR_VCF_Level[i] = ADSRVoices[i].adsr_vcf_voice.getWave();
  }
  ADSR_set_parameters();
}
```

### 5.4. Parameter updates at low rate

Debounced push for EnvDCO **and** EnvVCA/EnvVCF A/D/S/R when control values change (~200 Hz):

```cpp
inline void ADSR_set_parameters() {
  if ((tADSR - tADSR_params) > 5) {
    // Compare ADSR1_* and ADSR_VCA_* / ADSR_VCF_* against static last_* caches
    // On change, setAttack/setDecay/setSustain/setRelease on all voices
    tADSR_params = tADSR;
  }
}
```

See full implementation in [`DCO/adsr.ino`](../DCO/adsr.ino) (EnvVCA/EnvVCF debounce blocks). Parameter refresh is **not** done on every note edge.

---

## 6. Internal workings (high‑level)

### 6.1. Bézier precomputation

- Bézier curves are defined by:
  - Start point `A = (0, maxVal)`
  - End point `B = (maxVal, 0)`
  - Two control points `P1`, `P2` per curve type.
- For each of the 8 curve types and each `i` in `[0, ARRAY_SIZE-1]`:
  1. Compute a target `x` value along the line.
  2. Use a binary search (`findYForX`) along the Bézier parameter `t` to find the point where the Bézier curve’s `x` matches `xTarget`.
  3. Store the corresponding `y` in `_curve_tables[type][i]`.

This happens once at startup and uses `float`, but it’s out of the runtime hot path.

### 6.2. Time → table index

The ADSR runs as a small state machine with an explicit phase and phase‑start time:

- `ADSR_PHASE_ATTACK`
- `ADSR_PHASE_DECAY`
- `ADSR_PHASE_SUSTAIN`
- `ADSR_PHASE_RELEASE`
- `ADSR_PHASE_IDLE`

For each call to `getWave()`:

1. Read the current time in **ticks** (µs or ms) and compute `delta = now - t_phase_start` for the current phase.
2. **Realtime within phase, isolated across phases**:
   - Attack index uses the current `attack` time; changing `attack` while in ATTACK morphs the remaining attack, but does not affect DECAY/RELEASE.
   - Decay index uses the current `decay` time; changing `decay` while in DECAY morphs the remaining decay, but does not affect RELEASE.
   - Release index uses the current `release` time; changing `release` while in RELEASE morphs the tail, but earlier phases are unaffected.
3. Convert `delta` to a table index using the active backend:
   - **Fixed (`ADSR_BEZIER_USE_FLOAT=0`):** Q24 fast path or uint64 division fallback.
   - **Optimized (`ADSR_BEZIER_USE_FLOAT=1`):** mul+shift index reciprocal for all phase lengths (uint64 divide only if reciprocal search fails).
4. Clamp `idx` to `[0, ARRAY_SIZE-1]`.

### 6.3. Table → output level

Per stage (simplified):

- **Attack**:  
  `out = attack_start + curveVal * (vertical_resolution - attack_start) / vertical_resolution`

- **Decay**:  
  `out = sustain + curveVal * (vertical_resolution - sustain) / vertical_resolution`

- **Release**:  
  `out = curveVal * release_start / vertical_resolution`

These are implemented with precomputed Q16 scales: truncating (fixed) or round-nearest (`FLOAT=1`) at setter/noteOn/noteOff time. Attack uses pre-reversed curve tables.

---

## 7. Tips for using the library

- **For RP2040 / M0+:** keep `ADSR_BEZIER_USE_FLOAT` at `0` (default).
- **For RP2350 / Pico 2:** set `ADSR_BEZIER_USE_FLOAT` to `1` for mul+shift index + round-nearest Q16 output (±1 step vs golden).
- **For best quality:** use micros timebase (`ADSR_BEZIER_USE_MICROS=1`).
- Define **`ARRAY_SIZE`** in your project before `#include <ADSR_Bezier.h>` (512 in DCO).
- Adjust `ARRAY_SIZE` for resolution vs RAM trade-off.
- Use `setResetAttack(true)` for percussive lines; `false` for legato.

---

## 8. License / Credits

- Original ADSR concept and early implementation by **mo‑thunderz**.
- This Bezier + RP2040‑optimized variant and documentation adapted for the DCO4 project.

Enjoy shaping envelopes!

