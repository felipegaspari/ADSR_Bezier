## ADSR Bezier (millis/micros, RP2040‑friendly)

ADSR Bezier is a lightweight, digitally‑controlled envelope generator based on precomputed Bézier lookup tables.  
It is designed to be:

- **Fast at runtime** (integer math only in the hot path, RP2040‑friendly).
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
- **Timebase**:
  - Internal timing can be **microseconds** (`micros()`) or **milliseconds** (`millis()`).
  - Selected at compile time via `ADSR_BEZIER_USE_MICROS`.
- **Curves**:
  - Attack, decay and release each read from a Bézier‑generated lookup table.
  - 8 different curve types are supported (`0…7`), selected separately for A/D/R.

Internally, each call to `getWave()`:

1. Computes the elapsed time since `noteOn()` / `noteOff()` using the chosen timebase.
2. Converts elapsed time to a **table index** using either:
   - A precomputed Q24 fixed‑point scale (fast path, for shorter times), or
   - Exact 64‑bit integer division (accurate path, for long times).
3. Reads the appropriate table value for the current stage (attack/decay/release).
4. Linearly maps that curve value to the requested output range using Q16 fixed‑point math.

No floating‑point math or divisions are performed in the audio/control‑rate hot path.

---

## 2. Installation

### 2.1. As a generic Arduino library

1. Create a folder in your Arduino libraries directory, e.g. `ADSR_Bezier`.
2. Copy `ADSR_Bezier.h`, `ADSR_Bezier.cpp` (if present), and this `README.md` into that folder.
3. In your sketch:

```cpp
#include "ADSR_Bezier.h"
```

### 2.2. Inside this DCO project

In this project the Bezier ADSR is integrated via `adsr.h`:

- `adsr.h` includes `src/ADSR_Bezier/ADSR_Bezier.h`.
- The DCO project uses a global table generator (`adsrCreateTables`) to fill the Bézier lookup tables at startup.

You do not need to do anything extra here – just call `init_ADSR()` from your main sketch as shown below.

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

- **`vertical_resolution`**: maximum envelope value (e.g. `4000` for a DAC or fixed‑point control range).
- **`attack_alpha`, `attack_decay_release`**: legacy parameters for the original exponential curve tables (not used when you provide your own Bézier tables; can be left as defaults).
- **`bezier`**:
  - `true`: use Bézier tables.
  - `false`: use the original exponential tables (if compiled in).
- **Curve types** (`bezier_attack_type`, `bezier_decay_type`, `bezier_release_type`):  
  Index into `_curve_tables[8]` (0–7), letting you choose separate curves for A, D, and R.

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
#define ADSR_BEZIER_USE_MICROS 1
#include "src/ADSR_Bezier/ADSR_Bezier.h"
```

  - Internally uses `micros()` for timing.
  - Time resolution: 1 µs.
  - Parameter units remain **milliseconds**; conversion is done behind the scenes.

- **Millis mode (backwards‑compatible)**:

```cpp
#define ADSR_BEZIER_USE_MICROS 0
#include "src/ADSR_Bezier/ADSR_Bezier.h"
```

  - Internally uses `millis()` for timing.
  - Coarser resolution, but closer to the original Arduino ADSR examples.

The rest of your code (parameter units, `noteOn()`, `getWave()`) does not change between modes.

---

## 5. Example: DCO synth ADSR integration (RP2040)

This section shows how the library is used in the DCO synth project in this repo.

### 5.1. Global ADSR configuration (`adsr.h`)

Key parts of `adsr.h` (simplified):

```cpp
#define ADSR_1_DACSIZE 4000
#define ARRAY_SIZE 512

// ADSR Bezier library (provides global curve tables and ADSR class)
#include "src/ADSR_Bezier/ADSR_Bezier.h"

// Per-voice ADSR instances
adsr adsr1_voice_0(ADSR_1_DACSIZE, ADSR1_curve1, ADSR1_curve2, false, 7, 7, 7);
adsr adsr1_voice_1(ADSR_1_DACSIZE, ADSR1_curve1, ADSR1_curve2, false, 7, 7, 7);
adsr adsr1_voice_2(ADSR_1_DACSIZE, ADSR1_curve1, ADSR1_curve2, false, 7, 7, 7);
adsr adsr1_voice_3(ADSR_1_DACSIZE, ADSR1_curve1, ADSR1_curve2, false, 7, 7, 7);
```

Notes:

- The ADSR Bezier library now owns the global Bézier tables and their generation.
- Each voice has its own `adsr` instance with `vertical_resolution = 4000`.

### 5.2. Initialization (`adsr.ino`)

```cpp
void init_ADSR() {
  // Initialize ADSR Bézier lookup tables in the library
  adsrBezierInitTables(ADSR_1_CC, ARRAY_SIZE, _curve_tables);

  for (int i = 0; i < LIN_TO_EXP_TABLE_SIZE; i++) {
    linToLogLookup[i] = linearToLogarithmic(i, 10, maxADSRControlValue);
  }

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);    // ms
    ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);      // ms
    ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);  // 0..4000
    ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);  // ms
    ADSRVoices[i].adsr1_voice.setResetAttack(ADSRRestart);
  }
}
```

### 5.3. Per‑voice update loop

```cpp
inline void ADSR_update() {
  tADSR = millis();
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    if (noteEnd[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff();
      noteEnd[i] = 0;
    } else if (noteStart[i] == 1) {
      ADSRVoices[i].adsr1_voice.noteOff();
      ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);
      ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);
      ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
      ADSRVoices[i].adsr1_voice.noteOn();
      noteStart[i] = 0;
    }
    ADSR1Level[i] = ADSRVoices[i].adsr1_voice.getWave();
  }
  ADSR_set_parameters();
}
```

Here:

- `noteStart[i]` / `noteEnd[i]` come from the MIDI/voice allocator.
- Each voice gets its ADSR envelope updated and stored in `ADSR1Level[i]`.
- `ADSR1Level[i]` is then used inside the DCO voice engine to modulate amplitude and other parameters.

### 5.4. Parameter updates at low rate

```cpp
inline void ADSR_set_parameters() {
  if ((tADSR - tADSR_params) > 5) {
    static uint16_t last_attack  = 0xFFFF;
    static uint16_t last_decay   = 0xFFFF;
    static uint16_t last_sustain = 0xFFFF;
    static uint16_t last_release = 0xFFFF;

    bool attack_changed  = (ADSR1_attack  != last_attack);
    bool decay_changed   = (ADSR1_decay   != last_decay);
    bool sustain_changed = (ADSR1_sustain != last_sustain);
    bool release_changed = (ADSR1_release != last_release);

    if (attack_changed || decay_changed || sustain_changed || release_changed) {
      for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
        if (attack_changed)  ADSRVoices[i].adsr1_voice.setAttack(ADSR1_attack);
        if (decay_changed)   ADSRVoices[i].adsr1_voice.setDecay(ADSR1_decay);
        if (sustain_changed) ADSRVoices[i].adsr1_voice.setSustain(ADSR1_sustain);
        if (release_changed) ADSRVoices[i].adsr1_voice.setRelease(ADSR1_release);
      }
      last_attack  = ADSR1_attack;
      last_decay   = ADSR1_decay;
      last_sustain = ADSR1_sustain;
      last_release = ADSR1_release;
    }
    tADSR_params = tADSR;
  }
}
```

This ensures parameter changes (e.g. coming over Serial) are:

- Applied to all voices.
- Debounced to run only when values actually change.
- Limited to ~200 Hz (every 5 ms) so they don’t add overhead to the main control loop.

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
3. Depending on the phase’s (current) time and Q24 threshold:
   - **Q24 path (fast)** for shorter times:  
     `idx ≈ (delta * scale_q24) >> 24`  
     where `scale_q24` was precomputed from the active A/D/R time in the corresponding setter.
   - **Exact path (accurate)** for long times:  
     `idx = ((ARRAY_SIZE - 1) * delta) / time_ticks`.
4. Clamp `idx` to `[0, ARRAY_SIZE-1]`.

### 6.3. Table → output level

Per stage (simplified):

- **Attack**:  
  `out = attack_start + curveVal * (vertical_resolution - attack_start) / vertical_resolution`

- **Decay**:  
  `out = sustain + curveVal * (vertical_resolution - sustain) / vertical_resolution`

- **Release**:  
  `out = curveVal * release_start / vertical_resolution`

These are implemented as Q16 fixed‑point multiplies and shifts with precomputed scales, so only integer math is used at runtime.

---

## 7. Tips for using the library

- **For best quality**: use micros timebase and keep Q24 thresholds conservative (or disabled) if you use very long envelopes.
- **For benchmarking**: set the Q24 threshold high to force Q24 over most of the range, and compare against a threshold of `0`.
- **For other projects**:
  - Reuse the `adsrCreateTables()` pattern to generate your own `_curve_tables`.
  - Adjust `ARRAY_SIZE` for a resolution vs RAM trade‑off.
  - Use `setResetAttack(true)` for percussive mono lines; `false` for legato behavior.

---

## 8. License / Credits

- Original ADSR concept and early implementation by **mo‑thunderz**.
- This Bezier + RP2040‑optimized variant and documentation adapted for the DCO4 project.

Enjoy shaping envelopes!

