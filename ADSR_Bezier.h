#ifndef ADSR_BEZIER_H
#define ADSR_BEZIER_H

//------------------------------------------------------------------//
// ADSR class for Arduino
// by mo-thunderz
// version 1.2
// last update: 14.08.2022
// modified to exploit hardware FPU + Fixed Point Hybrid
//------------------------------------------------------------------//

// Use Arduino timing functions internally
#include "Arduino.h"

// =================================================================
// COMPILE-TIME CONFIGURATION & MATH BACKENDS
// =================================================================

/**
 * @def ADSR_BEZIER_USE_MICROS
 * @brief Select internal timing resolution:
 *        1 -> use micros() internally (high resolution microsecond timebase)
 *        0 -> use millis() internally (backwards-compatible millisecond timebase)
 */
#ifndef ADSR_BEZIER_USE_MICROS
#define ADSR_BEZIER_USE_MICROS 1
#endif

/**
 * @def ADSR_BEZIER_RESET_TRANSITION_US
 * @brief Default anti-click slew transition duration in microseconds when restarting
 *        an envelope from a non-zero level with resetAttack enabled.
 *        - Set to 0 to disable (hard immediate jump to 0).
 *        - Default is 2000 us (2.0 ms) which eliminates DC clicks without perceptible latency.
 */
#ifndef ADSR_BEZIER_RESET_TRANSITION_US
#define ADSR_BEZIER_RESET_TRANSITION_US 3000UL
#endif

/**
 * @def ADSR_BEZIER_USE_FLOAT
 * @brief Math backend selector:
 *        0 -> Fixed-point phase index + Q16 amplitude scaling (fastest on integer MCUs / Cortex-M0+).
 *        1 -> Hardware FPU time rate / fixed-point amplitude hybrid (optimized for Cortex-M4F/M7/RP2350).
 */
#ifndef ADSR_BEZIER_USE_FLOAT
#define ADSR_BEZIER_USE_FLOAT 0
#endif

/**
 * @def ADSR_BEZIER_PHASE_SHIFT
 * @brief Fixed-point phase accumulator bitshift.
 *        - 22: uint32 multiplication (fast 32-bit arithmetic hot path).
 *        - 24: uint64 multiplication (ultra-smooth interpolation for long A/D/R times).
 */
#ifndef ADSR_BEZIER_PHASE_SHIFT
#define ADSR_BEZIER_PHASE_SHIFT 22
#endif

#if ADSR_BEZIER_PHASE_SHIFT > 22
#define ADSR_BEZIER_PHASE_SCALE_U64 1
#else
#define ADSR_BEZIER_PHASE_SCALE_U64 0
#endif

/**
 * @def ADSR_BEZIER_UPDATE_Q15_CACHE
 * @brief Controls whether the cached Q15 output is updated on every getWave() call.
 *        1 -> refresh Q15 in getWave (default).
 *        0 -> skip for ADSR_update A/B (u12 path only).
 *        Ignored when ADSR_BEZIER_NATIVE_Q15=1 (primary output is already native Q15).
 */
#ifndef ADSR_BEZIER_UPDATE_Q15_CACHE
#define ADSR_BEZIER_UPDATE_Q15_CACHE 1
#endif

/**
 * @def ADSR_BEZIER_NATIVE_Q15
 * @brief Amplitude domain selector:
 *        0 -> DAC-primary (constructor vertical_resolution) + optional Q15 cache (default / DCO shipping).
 *        1 -> Native Q15 amplitude (peak ADSR_Q15_PEAK); getWave returns Q15 tap 0..ADSR_Q15_ONE;
 *             setSustain units are in Q15 peak units.
 */
#ifndef ADSR_BEZIER_NATIVE_Q15
#define ADSR_BEZIER_NATIVE_Q15 0
#endif

/**
 * @def ADSR_BEZIER_Q15_DYADIC
 * @brief When NATIVE_Q15=1, sets peak amplitude to 32768 (1<<15) to enable dyadic >>15 bitshifts
 *        and eliminate runtime divisions. Output tap remains clamped to 32767.
 *        Set to 0 to force non-dyadic 32767 peak.
 */
#ifndef ADSR_BEZIER_Q15_DYADIC
#define ADSR_BEZIER_Q15_DYADIC 1
#endif

// =============================================================================
// SRAM HOT PATH PLACEMENT (RP2040 / RP2350 / ARM Cortex-M)
// =============================================================================
// 1 = RP2040 __not_in_flash_func on getWave / noteOn / noteOff (define before include).
// 0 = portable / flash (library default). No-op if the attribute is missing (AVR).
// Curve tables stay BSS RAM either way; adsrBezierInitTables is boot-only (not pinned).
#ifndef ADSR_BEZIER_SRAM_HOT
#define ADSR_BEZIER_SRAM_HOT 0
#endif

#if ADSR_BEZIER_SRAM_HOT

  /* ---------- Raspberry Pi Pico / RP2040 / RP2350 ---------- */
  #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
    #ifndef __not_in_flash_func
      #define __not_in_flash_func(fn) fn
    #endif
    #define ADSR_BEZIER_HOT(fn) __not_in_flash_func(fn)

  /* ---------- STM32H7 (ITCM – fastest instruction RAM) ---------- */
  #elif defined(STM32H7) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
    /* Function goes into .itcmram section (linker + startup copy required) */
    #define ADSR_BEZIER_HOT(fn) __attribute__((section(".itcmram"), noinline, used)) fn

  /* ---------- Fallback ---------- */
  #else
    #define ADSR_BEZIER_HOT(fn) fn
  #endif

#else
  /* Portable fallback (no special placement) */
  #define ADSR_BEZIER_HOT(fn) fn
#endif

/* Always-inline attribute to guarantee zero Flash branching in hot loops */
#ifndef ADSR_ALWAYS_INLINE
  #if defined(__GNUC__) || defined(__clang__)
    #define ADSR_ALWAYS_INLINE __attribute__((always_inline)) inline
  #else
    #define ADSR_ALWAYS_INLINE inline
  #endif
#endif


// Emit active config once per translation unit (visible in compile logs)
#ifndef ADSR_BEZIER_CONFIG_REPORTED
#define ADSR_BEZIER_CONFIG_REPORTED
#if ADSR_BEZIER_USE_FLOAT
#pragma message("ADSR_Bezier: math=native float time / Q16 amp hybrid (Optimized for FPU) (ADSR_BEZIER_USE_FLOAT=1)")
#else
#if ADSR_BEZIER_PHASE_SCALE_U64
#pragma message("ADSR_Bezier: math=fixed Q24/Q16 uint64 phase (ADSR_BEZIER_PHASE_SHIFT>22)")
#else
#pragma message("ADSR_Bezier: math=fixed Q22/Q16 uint32 phase (ADSR_BEZIER_USE_FLOAT=0)")
#endif
#endif
#if ADSR_BEZIER_USE_MICROS
#pragma message("ADSR_Bezier: timebase=micros (ADSR_BEZIER_USE_MICROS=1)")
#else
#pragma message("ADSR_Bezier: timebase=millis (ADSR_BEZIER_USE_MICROS=0)")
#endif
#if ADSR_BEZIER_NATIVE_Q15
#if ADSR_BEZIER_Q15_DYADIC
#pragma message("ADSR_Bezier: amp=native Q15 dyadic peak 32768 (ADSR_BEZIER_NATIVE_Q15=1, Q15_DYADIC=1)")
#else
#pragma message("ADSR_Bezier: amp=native Q15 peak 32767 (ADSR_BEZIER_NATIVE_Q15=1, Q15_DYADIC=0)")
#endif
#elif !ADSR_BEZIER_UPDATE_Q15_CACHE
#pragma message("ADSR_Bezier: Q15 cache OFF (ADSR_BEZIER_UPDATE_Q15_CACHE=0) — A/B only")
#else
#pragma message("ADSR_Bezier: amp=DAC primary + Q15 cache (ADSR_BEZIER_NATIVE_Q15=0)")
#endif
#if ADSR_BEZIER_RESET_TRANSITION_US > 0
#pragma message("ADSR_Bezier: anti-click reset transition ON")
#else
#pragma message("ADSR_Bezier: anti-click reset transition OFF (hard jump)")
#endif
#if ADSR_BEZIER_SRAM_HOT
#pragma message("ADSR_Bezier: SRAM hot path ON (ADSR_BEZIER_SRAM_HOT=1) — getWave/noteOn/noteOff .time_critical")
#else
#pragma message("ADSR_Bezier: SRAM hot path OFF (ADSR_BEZIER_SRAM_HOT=0) — library default")
#endif
#endif

// Lookup table point count per curve
#ifndef ARRAY_SIZE
#define ARRAY_SIZE 1024
#endif

// Unipolar Q15 bus full scale (0..ADSR_Q15_ONE ≈ 0..1). Matches mo-lfo MO_LFO_Q15_ONE magnitude.
static constexpr int16_t ADSR_Q15_ONE = 32767;
#if ADSR_BEZIER_NATIVE_Q15 && ADSR_BEZIER_Q15_DYADIC
// Internal amp / table peak (1<<15); getWave/levelQ15 still publish 0..ADSR_Q15_ONE.
static constexpr int ADSR_Q15_PEAK = 32768;
#else
static constexpr int ADSR_Q15_PEAK = (int)ADSR_Q15_ONE;
#endif

// =================================================================
// CURVE PRESET DEFINITIONS & STRING LOOKUPS
// =================================================================

/**
 * @brief Total number of pre-authored Bézier curve profiles.
 */
static constexpr uint8_t ADSR_NUM_CURVES = 8;

/**
 * @enum ADSRCurveType
 * @brief Named Bézier curve profiles (0 to 7).
 */
enum ADSRCurveType : uint8_t
{
    /** @brief 0: Classic natural exponential curve. Standard natural analog decay. */
    ADSR_CURVE_EXP_NATURAL    = 0,

    /** @brief 1: Smooth/medium exponential curve. Softer slope knee. */
    ADSR_CURVE_EXP_SMOOTH     = 1,

    /** @brief 2: Sharp/hyperbolic drop. Extremely punchy for percussive sounds. */
    ADSR_CURVE_PERCUSSIVE     = 2,

    /** @brief 3: Convex / Logarithmic curve. Holds high level before rapid end drop. */
    ADSR_CURVE_LOG_CONVEX     = 3,

    /** @brief 4: Soft Sigmoid S-curve. Gentle start and landing inflection. */
    ADSR_CURVE_S_CURVE_SOFT   = 4,

    /** @brief 5: Steep Sigmoid S-curve. Pronounced middle transition. */
    ADSR_CURVE_S_CURVE_STEEP  = 5,

    /** @brief 6: Rounded gentle curve. Smooth analog-like transition. */
    ADSR_CURVE_ROUNDED        = 6,

    /** @brief 7: Pure linear slope. Constant ramp rate. */
    ADSR_CURVE_LINEAR         = 7
};

/**
 * @brief Array of human-readable curve names (useful for OLED/LCD UI display).
 */
inline const char* const ADSR_CURVE_NAMES[ADSR_NUM_CURVES] = {
    "Natural Exp",
    "Smooth Exp",
    "Percussive",
    "Log / Convex",
    "Soft S-Curve",
    "Steep S-Curve",
    "Rounded",
    "Linear"
};

/**
 * @brief Get the string name of a curve preset.
 * @param curve Curve index or ADSRCurveType enum.
 * @return const char* String description (e.g. "Natural Exp", "Linear").
 */
inline const char* adsrGetCurveName(uint8_t curve)
{
    if (curve >= ADSR_NUM_CURVES)
        curve = ADSR_NUM_CURVES - 1;
    return ADSR_CURVE_NAMES[curve];
}

// Global Bézier lookup tables (inline storage allocated across translation units)
SRAM_DATA int _curve_tables[8][ARRAY_SIZE];

// =================================================================
// ADSR ENVELOPE CLASS DEFINITION
// =================================================================

/**
 * @class adsr
 * @brief High-performance ADSR envelope generator with Bézier curve profiling and anti-click slew.
 */
class adsr
{
public:
    /**
     * @brief Construct a new ADSR envelope instance.
     * 
     * ### Available Curve Presets:
     * - `0` / **`ADSR_CURVE_EXP_NATURAL`**   : "Natural Exp" (Classic analog decay)
     * - `1` / **`ADSR_CURVE_EXP_SMOOTH`**    : "Smooth Exp" (Soft exponential knee)
     * - `2` / **`ADSR_CURVE_PERCUSSIVE`**    : "Percussive" (Sharp punchy hyperbolic drop)
     * - `3` / **`ADSR_CURVE_LOG_CONVEX`**    : "Log / Convex" (Holds high before sudden drop)
     * - `4` / **`ADSR_CURVE_S_CURVE_SOFT`**  : "Soft S-Curve" (Smooth sigmoidal transition)
     * - `5` / **`ADSR_CURVE_S_CURVE_STEEP`** : "Steep S-Curve" (Aggressive inflection)
     * - `6` / **`ADSR_CURVE_ROUNDED`**       : "Rounded" (Gentle convex slope)
     * - `7` / **`ADSR_CURVE_LINEAR`**        : "Linear" (Constant straight ramp)
     * 
     * @param vertical_resolution Peak output resolution (e.g. 4095 for 12-bit DAC, 32767 for Q15).
     * @param attack_curve        Attack curve preset (0–7 or ADSRCurveType). Default is `ADSR_CURVE_EXP_NATURAL`.
     * @param decay_curve         Decay curve preset (0–7 or ADSRCurveType). Default is `ADSR_CURVE_EXP_NATURAL`.
     * @param release_curve       Release curve preset (0–7 or ADSRCurveType). Default is `ADSR_CURVE_EXP_NATURAL`.
     */
    adsr(int vertical_resolution, 
         uint8_t attack_curve = ADSR_CURVE_EXP_NATURAL, 
         uint8_t decay_curve = ADSR_CURVE_EXP_NATURAL, 
         uint8_t release_curve = ADSR_CURVE_EXP_NATURAL)
    {
        _dac_export_vr = (vertical_resolution > 0) ? vertical_resolution : (int)ADSR_Q15_ONE;
#if ADSR_BEZIER_NATIVE_Q15
        // Amp peak is Q15 domain (ADSR_Q15_PEAK); ctor arg kept only for levelDac() export scale.
        _vertical_resolution = (int)ADSR_Q15_PEAK;
        _sustain = (int)ADSR_Q15_PEAK / 2;
        _to_q15_mul = 0;
        if (_dac_export_vr > 0)
            _to_dac_mul =
              ((uint32_t)_dac_export_vr << 16) / (uint32_t)ADSR_Q15_PEAK;
        else
            _to_dac_mul = 0;
#else
        _vertical_resolution = vertical_resolution; // DAC_Size
        _sustain = vertical_resolution / 2;
        // Q15 cache: (level * mul) >> 16 ≈ level * ADSR_Q15_ONE / vertical_resolution
        if (_vertical_resolution > 0)
            _to_q15_mul =
              ((uint32_t)ADSR_Q15_ONE << 16) / (uint32_t)_vertical_resolution;
        else
            _to_q15_mul = 0;
        _to_dac_mul = 0;
#endif
        _attack = 100000;  // take 100ms as initial value for Attack
        _decay = 100000;   // take 100ms as initial value for Decay
        _release = 100000; // take 100ms as initial value for Release
        _configured_release = _release;

        // Initialize anti-click reset transition time from compile macro
#if ADSR_BEZIER_USE_MICROS
        setResetTransitionUs(ADSR_BEZIER_RESET_TRANSITION_US);
#else
        setResetTransitionMs((ADSR_BEZIER_RESET_TRANSITION_US + 999UL) / 1000UL);
#endif

        _bezier_attack_type = attack_curve;
        _bezier_decay_type = decay_curve;
        _bezier_release_type = release_curve;
        bindCurvePtrs();

        _adsr_output = 0;
        _adsr_output_q15 = 0;
        _adsr_output_q15_src = -1;
    }

#if ADSR_BEZIER_NATIVE_Q15
    /**
     * @brief Invalidate cached Q15 output (no-op in native Q15 mode).
     */
    void invalidateQ15Cache() {}
#else
    /**
     * @brief Invalidate cached Q15 output forcing recalculation on next read.
     */
    void invalidateQ15Cache() { _adsr_output_q15_src = -1; }
#endif

    /**
     * @brief Set the curve shape preset for the Attack phase.
     * @param curve_type Curve index (0 to 7) or ADSRCurveType.
     */
    void adsrCurveAttack(uint8_t curve_type)
    {
        _bezier_attack_type = curve_type;
        bindCurvePtrs();
    }

    /**
     * @brief Set the curve shape preset for the Decay phase.
     * @param curve_type Curve index (0 to 7) or ADSRCurveType.
     */
    void adsrCurveDecay(uint8_t curve_type)
    {
        _bezier_decay_type = curve_type;
        bindCurvePtrs();
    }

    /**
     * @brief Set the curve shape preset for the Release phase at runtime.
     *        If called mid-release, seamlessly re-anchors to avoid audio clicks/pops.
     * @param curve_type Curve index (0 to 7) or ADSRCurveType.
     */
    void ADSR_BEZIER_HOT(adsrCurveRelease)(uint8_t curve_type)
    {
        if (curve_type >= ADSR_NUM_CURVES) curve_type = ADSR_NUM_CURVES - 1;
        if (_bezier_release_type == curve_type) return;

        // If switched mid-release, re-anchor from current output over remaining time
        if (_phase == ADSR_PHASE_RELEASE)
        {
            unsigned long now;
            #if ADSR_BEZIER_USE_MICROS
                        now = micros();
            #else
                        now = millis();
            #endif
                        unsigned long elapsed = now - _t_phase_start;
                        
                        // Anti-skew clamp
                        if (elapsed > 0x7FFFFFFFUL) elapsed = 0;
                        
                        // Calculate remaining release time
                        if (elapsed < _release) {
                            _release -= elapsed;
                        } else {
                            _release = 0;
                        }

            _release_start = _adsr_output;
            _t_phase_start = now;

            // Recalculate phase advance rate for remaining time
#if ADSR_BEZIER_USE_FLOAT
            _release_rate_f = (_release > 0) ? ((float)(ARRAY_SIZE - 1) / (float)_release) : 0.0f;
#else
            _release_scale_phase = phaseScale(_release);
#endif
            int32_t rs = (int32_t)_release_start;
            if (rs < 0) rs = 0;
            if (rs > _vertical_resolution) rs = _vertical_resolution;

            _release_range_scale_q16 = rangeScaleQ16(rs, _vertical_resolution);
            invalidateQ15Cache();
        }

        _bezier_release_type = curve_type;
        bindCurvePtrs();
    }

    /**
     * @brief Sets the envelope output mode.
     * @param mode 0: Normal (0..32767)
     *             1: Centered (-32768..32767)
     *             2: Inverted (0..-32767)
     */
     void setMode(uint8_t mode)
     {
         if (_mode != mode) {
             _mode = mode;
             invalidateQ15Cache(); // Force recalculation on next fetch
         }
     }
 
     uint8_t getMode() const { return _mode; }


    /**
     * @brief Enable or disable attack phase restart behavior on Note On.
     * @param l_reset_attack true: reset envelope level toward 0 on retrigger.
     *                       false: analog legato mode (re-attack begins from current envelope level).
     */
    void setResetAttack(bool l_reset_attack)
    {
        _reset_attack = l_reset_attack;
    }

    /**
     * @brief Check whether resetAttack mode is currently enabled.
     * @return bool True if re-attack resets toward 0.
     */
    bool getResetAttack() const
    {
        return _reset_attack;
    }

    /**
     * @brief Set the anti-click slew transition duration in microseconds.
     *        When resetAttack is true and Note On arrives while output > 0, this duration
     *        is used to softly ramp down to 0 before launching the Attack phase.
     * @param l_reset_us Transition duration in microseconds (0 = hard instant jump).
     */
    void setResetTransitionUs(unsigned long l_reset_us)
    {
#if ADSR_BEZIER_USE_MICROS
        _reset_transition = l_reset_us;
#else
        _reset_transition = (l_reset_us + 999UL) / 1000UL;
#endif
#if ADSR_BEZIER_USE_FLOAT
        _reset_rate_f = (_reset_transition > 0) ? ((float)(ARRAY_SIZE - 1) / (float)_reset_transition) : 0.0f;
#else
        _reset_scale_phase = phaseScale(_reset_transition);
#endif
    }

    /**
     * @brief Set the anti-click slew transition duration in milliseconds.
     * @param l_reset_ms Transition duration in milliseconds (0 = hard instant jump).
     */
    void setResetTransitionMs(unsigned long l_reset_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        _reset_transition = l_reset_ms * 1000UL;
#else
        _reset_transition = l_reset_ms;
#endif
#if ADSR_BEZIER_USE_FLOAT
        _reset_rate_f = (_reset_transition > 0) ? ((float)(ARRAY_SIZE - 1) / (float)_reset_transition) : 0.0f;
#else
        _reset_scale_phase = phaseScale(_reset_transition);
#endif
    }

    /**
     * @brief Get the configured anti-click reset transition duration in microseconds.
     * @return unsigned long Transition duration in microseconds.
     */
    unsigned long getResetTransitionUs() const
    {
#if ADSR_BEZIER_USE_MICROS
        return _reset_transition;
#else
        return _reset_transition * 1000UL;
#endif
    }

    /**
     * @brief Get the configured anti-click reset transition duration in milliseconds.
     * @return unsigned long Transition duration in milliseconds.
     */
    unsigned long getResetTransitionMs() const
    {
#if ADSR_BEZIER_USE_MICROS
        return (_reset_transition + 999UL) / 1000UL;
#else
        return _reset_transition;
#endif
    }

    void setAttack(unsigned long l_attack_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        unsigned long attack_ticks = l_attack_ms * 1000UL;
#else
        unsigned long attack_ticks = l_attack_ms;
#endif
        if (_phase == ADSR_PHASE_ATTACK && attack_ticks != _attack)
        {
            _attack_start = _adsr_output;
#if ADSR_BEZIER_USE_MICROS
            _t_phase_start = micros();
#else
            _t_phase_start = millis();
#endif
            int32_t range = (int32_t)_vertical_resolution - (int32_t)_attack_start;
            if (range < 0) range = 0;
            _attack_range_scale_q16 = rangeScaleQ16(range, _vertical_resolution);
            invalidateQ15Cache();
        }
        _attack = attack_ticks;

#if ADSR_BEZIER_USE_FLOAT
        if (_attack > 0)
            _attack_rate_f = (float)(ARRAY_SIZE - 1) / (float)_attack;
        else
            _attack_rate_f = 0.0f;
#else
        _attack_scale_phase = phaseScale(_attack);
#endif
    }

    void setDecay(unsigned long l_decay_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        unsigned long decay_ticks = l_decay_ms * 1000UL; 
#else
        unsigned long decay_ticks = l_decay_ms;          
#endif
        if (_phase == ADSR_PHASE_DECAY && decay_ticks != _decay)
        {
            _decay_start = _adsr_output;
#if ADSR_BEZIER_USE_MICROS
            _t_phase_start = micros();
#else
            _t_phase_start = millis();
#endif
            int32_t dr = (int32_t)_decay_start - (int32_t)_sustain;
            if (dr < 0) dr = 0;
            _decay_range_scale_q16 = rangeScaleQ16(dr, _vertical_resolution);
            invalidateQ15Cache();
        }
        _decay = decay_ticks;

#if ADSR_BEZIER_USE_FLOAT
        if (_decay > 0)
            _decay_rate_f = (float)(ARRAY_SIZE - 1) / (float)_decay;
        else
            _decay_rate_f = 0.0f;
#else
        _decay_scale_phase = phaseScale(_decay);
#endif
    }

    void setSustain(int l_sustain)
    {
        if (l_sustain < 0) l_sustain = 0;
        if (l_sustain >= _vertical_resolution) l_sustain = _vertical_resolution;
        if (_sustain == l_sustain) return;
        _sustain = l_sustain;

        if (_phase == ADSR_PHASE_DECAY) {
            _decay_start = _adsr_output;
#if ADSR_BEZIER_USE_MICROS
            _t_phase_start = micros();
#else
            _t_phase_start = millis();
#endif
        } else if (_phase != ADSR_PHASE_SUSTAIN && _phase != ADSR_PHASE_RELEASE) {
            _decay_start = _vertical_resolution;
        }

        int32_t range = (int32_t)_decay_start - (int32_t)_sustain;
        if (range < 0) range = 0;

        _decay_range_scale_q16 = rangeScaleQ16(range, _vertical_resolution);
        invalidateQ15Cache();
    }

    void setRelease(unsigned long l_release_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        unsigned long release_ticks = l_release_ms * 1000UL; 
#else
        unsigned long release_ticks = l_release_ms;          
#endif
        if (_phase == ADSR_PHASE_RELEASE && release_ticks != _configured_release)
        {
            _release_start = _adsr_output;
#if ADSR_BEZIER_USE_MICROS
            _t_phase_start = micros();
#else
            _t_phase_start = millis();
#endif
            int32_t rs = (int32_t)_release_start;
            if (rs < 0) rs = 0;
            if (rs > _vertical_resolution) rs = _vertical_resolution;
            _release_range_scale_q16 = rangeScaleQ16(rs, _vertical_resolution);
            invalidateQ15Cache();
        }
        _release = release_ticks;
        _configured_release = _release;

#if ADSR_BEZIER_USE_FLOAT
        if (_release > 0)
            _release_rate_f = (float)(ARRAY_SIZE - 1) / (float)_release;
        else
            _release_rate_f = 0.0f;
#else
        _release_scale_phase = phaseScale(_release);
#endif
    }

    /**
     * @brief Trigger Note On event using current internal timestamp.
     *        If resetAttack is enabled and current level > 0, safely initiates
     *        a soft ramp-down transition before advancing to the Attack phase.
     */
     void ADSR_BEZIER_HOT(noteOn)()
     {
         unsigned long now;
 #if ADSR_BEZIER_USE_MICROS
         now = micros();
 #else
         now = millis();
 #endif
         _notes_pressed = 1;
 
         if (_reset_attack)
         {
             // Restart ON: fade down to 0 first
             if (_adsr_output <= 0 || _reset_transition == 0)
             {
                 _attack_start = 0;
                 _phase = ADSR_PHASE_ATTACK;
                 _t_phase_start = now;
 
                 _active_attack = _attack;
 #if ADSR_BEZIER_USE_FLOAT
                 _active_attack_rate_f = _attack_rate_f;
 #else
                 _active_attack_scale_phase = _attack_scale_phase;
 #endif
                 _attack_range_scale_q16 = rangeScaleQ16((int32_t)_vertical_resolution, _vertical_resolution);
                 invalidateQ15Cache();
             }
             else
             {
                 _reset_start_level = _adsr_output;
                 if (_reset_start_level > _vertical_resolution) _reset_start_level = _vertical_resolution;
 
                 _phase = ADSR_PHASE_RESET_TRANSITION;
                 _t_phase_start = now;
 
                 _reset_range_scale_q16 = rangeScaleQ16((int32_t)_reset_start_level, _vertical_resolution);
                 invalidateQ15Cache();
             }
         }
         else
         {
             // Restart OFF (Legato): Ramp from CURRENT level to Peak (100%)
             _attack_start = _adsr_output;
             if (_attack_start < 0) _attack_start = 0;
             if (_attack_start > _vertical_resolution) _attack_start = _vertical_resolution;
 
             _phase = ADSR_PHASE_ATTACK;
             _t_phase_start = now;
 
             // If attack is 0, use the anti-click transition time to ramp up safely
             if (_attack == 0 && _reset_transition > 0)
             {
                 _active_attack = _reset_transition;
 #if ADSR_BEZIER_USE_FLOAT
                 _active_attack_rate_f = _reset_rate_f;
 #else
                 _active_attack_scale_phase = _reset_scale_phase;
 #endif
             }
             else
             {
                 _active_attack = _attack;
 #if ADSR_BEZIER_USE_FLOAT
                 _active_attack_rate_f = _attack_rate_f;
 #else
                 _active_attack_scale_phase = _attack_scale_phase;
 #endif
             }
 
             int32_t range = (int32_t)_vertical_resolution - (int32_t)_attack_start;
             if (range < 0) range = 0;
 
             _attack_range_scale_q16 = rangeScaleQ16(range, _vertical_resolution);
             invalidateQ15Cache();
         }
     }

    /**
     * @brief Trigger Note Off event using current internal timestamp.
     *        Transitions envelope to the Release phase, capturing the current output anchor.
     */
    void ADSR_BEZIER_HOT(noteOff)()
    {
        _notes_pressed--;
        if (_notes_pressed <= 0)
        {
            unsigned long now;
#if ADSR_BEZIER_USE_MICROS
            now = micros();
#else
            now = millis();
#endif
            _release = _configured_release;
            _release_start = _adsr_output;
            _notes_pressed = 0;

            _phase = ADSR_PHASE_RELEASE;
            _t_phase_start = now;

            int32_t rs = (int32_t)_release_start;
            if (rs < 0) rs = 0;
            if (rs > _vertical_resolution) rs = _vertical_resolution;

            _release_range_scale_q16 = rangeScaleQ16(rs, _vertical_resolution);
            invalidateQ15Cache();
        }
    }

    /**
     * @brief Advance envelope and compute current sample using internal timebase.
     * @return int Output value in native domain (DAC counts or native Q15).
     */
    int ADSR_BEZIER_HOT(getWave)()
    {
#if ADSR_BEZIER_USE_MICROS
        return getWave(micros());
#else
        return getWave(millis());
#endif
    }


    /**
     * @brief Advance envelope and compute current sample using a caller-supplied timestamp.
     * @param l_ticks Current timestamp in ticks (micros or millis matching ADSR_BEZIER_USE_MICROS).
     * @return int Output value in native domain.
     */
     int ADSR_BEZIER_HOT(getWave)(unsigned long l_ticks)
     {
 #if ADSR_BEZIER_NATIVE_Q15
         // Sustain/idle: no index/table/mul; publish Q15 tap (0..ADSR_Q15_ONE).
         if (_phase == ADSR_PHASE_SUSTAIN) {
             _adsr_output = _sustain;
             int32_t q = nativeTapQ15(_sustain);
             if (_mode == 1) q = (q - 16384) << 1;
             else if (_mode == 2) q = -q;
             _adsr_output_q15 = (int16_t)q;
             return (int)_adsr_output_q15;
         }
         if (_phase == ADSR_PHASE_IDLE) {
             _adsr_output = 0;
             int32_t q = 0;
             if (_mode == 1) q = -32768;
             else if (_mode == 2) q = 0;
             _adsr_output_q15 = (int16_t)q;
             return (int)_adsr_output_q15;
         }
 #endif
 
         // --- TIMING SKEW PROTECTION ---
         // If caller uses a cached timestamp slightly older than the internal 
         // micros() fetched during noteOn(), delta will underflow to a huge value.
         // We clamp these negative skews to 0 to prevent instantly skipping phases.
         unsigned long delta = l_ticks - _t_phase_start;
         if (delta > 0x7FFFFFFFUL) {
             delta = 0; 
         }
 
         switch (_phase)
         {
            case ADSR_PHASE_RESET_TRANSITION:
            {
                if (delta >= _reset_transition)
                {
                    // 1. Reset floor to 0
                    _adsr_output = 0;
                    _attack_start = 0;
                    _t_phase_start = l_ticks;
                    _attack_range_scale_q16 = rangeScaleQ16((int32_t)_vertical_resolution, _vertical_resolution);
    
                    // 2. Properly initialize active attack parameters for the Attack phase
                    if (_attack == 0 && _reset_transition > 0)
                    {
                        _active_attack = _reset_transition;
    #if ADSR_BEZIER_USE_FLOAT
                        _active_attack_rate_f = _reset_rate_f;
    #else
                        _active_attack_scale_phase = _reset_scale_phase;
    #endif
                    }
                    else
                    {
                        _active_attack = _attack;
    #if ADSR_BEZIER_USE_FLOAT
                        _active_attack_rate_f = _attack_rate_f;
    #else
                        _active_attack_scale_phase = _attack_scale_phase;
    #endif
                    }
    
                    // 3. Launch Attack phase from 0
                    _phase = ADSR_PHASE_ATTACK;
                    break;
                }
    
    #if ADSR_BEZIER_USE_FLOAT
                uint32_t idx = (uint32_t)((float)delta * _reset_rate_f);
                if (idx >= ARRAY_SIZE) idx = ARRAY_SIZE - 1;
    #else
                uint32_t idx = phaseIndexFixed(delta, _reset_transition, _reset_scale_phase);
    #endif
                // Linear fade down to zero
                int curveVal = _curve_tables[ADSR_CURVE_LINEAR][(int)idx];
                int32_t out = (int32_t)(((uint32_t)curveVal * _reset_range_scale_q16) >> 16);
                _adsr_output = (int)out;
                break;
            }
         
         case ADSR_PHASE_ATTACK:
         {
             if (_active_attack == 0 || delta >= _active_attack)
             {
                 _adsr_output = _vertical_resolution;
                 if (_decay > 0) {
                     _phase = ADSR_PHASE_DECAY;
                     _t_phase_start = l_ticks;
                     _decay_start = _vertical_resolution;
                     int32_t dr = (int32_t)_decay_start - (int32_t)_sustain;
                     if (dr < 0) dr = 0;
                     _decay_range_scale_q16 = rangeScaleQ16(dr, _vertical_resolution);
                 } else {
                     _phase = ADSR_PHASE_SUSTAIN;
                 }
                 break;
             }
 
 #if ADSR_BEZIER_USE_FLOAT
             uint32_t idx = (uint32_t)((float)delta * _active_attack_rate_f);
             if (idx >= ARRAY_SIZE) idx = ARRAY_SIZE - 1;
 #else
             uint32_t idx = phaseIndexFixed(delta, _active_attack, _active_attack_scale_phase);
 #endif
             // Read curve table backwards to ramp from 0 to max
             int curveVal = _curve_attack_ptr[(ARRAY_SIZE - 1) - (int)idx];
 
             int32_t out = (int32_t)_attack_start +
               (int32_t)(((uint32_t)curveVal * _attack_range_scale_q16) >> 16);
 
             _adsr_output = (int)out;
             break;
         }
 
         case ADSR_PHASE_DECAY:
         {
             if (_decay == 0)
             {
                 _adsr_output = _sustain;
                 _phase = ADSR_PHASE_SUSTAIN;
                 break;
             }
 
             if (delta >= _decay)
             {
                 _adsr_output = _sustain;
                 _phase = ADSR_PHASE_SUSTAIN;
                 break;
             }
 
 #if ADSR_BEZIER_USE_FLOAT
             uint32_t idx = (uint32_t)((float)delta * _decay_rate_f);
             if (idx >= ARRAY_SIZE) idx = ARRAY_SIZE - 1;
 #else
             uint32_t idx = phaseIndexFixed(delta, _decay, _decay_scale_phase);
 #endif
 
             int curveVal = _curve_decay_ptr[(int)idx];
 
             int32_t out = (int32_t)_sustain +
               (int32_t)(((uint32_t)curveVal * _decay_range_scale_q16) >> 16);
 
             _adsr_output = (int)out;
             break;
         }
 
         case ADSR_PHASE_SUSTAIN:
         {
             _adsr_output = _sustain;
             break;
         }
 
         case ADSR_PHASE_RELEASE:
         {
             if (_release == 0)
             {
                 _adsr_output = 0;
                 _phase = ADSR_PHASE_IDLE;
                 break;
             }
 
             if (delta >= _release)
             {
                 _adsr_output = 0;
                 _phase = ADSR_PHASE_IDLE;
                 break;
             }
 
 #if ADSR_BEZIER_USE_FLOAT
             uint32_t idx = (uint32_t)((float)delta * _release_rate_f);
             if (idx >= ARRAY_SIZE) idx = ARRAY_SIZE - 1;
 #else
             uint32_t idx = phaseIndexFixed(delta, _release, _release_scale_phase);
 #endif
 
             int curveVal = _curve_release_ptr[(int)idx];
 
             int32_t out =
               (int32_t)(((uint32_t)curveVal * _release_range_scale_q16) >> 16);
 
             _adsr_output = (int)out;
             break;
         }
 
 #if !ADSR_BEZIER_NATIVE_Q15
         case ADSR_PHASE_IDLE:
         default:
         {
             _adsr_output = 0;
             break;
         }
 #else
         default:
             _adsr_output = 0;
             break;
 #endif
         }
 
         #if ADSR_BEZIER_NATIVE_Q15
         // A/D/R: publish bus tap (curve×scale stays in [0, vr] when tables/scales valid).
         int32_t q = nativeTapQ15(_adsr_output);
         if (_mode == 1) q = (q - 16384) << 1;
         else if (_mode == 2) q = -q;
         _adsr_output_q15 = (int16_t)q;
         return (int)_adsr_output_q15;
 #elif ADSR_BEZIER_UPDATE_Q15_CACHE
         // Q15 cache: skip mul when DAC level unchanged (sustain/idle).
         if (_adsr_output != _adsr_output_q15_src) {
             _adsr_output_q15_src = _adsr_output;
             uint32_t q_raw = ((uint32_t)_adsr_output * _to_q15_mul) >> 16;
             if (q_raw > (uint32_t)ADSR_Q15_ONE) q_raw = (uint32_t)ADSR_Q15_ONE;
             
             int32_t q = (int32_t)q_raw;
             if (_mode == 1) q = (q - 16384) << 1;
             else if (_mode == 2) q = -q;
             
             _adsr_output_q15 = (int16_t)q;
         }
 #endif
         return _adsr_output;
     }

    /**
     * @brief Get the Q15 amplitude representation from the last getWave() call.
     * @return int16_t Unipolar Q15 output (0..32767).
     */
    int16_t ADSR_BEZIER_HOT(levelQ15)() const
    {
        return _adsr_output_q15;
    }

    /**
     * @brief Get the DAC-domain amplitude level.
     * @return int Scaled integer value matching constructor vertical_resolution.
     */
    int ADSR_BEZIER_HOT(levelDac)() const
    {
#if ADSR_BEZIER_NATIVE_Q15
        return (int)(((uint32_t)_adsr_output * _to_dac_mul) >> 16);
#else
        return _adsr_output;
#endif
    }

    /**
     * @brief Advance envelope and return unipolar Q15 level using internal timestamp.
     * @note Do not call both getWave() and getWaveQ15() in the same tick.
     * @return int16_t Output in Q15 format (0..32767).
     */
    int16_t ADSR_BEZIER_HOT(getWaveQ15)()
    {
        getWave();
        return _adsr_output_q15;
    }

    /**
     * @brief Advance envelope and return unipolar Q15 level using a caller-supplied timestamp.
     * @param t Current timestamp in ticks.
     * @return int16_t Output in Q15 format (0..32767).
     */
    int16_t ADSR_BEZIER_HOT(getWaveQ15)(unsigned long t)
    {
        getWave(t);
        return _adsr_output_q15;
    }

private:

/** @brief Envelope output mode */
    uint8_t _mode = 0;

#if ADSR_BEZIER_NATIVE_Q15
    /**
     * @brief Clamps native integer level to unipolar Q15 bus limits (0..ADSR_Q15_ONE).
     */
    static ADSR_ALWAYS_INLINE int16_t nativeTapQ15(int level)
    {
        if (level <= 0)
            return 0;
        if (level >= (int)ADSR_Q15_ONE)
            return ADSR_Q15_ONE;
        return (int16_t)level;
    }
#endif

    /**
     * @brief Binds internal table pointers directly to the active curve LUT arrays.
     */
    void bindCurvePtrs()
    {
        uint8_t a = (uint8_t)_bezier_attack_type;
        uint8_t d = (uint8_t)_bezier_decay_type;
        uint8_t r = (uint8_t)_bezier_release_type;
        if (a >= ADSR_NUM_CURVES) a = ADSR_NUM_CURVES - 1;
        if (d >= ADSR_NUM_CURVES) d = ADSR_NUM_CURVES - 1;
        if (r >= ADSR_NUM_CURVES) r = ADSR_NUM_CURVES - 1;
        _curve_attack_ptr = _curve_tables[a];
        _curve_decay_ptr = _curve_tables[d];
        _curve_release_ptr = _curve_tables[r];
    }

    /**
     * @brief Calculates Q16 amplitude scale factor: (range << 16) / vr.
     *        Dyadic peak 32768 optimizes down to `range << 1` with zero divide.
     */
    static ADSR_ALWAYS_INLINE uint32_t rangeScaleQ16(int32_t range, int vr)
    {
        if (range <= 0 || vr <= 0)
            return 0;
#if ADSR_BEZIER_NATIVE_Q15 && ADSR_BEZIER_Q15_DYADIC
        if (vr == ADSR_Q15_PEAK)
            return (uint32_t)range << 1;
#endif
        return (uint32_t)(((uint64_t)range << 16) / (uint32_t)vr);
    }

#if !ADSR_BEZIER_USE_FLOAT
#if ADSR_BEZIER_PHASE_SCALE_U64
    using phase_scale_t = uint64_t;
#else
    using phase_scale_t = uint32_t;
#endif

    /**
     * @brief Calculates rounded phase scale factor: ((ARRAY_SIZE - 1) << SHIFT) / ticks.
     */
    static phase_scale_t phaseScale(unsigned long phase_ticks)
    {
        if (phase_ticks == 0)
            return 0;
        return (phase_scale_t)((((uint64_t)(ARRAY_SIZE - 1) << ADSR_BEZIER_PHASE_SHIFT) +
                                (phase_ticks >> 1)) /
                               (uint64_t)phase_ticks);
    }

    /**
     * @brief Converts elapsed delta ticks to a LUT table index via fixed-point multiply-shift.
     */
    static ADSR_ALWAYS_INLINE uint32_t phaseIndexFixed(unsigned long delta, unsigned long phase_ticks, phase_scale_t scale)
    {
        uint32_t idx;
        if (scale != 0)
        {
#if ADSR_BEZIER_PHASE_SCALE_U64
            idx = (uint32_t)(((uint64_t)delta * scale) >> ADSR_BEZIER_PHASE_SHIFT);
#else
            idx = ((uint32_t)delta * (uint32_t)scale) >> ADSR_BEZIER_PHASE_SHIFT;
#endif
        }
        else if (phase_ticks > 0)
        {
            idx = (uint32_t)(((uint64_t)(ARRAY_SIZE - 1) * (uint64_t)delta) / (uint64_t)phase_ticks);
        }
        else
        {
            idx = 0;
        }
        if (idx >= ARRAY_SIZE)
            idx = ARRAY_SIZE - 1;
        return idx;
    }
#endif

    uint8_t _bezier_attack_type;
    uint8_t _bezier_decay_type;
    uint8_t _bezier_release_type;
    int *_curve_attack_ptr = nullptr;
    int *_curve_decay_ptr = nullptr;
    int *_curve_release_ptr = nullptr;

    int _vertical_resolution; // amp peak: DAC vr (NATIVE=0) or ADSR_Q15_PEAK (NATIVE=1)
    int _dac_export_vr;       // ctor DAC size; used by levelDac() when NATIVE=1
    uint32_t _to_dac_mul = 0; // (dac_export_vr << 16) / ADSR_Q15_PEAK when NATIVE=1
    unsigned long _attack = 0;
    unsigned long _active_attack = 0; // Active attack phase, to prevent clicks
    unsigned long _decay = 0; 
    int _sustain = 0;         // DAC counts (NATIVE=0) or Q15 (NATIVE=1)
    unsigned long _release = 0;
    unsigned long _configured_release = 0;
    bool _reset_attack = false;
    unsigned long _reset_transition = 0; // Anti-click slew duration (ticks)
    int _reset_start_level = 0;          // Output anchor level at start of reset transition

#if !ADSR_BEZIER_USE_FLOAT
    phase_scale_t _attack_scale_phase = 0;
    phase_scale_t _active_attack_scale_phase = 0;
    phase_scale_t _decay_scale_phase = 0;
    phase_scale_t _release_scale_phase = 0;
    phase_scale_t _reset_scale_phase = 0;
#endif

    /**
     * @enum ADSRPhase
     * @brief Internal envelope generator lifecycle states.
     */
    enum ADSRPhase
    {
        ADSR_PHASE_IDLE = 0,
        ADSR_PHASE_RESET_TRANSITION,
        ADSR_PHASE_ATTACK,
        ADSR_PHASE_DECAY,
        ADSR_PHASE_SUSTAIN,
        ADSR_PHASE_RELEASE
    };

    ADSRPhase _phase = ADSR_PHASE_IDLE;
    unsigned long _t_phase_start = 0;

#if ADSR_BEZIER_USE_FLOAT
    float _attack_rate_f = 0.0f;
    float _active_attack_rate_f = 0.0f;
    float _decay_rate_f = 0.0f;
    float _release_rate_f = 0.0f;
    float _reset_rate_f = 0.0f;
#endif

    // Unified Amplitude scaling - universally fast on all platforms
    uint32_t _attack_range_scale_q16 = 0;
    uint32_t _decay_range_scale_q16 = 0;
    uint32_t _release_range_scale_q16 = 0;
    uint32_t _reset_range_scale_q16 = 0;

    int _adsr_output;
    int16_t _adsr_output_q15 = 0;
    int _adsr_output_q15_src = -1;  // last DAC level converted to Q15
    // (ADSR_Q15_ONE << 16) / vertical_resolution — set in ctor
    uint32_t _to_q15_mul = 0;
    int _release_start;
    int _attack_start;
    int _decay_start;
    int _notes_pressed = 0;
};

// =================================================================
// BÉZIER TABLE GENERATION HELPERS
// =================================================================

/**
 * @struct ADSRBezierPoint
 * @brief Lightweight 2D floating-point coordinate for Bézier solver.
 */
struct ADSRBezierPoint
{
    float x, y;
};

/**
 * @brief Evaluates cubic Bézier coordinates at normalized progress parameter t.
 * @param A  Start point coordinate (0, maxVal).
 * @param P1 First control handle coordinate.
 * @param P2 Second control handle coordinate.
 * @param B  End point coordinate (maxVal, 0).
 * @param t  Normalized parametric progress in [0.0, 1.0].
 * @return ADSRBezierPoint Evaluated 2D position.
 */
inline ADSRBezierPoint adsrBezierCubic(const ADSRBezierPoint &A,
                                       const ADSRBezierPoint &P1,
                                       const ADSRBezierPoint &P2,
                                       const ADSRBezierPoint &B,
                                       float t)
{
    float one_minus_t = 1.0f - t;
    float one_minus_t_squared = one_minus_t * one_minus_t;
    float t_squared = t * t;

    float x = one_minus_t_squared * one_minus_t * A.x +
              3.0f * one_minus_t_squared * t * P1.x +
              3.0f * one_minus_t * t_squared * P2.x +
              t_squared * t * B.x;

    float y = one_minus_t_squared * one_minus_t * A.y +
              3.0f * one_minus_t_squared * t * P1.y +
              3.0f * one_minus_t * t_squared * P2.y +
              t_squared * t * B.y;

    return {x, y};
}

/**
 * @brief Binary-search inversion to solve Y given target X along a Bézier curve.
 * @param A       Start point coordinate.
 * @param P1      First control handle coordinate.
 * @param P2      Second control handle coordinate.
 * @param B       End point coordinate.
 * @param xTarget Target X progress coordinate.
 * @param tol     Convergence tolerance.
 * @return float  Solved Y coordinate corresponding to xTarget.
 */
 inline float adsrBezierFindYForX(const ADSRBezierPoint &A,
    const ADSRBezierPoint &P1,
    const ADSRBezierPoint &P2,
    const ADSRBezierPoint &B,
    float xTarget,
    float tol = 1e-5f)
{
float tLow = 0.0f;
float tHigh = 1.0f;
float yResult = A.y;

// 16 iterations gives 1/65536 precision (plenty for 12-bit DACs)
for (int iter = 0; iter < 16 && (tHigh - tLow) > tol; ++iter)
{
float tMid = (tLow + tHigh) * 0.5f;
ADSRBezierPoint midPoint = adsrBezierCubic(A, P1, P2, B, tMid);
yResult = midPoint.y;

if (midPoint.x < xTarget)
{
tLow = tMid;
}
else
{
tHigh = tMid;
}
}

return yResult;
}

/**
 * @brief Generates 8 pre-authored Bézier curve profiles into the global lookup tables.
 *        P1/P2 literals are authored near 12-bit CV (~4095) and scaled by maxVal/4096 (2^12)
 *        so shapes remain consistent at Q15 peak with an exact dyadic factor.
 * @param maxVal    Maximum amplitude value (e.g. vertical_resolution or ADSR_Q15_PEAK).
 * @param numPoints Number of points per curve array (default: ARRAY_SIZE = 1024).
 */
inline void adsrBezierInitTables(float maxVal, int numPoints = ARRAY_SIZE)
{
    ADSRBezierPoint A = {0.0f, maxVal};
    ADSRBezierPoint B = {maxVal, 0.0f};

    // Scale reference = 2^12 (not 4095): exact float reciprocal; ≈ authored frame.
    static constexpr float kAuthPeak = 4096.0f;
    const float s = maxVal / kAuthPeak;

    const ADSRBezierPoint P1_auth[8] = {
        {250.0f, 1500.0f}, {840.0f, 1780.0f}, {400.0f, 430.0f},  {2170.0f, 3610.0f},
        {400.0f, 1380.0f}, {1140.0f, 3750.0f}, {200.0f, 2700.0f}, {0.0f, 4095.0f}};

    const ADSRBezierPoint P2_auth[8] = {
        {1500.0f, 250.0f}, {1160.0f, 210.0f}, {920.0f, 420.0f},  {3730.0f, 2610.0f},
        {3830.0f, 2890.0f}, {1850.0f, 1080.0f}, {720.0f, 3050.0f}, {4095.0f, 0.0f}};

    for (int j = 0; j < 8; ++j)
    {
        ADSRBezierPoint P1 = {P1_auth[j].x * s, P1_auth[j].y * s};
        ADSRBezierPoint P2 = {P2_auth[j].x * s, P2_auth[j].y * s};

        float multiplier = (float)(maxVal + 1.0f) / (float)(numPoints - 1);

        for (int i = 0; i < numPoints; ++i)
        {
            float xTarget = multiplier * (float)i;
            float yResult = adsrBezierFindYForX(A, P1, P2, B, xTarget);

            _curve_tables[j][i] = (int)roundf(yResult);
        }
    }
}

#endif // ADSR_BEZIER_H