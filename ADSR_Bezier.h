#ifndef ADSR_BEZIER_H
#define ADSR_BEZIER_H

//------------------------------------------------------------------//
// ADSR Envelope Generator for Arduino (Bézier Curve Engine)
// Hybrid Fixed-Point / Hardware FPU Math Architecture
//------------------------------------------------------------------//

#include "Arduino.h"

// =================================================================
// CONFIGURATION MACROS
// =================================================================

/**
 * @def ADSR_BEZIER_USE_MICROS
 * @brief Select timing resolution backend.
 *        1 = Use micros() internally (high resolution, recommended).
 *        0 = Use millis() internally (legacy compatibility).
 */
#ifndef ADSR_BEZIER_USE_MICROS
#define ADSR_BEZIER_USE_MICROS 1
#endif

/**
 * @def ADSR_BEZIER_USE_FLOAT
 * @brief Math backend selector.
 *        0 = Fixed-point phase index + Q16 amplitude (ultra-fast integer math).
 *        1 = Floating-point time accumulator / fixed amp hybrid (optimized for MCUs with FPU).
 */
#ifndef ADSR_BEZIER_USE_FLOAT
#define ADSR_BEZIER_USE_FLOAT 0
#endif

/**
 * @def ADSR_BEZIER_PHASE_SHIFT
 * @brief Fixed-point phase precision shift (used when ADSR_BEZIER_USE_FLOAT=0).
 *        22 = uint32 multiplication (fast 32-bit integer math).
 *        24 = uint64 multiplication (smoother long envelope ramps).
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
 * @brief Automatically compute and cache Q15 output on every getWave() call.
 *        1 = Enabled (default).
 *        0 = Disabled (saves cycles if only raw DAC counts are needed).
 */
#ifndef ADSR_BEZIER_UPDATE_Q15_CACHE
#define ADSR_BEZIER_UPDATE_Q15_CACHE 1
#endif

/**
 * @def ADSR_BEZIER_NATIVE_Q15
 * @brief Primary amplitude domain selector.
 *        0 = DAC-primary domain (defined by constructor vertical_resolution).
 *        1 = Native Q15 domain (peak amplitude is ADSR_Q15_PEAK).
 */
#ifndef ADSR_BEZIER_NATIVE_Q15
#define ADSR_BEZIER_NATIVE_Q15 0
#endif

/**
 * @def ADSR_BEZIER_Q15_DYADIC
 * @brief Peak scale format when ADSR_BEZIER_NATIVE_Q15=1.
 *        1 = Peak is 32768 (allows bit-shift >>15 operations).
 *        0 = Peak is 32767.
 */
#ifndef ADSR_BEZIER_Q15_DYADIC
#define ADSR_BEZIER_Q15_DYADIC 1
#endif

/**
 * @def ADSR_BEZIER_SRAM_HOT
 * @brief Places time-critical functions into SRAM (RP2040 / Cortex-M RAM optimization).
 *        1 = Tag getWave/noteOn/noteOff with __not_in_flash_func.
 *        0 = Standard flash execution (default).
 */
#ifndef ADSR_BEZIER_SRAM_HOT
#define ADSR_BEZIER_SRAM_HOT 0
#endif
#if ADSR_BEZIER_SRAM_HOT
#ifndef __not_in_flash_func
#define __not_in_flash_func(fn) fn
#endif
#define ADSR_BEZIER_HOT(fn) __not_in_flash_func(fn)
#else
#define ADSR_BEZIER_HOT(fn) fn
#endif

// =================================================================
// CURVE PRESET DEFINITIONS
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
extern const char* const ADSR_CURVE_NAMES[ADSR_NUM_CURVES];

/**
 * @brief Get the string name of a curve preset.
 * @param curve Curve index or ADSRCurveType enum.
 * @return const char* String description (e.g. "Natural Exp", "Linear").
 */
const char* adsrGetCurveName(uint8_t curve);

// Lookup table resolution (number of sample points per curve)
#ifndef ARRAY_SIZE
#define ARRAY_SIZE 1024
#endif

// Unipolar Q15 bus full-scale definition (0 .. 32767)
static constexpr int16_t ADSR_Q15_ONE = 32767;

#if ADSR_BEZIER_NATIVE_Q15 && ADSR_BEZIER_Q15_DYADIC
static constexpr int ADSR_Q15_PEAK = 32768;
#else
static constexpr int ADSR_Q15_PEAK = (int)ADSR_Q15_ONE;
#endif

// Global Bézier lookup tables (8 decay/release curves + 8 mirrored attack curves)
extern int _curve_tables[8][ARRAY_SIZE];
extern int _curve_attack_tables[8][ARRAY_SIZE];

/**
 * @class adsr
 * @brief High-performance ADSR envelope generator with Bézier curve profiling.
 */
class adsr
{
public:
    /**
     * @brief Construct a new ADSR envelope instance.
     * 
     * @param vertical_resolution Peak output resolution (e.g. 4095 for 12-bit DAC, 255 for PWM, 32767 for Q15).
     * @param attack_curve        Curve shape for Attack (0-7 or ADSRCurveType). Default is ADSR_CURVE_EXP_NATURAL.
     * @param decay_curve         Curve shape for Decay (0-7 or ADSRCurveType). Default is ADSR_CURVE_EXP_NATURAL.
     * @param release_curve       Curve shape for Release (0-7 or ADSRCurveType). Default is ADSR_CURVE_EXP_NATURAL.
     */
    adsr(int vertical_resolution, 
         uint8_t attack_curve = ADSR_CURVE_EXP_NATURAL, 
         uint8_t decay_curve = ADSR_CURVE_EXP_NATURAL, 
         uint8_t release_curve = ADSR_CURVE_EXP_NATURAL)
    {
        _dac_export_vr = (vertical_resolution > 0) ? vertical_resolution : (int)ADSR_Q15_ONE;

#if ADSR_BEZIER_NATIVE_Q15
        _vertical_resolution = (int)ADSR_Q15_PEAK;
        _sustain = (int)ADSR_Q15_PEAK / 2;
        _to_q15_mul = 0;
        _to_dac_mul = (_dac_export_vr > 0) ? (((uint32_t)_dac_export_vr << 16) / (uint32_t)ADSR_Q15_PEAK) : 0;
#else
        _vertical_resolution = vertical_resolution;
        _sustain = vertical_resolution / 2;
        _to_q15_mul = (_vertical_resolution > 0) ? (((uint32_t)ADSR_Q15_ONE << 16) / (uint32_t)_vertical_resolution) : 0;
        _to_dac_mul = 0;
#endif
        _attack = 100000;  // Default 100ms
        _decay = 100000;   // Default 100ms
        _release = 100000; // Default 100ms

        _bezier_attack_type = attack_curve;
        _bezier_decay_type = decay_curve;
        _bezier_release_type = release_curve;
        bindCurvePtrs();

        _adsr_output = 0;
        _adsr_output_q15 = 0;
        _adsr_output_q15_src = -1;
    }

    /**
     * @brief Invalidate internal Q15 cache (used when parameters change).
     */
#if ADSR_BEZIER_NATIVE_Q15
    void invalidateQ15Cache() {}
#else
    void invalidateQ15Cache() { _adsr_output_q15_src = -1; }
#endif

    /**
     * @brief Set the curve shape preset for the Attack phase.
     * @param curve_type Curve preset (0 to 7 or ADSRCurveType).
     */
    void adsrCurveAttack(uint8_t curve_type)
    {
        _bezier_attack_type = curve_type;
        bindCurvePtrs();
    }

    /**
     * @brief Set the curve shape preset for the Decay phase.
     * @param curve_type Curve preset (0 to 7 or ADSRCurveType).
     */
    void adsrCurveDecay(uint8_t curve_type)
    {
        _bezier_decay_type = curve_type;
        bindCurvePtrs();
    }

    /**
     * @brief Set the curve shape preset for the Release phase.
     * @param curve_type Curve preset (0 to 7 or ADSRCurveType).
     */
    void adsrCurveRelease(uint8_t curve_type)
    {
        _bezier_release_type = curve_type;
        bindCurvePtrs();
    }

    /**
     * @brief Configure envelope behavior upon retriggering.
     * @param reset_attack If true, restarts attack from 0. If false (default), ramps smoothly from current output level.
     */
    void setResetAttack(bool reset_attack)
    {
        _reset_attack = reset_attack;
    }

    /**
     * @brief Set the Attack time in milliseconds.
     * @param attack_ms Duration of attack phase in ms.
     */
    void setAttack(unsigned long attack_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        _attack = attack_ms * 1000UL;
#else
        _attack = attack_ms;
#endif

#if ADSR_BEZIER_USE_FLOAT
        _attack_rate_f = (_attack > 0) ? ((float)(ARRAY_SIZE - 1) / (float)_attack) : 0.0f;
#else
        _attack_scale_phase = phaseScale(_attack);
#endif
    }

    /**
     * @brief Set the Decay time in milliseconds.
     * @param decay_ms Duration of decay phase in ms.
     */
    void setDecay(unsigned long decay_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        _decay = decay_ms * 1000UL; 
#else
        _decay = decay_ms;          
#endif

#if ADSR_BEZIER_USE_FLOAT
        _decay_rate_f = (_decay > 0) ? ((float)(ARRAY_SIZE - 1) / (float)_decay) : 0.0f;
#else
        _decay_scale_phase = phaseScale(_decay);
#endif
    }

    /**
     * @brief Set the Sustain level.
     * @param sustain_level Sustain amplitude level (0 to vertical_resolution).
     */
    void setSustain(int sustain_level)
    {
        if (sustain_level < 0) sustain_level = 0;
        if (sustain_level >= _vertical_resolution) sustain_level = _vertical_resolution;
        _sustain = sustain_level;

        int32_t range = (int32_t)_vertical_resolution - (int32_t)_sustain;
        if (range < 0) range = 0;

        _decay_range_scale_q16 = rangeScaleQ16(range, _vertical_resolution);
        invalidateQ15Cache();
    }

    /**
     * @brief Set the Release time in milliseconds.
     * @param release_ms Duration of release phase in ms.
     */
    void setRelease(unsigned long release_ms)
    {
#if ADSR_BEZIER_USE_MICROS
        _release = release_ms * 1000UL; 
#else
        _release = release_ms;          
#endif

#if ADSR_BEZIER_USE_FLOAT
        _release_rate_f = (_release > 0) ? ((float)(ARRAY_SIZE - 1) / (float)_release) : 0.0f;
#else
        _release_scale_phase = phaseScale(_release);
#endif
    }

    /**
     * @brief Trigger envelope Gate ON (starts Attack phase).
     */
    void ADSR_BEZIER_HOT(noteOn)()
    {
        unsigned long now = 
#if ADSR_BEZIER_USE_MICROS
            micros();
#else
            millis();
#endif
        _attack_start = _reset_attack ? 0 : _adsr_output;
        _notes_pressed = 1;

        _phase = ADSR_PHASE_ATTACK;
        _t_phase_start = now;

        int32_t range = (int32_t)_vertical_resolution - (int32_t)_attack_start;
        if (range < 0) range = 0;

        _attack_range_scale_q16 = rangeScaleQ16(range, _vertical_resolution);
        invalidateQ15Cache();
    }

    /**
     * @brief Trigger envelope Gate OFF (starts Release phase).
     */
    void ADSR_BEZIER_HOT(noteOff)()
    {
        _notes_pressed--;
        if (_notes_pressed <= 0)
        {
            unsigned long now = 
#if ADSR_BEZIER_USE_MICROS
                micros();
#else
                millis();
#endif
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
     * @brief Advance envelope state using internal timer (micros or millis).
     * @return Current envelope amplitude value in native units (0 .. vertical_resolution).
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
     * @brief Advance envelope state using an external timestamp.
     * @param current_ticks Current timestamp (in micros or millis matching ADSR_BEZIER_USE_MICROS).
     * @return Current envelope amplitude value in native units (0 .. vertical_resolution).
     */
    int ADSR_BEZIER_HOT(getWave)(unsigned long current_ticks)
    {
#if ADSR_BEZIER_NATIVE_Q15
        if (_phase == ADSR_PHASE_SUSTAIN) {
            _adsr_output = _sustain;
            _adsr_output_q15 = nativeTapQ15(_sustain);
            return (int)_adsr_output_q15;
        }
        if (_phase == ADSR_PHASE_IDLE) {
            _adsr_output = 0;
            _adsr_output_q15 = 0;
            return 0;
        }
#endif
        unsigned long delta = 0;

        switch (_phase)
        {
        case ADSR_PHASE_ATTACK:
        {
            if (_attack == 0)
            {
                _adsr_output = _vertical_resolution;
                _phase = (_decay > 0) ? ADSR_PHASE_DECAY : ADSR_PHASE_SUSTAIN;
                _t_phase_start = current_ticks;
                break;
            }

            delta = current_ticks - _t_phase_start;

            if (delta >= _attack)
            {
                _adsr_output = _vertical_resolution;
                _phase = (_decay > 0) ? ADSR_PHASE_DECAY : ADSR_PHASE_SUSTAIN;
                _t_phase_start = current_ticks;
                break;
            }

#if ADSR_BEZIER_USE_FLOAT
            uint32_t idx = (uint32_t)((float)delta * _attack_rate_f);
            if (idx >= ARRAY_SIZE) idx = ARRAY_SIZE - 1;
#else
            uint32_t idx = phaseIndexFixed(delta, _attack, _attack_scale_phase);
#endif
            int curveVal = _curve_attack_ptr[(int)idx];
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

            delta = current_ticks - _t_phase_start;

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

            delta = current_ticks - _t_phase_start;

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
            int32_t out = (int32_t)(((uint32_t)curveVal * _release_range_scale_q16) >> 16);

            _adsr_output = (int)out;
            break;
        }

        case ADSR_PHASE_IDLE:
        default:
            _adsr_output = 0;
            break;
        }

#if ADSR_BEZIER_NATIVE_Q15
        _adsr_output_q15 = nativeTapQ15(_adsr_output);
        return (int)_adsr_output_q15;
#elif ADSR_BEZIER_UPDATE_Q15_CACHE
        if (_adsr_output != _adsr_output_q15_src) {
            _adsr_output_q15_src = _adsr_output;
            uint32_t q = ((uint32_t)_adsr_output * _to_q15_mul) >> 16;
            if (q > (uint32_t)ADSR_Q15_ONE) q = (uint32_t)ADSR_Q15_ONE;
            _adsr_output_q15 = (int16_t)q;
        }
#endif
        return _adsr_output;
    }

    /**
     * @brief Get the last computed envelope level in fixed-point Q15 format (0 .. 32767).
     * @return Output amplitude formatted as int16_t (Q15).
     */
    int16_t levelQ15() const { return _adsr_output_q15; }

    /**
     * @brief Get the last computed envelope level scaled to DAC resolution.
     * @return Output amplitude scaled to constructor resolution.
     */
    int levelDac() const
    {
#if ADSR_BEZIER_NATIVE_Q15
        return (int)(((uint32_t)_adsr_output * _to_dac_mul) >> 16);
#else
        return _adsr_output;
#endif
    }

    /**
     * @brief Advance the envelope and directly return the Q15 formatted level.
     * @return Amplitude value in Q15 format (0 .. 32767).
     */
    int16_t getWaveQ15()
    {
        getWave();
        return _adsr_output_q15;
    }

    /**
     * @brief Advance the envelope with a timestamp and directly return the Q15 formatted level.
     * @param current_ticks Current timestamp.
     * @return Amplitude value in Q15 format (0 .. 32767).
     */
    int16_t getWaveQ15(unsigned long current_ticks)
    {
        getWave(current_ticks);
        return _adsr_output_q15;
    }

private:
#if ADSR_BEZIER_NATIVE_Q15
    static int16_t nativeTapQ15(int level)
    {
        if (level <= 0) return 0;
        if (level >= (int)ADSR_Q15_ONE) return ADSR_Q15_ONE;
        return (int16_t)level;
    }
#endif

    void bindCurvePtrs()
    {
        uint8_t a = (_bezier_attack_type >= ADSR_NUM_CURVES) ? (ADSR_NUM_CURVES - 1) : (uint8_t)_bezier_attack_type;
        uint8_t d = (_bezier_decay_type >= ADSR_NUM_CURVES) ? (ADSR_NUM_CURVES - 1) : (uint8_t)_bezier_decay_type;
        uint8_t r = (_bezier_release_type >= ADSR_NUM_CURVES) ? (ADSR_NUM_CURVES - 1) : (uint8_t)_bezier_release_type;
        _curve_attack_ptr = _curve_attack_tables[a];
        _curve_decay_ptr = _curve_tables[d];
        _curve_release_ptr = _curve_tables[r];
    }

    static uint32_t rangeScaleQ16(int32_t range, int vr)
    {
        if (range <= 0 || vr <= 0) return 0;
#if ADSR_BEZIER_NATIVE_Q15 && ADSR_BEZIER_Q15_DYADIC
        if (vr == ADSR_Q15_PEAK) return (uint32_t)range << 1;
#endif
        return (uint32_t)(((uint64_t)range << 16) / (uint32_t)vr);
    }

#if !ADSR_BEZIER_USE_FLOAT
#if ADSR_BEZIER_PHASE_SCALE_U64
    using phase_scale_t = uint64_t;
#else
    using phase_scale_t = uint32_t;
#endif

    static phase_scale_t phaseScale(unsigned long phase_ticks)
    {
        if (phase_ticks == 0) return 0;
        return (phase_scale_t)((((uint64_t)(ARRAY_SIZE - 1) << ADSR_BEZIER_PHASE_SHIFT) +
                                (phase_ticks >> 1)) / (uint64_t)phase_ticks);
    }

    static uint32_t phaseIndexFixed(unsigned long delta, unsigned long phase_ticks, phase_scale_t scale)
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
        return (idx >= ARRAY_SIZE) ? (ARRAY_SIZE - 1) : idx;
    }
#endif

    uint8_t _bezier_attack_type;
    uint8_t _bezier_decay_type;
    uint8_t _bezier_release_type;
    int *_curve_attack_ptr = nullptr;
    int *_curve_decay_ptr = nullptr;
    int *_curve_release_ptr = nullptr;

    int _vertical_resolution;
    int _dac_export_vr;
    uint32_t _to_dac_mul = 0;
    unsigned long _attack = 0;
    unsigned long _decay = 0; 
    int _sustain = 0;
    unsigned long _release = 0;
    bool _reset_attack = false;

#if !ADSR_BEZIER_USE_FLOAT
    phase_scale_t _attack_scale_phase = 0;
    phase_scale_t _decay_scale_phase = 0;
    phase_scale_t _release_scale_phase = 0;
#endif

    enum ADSRPhase
    {
        ADSR_PHASE_IDLE = 0,
        ADSR_PHASE_ATTACK,
        ADSR_PHASE_DECAY,
        ADSR_PHASE_SUSTAIN,
        ADSR_PHASE_RELEASE
    };

    ADSRPhase _phase = ADSR_PHASE_IDLE;
    unsigned long _t_phase_start = 0;

#if ADSR_BEZIER_USE_FLOAT
    float _attack_rate_f = 0.0f;
    float _decay_rate_f = 0.0f;
    float _release_rate_f = 0.0f;
#endif

    uint32_t _attack_range_scale_q16 = 0;
    uint32_t _decay_range_scale_q16 = 0;
    uint32_t _release_range_scale_q16 = 0;

    int _adsr_output;
    int16_t _adsr_output_q15 = 0;
    int _adsr_output_q15_src = -1;
    uint32_t _to_q15_mul = 0;
    int _release_start;
    int _attack_start;
    int _notes_pressed = 0;
};

/**
 * @brief Precomputes the 8 Bézier lookup tables into SRAM at system boot.
 * 
 * @param max_val    Peak vertical resolution (e.g. 4095.0f or 32768.0f).
 * @param num_points Number of points per curve (defaults to ARRAY_SIZE = 1024).
 */
void adsrBezierInitTables(float max_val, int num_points = ARRAY_SIZE);

#endif // ADSR_BEZIER_H