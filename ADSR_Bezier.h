//----------------------------------//
// ADSR class for Arduino
// by mo-thunderz
// version 1.2
// last update: 14.08.2022
//----------------------------------//

// Use Arduino timing functions internally
#include "Arduino.h"

// Select timebase:
// 1 -> use micros() internally (high resolution)
// 0 -> use millis() internally (backwards-compatible behavior)
#ifndef ADSR_BEZIER_USE_MICROS
#define ADSR_BEZIER_USE_MICROS 1
#endif
// #include <math.h>

#ifndef ADSR
#define ADSR

// for array for lookup table
#ifndef ARRAY_SIZE
#define ARRAY_SIZE 1024
#endif

// number of time points
// #define ATTACK_ALPHA 0.997                  // varies between 0.9 (steep curve) and 0.9995 (straight line)
// #define ATTACK_DECAY_RELEASE 0.997          // fits to ARRAY_SIZE 1024

// #define ARRAY_SIZE 1024                   // number of time points
// #define ATTACK_ALPHA 0.9975           // varies between 0.9 (steep curve) and 0.9995 (straight line)
// #define ATTACK_DECAY_RELEASE 0.9975

// Global curve table pointer (defined later in this header)
extern int *_curve_tables[8];

// Midi trigger -> on/off
class adsr
{

public:
    // constructor

    struct Point
    {
        float x, y;
    };

    adsr(int l_vertical_resolution, float attack_alpha, float attack_decay_release, bool bezier, int bezier_attack_type, int bezier_decay_type, int bezier_release_type)
    {
        _vertical_resolution = l_vertical_resolution; // store vertical resolution (DAC_Size)
        _attack = 100000;                             // take 100ms as initial value for Attack
        _sustain = l_vertical_resolution / 2;         // take half the DAC_size as initial value for sustain
        _decay = 100000;                              // take 100ms as initial value for Decay
        _release = 100000;                            // take 100ms as initial value for Release

        if (bezier == true)
        {
            // for (int i = 0; i < ARRAY_SIZE; i++)
            // { // Create look-up table for Attack
            //     _attack_table[i] = i;
            //     _decay_release_table[i] = _vertical_resolution - i;
            // }

            // for (int i = 0; i < ARRAY_SIZE - 1; i++)
            // { // Create look-up table for Decay
            //     _attack_table[i + 1] = (1.0 - attack_alpha) * (_vertical_resolution) + attack_alpha * _attack_table[i];
            //     _decay_release_table[i + 1] = attack_decay_release * _decay_release_table[i];
            // }

            // for (int i = 0; i < ARRAY_SIZE; i++)
            // { // normalize table to min and max
            //     _attack_table[i] = map(_attack_table[i], 0, _attack_table[ARRAY_SIZE - 1], 0, _vertical_resolution);
            //     _decay_release_table[i] = map(_decay_release_table[i], _decay_release_table[ARRAY_SIZE - 1], _decay_release_table[0], 0, _vertical_resolution);
            // }
        }
        else
        {

            // adsrCreateTables(l_vertical_resolution, ARRAY_SIZE);
        }
    }

    // void adsrCreateTables(float maxVal, int numPoints)
    //{

    // Point A = {0, maxVal}; // Punto inicial
    // Point B = {maxVal, 0};
    // Point P1[8] = {{250, 1500}, {840, 1780}, {400, 430}, {2170, 3610}, {400, 1380}, {1140, 3750}, {200, 2700}, {0, 4095}};
    // Point P2[8] = {{1500, 250}, {1160, 210}, {920, 420}, {3730, 2610}, {3830, 2890}, {1850, 1080}, {720, 3050}, {4095, 0}};

    // for (int j = 0; j < 8; j++)
    // {

    //     float multiplier = (float)(maxVal + 1) / (float)(numPoints - 1);

    //     // Imprimir los puntos de la curva
    //     for (float i = 0; i < numPoints; i++)
    //     {
    //         float xTarget = multiplier * i;
    //         float yResult = findYForX(A, P1[j], P2[j], B, xTarget);

    //         _curve_tables[j][(int)i] = (int)round(yResult);
    //     }
    // }
    //}

    void adsrCurveAttack(uint8_t curveType)
    {
        _bezier_attack_type = curveType;
    }

    void adsrCurveDecay(uint8_t curveType)
    {
        _bezier_decay_type = curveType;
    }

    void adsrCurveRelease(uint8_t curveType)
    {
        _bezier_release_type = curveType;
    }

    void setResetAttack(bool l_reset_attack)
    {
        _reset_attack = l_reset_attack;
    }

    // Attack time in milliseconds (same external semantics as millis-based ADSR)
    void setAttack(unsigned long l_attack_ms)
    {
        // Convert to internal timebase (ticks)
#if ADSR_BEZIER_USE_MICROS
        unsigned long attack_ticks = l_attack_ms * 1000UL; // µs
#else
        unsigned long attack_ticks = l_attack_ms;          // ms
#endif
        _attack = attack_ticks;

        // Precompute fixed-point scale for fast time->index mapping (Q24 format)
        // idx ~= delta_ticks * ((ARRAY_SIZE-1) / _attack)
        if (_attack > 0 && _attack <= _time_q24_max_ticks)
        {
            _attack_scale_q24 = (((uint64_t)(ARRAY_SIZE - 1)) << 24) / (uint64_t)_attack;
        }
        else
        {
            _attack_scale_q24 = 0;
        }
    }

    // Decay time in milliseconds
    void setDecay(unsigned long l_decay_ms)
    {
        // Convert to internal timebase (ticks)
#if ADSR_BEZIER_USE_MICROS
        unsigned long decay_ticks = l_decay_ms * 1000UL; // µs
#else
        unsigned long decay_ticks = l_decay_ms;          // ms
#endif
        _decay = decay_ticks;

        if (_decay > 0 && _decay <= _time_q24_max_ticks)
        {
            _decay_scale_q24 = (((uint64_t)(ARRAY_SIZE - 1)) << 24) / (uint64_t)_decay;
        }
        else
        {
            _decay_scale_q24 = 0;
        }
    }

    void setSustain(int l_sustain)
    {
        if (l_sustain < 0)
            l_sustain = 0;
        if (l_sustain >= _vertical_resolution)
            l_sustain = _vertical_resolution;
        _sustain = l_sustain;

        // Precompute decay output range scale: from sustain up to full level
        // out = sustain + curveVal * (vertical_resolution - sustain) / vertical_resolution
        int32_t range = (int32_t)_vertical_resolution - (int32_t)_sustain;
        if (range < 0)
            range = 0;
        if (_vertical_resolution > 0)
        {
            _decay_range_scale_q16 = (int32_t)(((int32_t)range << 16) / _vertical_resolution);
        }
        else
        {
            _decay_range_scale_q16 = 0;
        }
    }

    // Release time in milliseconds
    void setRelease(unsigned long l_release_ms)
    {
        // Convert to internal timebase (ticks)
#if ADSR_BEZIER_USE_MICROS
        unsigned long release_ticks = l_release_ms * 1000UL; // µs
#else
        unsigned long release_ticks = l_release_ms;          // ms
#endif
        _release = release_ticks;

        if (_release > 0 && _release <= _time_q24_max_ticks)
        {
            _release_scale_q24 = (((uint64_t)(ARRAY_SIZE - 1)) << 24) / (uint64_t)_release;
        }
        else
        {
            _release_scale_q24 = 0;
        }
    }

    // Use current micros() timestamp internally
    void noteOn()
    {
        unsigned long now;
#if ADSR_BEZIER_USE_MICROS
        now = micros();
#else
        now = millis();
#endif
        _t_note_on = now; // set new timestamp for note_on
        if (_reset_attack)     // set start value new Attack
            _attack_start = 0; // if _reset_attack equals true, a new trigger starts with 0
        else
            _attack_start = _adsr_output; // if _reset_attack equals false, a new trigger starts with the current value
        _notes_pressed++;                 // increase number of pressed notes with one

        // Start attack phase
        _phase = ADSR_PHASE_ATTACK;
        _t_phase_start = now;

        // Precompute attack output range scale: from attack_start up to full level
        // out = attack_start + curveVal * (vertical_resolution - attack_start) / vertical_resolution
        int32_t range = (int32_t)_vertical_resolution - (int32_t)_attack_start;
        if (range < 0)
            range = 0;
        if (_vertical_resolution > 0)
        {
            _attack_range_scale_q16 = (int32_t)(((int32_t)range << 16) / _vertical_resolution);
        }
        else
        {
            _attack_range_scale_q16 = 0;
        }
    }

    void noteOff()
    {
        _notes_pressed--;
        if (_notes_pressed <= 0)
        {                                  // if all notes are depressed - start release
            unsigned long now;
#if ADSR_BEZIER_USE_MICROS
            now = micros();
#else
            now = millis();
#endif
            _t_note_off = now;             // set timestamp for note off
            _release_start = _adsr_output; // set start value for release
            _notes_pressed = 0;

            // Start release phase
            _phase = ADSR_PHASE_RELEASE;
            _t_phase_start = now;

            // Precompute release output range scale: from release_start down to 0
            // out = curveVal * release_start / vertical_resolution
            int32_t rs = (int32_t)_release_start;
            if (rs < 0)
                rs = 0;
            if (rs > _vertical_resolution)
                rs = _vertical_resolution;
            if (_vertical_resolution > 0)
            {
                _release_range_scale_q16 = (int32_t)((rs << 16) / _vertical_resolution);
            }
            else
            {
                _release_range_scale_q16 = 0;
            }
        }
    }

    // Compute ADSR value based on current timebase (micros or millis)
    int getWave()
    {
        unsigned long l_ticks;
#if ADSR_BEZIER_USE_MICROS
        l_ticks = micros();
#else
        l_ticks = millis();
#endif
        unsigned long delta = 0;

        switch (_phase)
        {
        case ADSR_PHASE_ATTACK:
        {
            if (_attack == 0)
            {
                // Immediate attack -> go to next phase
                _adsr_output = _vertical_resolution;
                if (_decay > 0)
                {
                    _phase = ADSR_PHASE_DECAY;
                    _t_phase_start = l_ticks;
                }
                else
                {
                    _phase = ADSR_PHASE_SUSTAIN;
                }
                break;
            }

            delta = l_ticks - _t_phase_start;

            if (delta >= _attack)
            {
                // End of attack -> full level
                _adsr_output = _vertical_resolution;
                if (_decay > 0)
                {
                    _phase = ADSR_PHASE_DECAY;
                    _t_phase_start = l_ticks;
                }
                else
                {
                    _phase = ADSR_PHASE_SUSTAIN;
                }
                break;
            }

            // Time->index mapping for attack
            uint32_t idx;
            if (_attack > 0 && _attack <= _time_q24_max_ticks && _attack_scale_q24 != 0)
            {
                idx = (uint32_t)(((uint64_t)delta * _attack_scale_q24) >> 24);
            }
            else
            {
                idx = (uint32_t)(((uint64_t)(ARRAY_SIZE - 1) * (uint64_t)delta) / (uint64_t)_attack);
            }
            if (idx >= ARRAY_SIZE)
                idx = ARRAY_SIZE - 1;

            // Attack curve runs "backwards" through the table
            int curveVal = _curve_tables[_bezier_attack_type][(ARRAY_SIZE - 1) - (int)idx];

            // Map to output
            int32_t out = (int32_t)_attack_start +
                          (int32_t)(((int32_t)curveVal * _attack_range_scale_q16) >> 16);
            if (out < 0)
                out = 0;
            if (out > _vertical_resolution)
                out = _vertical_resolution;
            _adsr_output = (int)out;
            break;
        }

        case ADSR_PHASE_DECAY:
        {
            if (_decay == 0)
            {
                // Immediate decay -> sustain
                _adsr_output = _sustain;
                _phase = ADSR_PHASE_SUSTAIN;
                break;
            }

            delta = l_ticks - _t_phase_start;

            if (delta >= _decay)
            {
                // End of decay -> sustain
                _adsr_output = _sustain;
                _phase = ADSR_PHASE_SUSTAIN;
                break;
            }

            uint32_t idx;
            if (_decay > 0 && _decay <= _time_q24_max_ticks && _decay_scale_q24 != 0)
            {
                idx = (uint32_t)(((uint64_t)delta * _decay_scale_q24) >> 24);
            }
            else
            {
                idx = (uint32_t)(((uint64_t)(ARRAY_SIZE - 1) * (uint64_t)delta) / (uint64_t)_decay);
            }
            if (idx >= ARRAY_SIZE)
                idx = ARRAY_SIZE - 1;

            int curveVal = _curve_tables[_bezier_decay_type][(int)idx];

            int32_t out = (int32_t)_sustain +
                          (int32_t)(((int32_t)curveVal * _decay_range_scale_q16) >> 16);
            if (out < 0)
                out = 0;
            if (out > _vertical_resolution)
                out = _vertical_resolution;
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

            delta = l_ticks - _t_phase_start;

            if (delta >= _release)
            {
                _adsr_output = 0;
                _phase = ADSR_PHASE_IDLE;
                break;
            }

            uint32_t idx;
            if (_release > 0 && _release <= _time_q24_max_ticks && _release_scale_q24 != 0)
            {
                idx = (uint32_t)(((uint64_t)delta * _release_scale_q24) >> 24);
            }
            else
            {
                idx = (uint32_t)(((uint64_t)(ARRAY_SIZE - 1) * (uint64_t)delta) / (uint64_t)_release);
            }
            if (idx >= ARRAY_SIZE)
                idx = ARRAY_SIZE - 1;

            int curveVal = _curve_tables[_bezier_release_type][(int)idx];

            int32_t out = (int32_t)(((int32_t)curveVal * _release_range_scale_q16) >> 16);
            if (out < 0)
                out = 0;
            if (out > _vertical_resolution)
                out = _vertical_resolution;
            _adsr_output = (int)out;
            break;
        }

        case ADSR_PHASE_IDLE:
        default:
        {
            _adsr_output = 0;
            break;
        }
        }
        return _adsr_output;
    }

    // Función que calcula un punto en la curva de Bézier cúbica para un valor dado de t
    Point bezierCubic(const Point &A, const Point &P1, const Point &P2, const Point &B, float t)
    {
        float one_minus_t = 1.0f - t;
        float one_minus_t_squared = one_minus_t * one_minus_t;
        float t_squared = t * t;
        float x = one_minus_t_squared * one_minus_t * A.x +
                  3 * one_minus_t_squared * t * P1.x +
                  3 * one_minus_t * t_squared * P2.x +
                  t_squared * t * B.x;
        float y = one_minus_t_squared * one_minus_t * A.y +
                  3 * one_minus_t_squared * t * P1.y +
                  3 * one_minus_t * t_squared * P2.y +
                  t_squared * t * B.y;
        return {x, y};
    }

    // Función para encontrar el valor de y dado un valor de x en la curva de Bézier
    float findYForX(const Point &A, const Point &P1, const Point &P2, const Point &B, float xTarget, float tol = 1e-6)
    {
        float tLow = 0.0f;
        float tHigh = 1.0f;
        float tMid;

        while ((tHigh - tLow) > tol)
        {
            tMid = (tLow + tHigh) / 2.0f;
            Point midPoint = bezierCubic(A, P1, P2, B, tMid);
            if (midPoint.x < xTarget)
            {
                tLow = tMid;
            }
            else
            {
                tHigh = tMid;
            }
        }

        Point resultPoint = bezierCubic(A, P1, P2, B, tMid);
        return resultPoint.y;
    }

private:
    int _bezier_attack_type;
    int _bezier_decay_type;
    int _bezier_release_type;

    int _vertical_resolution;   // number of bits for output, control, etc
    unsigned long _attack = 0;  // 0 to 20 sec (in microseconds)
    unsigned long _decay = 0;   // 1ms to 60 sec  (in microseconds)
    int _sustain = 0;           // 0 to -60dB -> then -inf
    unsigned long _release = 0; // 1ms to 60 sec (in microseconds)
    bool _reset_attack = false; // if _reset_attack is "true" a new trigger starts with 0, if _reset_attack is false it starts with the current output value

    // Threshold for using Q24 fixed-point vs exact division (in internal ticks)
#if ADSR_BEZIER_USE_MICROS
    static constexpr unsigned long _time_q24_max_ticks = 2000000UL; // 2 seconds in µs
#else
    static constexpr unsigned long _time_q24_max_ticks = 2000UL;    // 2 seconds in ms
#endif

    // Precomputed fixed-point (Q24) scales for fast time->index conversion
    uint64_t _attack_scale_q24 = 0;
    uint64_t _decay_scale_q24  = 0;
    uint64_t _release_scale_q24 = 0;

    // Internal ADSR phase state
    enum ADSRPhase
    {
        ADSR_PHASE_IDLE = 0,
        ADSR_PHASE_ATTACK,
        ADSR_PHASE_DECAY,
        ADSR_PHASE_SUSTAIN,
        ADSR_PHASE_RELEASE
    };

    ADSRPhase _phase = ADSR_PHASE_IDLE;

    // Phase start time (ticks) for the current stage
    unsigned long _t_phase_start = 0;

    // Precomputed fixed-point (Q16) scales for fast curve->output mapping
    int32_t _attack_range_scale_q16 = 0;
    int32_t _decay_range_scale_q16 = 0;
    int32_t _release_range_scale_q16 = 0;

    // time stamp for note on and note off
    unsigned long _t_note_on = 0;
    unsigned long _t_note_off = 0;

    // internal values needed to transition to new pulse (attack) and to release at any point in time
    int _adsr_output;
    int _release_start;
    int _attack_start;
    int _notes_pressed = 0;
};

// ---------------------------------------------------------------------------
// Bézier table generation helpers
// ---------------------------------------------------------------------------

// Global Bézier lookup tables shared by all ADSR instances.
// ARRAY_SIZE is provided by the including project before this header.
int _curve0_table[ARRAY_SIZE];
int _curve1_table[ARRAY_SIZE];
int _curve2_table[ARRAY_SIZE];
int _curve3_table[ARRAY_SIZE];
int _curve4_table[ARRAY_SIZE];
int _curve5_table[ARRAY_SIZE];
int _curve6_table[ARRAY_SIZE];
int _curve7_table[ARRAY_SIZE];

int *_curve_tables[8] = {
    _curve0_table, _curve1_table, _curve2_table, _curve3_table,
    _curve4_table, _curve5_table, _curve6_table, _curve7_table};

// Lightweight point type used for table generation
struct ADSRBezierPoint
{
    float x, y;
};

// Evaluate a cubic Bézier at parameter t in [0, 1]
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

// Find y for a given x on the cubic Bézier using binary search on t
inline float adsrBezierFindYForX(const ADSRBezierPoint &A,
                                 const ADSRBezierPoint &P1,
                                 const ADSRBezierPoint &P2,
                                 const ADSRBezierPoint &B,
                                 float xTarget,
                                 float tol = 1e-5f)
{
    float tLow = 0.0f;
    float tHigh = 1.0f;
    float tMid = 0.0f;

    while ((tHigh - tLow) > tol)
    {
        tMid = (tLow + tHigh) * 0.5f;
        ADSRBezierPoint midPoint = adsrBezierCubic(A, P1, P2, B, tMid);
        if (midPoint.x < xTarget)
        {
            tLow = tMid;
        }
        else
        {
            tHigh = tMid;
        }
    }

    ADSRBezierPoint resultPoint = adsrBezierCubic(A, P1, P2, B, tMid);
    return resultPoint.y;
}

// Generate 8 Bézier curves into the provided curve_tables (size [8][numPoints])
// maxVal: maximum y value (e.g. vertical_resolution)
// numPoints: number of points per curve (ARRAY_SIZE)
inline void adsrBezierInitTables(float maxVal, int numPoints, int *curve_tables[8])
{
    ADSRBezierPoint A = {0.0f, maxVal};
    ADSRBezierPoint B = {maxVal, 0.0f};

    ADSRBezierPoint P1[8] = {
        {250.0f, 1500.0f}, {840.0f, 1780.0f}, {400.0f, 430.0f},  {2170.0f, 3610.0f},
        {400.0f, 1380.0f}, {1140.0f, 3750.0f}, {200.0f, 2700.0f}, {0.0f, 4095.0f}};

    ADSRBezierPoint P2[8] = {
        {1500.0f, 250.0f}, {1160.0f, 210.0f}, {920.0f, 420.0f},  {3730.0f, 2610.0f},
        {3830.0f, 2890.0f}, {1850.0f, 1080.0f}, {720.0f, 3050.0f}, {4095.0f, 0.0f}};

    for (int j = 0; j < 8; ++j)
    {
        float multiplier = (float)(maxVal + 1.0f) / (float)(numPoints - 1);

        for (int i = 0; i < numPoints; ++i)
        {
            float xTarget = multiplier * (float)i;
            float yResult = adsrBezierFindYForX(A, P1[j], P2[j], B, xTarget);

            curve_tables[j][i] = (int)roundf(yResult);
        }
    }
}

#endif
