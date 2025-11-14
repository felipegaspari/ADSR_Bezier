//----------------------------------//
// ADSR class for Arduino
// by mo-thunderz
// version 1.2
// last update: 14.08.2022
//----------------------------------//

// Use Arduino timing functions (millis) internally
#include "Arduino.h"
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
        _attack = 100;                             // take 100ms as initial value for Attack
        _sustain = l_vertical_resolution / 2;         // take half the DAC_size as initial value for sustain
        _decay = 100;                              // take 100ms as initial value for Decay
        _release = 100;                            // take 100ms as initial value for Release

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

    void setAttack(unsigned long l_attack)
    {
        _attack = l_attack;

        // Precompute fixed-point scale for fast time->index mapping (Q6 format)
        // idx ~= delta_ms * ((ARRAY_SIZE-1) / _attack)
        // Using Q6 (shift by 6) keeps the product within 32 bits even for long times.
        if (_attack > 0)
        {
            _attack_scale_q6 = ((uint32_t)(ARRAY_SIZE - 1) << 6) / _attack;
        }
        else
        {
            _attack_scale_q6 = 0;
        }

        // Keep cached sum for faster stage checks in getWave()
        _attack_plus_decay = _attack + _decay;
    }

    void setDecay(unsigned long l_decay)
    {
        _decay = l_decay;

        if (_decay > 0)
        {
            _decay_scale_q6 = ((uint32_t)(ARRAY_SIZE - 1) << 6) / _decay;
        }
        else
        {
            _decay_scale_q6 = 0;
        }

        // Keep cached sum for faster stage checks in getWave()
        _attack_plus_decay = _attack + _decay;
    }

    void setSustain(int l_sustain)
    {
        if (l_sustain < 0)
            l_sustain = 0;
        if (l_sustain >= _vertical_resolution)
            l_sustain = _vertical_resolution;
        _sustain = l_sustain;
    }

    void setRelease(unsigned long l_release)
    {
        _release = l_release;

        if (_release > 0)
        {
            _release_scale_q6 = ((uint32_t)(ARRAY_SIZE - 1) << 6) / _release;
        }
        else
        {
            _release_scale_q6 = 0;
        }
    }

    // Use current millis() timestamp internally
    void noteOn()
    {
        unsigned long now = millis();
        _t_note_on = now; // set new timestamp for note_on
        if (_reset_attack)     // set start value new Attack
            _attack_start = 0; // if _reset_attack equals true, a new trigger starts with 0
        else
            _attack_start = _adsr_output; // if _reset_attack equals false, a new trigger starts with the current value
        _notes_pressed++;                 // increase number of pressed notes with one
    }

    void noteOff()
    {
        _notes_pressed--;
        if (_notes_pressed <= 0)
        {                                  // if all notes are depressed - start release
            unsigned long now = millis();
            _t_note_off = now;             // set timestamp for note off
            _release_start = _adsr_output; // set start value for release
            _notes_pressed = 0;
        }
    }

    // Compute ADSR value based on current millis()
    int getWave()
    {
        unsigned long l_millis = millis();
        unsigned long delta = 0;

        if (_t_note_off < _t_note_on)
        { // if note is pressed
            // Total time since note on
            delta = l_millis - _t_note_on;

            // Attack
            if ((delta < _attack) && (_attack > 0))
            {
                // Integer time->index mapping using precomputed Q6 scale
                uint32_t idx = (delta * _attack_scale_q6) >> 6;
                if (idx >= ARRAY_SIZE)
                    idx = ARRAY_SIZE - 1;

                // Attack curve runs "backwards" through the table
                _adsr_output = map(
                    _curve_tables[_bezier_attack_type][(ARRAY_SIZE - 1) - (int)idx],
                    0, _vertical_resolution,
                    _attack_start, _vertical_resolution);
            }
            else if (delta < _attack_plus_decay)
            { // Decay
                // Time since start of decay
                delta -= _attack;

                if (_decay > 0)
                {
                    uint32_t idx = (delta * _decay_scale_q6) >> 6;
                    if (idx >= ARRAY_SIZE)
                        idx = ARRAY_SIZE - 1;

                    _adsr_output = map(
                        _curve_tables[_bezier_decay_type][(int)idx],
                        0, _vertical_resolution,
                        _sustain, _vertical_resolution);
                }
                else
                {
                    _adsr_output = _sustain;
                }
            }
            else
                _adsr_output = _sustain;
        }
        else if (_t_note_off > _t_note_on)
        { // if note not pressed
            delta = l_millis - _t_note_off;
            if ((delta < _release) && (_release > 0)) // release
            {
                uint32_t idx = (delta * _release_scale_q6) >> 6;
                if (idx >= ARRAY_SIZE)
                    idx = ARRAY_SIZE - 1;

                _adsr_output = map(
                    _curve_tables[_bezier_release_type][(int)idx],
                    0, _vertical_resolution,
                    0, _release_start);
            }
            else
                _adsr_output = 0; // note off
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
    unsigned long _attack = 0;  // 0 to 20 sec
    unsigned long _decay = 0;   // 1ms to 60 sec
    int _sustain = 0;           // 0 to -60dB -> then -inf
    unsigned long _release = 0; // 1ms to 60 sec
    bool _reset_attack = false; // if _reset_attack is "true" a new trigger starts with 0, if _reset_attack is false it starts with the current output value

    // Cached sums and precomputed fixed-point (Q6) scales for fast time->index conversion
    unsigned long _attack_plus_decay = 0;
    uint32_t _attack_scale_q6 = 0;
    uint32_t _decay_scale_q6 = 0;
    uint32_t _release_scale_q6 = 0;

    // time stamp for note on and note off
    unsigned long _t_note_on = 0;
    unsigned long _t_note_off = 0;

    // internal values needed to transition to new pulse (attack) and to release at any point in time
    int _adsr_output;
    int _release_start;
    int _attack_start;
    int _notes_pressed = 0;
};

#endif
