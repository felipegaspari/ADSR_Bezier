//----------------------------------//
// ADSR class for Arduino
// by mo-thunderz
// version 1.2
// last update: 14.08.2022
//----------------------------------//

#include "Arduino.h"
#include <math.h>

#ifndef ADSR
#define ADSR

// for array for lookup table
#ifndef ARRAY_SIZE
#define ARRAY_SIZE 1024
#endif

// number of time points
//#define ATTACK_ALPHA 0.997                  // varies between 0.9 (steep curve) and 0.9995 (straight line)
//#define ATTACK_DECAY_RELEASE 0.997          // fits to ARRAY_SIZE 1024

//#define ARRAY_SIZE 1024                   // number of time points
//#define ATTACK_ALPHA 0.9975           // varies between 0.9 (steep curve) and 0.9995 (straight line)
//#define ATTACK_DECAY_RELEASE 0.9975   

// Midi trigger -> on/off
class adsr
{

    public:
        // constructor

struct Point {
    float x, y;
};

// Función que calcula un punto en la curva de Bézier cúbica para un valor dado de t
Point bezierCubic(const Point& A, const Point& P1, const Point& P2, const Point& B, float t) {
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
float findYForX(const Point& A, const Point& P1, const Point& P2, const Point& B, float xTarget, float tol = 1e-5) {
    float tLow = 0.0f;
    float tHigh = 1.0f;
    float tMid;

    while ((tHigh - tLow) > tol) {
        tMid = (tLow + tHigh) / 2.0f;
        Point midPoint = bezierCubic(A, P1, P2, B, tMid);
        if (midPoint.x < xTarget) {
            tLow = tMid;
        } else {
            tHigh = tMid;
        }
    }

    Point resultPoint = bezierCubic(A, P1, P2, B, tMid);
    return resultPoint.y;
}

adsr(int l_vertical_resolution, float attack_alpha, float attack_decay_release, bool bezier, int bezier_attack_type, int bezier_decay_release_type)
{
    _vertical_resolution = l_vertical_resolution; // store vertical resolution (DAC_Size)
    _attack = 100000;                             // take 100ms as initial value for Attack
    _sustain = l_vertical_resolution / 2;         // take half the DAC_size as initial value for sustain
    _decay = 100000;                              // take 100ms as initial value for Decay
    _release = 100000;                            // take 100ms as initial value for Release

    if (bezier == true)
    {
        for (int i = 0; i < ARRAY_SIZE; i++)
        { // Create look-up table for Attack
            _attack_table[i] = i;
            _decay_release_table[i] = _vertical_resolution - i;
        }

        for (int i = 0; i < ARRAY_SIZE - 1; i++)
        { // Create look-up table for Decay
            _attack_table[i + 1] = (1.0 - attack_alpha) * (_vertical_resolution) + attack_alpha * _attack_table[i];
            _decay_release_table[i + 1] = attack_decay_release * _decay_release_table[i];
        }

        for (int i = 0; i < ARRAY_SIZE; i++)
        { // normalize table to min and max
            _attack_table[i] = map(_attack_table[i], 0, _attack_table[ARRAY_SIZE - 1], 0, _vertical_resolution);
            _decay_release_table[i] = map(_decay_release_table[i], _decay_release_table[ARRAY_SIZE - 1], _decay_release_table[0], 0, _vertical_resolution);
        }
    }
    else
    {

        Point P1_attack;
        Point P2_attack;

        Point P1_decay;
        Point P2_decay;

        switch (bezier_attack_type)
        {
        case 0: // Standard soft
            P1_attack = {210, 1700};
            P2_attack = {620, 3970};
            break;
        case 1: // Softer start
            P1_attack = {1610, 2940};
            P2_attack = {2160, 3960};
            break;
        case 2: // very steep
            P1_attack = {330, 4220};
            P2_attack = {430, 3710};
            break;
        case 3: // concave
            P1_attack = {4390, 300};
            P2_attack = {3610, 1730};
            break;
        case 4: // fast start, dead middle, aggresive rise
            P1_attack = {860, 1875};
            P2_attack = {4050, 720};
            break;
        default: // fast start, dead middle, aggresive rise
            P1_attack = {860, 1875};
            P2_attack = {4050, 720};
            break;
        }

        switch (bezier_decay_release_type)
        {
        case 0:
            P1_decay = {170, 1650}; // Standard, soft
            P2_decay = {1290, 50};
            break;
        case 1: // Softer start
            P1_decay = {840, 1780};
            P2_decay = {1160, 210};
            break;
        case 2: // very steep
            P1_decay = {160, 360};
            P2_decay = {-80, 470};
            break;
        case 3: // convex
            P1_decay = {2170, 3610};
            P2_decay = {3730, 2610};
            break;
        case 4: // fast start, dead middle, aggresive dive
            P1_decay = {400, 1380};
            P2_decay = {3830, 2890};
            break;
        default: // fast start, dead middle, aggresive dive
            P1_decay = {400, 1380};
            P2_decay = {3830, 2890};
            break;
        }

        adsrCurveAttack(P1_attack, P2_attack, l_vertical_resolution, ARRAY_SIZE);

        adsrCurveDecayRelease(P1_decay, P2_decay, l_vertical_resolution, ARRAY_SIZE);
    }
}

void adsrCurveAttack(Point P1, Point P2, float maxVal, int numPoints)
{

    Point A = {0, 0}; // Punto inicial
    Point B = {maxVal, maxVal};

    float multiplier = (float)(maxVal + 1) / (float)(numPoints - 1);

    // Imprimir los puntos de la curva
    for (float i = 0; i < numPoints; i++)
    {
        float xTarget = multiplier * i;
        float yResult = findYForX(A, P1, P2, B, xTarget);

        _attack_table[(int)i] = (int)round(yResult);
    }
}

void adsrCurveDecayRelease(Point P1, Point P2, float maxVal, int numPoints)
{

    Point A = {0, maxVal}; // Punto inicial
    Point B = {maxVal, 0};

    float multiplier = (float)(maxVal + 1) / (float)(numPoints - 1);

    // Imprimir los puntos de la curva
    for (float i = 0; i < numPoints; i++)
    {
        float xTarget = multiplier * i;
        float yResult = findYForX(A, P1, P2, B, xTarget);

        _decay_release_table[(int)i] = (int)round(yResult);
    }
}

void changeCurves(int l_vertical_resolution, float attack_alpha, float attack_decay_release)
{
    _vertical_resolution = l_vertical_resolution; // store vertical resolution (DAC_Size)
    _attack = 100000;                             // take 100ms as initial value for Attack
    _sustain = l_vertical_resolution / 2;         // take half the DAC_size as initial value for sustain
    _decay = 100000;                              // take 100ms as initial value for Decay
    _release = 100000;                            // take 100ms as initial value for Release

    for (int i = 0; i < ARRAY_SIZE; i++)
    { // Create look-up table for Attack
        _attack_table[i] = i;
        _decay_release_table[i] = _vertical_resolution  - i;
    }

    for (int i = 0; i < ARRAY_SIZE - 1; i++)
    { // Create look-up table for Decay
        _attack_table[i + 1] = (1.0 - attack_alpha) * (_vertical_resolution ) + attack_alpha * _attack_table[i];
        _decay_release_table[i + 1] = attack_decay_release * _decay_release_table[i];
    }

    for (int i = 0; i < ARRAY_SIZE; i++)
    { // normalize table to min and max
        _attack_table[i] = map(_attack_table[i], 0, _attack_table[ARRAY_SIZE - 1], 0, _vertical_resolution );
        _decay_release_table[i] = map(_decay_release_table[i], _decay_release_table[ARRAY_SIZE - 1], _decay_release_table[0], 0, _vertical_resolution );
    }
}

void setResetAttack(bool l_reset_attack)
{
    _reset_attack = l_reset_attack;
}

void setAttack(unsigned long l_attack)
{
    _attack = l_attack;
}

void setDecay(unsigned long l_decay)
{
    _decay = l_decay;
}

void setSustain(int l_sustain)
{
    if (l_sustain < 0)
        l_sustain = 0;
    if (l_sustain >= _vertical_resolution)
        l_sustain = _vertical_resolution ;
    _sustain = l_sustain;
}

void setRelease(unsigned long l_release)
{
    _release = l_release;
}

void noteOn(unsigned long l_micros)
{
    _t_note_on = l_micros; // set new timestamp for note_on
    if (_reset_attack)     // set start value new Attack
        _attack_start = 0; // if _reset_attack equals true, a new trigger starts with 0
    else
        _attack_start = _adsr_output; // if _reset_attack equals false, a new trigger starts with the current value
    _notes_pressed++;                 // increase number of pressed notes with one
}

void noteOff(unsigned long l_micros)
{
    _notes_pressed--;
    if (_notes_pressed <= 0)
    {                                  // if all notes are depressed - start release
        _t_note_off = l_micros;        // set timestamp for note off
        _release_start = _adsr_output; // set start value for release
        _notes_pressed = 0;
    }
}

int getWave(unsigned long l_micros)
{
    unsigned long delta = 0;
    if (_t_note_off < _t_note_on)
    { // if note is pressed
        delta = l_micros - _t_note_on;
        if (delta < _attack)                                                                                                                                                 // Attack
            _adsr_output = map(_attack_table[(int)floor((float)ARRAY_SIZE * (float)delta / (float)_attack)], 0, _vertical_resolution, _attack_start, _vertical_resolution ); //
        else if (delta < _attack + _decay)
        { // Decay
            delta = l_micros - _t_note_on - _attack;
            _adsr_output = map(_decay_release_table[(int)floor((float)ARRAY_SIZE * (float)delta / (float)_decay)], 0, _vertical_resolution, _sustain, _vertical_resolution );
        }
        else
            _adsr_output = _sustain;
    }
    if (_t_note_off > _t_note_on)
    { // if note not pressed
        delta = l_micros - _t_note_off;
        if (delta < _release) // release
            _adsr_output = map(_decay_release_table[(int)floor((float)ARRAY_SIZE * (float)delta / (float)_release)], 0, _vertical_resolution , 0, _release_start);
        else
            _adsr_output = 0; // note off
    }
    return _adsr_output;
}
    private:

        int _attack_table[ARRAY_SIZE];
        int _decay_release_table[ARRAY_SIZE];

        int _vertical_resolution;                               // number of bits for output, control, etc
        unsigned long _attack = 0;                              // 0 to 20 sec
        unsigned long _decay = 0;                               // 1ms to 60 sec
        int _sustain = 0;                                       // 0 to -60dB -> then -inf
        unsigned long _release = 0;                             // 1ms to 60 sec
        bool _reset_attack = false;                             // if _reset_attack is "true" a new trigger starts with 0, if _reset_attack is false it starts with the current output value

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
