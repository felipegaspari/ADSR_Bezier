//----------------------------------//
// ADSR class for Arduino
// by mo-thunderz
// version 1.2
// last update: 14.08.2020
//----------------------------------//

#include "Arduino.h"
#include "adsr_fela_bezier.h"
#include <math.h>

adsr::Point bezierCubic(adsr::Point A, adsr::Point P1, adsr::Point P2, adsr::Point B, double t)
{
    double x = pow(1 - t, 3) * A.x + 3 * pow(1 - t, 2) * t * P1.x + 3 * (1 - t) * t * t * P2.x + pow(t, 3) * B.x;
    double y = pow(1 - t, 3) * A.y + 3 * pow(1 - t, 2) * t * P1.y + 3 * (1 - t) * t * t * P2.y + pow(t, 3) * B.y;
    return {x, y};
}

// Función para encontrar el valor de y dado un valor de x en la curva de Bézier
double findYForX(adsr::Point A, adsr::Point P1, adsr::Point P2, adsr::Point B, double xTarget, double tol = 1e-6)
{
    double tLow = 0.0;
    double tHigh = 1.0;
    double tMid = 0.0;

    while ((tHigh - tLow) > tol)
    {
        tMid = (tLow + tHigh) / 2.0;
        adsr::Point midPoint = bezierCubic(A, P1, P2, B, tMid);
        if (midPoint.x < xTarget)
        {
            tLow = tMid;
        }
        else
        {
            tHigh = tMid;
        }
    }
    adsr::Point resultPoint = bezierCubic(A, P1, P2, B, tMid);
    return resultPoint.y;
}

adsr::Point bezierQuadratic(adsr::Point A, adsr::Point P1, adsr::Point B, double t)
{
    double x = pow(1 - t, 2) * A.x + 2 * (1 - t) * t * P1.x + t * t * B.x;
    double y = pow(1 - t, 2) * A.y + 2 * (1 - t) * t * P1.y + t * t * B.y;
    return {x, y};
}

// Función para encontrar el valor de y dado un valor de x en la curva de Bézier
double findYForXQuadratic(adsr::Point A, adsr::Point P1, adsr::Point B, double xTarget, double tol = 1e-6)
{
    double tLow = 0.0;
    double tHigh = 1.0;
    double tMid = 0.0;

    while ((tHigh - tLow) > tol)
    {
        tMid = (tLow + tHigh) / 2.0;
        adsr::Point midPoint = bezierQuadratic(A, P1, B, tMid);
        if (midPoint.x < xTarget)
        {
            tLow = tMid;
        }
        else
        {
            tHigh = tMid;
        }
    }
    adsr::Point resultPoint = bezierQuadratic(A, P1, B, tMid);
    return resultPoint.y;
}

adsr::adsr(int l_vertical_resolution, float attack_alpha, float attack_decay_release, bool bezier, int bezier_attack_type, int bezier_decay_release_type)
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

        adsr::Point P1_attack;
        adsr::Point P2_attack;

        adsr::Point P1_decay;
        adsr::Point P2_decay;

        switch (bezier_attack_type)
        {
        case 0: // Standard, soft
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

        adsr::adsrCurveAttack(P1_attack, P2_attack, l_vertical_resolution, ARRAY_SIZE);

        adsr::adsrCurveDecayRelease(P1_decay, P2_decay, l_vertical_resolution, ARRAY_SIZE);
    }
}

void adsr::adsrCurveAttack(adsr::Point P1, adsr::Point P2, double maxVal, int numPoints)
{

    adsr::Point A = {0, 0}; // Punto inicial
    adsr::Point B = {maxVal, maxVal};

    double multiplier = (double)(maxVal + 1) / (double)(numPoints - 1);

    // Imprimir los puntos de la curva
    for (double i = 0; i < numPoints; i++)
    {
        double xTarget = multiplier * i;
        double yResult = findYForX(A, P1, P2, B, xTarget);

        _attack_table[(int)i] = (int)round(yResult);
    }
}

void adsr::adsrCurveDecayRelease(adsr::Point P1, adsr::Point P2, double maxVal, int numPoints)
{

    adsr::Point A = {0, maxVal}; // Punto inicial
    adsr::Point B = {maxVal, 0};

    double multiplier = (double)(maxVal + 1) / (double)(numPoints - 1);

    // Imprimir los puntos de la curva
    for (double i = 0; i < numPoints; i++)
    {
        double xTarget = multiplier * i;
        double yResult = findYForX(A, P1, P2, B, xTarget);

        _decay_release_table[(int)i] = (int)round(yResult);
    }
}

void adsr::changeCurves(int l_vertical_resolution, float attack_alpha, float attack_decay_release)
{
    _vertical_resolution = l_vertical_resolution; // store vertical resolution (DAC_Size)
    _attack = 100000;                             // take 100ms as initial value for Attack
    _sustain = l_vertical_resolution / 2;         // take half the DAC_size as initial value for sustain
    _decay = 100000;                              // take 100ms as initial value for Decay
    _release = 100000;                            // take 100ms as initial value for Release

    for (int i = 0; i < ARRAY_SIZE; i++)
    { // Create look-up table for Attack
        _attack_table[i] = i;
        _decay_release_table[i] = _vertical_resolution - 1 - i;
    }

    for (int i = 0; i < ARRAY_SIZE - 1; i++)
    { // Create look-up table for Decay
        _attack_table[i + 1] = (1.0 - attack_alpha) * (_vertical_resolution - 1) + attack_alpha * _attack_table[i];
        _decay_release_table[i + 1] = attack_decay_release * _decay_release_table[i];
    }

    for (int i = 0; i < ARRAY_SIZE; i++)
    { // normalize table to min and max
        _attack_table[i] = map(_attack_table[i], 0, _attack_table[ARRAY_SIZE - 1], 0, _vertical_resolution - 1);
        _decay_release_table[i] = map(_decay_release_table[i], _decay_release_table[ARRAY_SIZE - 1], _decay_release_table[0], 0, _vertical_resolution - 1);
    }
}

void adsr::setResetAttack(bool l_reset_attack)
{
    _reset_attack = l_reset_attack;
}

void adsr::setAttack(unsigned long l_attack)
{
    _attack = l_attack;
}

void adsr::setDecay(unsigned long l_decay)
{
    _decay = l_decay;
}

void adsr::setSustain(int l_sustain)
{
    if (l_sustain < 0)
        l_sustain = 0;
    if (l_sustain >= _vertical_resolution)
        l_sustain = _vertical_resolution - 1;
    _sustain = l_sustain;
}

void adsr::setRelease(unsigned long l_release)
{
    _release = l_release;
}

void adsr::noteOn(unsigned long l_micros)
{
    _t_note_on = l_micros; // set new timestamp for note_on
    if (_reset_attack)     // set start value new Attack
        _attack_start = 0; // if _reset_attack equals true, a new trigger starts with 0
    else
        _attack_start = _adsr_output; // if _reset_attack equals false, a new trigger starts with the current value
    _notes_pressed++;                 // increase number of pressed notes with one
}

void adsr::noteOff(unsigned long l_micros)
{
    _notes_pressed--;
    if (_notes_pressed <= 0)
    {                                  // if all notes are depressed - start release
        _t_note_off = l_micros;        // set timestamp for note off
        _release_start = _adsr_output; // set start value for release
        _notes_pressed = 0;
    }
}

int adsr::getWave(unsigned long l_micros)
{
    unsigned long delta = 0;
    if (_t_note_off < _t_note_on)
    { // if note is pressed
        delta = l_micros - _t_note_on;
        if (delta < _attack)                                                                                                                                                 // Attack
            _adsr_output = map(_attack_table[(int)floor(ARRAY_SIZE * (float)delta / (float)_attack)], 0, _vertical_resolution - 1, _attack_start, _vertical_resolution - 1); //
        else if (delta < _attack + _decay)
        { // Decay
            delta = l_micros - _t_note_on - _attack;
            _adsr_output = map(_decay_release_table[(int)floor(ARRAY_SIZE * (float)delta / (float)_decay)], 0, _vertical_resolution - 1, _sustain, _vertical_resolution - 1);
        }
        else
            _adsr_output = _sustain;
    }
    if (_t_note_off > _t_note_on)
    { // if note not pressed
        delta = l_micros - _t_note_off;
        if (delta < _release) // release
            _adsr_output = map(_decay_release_table[(int)floor(ARRAY_SIZE * (float)delta / (float)_release)], 0, _vertical_resolution - 1, 0, _release_start);
        else
            _adsr_output = 0; // note off
    }
    return _adsr_output;
}