//------------------------------------------------------------------//
// ADSR Envelope Generator for Arduino (Bézier Curve Engine)
// Table generation & cubic Bézier interpolation functions
//------------------------------------------------------------------//

#include "ADSR_Bezier.h"

// Global human-readable curve names
const char* const ADSR_CURVE_NAMES[ADSR_NUM_CURVES] = {
    "Natural Exp",
    "Smooth Exp",
    "Percussive",
    "Log / Convex",
    "Soft S-Curve",
    "Steep S-Curve",
    "Rounded",
    "Linear"
};

const char* adsrGetCurveName(uint8_t curve)
{
    if (curve >= ADSR_NUM_CURVES)
        curve = ADSR_NUM_CURVES - 1;
    return ADSR_CURVE_NAMES[curve];
}

// Global memory allocation for Bézier curve lookup tables
int _curve_tables[8][ARRAY_SIZE];
int _curve_attack_tables[8][ARRAY_SIZE];

/**
 * @struct ADSRBezierPoint
 * @brief Floating-point 2D coordinate for cubic Bézier solver.
 */
struct ADSRBezierPoint
{
    float x, y;
};

/**
 * @brief Evaluates cubic Bézier coordinates at normalized progress parameter t.
 */
static inline ADSRBezierPoint adsrBezierCubic(const ADSRBezierPoint &A,
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
 */
static inline float adsrBezierFindYForX(const ADSRBezierPoint &A,
                                        const ADSRBezierPoint &P1,
                                        const ADSRBezierPoint &P2,
                                        const ADSRBezierPoint &B,
                                        float xTarget,
                                        float tol = 1e-5f)
{
    float tLow = 0.0f;
    float tHigh = 1.0f;
    float tMid = 0.0f;

    for (int iter = 0; iter < 64 && (tHigh - tLow) > tol; ++iter)
    {
        tMid = (tLow + tHigh) * 0.5f;
        ADSRBezierPoint midPoint = adsrBezierCubic(A, P1, P2, B, tMid);
        if (midPoint.x < xTarget)
            tLow = tMid;
        else
            tHigh = tMid;
    }

    return adsrBezierCubic(A, P1, P2, B, tMid).y;
}

void adsrBezierInitTables(float max_val, int num_points)
{
    ADSRBezierPoint A = {0.0f, max_val};
    ADSRBezierPoint B = {max_val, 0.0f};

    static constexpr float kAuthPeak = 4096.0f;
    const float s = max_val / kAuthPeak;

    // 8 Pre-authored control point profiles for distinct acoustic slopes
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

        float multiplier = (float)(max_val + 1.0f) / (float)(num_points - 1);

        for (int i = 0; i < num_points; ++i)
        {
            float xTarget = multiplier * (float)i;
            float yResult = adsrBezierFindYForX(A, P1, P2, B, xTarget);
            _curve_tables[j][i] = (int)roundf(yResult);
        }
        for (int i = 0; i < num_points; ++i)
        {
            _curve_attack_tables[j][i] = _curve_tables[j][num_points - 1 - i];
        }
    }
}