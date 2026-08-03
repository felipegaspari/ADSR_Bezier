// Host-side regression test: main (fixed Q24/Q16) vs float-version hot-path math.
// Build: g++ -std=c++17 -O2 -o compare compare.cpp && ./compare

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE 512
#endif

static constexpr unsigned long TIME_Q24_MAX_TICKS = 2000000UL; // 2s in us

static uint32_t fixedIndexFromDelta(unsigned long delta,
                                    unsigned long phase_ticks,
                                    int array_size)
{
    uint32_t idx;
    if (phase_ticks > 0 && phase_ticks <= TIME_Q24_MAX_TICKS)
    {
        uint64_t scale_q24 =
            (((uint64_t)(array_size - 1)) << 24) / (uint64_t)phase_ticks;
        idx = (uint32_t)(((uint64_t)delta * scale_q24) >> 24);
    }
    else if (phase_ticks > 0)
    {
        idx = (uint32_t)(((uint64_t)(array_size - 1) * (uint64_t)delta) /
                         (uint64_t)phase_ticks);
    }
    else
    {
        idx = 0;
    }
    if (idx >= (uint32_t)array_size)
        idx = (uint32_t)array_size - 1;
    return idx;
}

static uint32_t exactIndexFromDelta(unsigned long delta,
                                    unsigned long phase_ticks,
                                    int array_size)
{
    if (phase_ticks == 0)
        return 0;
    uint32_t idx = (uint32_t)(((uint64_t)(array_size - 1) * (uint64_t)delta) /
                              (uint64_t)phase_ticks);
    if (idx >= (uint32_t)array_size)
        idx = (uint32_t)array_size - 1;
    return idx;
}

static uint32_t floatIndexFromDelta(unsigned long delta,
                                    unsigned long phase_ticks,
                                    float idx_scale,
                                    int array_size)
{
    uint32_t idx;
    if (phase_ticks > 16777216UL)
    {
        idx = (uint32_t)(((uint64_t)(array_size - 1) * (uint64_t)delta) /
                         (uint64_t)phase_ticks);
    }
    else if (idx_scale > 0.0f)
    {
        idx = (uint32_t)floorf((float)delta * idx_scale);
    }
    else
    {
        idx = 0;
    }
    if (idx >= (uint32_t)array_size)
        idx = (uint32_t)array_size - 1;
    return idx;
}

static int32_t fixedRangeOutput(int32_t start, int curve_val, int32_t range_q16)
{
    return start + (int32_t)(((int32_t)curve_val * range_q16) >> 16);
}

static int32_t floatRangeOutput(int32_t start, int curve_val, float range_scale)
{
    return start + (int32_t)((float)curve_val * range_scale);
}

static int sweepIndex(const char *label,
                      unsigned long phase_ticks_us,
                      int array_size,
                      int *max_vs_fixed,
                      int *max_vs_exact)
{
    float idx_scale = (phase_ticks_us > 0)
                          ? (float)((double)(array_size - 1) / (double)phase_ticks_us)
                          : 0.0f;
    int local_fixed = 0;
    int local_exact = 0;
    unsigned long step = phase_ticks_us / (unsigned long)(array_size * 4);
    if (step == 0)
        step = 1;

    for (unsigned long delta = 0; delta < phase_ticks_us; delta += step)
    {
        uint32_t fixed_idx =
            fixedIndexFromDelta(delta, phase_ticks_us, array_size);
        uint32_t exact_idx =
            exactIndexFromDelta(delta, phase_ticks_us, array_size);
        uint32_t float_idx =
            floatIndexFromDelta(delta, phase_ticks_us, idx_scale, array_size);

        int diff_fixed = (int)fixed_idx - (int)float_idx;
        if (diff_fixed < 0)
            diff_fixed = -diff_fixed;
        int diff_exact = (int)exact_idx - (int)float_idx;
        if (diff_exact < 0)
            diff_exact = -diff_exact;

        if (diff_fixed > local_fixed)
            local_fixed = diff_fixed;
        if (diff_exact > local_exact)
            local_exact = diff_exact;
    }

    if (local_fixed > *max_vs_fixed)
        *max_vs_fixed = local_fixed;
    if (local_exact > *max_vs_exact)
        *max_vs_exact = local_exact;

    printf("  index %s (%lu us): vs main fixed=%d, vs exact int=%d\n", label,
           phase_ticks_us, local_fixed, local_exact);
    return local_exact;
}

static int sweepOutput(int vertical_resolution, int *max_vs_fixed, int *max_vs_exact)
{
    int local_fixed = 0;
    int local_exact = 0;

    for (int range = 0; range <= vertical_resolution; range += 137)
    {
        int32_t range_q16 =
            (vertical_resolution > 0)
                ? (int32_t)(((int32_t)range << 16) / vertical_resolution)
                : 0;
        float range_scale = (vertical_resolution > 0)
                                ? (float)range / (float)vertical_resolution
                                : 0.0f;

        for (int curve = 0; curve <= vertical_resolution; curve += 211)
        {
            int32_t fixed_out = fixedRangeOutput(1000, curve, range_q16);
            int32_t float_out = floatRangeOutput(1000, curve, range_scale);
            int32_t exact_out =
                1000 + (int32_t)(((int64_t)curve * (int64_t)range) /
                                 (int64_t)vertical_resolution);

            int diff_fixed = (int)fixed_out - (int)float_out;
            if (diff_fixed < 0)
                diff_fixed = -diff_fixed;
            int diff_exact = (int)exact_out - (int)float_out;
            if (diff_exact < 0)
                diff_exact = -diff_exact;

            if (diff_fixed > local_fixed)
                local_fixed = diff_fixed;
            if (diff_exact > local_exact)
                local_exact = diff_exact;
        }
    }

    if (local_fixed > *max_vs_fixed)
        *max_vs_fixed = local_fixed;
    if (local_exact > *max_vs_exact)
        *max_vs_exact = local_exact;

    printf("  output (vertical=%d): vs main fixed=%d, vs exact int=%d\n",
           vertical_resolution, local_fixed, local_exact);
    return local_exact;
}

int main()
{
    const unsigned long times_us[] = {
        1000UL, 100000UL, 2000000UL, 10000000UL, 20000000UL, 60000000UL,
    };
    const unsigned long dco_times_us[] = {
        1000UL, 100000UL, 2000000UL, 10000000UL,
    };

    int max_idx_vs_fixed = 0;
    int max_idx_vs_exact = 0;
    int max_out_vs_fixed = 0;
    int max_out_vs_exact = 0;
    int fail = 0;

    printf("ADSR_Bezier fixed vs float comparison (ARRAY_SIZE=%d)\n\n",
           ARRAY_SIZE);

    printf("Time->index sweeps:\n");
    for (unsigned long t : times_us)
        sweepIndex("phase", t, ARRAY_SIZE, &max_idx_vs_fixed, &max_idx_vs_exact);

    printf("\nCurve->output sweeps:\n");
    sweepOutput(4000, &max_out_vs_fixed, &max_out_vs_exact);
    sweepOutput(4095, &max_out_vs_fixed, &max_out_vs_exact);

    printf("\nSummary:\n");
    printf("  max index diff vs main fixed:  %d\n", max_idx_vs_fixed);
    printf("  max index diff vs exact int:   %d\n", max_idx_vs_exact);
    printf("  max output diff vs main fixed: %d\n", max_out_vs_fixed);
    printf("  max output diff vs exact int:  %d\n", max_out_vs_exact);

    max_idx_vs_exact = 0;
    printf("\nDCO-realistic index check (float vs exact integer):\n");
    for (unsigned long t : dco_times_us)
        sweepIndex("dco", t, ARRAY_SIZE, &max_idx_vs_fixed, &max_idx_vs_exact);

    if (max_idx_vs_exact != 0)
    {
        printf("\nFAIL: float index differs from exact integer (max=%d)\n",
               max_idx_vs_exact);
        fail = 1;
    }

    if (max_out_vs_exact != 0)
    {
        printf("\nFAIL: float output differs from exact integer (max=%d)\n",
               max_out_vs_exact);
        fail = 1;
    }

    if (!fail)
        printf("\nPASS: float-version matches exact envelope math.\n");
    if (max_idx_vs_fixed > 0)
        printf("NOTE: up to %d index step drift vs main Q24 at <=2s (expected).\n",
               max_idx_vs_fixed);

    return fail ? 1 : 0;
}
