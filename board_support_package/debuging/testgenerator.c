/**
 ******************************************************************************
 * @file    testgenerator.c
 * @brief   Portable debug waveform generator implementation (see header).
 *          All comments in ASCII English.
 ******************************************************************************
 */

#include "testgenerator.h"
#include <math.h>

void CTestGenerator_init(CTestGenerator *t, float amplitude_, float freq_hz_,
                         CTestGeneratorWaveForm_t waveform_)
{
    t->amplitude = amplitude_;
    t->offset    = 0.0f;
    t->freq_hz   = freq_hz_;
    t->waveform  = waveform_;
    t->m_time    = 0.0f;
    t->lcg_state = 0x1234ABCDu;
    t->value     = 0.0f;
}

float CTestGenerator_Process(CTestGenerator *t, float elapsed_time)
{
    float result = 0.0f;
    float phase  = 0.0f;
    float f;

    /* Frequency guard: a non-positive frequency freezes the phase at zero */
    if (t->freq_hz > 0.0f) {
        t->m_time += elapsed_time;
        f = 1.0f / t->freq_hz;
        while (t->m_time >= f) {
            t->m_time -= f;
        }
        phase = t->m_time / f;              /* Normalized 0..1 within period */
    }

    switch (t->waveform) {
    case T_W_MEANDER:
        result = (phase < 0.5f) ? (t->amplitude + t->offset)
                                : (-t->amplitude + t->offset);
        break;

    case T_W_SINE:
        result = t->amplitude * sinf(2.0f * (float)M_PI * phase) + t->offset;
        break;

    case T_W_SAW:
        /* Symmetric bipolar ramp: -A at phase 0, +A before phase 1 */
        result = t->amplitude * (2.0f * phase - 1.0f) + t->offset;
        break;

    case T_W_TRIANGLE:
        if (phase < 0.25f) {
            result = -t->amplitude + 4.0f * t->amplitude * phase;
        } else if (phase < 0.75f) {
            result = t->amplitude - 4.0f * t->amplitude * (phase - 0.25f);
        } else {
            result = -t->amplitude + 4.0f * t->amplitude * (phase - 0.75f);
        }
        result += t->offset;
        break;

    case T_W_LEVEL:
        result = t->amplitude + t->offset;
        break;

    case T_W_NOISE: {
        /* Numerical Recipes LCG: deterministic, reproducible across runs */
        t->lcg_state = t->lcg_state * 1664525u + 1013904223u;
        float u = (float)(t->lcg_state >> 8) / 16777216.0f;   /* 0..1 */
        result = t->amplitude * (2.0f * u - 1.0f) + t->offset;
        break;
    }

    case T_W_NONE:
    default:
        result = 0.0f;
        break;
    }

    t->value = result;
    return result;
}
