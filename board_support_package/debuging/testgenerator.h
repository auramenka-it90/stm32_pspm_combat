/**
 ******************************************************************************
 * @file    testgenerator.h
 * @brief   Portable debug waveform generator for DAC bring-up.
 *          Values are expressed on the bipolar BC DAC scale:
 *          full scale = +/-32768 code units, zero = mid-scale.
 *          Port of the MU_GSP debuging/testgenerator module with the brace
 *          bug fixed (LEVEL/NOISE were trapped inside the TRIANGLE case) and
 *          rand() replaced by a deterministic per-instance LCG.
 *          All comments in ASCII English.
 ******************************************************************************
 */

#ifndef DEBUGING_TESTGENERATOR_H_
#define DEBUGING_TESTGENERATOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    T_W_NONE     = 0, /* Output 0                        */
    T_W_MEANDER  = 1, /* Square wave +A / -A             */
    T_W_SINE     = 2, /* Sine                            */
    T_W_SAW      = 3, /* Symmetric ramp -A..+A           */
    T_W_TRIANGLE = 4, /* Triangle -A..+A..-A             */
    T_W_LEVEL    = 5, /* Constant LEVEL = offset + A     */
    T_W_NOISE    = 6  /* Deterministic pseudo noise      */
} CTestGeneratorWaveForm_t;

typedef struct {
    float amplitude;              /* Wave amplitude, code units (+/-32768)   */
    float offset;                 /* DC bias, code units (+/-32768)          */
    float freq_hz;                /* Wave frequency, Hz (> 0)                */
    CTestGeneratorWaveForm_t waveform;
    float m_time;                 /* Internal running phase time, seconds    */
    uint32_t lcg_state;           /* Noise generator state (deterministic)   */
    float value;                  /* Result of the last Process() call       */
} CTestGenerator;

/* Reset the generator: phase to zero, noise seed to a fixed value. */
void  CTestGenerator_init(CTestGenerator *t, float amplitude_, float freq_hz_,
                          CTestGeneratorWaveForm_t waveform_);

/* Advance the model by elapsed_time seconds and return the new sample. */
float CTestGenerator_Process(CTestGenerator *t, float elapsed_time);

#ifdef __cplusplus
}
#endif

#endif /* DEBUGING_TESTGENERATOR_H_ */
