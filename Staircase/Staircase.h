
/*
 * Staircase.h
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 *
 * Copyright (C) 2025 brummer <brummer@web.de>
 */


/****************************************************************
        Staircase.h - distortion

****************************************************************/


#pragma once
#include <cmath>

class LM_EII12 {
public:
    float *cutoff;
    float *drive;
    float *amount;
    float  *onOff;

    void setSampleRate(float sr) {
        sampleRate = sr;
        float wc = 2.0f * M_PI * cutoffState;
        cutoff = &cutoffState;
        float k  = wc / (wc + sr);
        a = k;
        b = 1.f - k;
    }

    inline void process(float* output, uint32_t n_samples) {
        float wc = 2.0f * M_PI * *cutoff;
        float k  = wc / (wc + sampleRate);
        a = k;
        b = 1.f - k;
        for (uint32_t i = 0; i < n_samples; i++) {
            float tmp0 = output[i];
            output[i] = process(tmp0);
        }
    }

private:
    float cutoffState = 12000.0f;
    float sampleRate = 48000.0f;

    float lp = 0.0f, a=0.0f, b=0.0f;
    constexpr static float mu = 255.f;
    constexpr static float q = 1.0f / 2048.0f;

    inline float tanh_fast(float x) {
        float x2 = x * x;
        return x * (27.0f + x2) / (27.0f + 9.0f * x2);
    }

    inline float postSaturate(float x) {
        return x / (*drive + std::fabs(x));
    }

    inline float process(float x) {
        x *= *drive;
        float s = copysignf(1.f, x);
        x = s * log1p(mu * fabsf(x)) / log1p(mu);
        x = std::round(x / q) * q;
        x = tanh_fast(x * *drive);
        lp = 1e-15f + a * x + b * lp - 1e-15f;
        lp *= *amount;
        return postSaturate(lp);
    }
};

