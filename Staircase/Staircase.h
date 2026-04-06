
/*
 * Staircase.h
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 *
 * Copyright (C) 2025 brummer <brummer@web.de>
 */


/****************************************************************
        Staircase.h - compander & distortion with hp/lp filters

****************************************************************/

#pragma once
#include <cmath>
#include <algorithm>
#include "StreamingResampler.h"

class LM_EII12 {
public:
    float *highcut = nullptr;
    float *lowcut = nullptr;
    float *drive = nullptr;
    float *amount = nullptr;
    float *onOff = nullptr;
    float *hpSlope = nullptr;
    float *lpSlope = nullptr;
    float *distMode = nullptr;

    void setSampleRate(float sr) {
        sampleRate = sr;
        resUp.setup(1, 8192, sampleRate, 2*sampleRate);
        resDown.setup(1, 8192, 2*sampleRate, sampleRate);
    }

    inline void process(float* output, uint32_t n_samples) {
        // fetch controller values from host
        const float highcutVal = highcut ? *highcut : 2100.0f;
        const float lowcutVal = lowcut ? *lowcut : 220.0f;
        const float driveVal = drive ? *drive : 1.2f;
        const float amountVal = amount ? *amount : 0.75f;
        const int hpSlopesVal = hpSlope ? (int)*hpSlope : 1;
        const int lpSlopesVal = lpSlope ? (int)*lpSlope : 1;        
        const int hpStages = slopeToStages(hpSlopesVal);
        const int lpStages = slopeToStages(lpSlopesVal);

        // check if dist model have changed
        if (distModeVal != (int)*distMode) {
            distModeVal = distMode ? (int)*distMode : 0;
            setMode();
        }
        // check if highpass filter stages have changed
        if (hpStages != lastHpStages) {
            for (int i = hpStages; i < MAX_STAGES; ++i) {
                hp_state[i] = 1e-15f;
                hp_post_state[i] = 1e-15f;
            }
            lastHpStages = hpStages;
        }
        // check if lowpass filter stages have changed
        if (lpStages != lastLpStages) {
            for (int i = lpStages; i < MAX_STAGES; ++i)
                lp_state[i] = 1e-15f;
            lastLpStages = lpStages;
        }
        // update filter coeffs if needed
        if (hpCoeffs != ((float)hpStages + lowcutVal)) {
            hpCoeffs = (float)hpStages + lowcutVal;
            updateHpCoeffs(hpStages, lowcutVal);
        }
        if (lpCoeffs != ((float)lpStages + highcutVal)) {
            lpCoeffs = (float)lpStages + highcutVal;
            updateLpCoeffs(lpStages, highcutVal);
        }
        // run pre highpass filter (remove mud)
        for (uint32_t i = 0; i < n_samples; i++) {
            float x = output[i];
            float y = x;
            for (int s = 0; s < hpStages; ++s) {
                y = onepole_hp(y, hp_state[s], ha);
            }
            output[i] = y;
        }
        // run distortion 2 x oversampled
        uint32_t r = resUp.getOutSize(n_samples);
        float buf[r];
        memset(buf, 0, r * sizeof(float));
        resUp.resample(output, buf, n_samples);
        for (uint32_t i = 0; i < r; i++) {
            float x = buf[i];
            buf[i] = (this->*dist)(x, driveVal, amountVal);
        }
        resDown.resample(buf, output, r);
        // run post highpass and lowpass filters
        for (uint32_t i = 0; i < n_samples; i++) {
            float x = output[i];
            float y1 = x;
            for (int s = 0; s < hpStages; ++s) {
                y1 = onepole_hp(y1, hp_post_state[s], hb);
            }
            x = y1;

            for (int s = 0; s < lpStages; ++s) {
                x = onepole_lp(x, lp_state[s], a, b);
            }
            output[i] = x;
        }
    }

private:
    float (LM_EII12::*dist)(float x, const float driveVal, const float amountVal);
    StreamingResampler resUp;
    StreamingResampler resDown;
    float sampleRate = 48000.0f;
    int distModeVal = -1;
    float hpCoeffs = 0.0f;
    float lpCoeffs = 0.0f;
    // Filter States
    static constexpr int MAX_STAGES = 6;
    float lp_state[MAX_STAGES] = {1e-15f};
    float hp_state[MAX_STAGES] = {1e-15f};
    float hp_post_state[MAX_STAGES] = {1e-15f};

    int lastHpStages = -1;
    int lastLpStages = -1;

    float a=0.0f, b=0.0f;
    float ha=0.0f, hb=0.0f;

    float dc_z = 0.0f;
    float dc_y = 0.0f;

    constexpr static float mu = 255.f;
    constexpr static float q = 1.0f / 2048.0f;

    enum DistMode {
        OFF,
        SOFT,
        CRUNCH,
        ROCK
    };

    void setMode() {
        switch (distModeVal)
        {
            case (OFF)    : this->dist = &LM_EII12::offDist;
            break;
            case (SOFT)   : this->dist = &LM_EII12::softDist;
            break;
            case (CRUNCH) : this->dist = &LM_EII12::crunchDist;
            break;
            case (ROCK)   : this->dist = &LM_EII12::rockDist;
            break;
            default       : this->dist = &LM_EII12::softDist;
            break;
        }        
    }

    // compute lowpass cutoff frequency compensation for used stages
    inline float compute_corrected_k(float wc, float fs, int stages) {
        float target = powf(2.0f, -1.0f / stages);
        float k = wc / (wc + fs);

        for (int i = 0; i < 8; ++i) {
            float b = 1.0f - k;
            float omega = wc / fs;
            float cos_w = cosf(omega);
            float denom = 1.0f + b*b - 2.0f * b * cos_w;
            float H2 = (k*k) / denom;
            float err = H2 - target;
            float dk = 1e-5f;
            float k2 = k + dk;
            float b2 = 1.0f - k2;
            float denom2 = 1.0f + b2*b2 - 2.0f * b2 * cos_w;
            float H2_2 = (k2*k2) / denom2;
            float deriv = (H2_2 - H2) / dk;
            k -= err / (deriv + 1e-12f);
            k = fmaxf(1e-6f, fminf(0.9999f, k));
        }
        return k;
    }

    // update lowpass filter cutoff frequencies
    inline void updateLpCoeffs(const int lpStages, const float highcutVal) {
        float wc = 2.0f * M_PI * highcutVal;
        float k = compute_corrected_k(wc, sampleRate, lpStages);
        a = k;
        b = 1.f - k;
    }

    // update highpass filter cutoff frequencies
    inline void updateHpCoeffs(const int hpStages, const float lowcutVal) {
        float correctedFc = lowcutVal / sqrtf((float)hpStages);
        float hwc = 2.0f * M_PI * correctedFc;
        float hk  = hwc / (hwc + sampleRate);
        ha = fmaxf(1e-6f, fminf(0.9999f, hk));

        float hbwc = 2.0f * M_PI * fmaxf(12.0f,correctedFc * 0.85f);
        float hbk  = hbwc / (hbwc + sampleRate);
        hb = fmaxf(1e-6f, fminf(0.9999f, hbk));
    }

    // convert controller values to stages
    inline int slopeToStages(int slope) {
        if (slope == 0)   return 1; // 6 dB
        if (slope == 1)   return 2; // 12 dB
        if (slope == 2)   return 4; // 24 dB
        return 6;                   // 36 dB
    }

    // one pole lowpass filter
    inline float onepole_lp(float x, float& z, float a, float b) {
        z = 1e-15f + a * x + b * z - 1e-15f;
        return z;
    }

    // one pole highpass filter
    inline float onepole_hp(float x, float& z, float a) {
        float y = x - z;
        z = 1e-15f + z + a * y - 1e-15f;
        return y;
    }

    // tanh
    inline float tanh_fast(float x) {
        float x2 = x * x;
        return x * (27.0f + x2) / (27.0f + 9.0f * x2);
    }

    // expf
    inline float fast_exp(float x) {
        x = fmaxf(-60.0f, fminf(60.0f, x));
        float x2 = x * x;
        return 1.0f + x + x2 * 0.5f + x * x2 * (1.0f/6.0f) + x2 * x2 * (1.0f/24.0f);
    }

    // smooth gain compensation for drive
    inline float postSaturate(float x, const float driveVal) {
        return x / (driveVal + std::fabs(x));
    }

    // last instance dc blocker
    inline float dc_block(float x) {
        float y = x - dc_z + 0.995f * dc_y;
        dc_z = 1e-15f + x - 1e-15f;
        dc_y = 1e-15f + y - 1e-15f;
        return y;
    }

    inline float offDist(float x, const float, const float) {
        return x;
    }

    inline float softDist(float x, const float driveVal, const float amountVal) {
        if (fabsf(x) < 1e-20f) return 0.0f;
        x *= driveVal;
        x = tanh_fast(x * driveVal);
        x = 1e-15f + x - 1e-15f;
        return x * amountVal;
    }

    inline float crunchDist(float x, const float driveVal, const float amountVal) {
        if (fabsf(x) < 1e-20f) return 0.0f;
        x *= driveVal;
        float s = copysignf(1.f, x);
        x = s * log1p(mu * fabsf(x)) / log1p(mu);
        x = std::round(x / q) * q;
        x = tanh_fast(x * driveVal);
        x *= amountVal;
        return postSaturate(x, driveVal);
    }

    inline float rockDist(float x, const float driveVal, const float amountVal) {
        if (fabsf(x) < 1e-20f) return 0.0f;
        x *= driveVal * 12.0f;
        x = std::clamp(x, -1.5f, 1.5f);
        x = x - (x * x * x) * 0.2f;
        x = std::clamp(x, -0.9f, 0.9f);
        float s = copysignf(1.0f, x);
        float a = (s > 0.0f) ? 2.0f : 1.5f;
        float t = -fabsf(x) * a;
        x = s * (1.0f - fast_exp(t));
        x = 1e-15f + x - 1e-15f;
        return dc_block(x * amountVal);
    }

    // compander, distortion and saturation
    inline float processSample(float x, const float driveVal, const float amountVal, const int distModeVal) {
        if (fabsf(x) < 1e-20f) return 0.0f;
        switch (distModeVal)
        {
            case (SOFT)   : return softDist(x, driveVal, amountVal);
            break;
            case (CRUNCH) : return crunchDist(x, driveVal, amountVal);
            break;
            case (ROCK)  : return rockDist(x, driveVal, amountVal);
            break;
        }
        return x;
    }
};
