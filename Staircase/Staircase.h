
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
        table1 = metalTable[kTableSize - 1];
        table0 = metalTable[0];
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
            for (int s = 0; s < hpStages; ++s) {
                x = onepole_hp(x, hp_state[s], ha);
            }
            output[i] = x;
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
            for (int s = 0; s < hpStages; ++s) {
                x = onepole_hp(x, hp_post_state[s], hb);
            }

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
    
    float table0;
    float table1;

    enum DistMode {
        OFF,
        SOFT,
        ROCK,
        METAL,
        CRUNCH
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
            case (METAL)   : this->dist = &LM_EII12::metalDist;
            break;
            default       : this->dist = &LM_EII12::offDist;
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

        float correctedFcb = fmaxf(12.0f, lowcutVal / (sqrtf((float)hpStages)* 0.65f));
        float hbwc = 2.0f * M_PI * correctedFcb;
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
        x *= driveVal * 2.0f;
        x = std::clamp(x, -1.5f, 1.5f);
        x = x - (x * x * x) * 0.2f;
        x = std::clamp(x, -0.9f, 0.75f);
        float s = copysignf(1.0f, x);
        float a = (s > 0.0f) ? 2.0f : 1.5f;
        float t = -fabsf(x) * a;
        x = s * (1.0f - fast_exp(t));
        x = tanh_fast(x * driveVal);
        x = 1e-15f + x - 1e-15f;
        return dc_block(x * amountVal);
    }

    inline  float metalDist(float x, const float driveVal, const float amountVal) {
        if (fabsf(x) < 1e-20f) return 0.0f;
        x *= driveVal;
        float scaled = 142.143f * std::fabs(x);
        int idx = int(scaled);
        float idx_f = float(idx);
        float y;
        if (idx < 0) {
            y = table0;
        } else if (idx > kTableSize - 2) {
            y = table1;
        } else {
            float frac = scaled - idx_f;
            y = metalTable[idx] * (1.0f - frac) + metalTable[idx + 1] * frac;
        }
        x = (x < 0.0f ? -1.0f : 1.0f) * std::fabs(y);
        return x * amountVal;
    }

private:
    static constexpr int kTableSize = 200;

    static constexpr float metalTable[kTableSize] = {
        0.000000000000,0.122079177170,0.184970061698,0.208502406061,0.228411122566,
        0.246627947982,0.263694032626,0.279424468753,0.293142395720,0.306227137551,
        0.318707890528,0.330612504688,0.341967545900,0.352798355078,0.363129104662,
        0.372982852498,0.382381593229,0.391346307322,0.399897007829,0.408052784996,
        0.415831848813,0.423251569602,0.430328516735,0.437078495565,0.443516582658,
        0.449657159393,0.455513944018,0.461100022233,0.466427876346,0.471509413108,
        0.476355990241,0.480978441763,0.485387102128,0.489591829270,0.493602026565,
        0.497426663801,0.501074297163,0.504553088312,0.507870822566,0.511034926268,
        0.514052483320,0.516930250995,0.519674674974,0.522291903739,0.524787802250,
        0.527167965047,0.529437728689,0.531602183681,0.533666185787,0.535634366889,
        0.537511145272,0.539300735514,0.541007157837,0.542634247111,0.544185661359,
        0.545664889952,0.547075261346,0.548419950540,0.549701986117,0.550924257029,
        0.552089518997,0.553200400700,0.554259409585,0.555268937500,0.556231265989,
        0.557148571410,0.558022929750,0.558856321294,0.559650634994,0.560407672724,
        0.561129153254,0.561816716121,0.562471925242,0.563096272444,0.563691180746,
        0.564258007570,0.564798047728,0.565312536351,0.565802651595,0.566269517313,
        0.566714205519,0.567137738813,0.567541092627,0.567925197433,0.568290940783,
        0.568639169319,0.568970690629,0.569286275083,0.569586657507,0.569872538867,
        0.570144587787,0.570403442080,0.570649710131,0.570883972292,0.571106782137,
        0.571318667731,0.571520132776,0.571711657762,0.571893701009,0.572066699722,
        0.572231070927,0.572387212441,0.572535503720,0.572676306738,0.572809966767,
        0.572936813170,0.573057160111,0.573171307281,0.573279540539,0.573382132578,
        0.573479343503,0.573571421438,0.573658603054,0.573741114124,0.573819169995,
        0.573892976099,0.573962728383,0.574028613770,0.574090810556,0.574149488822,
        0.574204810800,0.574256931250,0.574305997788,0.574352151231,0.574395525895,
        0.574436249912,0.574474445497,0.574510229237,0.574543712338,0.574575000884,
        0.574604196060,0.574631394395,0.574656687956,0.574680164577,0.574701908031,
        0.574721998237,0.574740511422,0.574757520306,0.574773094247,0.574787299413,
        0.574800198915,0.574811852960,0.574822318972,0.574831651735,0.574839903499,
        0.574847124112,0.574853361117,0.574858659872,0.574863063637,0.574866613682,
        0.574869349370,0.574871308255,0.574872526152,0.574873037231,0.574872874082,
        0.574872067796,0.574870648027,0.574868643065,0.574866079892,0.574862984251,
        0.574859380691,0.574855292637,0.574850742426,0.574845751373,0.574840339802,
        0.574834527108,0.574828331785,0.574821771479,0.574814863019,0.574807622461,
        0.574800065118,0.574792205599,0.574784057838,0.574775635128,0.574766950147,
        0.574758014992,0.574748841197,0.574739439769,0.574729821204,0.574719995518,
        0.574709972260,0.574699760544,0.574689369060,0.574678806100,0.574668079569,
        0.574657197015,0.574646165631,0.574634992283,0.574623683517,0.574612245580,
        0.574600684428,0.574589005747,0.574577214955,0.574565317225,0.574553317489,
        0.574541220453,0.574529030603,0.574516752223,0.574504389396,0.574491946017
    };

};
