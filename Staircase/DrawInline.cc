
/*
 * DrawInline.cc
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 *
 * Copyright (C) 2025 brummer <brummer@web.de>
 */


/****************************************************************
        DrawInline.cc - part of Staircase.cpp 
                        draw the LV2 Inline Display
****************************************************************/


static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float db_to_y(float db, float db_min, float db_max, int height) {
    float norm = (db - db_min) / (db_max - db_min);
    norm = clampf(norm, 0.0f, 1.0f);
    return (1.0f - norm) * height;
}

static float freq_to_x(float freq, float f_min, float f_max, int width) {
    const float x_pad = 3.0f;
    const float inv_log_range = 1.0f / log10f(f_max / f_min);
    if (freq < f_min) freq = f_min;
    if (freq > f_max) freq = f_max;
    float norm = log10f(freq / f_min) * inv_log_range;
    return  x_pad + norm * (width - 2.0f * x_pad);
}

static inline float display_tilt(float freq) {
    if (freq >= 1000.0f) return 0.0f;
    float t = log10f(freq / 1000.0f);
    return 20.0f * t;
}

static inline int slope_to_stages(int slope) {
    if (slope == 0)   return 1; // 6 dB
    if (slope == 1)   return 2; // 12 dB
    if (slope == 2)   return 4; // 24 dB
    return 6;                   // 36 dB
}

static inline float compute_corrected_k(float wc, float fs, int stages) {
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

static inline void one_pole_lp_complex(float freq, int stages, float cutoff, float fs,
                                    float* out_real, float* out_imag) {
    float wc = 2.0f * (float)M_PI * cutoff;
    float k = compute_corrected_k(wc, fs, stages);
    float a = k;
    float b = 1.0f - k;
    float omega = 2.0f * (float)M_PI * freq / fs;
    float denom_real = 1.0f - b * cosf(omega);
    float denom_imag =        b * sinf(omega);
    float denom_mag2 = denom_real*denom_real + denom_imag*denom_imag;
    *out_real =  a * denom_real / denom_mag2;
    *out_imag = -a * denom_imag / denom_mag2;
}

static inline void one_pole_hp_complex(float freq, float cutoff, float fs,
                                       float* out_real, float* out_imag) {
    float wc = 2.0f * (float)M_PI * cutoff;
    float k  = wc / (wc + fs);
    float b  = 1.0f - k;
    float omega = 2.0f * (float)M_PI * freq / fs;
    float cos_w = cosf(omega);
    float sin_w = sinf(omega);
    float num_real = 1.0f - cos_w;
    float num_imag =        sin_w;
    float denom_real = 1.0f - b * cos_w;
    float denom_imag =        b * sin_w;
    float denom_mag2 = denom_real * denom_real + denom_imag * denom_imag;
    *out_real = (num_real * denom_real + num_imag * denom_imag) / denom_mag2;
    *out_imag = (num_imag * denom_real - num_real * denom_imag) / denom_mag2;
}

static void draw_filter_overlay(cairo_t* cr, int width, int height,
            float sample_rate, const float f_min, const float f_max,
            const float db_min, const float db_max,
            float cutoff, int stages, int type) {

    cairo_set_line_width(cr, 2.0);
    int started_f = 0;

    for (int i = 0; i < width; ++i) {
        float norm = (float)i / (float)(width - 1);
        float freq = f_min * powf(f_max / f_min, norm);

        float mag = 1.0f;

        for (int s = 0; s < stages; ++s) {

            if (type == 1) {
                // Lowpass
                float lp_r, lp_i;
                one_pole_lp_complex(freq, stages, cutoff, sample_rate, &lp_r, &lp_i);
                float lp_mag = sqrtf(lp_r*lp_r + lp_i*lp_i);
                mag *= lp_mag;
                cairo_set_source_rgba(cr, 0.9, 0.6, 0.2, 0.65);
            } else {
                // Highpass
                float hp_r, hp_i;
                float correctedFc = cutoff / sqrtf((float)stages);
                one_pole_hp_complex(freq, correctedFc, sample_rate, &hp_r, &hp_i);
                float hp_mag = sqrtf(hp_r*hp_r + hp_i*hp_i);
                mag *= hp_mag;
                cairo_set_source_rgba(cr, 0.2, 0.6, 0.9, 0.65);
            }
        }

        float db = 20.0f * log10f(mag + 1e-20f);
        float y  = db_to_y(db, db_min, db_max, height);
        float x = freq_to_x(freq, f_min, f_max, width);

        if (!started_f) {
            cairo_move_to(cr, x, y);
            started_f = 1;
        } else {
            cairo_line_to(cr, x, y);
        }
    }

    cairo_stroke_preserve(cr);
    if (type == 1) {
        cairo_set_source_rgba(cr, 0.9, 0.6, 0.2, 0.07);
        cairo_line_to(cr, width, 0);
        cairo_line_to(cr,0, 0);
        cairo_close_path(cr);
        cairo_fill(cr);
    } else {
        cairo_set_source_rgba(cr, 0.2, 0.6, 0.9, 0.09);
        cairo_line_to(cr,0, 0);
        cairo_line_to(cr,0, height);
        cairo_close_path(cr);
        cairo_fill(cr);
    }

    // Cutoff Line
    float cx = freq_to_x(cutoff, f_min, f_max, width);

    if (type == 1)
        cairo_set_source_rgba(cr, 0.9, 0.6, 0.2, 0.5);
    else
        cairo_set_source_rgba(cr, 0.2, 0.6, 0.9, 0.5);

    cairo_set_line_width(cr, 1.0);
    cairo_move_to(cr, cx, 0);
    cairo_line_to(cr, cx, height);
    cairo_stroke(cr);
}

static void draw_inline(Xstaircase *self , cairo_t* cr) {

    const float* mags = self->getMagnitudes();
    int bins = self->getBins();
    float sample_rate =  self->sampleRate;
    int fft_size = bins * 2; // 2048;

    int width  = self->width;
    int height = self->height;

    const float f_min = 20.0f;
    const float f_max = 20000.0f;

    const float db_min = -75.0f;
    const float db_max = 0.0f;

    // Background
    cairo_set_source_rgb(cr, 0.08, 0.08, 0.08);
    cairo_rectangle(cr, 0, 0, width, height);
    cairo_fill(cr);

    // Grid: Frequencies
    cairo_set_source_rgba(cr, 1, 1, 1, 0.08);
    cairo_set_line_width(cr, 1.0);

    float freqs[] = {20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
    int num_freqs = sizeof(freqs) / sizeof(freqs[0]);

    for (int i = 0; i < num_freqs; ++i) {
        float x = freq_to_x(freqs[i], f_min, f_max, width);
        cairo_move_to(cr, x, 0);
        cairo_line_to(cr, x, height);
    }
    cairo_stroke(cr);

    // Grid: dB
    float db_lines[] = { -60, -48, -36, -24, -12, 0};
    int num_db = sizeof(db_lines) / sizeof(db_lines[0]);

    for (int i = 0; i < num_db; ++i) {
        float y = db_to_y(db_lines[i], db_min, db_max, height);
        cairo_move_to(cr, 0, y);
        cairo_line_to(cr, width, y);
    }
    cairo_stroke(cr);

    // LP + HP Filter overlay

    draw_filter_overlay(cr, width, height, sample_rate, f_min, f_max,
        db_min, db_max, self->highcut, slope_to_stages(self->lpslopes), 1);

    draw_filter_overlay(cr, width, height, sample_rate, f_min, f_max,
        db_min, db_max, self->lowcut, slope_to_stages(self->hpslopes), 0);

    // Spectrum Line
    cairo_pattern_t* lpat = cairo_pattern_create_linear(0, 0, 0, height);
    cairo_pattern_add_color_stop_rgba(lpat, 0.0, 0.75, 0.2, 0.9, 0.65);
    cairo_pattern_add_color_stop_rgba(lpat, 1.0, 0.45, 0.2, 0.75, 0.65);
    cairo_set_source(cr, lpat);
    cairo_set_line_width(cr, 1.0);
    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    int started = 0;

    for (int i = 1; i < bins; ++i) {

        float freq = (float)i * sample_rate / fft_size;

        if (freq < f_min || freq > f_max)
            continue;

        float x = freq_to_x(freq, f_min, f_max, width);
        float db = mags[i] ;//+ display_tilt(freq);
        float y = db_to_y(db, db_min, db_max, height);

        if (!started) {
            float x0 = freq_to_x(f_min, f_min, f_max, width);
            float db0 = mags[1] ;//+ display_tilt(f_min);
            float y0 = db_to_y(db0, db_min, db_max, height);
            cairo_move_to(cr, x0, y0);
            started = 1;
        } else {
            cairo_line_to(cr, x, y);
        }
    }
    cairo_stroke_preserve(cr);
    cairo_pattern_destroy(lpat);

    // Spectrum fill
    if (started) {
        //cairo_set_source_rgba(cr,  0.17, 0.82, 0.64, 0.15);
        cairo_line_to(cr, width, height);
        cairo_line_to(cr, 3, height);
        cairo_close_path(cr);
        cairo_pattern_t* pat = cairo_pattern_create_linear(0, 0, 0, height);
        cairo_pattern_add_color_stop_rgba(pat, 0.0, 0.75, 0.2, 0.9, 0.25);
        cairo_pattern_add_color_stop_rgba(pat, 1.0, 0.45, 0.2, 0.75, 0.05);
        cairo_set_source(cr, pat);
        cairo_fill(cr);
        cairo_pattern_destroy(pat);
    }
}
