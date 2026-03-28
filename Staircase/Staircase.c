
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define HIDE_NAME 

#define CONTROLS 4

#define GUI_ELEMENTS 0

#define TAB_ELEMENTS 0


#define PLUGIN_UI_URI "urn:brummer:stair_ui"


#include "lv2_plugin.h"

#include "lv2_plugin.cc"


static void build_hann(float* w, int N) {
    for (int i = 0; i < N; ++i) {
        w[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (N - 1)));
    }
}

static float compute_window_gain(const float* w, int N) {
    float sum = 0.0f;
    for (int i = 0; i < N; ++i)
        sum += w[i];
    return sum / (float)N;
}

FFTAnalyzer* fft_analyzer_create(int fft_size, float sample_rate) {

    FFTAnalyzer* a = (FFTAnalyzer*)calloc(1, sizeof(FFTAnalyzer));

    a->N = fft_size;
    a->bins = fft_size / 2;
    a->sample_rate = sample_rate;
    a->in     = (float*)fftwf_malloc(sizeof(float) * a->N);
    a->window = (float*)malloc(sizeof(float) * a->N);
    a->mags   = (float*)malloc(sizeof(float) * a->bins);
    a->out    = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (a->bins + 1));

    build_hann(a->window, a->N);

    float window_gain = compute_window_gain(a->window, a->N);
    a->norm_factor = (2.0f / (a->N * window_gain));
    a->plan = fftwf_plan_dft_r2c_1d(a->N, a->in, a->out, FFTW_MEASURE);
    a->fifo = (float*)calloc(a->N, sizeof(float));
    a->fifo_pos = 0;
    a->hop_size = a->N / 2;
    a->samples_since_last_fft = 0;
    a->smooth = (float*)malloc(sizeof(float) * a->bins);

    for (int i = 0; i < a->bins; ++i)
        a->smooth[i] = -90.0f;

    a->attack  = 0.6f;
    a->release = 0.05f;
    return a;
}

void fft_analyzer_destroy(FFTAnalyzer* a) {
    if (!a) return;
    fftwf_destroy_plan(a->plan);
    fftwf_free(a->in);
    fftwf_free(a->out);
    free(a->window);
    free(a->mags);
    free(a->fifo);
    free(a->smooth);
    free(a);
}

void fft_analyzer_process(FFTAnalyzer* a, const float* input, int n_samples) {
    for (int i = 0; i < n_samples; ++i) {

        float v = input[i];
        if (!isfinite(v)) v = 0.0f;
        a->fifo[a->fifo_pos++] = v;
        if (a->fifo_pos >= a->N)
            a->fifo_pos = 0;
        a->samples_since_last_fft++;
        if (a->samples_since_last_fft >= a->hop_size) {
            a->samples_since_last_fft = 0;
            int idx = a->fifo_pos;
            for (int j = 0; j < a->N; ++j) {
                float s = a->fifo[idx];
                a->in[j] = s * a->window[j];
                idx++;
                if (idx >= a->N)
                    idx = 0;
            }
            fftwf_execute(a->plan);
            for (int k = 0; k < a->bins; ++k) {
                float re = a->out[k][0];
                float im = a->out[k][1];
                float mag = sqrtf(re * re + im * im);
                mag *= a->norm_factor;
                if (!isfinite(mag) || mag <= 0.0f)
                    mag = 1e-20f;

                float db = 20.0f * log10f(mag);
                float prev = a->smooth[k];
                float t = (float)k / (float)a->bins;
                float release = a->release * (0.5f + t);
                float coeff = (db > prev) ? a->attack : release;
                float smoothed = prev + coeff * (db - prev);
                a->smooth[k] = smoothed;
                a->mags[k] = smoothed;
            }
            X11_UI* ui = (X11_UI*)a->private_ptr;
            expose_widget(ui->win);
            for (int i = 0; i < CONTROLS; i++) {
                expose_widget(ui->widget[i]);
            }
        }
    }
}

const float* fft_analyzer_get_magnitudes(FFTAnalyzer* a) {
    return a->mags;
}

int fft_analyzer_get_bins(FFTAnalyzer* a) {
    return a->bins;
}

void plugin_value_changed(X11_UI *ui, Widget_t *w, PortIndex index) {
    // do special stuff when needed
}

void plugin_set_window_size(int *w,int *h,const char * plugin_uri) {
    (*w) = 440; //set initial width of main window
    (*h) = 160; //set initial height of main window
}

const char* plugin_set_name() {
    return "Staircase"; //set plugin name to display on UI
}

void plugin_create_controller_widgets(X11_UI *ui, const char * plugin_uri) {
    float sr = ui->uiKnowSampleRate ? ui->uiSampleRate : 48000.0;
    ui->ana = fft_analyzer_create(2048, sr);
    ui->ana->private_ptr = ui;
    ui->widget[0] = add_lv2_toggle_button (ui->widget[0], ui->win, 2, "Enable", ui, 40,  49, 60, 60);

    ui->widget[1] = add_lv2_knob (ui->widget[1], ui->win, 3, "Drive", ui, 140,  49, 60, 80);
    set_adjustment(ui->widget[1]->adj, 1.2, 1.2, 0.1, 4.0, 0.01, CL_CONTINUOS);

    ui->widget[2] = add_lv2_knob (ui->widget[2], ui->win, 4, "Amount", ui, 240,  49, 60, 80);
    set_adjustment(ui->widget[2]->adj, 0.75, 0.75, 0.1, 1.0, 0.01, CL_CONTINUOS);

    ui->widget[3] = add_lv2_knob (ui->widget[3], ui->win, 5, "CutOff", ui, 340,  49, 60, 80);
    set_adjustment(ui->widget[3]->adj, 12000.0, 12000.0, 40.0, 22000.0, 0.01, CL_LOGARITHMIC);

}

void plugin_cleanup(X11_UI *ui) {
    fft_analyzer_destroy(ui->ana);
    // clean up used sources when needed
}


void plugin_port_event(LV2UI_Handle handle, uint32_t port_index,
                        uint32_t buffer_size, uint32_t format,
                        const void * buffer) {
    X11_UI* ui = (X11_UI*)handle;
    URIs* uris = &ui->uris;
    if (format == uris->atom_eventTransfer) {
        LV2_Atom* atom = (LV2_Atom*)buffer;
        if (atom->type == uris->atom_Object) {
            LV2_Atom_Object* obj      = (LV2_Atom_Object*)atom;
            if (obj->body.otype == uris->atom_Float) {
                const LV2_Atom* vector_data = NULL;
                const int n_props  = lv2_atom_object_get(obj,uris->atom_Vector, &vector_data, NULL);
                if (!n_props) return;
                const LV2_Atom_Vector* vec = (LV2_Atom_Vector*)LV2_ATOM_BODY(vector_data);
                if (vec->atom.type == uris->atom_Float) {
                    int n_elem = (vector_data->size - sizeof(LV2_Atom_Vector_Body)) / vec->atom.size;
                    float* audio;
                    audio = (float*) LV2_ATOM_BODY(&vec->atom);
                    fft_analyzer_process(ui->ana, audio, n_elem);
                }
            }
        }
    }
    // port value change message from host
    // do special stuff when needed
}

