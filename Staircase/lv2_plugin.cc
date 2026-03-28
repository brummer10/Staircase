/*
 *                           0BSD 
 * 
 *                    BSD Zero Clause License
 * 
 *  Copyright (c) 2019 Hermann Meyer
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.

 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 */


#include "lv2_plugin.h"

/*---------------------------------------------------------------------
-----------------------------------------------------------------------    
                the main LV2 handle->XWindow
-----------------------------------------------------------------------
----------------------------------------------------------------------*/

#ifdef USE_MIDI
void messenger_init(MidiMessenger *mm) {
    int i = 0;
    for (; i < 25; i++) {
        mm->send_cc[i] &= ~_FULL;
        mm->send_cc[i] |= _EMPTY;
    }
}

int next(MidiMessenger *mm, int i) {
    while (++i < 25) {
        if (mm->send_cc[i] & _FULL) {
            return i;
        }
    }
    return -1;
}

void fill(MidiMessenger *mm, uint8_t *midi_send, int i) {
    if (mm->me_num[i] == 3) {
        midi_send[2] =  mm->bg_num[i];
    }
    midi_send[1] = mm->pg_num[i];    // program value
    midi_send[0] = mm->cc_num[i];    // controller+ channel
    mm->send_cc[i] &= ~_FULL;
    mm->send_cc[i] |= _EMPTY;
}

bool send_midi_cc(MidiMessenger *mm, uint8_t _cc, const uint8_t _pg,
                            const uint8_t _bgn, const uint8_t _num) {

    for(int i = 0; i < 25; i++) {
        if (mm->send_cc[i] & _FULL) {
            if (mm->cc_num[i] == _cc && mm->pg_num[i] == _pg &&
                mm->bg_num[i] == _bgn && mm->me_num[i] == _num)
                return true;
        } else if (mm->send_cc[i] & _EMPTY) {
            mm->cc_num[i] = _cc;
            mm->pg_num[i] = _pg;
            mm->bg_num[i] = _bgn;
            mm->me_num[i] = _num;
            mm->send_cc[i] &= ~_EMPTY;
            mm->send_cc[i] |=_FULL;
            return true;
        }
    }
    return false;
}
#endif

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
    if (freq < f_min) freq = f_min;
    if (freq > f_max) freq = f_max;

    float norm = log10f(freq / f_min) / log10f(f_max / f_min);
    return  x_pad + norm * (width - 2.0f * x_pad);
}

static inline float display_tilt(float freq) {
    if (freq >= 1000.0f)
        return 0.0f;

    float t = log10f(freq / 1000.0f);
    return 20.0f * t;
}

static void draw_window(void *w_, void* user_data) {
    Widget_t *w = (Widget_t*)w_;
    X11_UI* ui = (X11_UI*)w->parent_struct;
    const float* mags = fft_analyzer_get_magnitudes(ui->ana);
    int bins = fft_analyzer_get_bins(ui->ana);
    float sample_rate =  ui->uiKnowSampleRate ? ui->uiSampleRate : 48000.0;
    int fft_size = 2048;
    cairo_t* cr = w->crb;

    int width  = w->width;
    int height = w->height;

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
    float db_lines[] = {-60, -40, -24, -12, 0};
    int num_db = sizeof(db_lines) / sizeof(db_lines[0]);

    for (int i = 0; i < num_db; ++i) {
        float y = db_to_y(db_lines[i], db_min, db_max, height);
        cairo_move_to(cr, 0, y);
        cairo_line_to(cr, width, y);
    }
    cairo_stroke(cr);

    // Spectrum Line
    cairo_set_source_rgb(cr, 0.2, 0.8, 0.3);
    cairo_set_line_width(cr, 2.0);

    int started = 0;

    for (int i = 1; i < bins; ++i) {

        float freq = (float)i * sample_rate / fft_size;

        if (freq < f_min || freq > f_max)
            continue;

        float x = freq_to_x(freq, f_min, f_max, width);
        float db = mags[i] + display_tilt(freq);
        float y = db_to_y(db, db_min, db_max, height);

        if (!started) {
            float x0 = freq_to_x(f_min, f_min, f_max, width);
            float db0 = mags[1] + display_tilt(f_min);
            float y0 = db_to_y(db0, db_min, db_max, height);
            cairo_move_to(cr, x0, y0);
            started = 1;
        } else {
            cairo_line_to(cr, x, y);
        }
    }
    cairo_stroke(cr);

    // Filled Area
    cairo_set_source_rgba(cr, 0.2, 0.8, 0.3, 0.15);

    started = 0;

    for (int i = 1; i < bins; ++i) {

        float freq = (float)i * sample_rate / fft_size;

        if (freq < f_min || freq > f_max)
            continue;

        float x = freq_to_x(freq, f_min, f_max, width);
        float db = mags[i] + display_tilt(freq);
        float y = db_to_y(db, db_min, db_max, height);

        if (!started) {
            float x0 = freq_to_x(f_min, f_min, f_max, width);
            cairo_move_to(cr, x0, height);
            cairo_line_to(cr, x0, y);
            started = 1;
        } else {
            cairo_line_to(cr, x, y);
        }
    }

    if (started) {
        cairo_line_to(cr, width, height);
        cairo_close_path(cr);
        cairo_fill(cr);
    }
}

// if controller value changed send notify to host
static void value_changed(void *w_, void* user_data) {
    Widget_t *w = (Widget_t*)w_;
    X11_UI* ui = (X11_UI*)w->parent_struct;
    float v = adj_get_value(w->adj);
    ui->write_function(ui->controller,w->data,sizeof(float),0,&v);
    plugin_value_changed(ui, w, (PortIndex)w->data);
}


Widget_t* add_lv2_knob(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_knob(p, label, x, y, width, height);
    w->flags = USE_TRANSPARENCY | FAST_REDRAW;
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}


Widget_t* add_lv2_combobox(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_combobox(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}


Widget_t* add_lv2_vmeter(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_vmeter(p, label, false, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    tooltip_set_text(w, label);
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_hmeter(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_hmeter(p, label, false, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    tooltip_set_text(w, label);
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_vslider(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_vslider(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_hslider(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_hslider(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_toggle_button(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_toggle_button(p, label, x, y, width, height);
    w->flags = USE_TRANSPARENCY | FAST_REDRAW;
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_image_toggle(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_switch_image_button(p, label, x, y, width, height);
    w->flags = USE_TRANSPARENCY | FAST_REDRAW;
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_button(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_button(p, label, x, y, width, height);
    w->flags = USE_TRANSPARENCY | FAST_REDRAW;
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_image_button(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_image_button(p, label, x, y, width, height);
    w->flags = USE_TRANSPARENCY | FAST_REDRAW;
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_valuedisplay(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_valuedisplay(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_label(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_label(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_frame(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_frame(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_image(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_image(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    return w;
}

Widget_t* add_lv2_waveview(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_waveview(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    w->func.value_changed_callback = value_changed;
    return w;
}

Widget_t* add_lv2_tabbox(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_tabbox(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    return w;
}

Widget_t* add_lv2_tab(Widget_t *w, Widget_t *p, PortIndex index, const char * label, X11_UI* ui) {
    w = tabbox_add_tab(p, label);
    w->parent_struct = ui;
    w->data = index;
    return w;
}

Widget_t* add_lv2_file_button(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_file_button(p, x, y, width, height, "", "");
    w->data = index;
    return w;
}

Widget_t* add_lv2_midikeyboard(Widget_t *w, Widget_t *p, PortIndex index, const char * label,
                                X11_UI* ui, int x, int y, int width, int height) {
    w = add_midi_keyboard(p, label, x, y, width, height);
    w->parent_struct = ui;
    w->data = index;
    return w;
}

void load_bg_image(X11_UI* ui, const char* image) {
    cairo_surface_t *getpng = cairo_image_surface_create_from_png (image);
    int width = cairo_image_surface_get_width(getpng);
    int height = cairo_image_surface_get_height(getpng);
    int width_t = ui->win->scale.init_width;
    int height_t = ui->win->scale.init_height;
    double x = (double)width_t/(double)width;
    double y = (double)height_t/(double)height;
    cairo_surface_destroy(ui->win->image);
    ui->win->image = NULL;

    ui->win->image = cairo_surface_create_similar (ui->win->surface, 
                        CAIRO_CONTENT_COLOR_ALPHA, width_t, height_t);
    cairo_t *cri = cairo_create (ui->win->image);
    cairo_scale(cri, x,y);    
    cairo_set_source_surface (cri, getpng,0,0);
    cairo_paint (cri);
    cairo_surface_destroy(getpng);
    cairo_destroy(cri);
}

void load_controller_image(Widget_t* w, const char* image) {
    cairo_surface_t *getpng = cairo_image_surface_create_from_png (image);
    int width = cairo_image_surface_get_width(getpng);
    int height = cairo_image_surface_get_height(getpng);
    cairo_surface_destroy(w->image);
    w->image = NULL;

    w->image = cairo_surface_create_similar (w->surface, 
                        CAIRO_CONTENT_COLOR_ALPHA, width, height);
    cairo_t *cri = cairo_create (w->image);
    cairo_set_source_surface (cri, getpng,0,0);
    cairo_paint (cri);
    cairo_surface_destroy(getpng);
    cairo_destroy(cri);
}

// init the xwindow and return the LV2UI handle
static LV2UI_Handle instantiate(const LV2UI_Descriptor * descriptor,
            const char * plugin_uri, const char * bundle_path,
            LV2UI_Write_Function write_function,
            LV2UI_Controller controller, LV2UI_Widget * widget,
            const LV2_Feature * const * features) {

    X11_UI* ui = (X11_UI*)malloc(sizeof(X11_UI));

    if (!ui) {
        fprintf(stderr,"ERROR: failed to instantiate plugin with URI %s\n", plugin_uri);
        return NULL;
    }

    ui->parentXwindow = 0;
    ui->private_ptr = NULL;
    ui->need_resize = 1;
    LV2_Options_Option *opts = NULL;

    int i = 0;
    for(;i<CONTROLS;i++)
        ui->widget[i] = NULL;
    i = 0;
    for(;i<GUI_ELEMENTS;i++)
        ui->elem[i] = NULL;

    i = 0;
    for (; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_UI__parent)) {
            ui->parentXwindow = features[i]->data;
        } else if (!strcmp(features[i]->URI, LV2_UI__resize)) {
            ui->resize = (LV2UI_Resize*)features[i]->data;
        } else if(!strcmp(features[i]->URI, LV2_OPTIONS__options)) {
            opts = (LV2_Options_Option*)features[i]->data;
        } else if (!strcmp(features[i]->URI, LV2_URID_URI "#map")) {
            ui->map = (LV2_URID_Map*)features[i]->data;
        }
    }
    if (opts != NULL) {
        const LV2_URID atom_Float = ui->map->map(ui->map->handle, LV2_ATOM__Float);
        const LV2_URID ui_sampleRate = ui->map->map(ui->map->handle, LV2_PARAMETERS__sampleRate);
        for (const LV2_Options_Option* o = opts; o->key; ++o) {
            if (o->context == LV2_OPTIONS_INSTANCE &&
              o->key == ui_sampleRate && o->type == atom_Float) {
                ui->uiKnowSampleRate = true;
                ui->uiSampleRate = *(float*)o->value;
                //fprintf(stderr, "SampleRate = %iHz\n",(int)*(float*)o->value);
            }
        }
    }
 
    if (ui->parentXwindow == NULL)  {
        fprintf(stderr, "ERROR: Failed to open parentXwindow for %s\n", plugin_uri);
        free(ui);
        return NULL;
    }

#ifdef USE_MIDI
    lv2_atom_forge_init(&ui->forge,ui->map);
    ui->atom_eventTransfer  = ui->map->map(ui->map->handle, LV2_ATOM__eventTransfer);
    ui->midi_MidiEvent = ui->map->map(ui->map->handle, LV2_MIDI__MidiEvent);
    ui->midiatom.type = ui->midi_MidiEvent;
    ui->midiatom.size = 3;
    messenger_init(&ui->mm);
#endif
    map_osclv2_uris(ui->map, &ui->uris);
    lv2_atom_forge_init(&ui->forge, ui->map);

    // init Xputty
    main_init(&ui->main);
    int w = 1;
    int h = 1;
    plugin_set_window_size(&w,&h,plugin_uri);
    // create the toplevel Window on the parentXwindow provided by the host
    ui->win = create_window(&ui->main, (Window)ui->parentXwindow, 0, 0, w, h);
    ui->win->parent_struct = ui;
#ifdef __linux__
    ui->win->flags |= DONT_PROPAGATE;
#endif
    ui->win->label = plugin_set_name();
    // connect the expose func
    ui->win->func.expose_callback = draw_window;
    // create controller widgets
    plugin_create_controller_widgets(ui,plugin_uri);
    // map all widgets into the toplevel Widget_t
    widget_show_all(ui->win);
    // set the widget pointer to the X11 Window from the toplevel Widget_t
    *widget = (void*)ui->win->widget;
    // request to resize the parentXwindow to the size of the toplevel Widget_t
    if (ui->resize){
        ui->resize->ui_resize(ui->resize->handle, w, h);
    }
    // store pointer to the host controller
    ui->controller = controller;
    // store pointer to the host write function
    ui->write_function = write_function;
    
    return (LV2UI_Handle)ui;
}

// cleanup after usage
static void cleanup(LV2UI_Handle handle) {
    X11_UI* ui = (X11_UI*)handle;
    plugin_cleanup(ui);
    // Xputty free all memory used
    main_quit(&ui->main);
    free(ui->private_ptr);
    free(ui);
}

static void null_callback(void *w_, void* user_data) {
    
}

/*---------------------------------------------------------------------
-----------------------------------------------------------------------    
                        LV2 interface
-----------------------------------------------------------------------
----------------------------------------------------------------------*/

// port value change message from host
static void port_event(LV2UI_Handle handle, uint32_t port_index,
                        uint32_t buffer_size, uint32_t format,
                        const void * buffer) {
    X11_UI* ui = (X11_UI*)handle;
    float value = *(float*)buffer;
    int i=0;
    for (;i<CONTROLS;i++) {
        if (ui->widget[i] && port_index == (uint32_t)ui->widget[i]->data) {
            // prevent event loop between host and plugin
            xevfunc store = ui->widget[i]->func.value_changed_callback;
            ui->widget[i]->func.value_changed_callback = null_callback;
            // Xputty check if the new value differs from the old one
            // and set new one, when needed
            adj_set_value(ui->widget[i]->adj, value);
            // activate value_change_callback back
            ui->widget[i]->func.value_changed_callback = store;
        }
   }
   plugin_port_event(handle, port_index, buffer_size, format, buffer);
}

#ifdef USE_MIDI
// send midi data to the midi output port 
void send_midi_data(X11_UI* ui, uint8_t controller,
                             uint8_t note, uint8_t velocity) {

    uint8_t obj_buf[OBJ_BUF_SIZE];
    uint8_t vec[3];
    vec[0] = controller;
    vec[1] = note;
    vec[2] = velocity; 
    lv2_atom_forge_set_buffer(&ui->forge, obj_buf, OBJ_BUF_SIZE);

    lv2_atom_forge_frame_time(&ui->forge,0);
    LV2_Atom* msg = (LV2_Atom*)lv2_atom_forge_raw(&ui->forge,&ui->midiatom,sizeof(LV2_Atom));
    lv2_atom_forge_raw(&ui->forge,vec, sizeof(vec));
    lv2_atom_forge_pad(&ui->forge,sizeof(vec)+sizeof(LV2_Atom));

    ui->write_function(ui->controller, ui->midi_port, lv2_atom_total_size(msg),
                       ui->atom_eventTransfer, msg);
}
#endif

// LV2 idle interface to host
static int ui_idle(LV2UI_Handle handle) {
    X11_UI* ui = (X11_UI*)handle;
    if (ui->need_resize == 1) {
        ui->need_resize = 2;
    } else if (ui->need_resize == 2) {
        int i=0;
        for (;i<CONTROLS;i++) {
            os_move_window(ui->main.dpy, ui->widget[i], ui->widget[i]->x, ui->widget[i]->y);
        }
        ui->need_resize = 0;
    }
    // Xputty event loop setup to run one cycle when called
    run_embedded(&ui->main);
#ifdef USE_MIDI
    int i = next(&ui->mm, -1);
    uint8_t data[3] = {0};
    while (i >= 0) {
        fill(&ui->mm, data, i);
        send_midi_data(ui, data[0], data[1], data[2]);
        i = next(&ui->mm, i);
    }
#endif
    return 0;
}

// LV2 resize interface to host
static int ui_resize(LV2UI_Feature_Handle handle, int w, int h) {
    X11_UI* ui = (X11_UI*)handle;
    // Xputty sends configure event to the toplevel widget to resize itself
    if (ui) send_configure_event(ui->win,0, 0, w, h);
    return 0;
}

// connect idle and resize functions to host
static const void* extension_data(const char* uri) {
    static const LV2UI_Idle_Interface idle = { ui_idle };
    static const LV2UI_Resize resize = { 0 ,ui_resize };
    if (!strcmp(uri, LV2_UI__idleInterface)) {
        return &idle;
    }
    if (!strcmp(uri, LV2_UI__resize)) {
        return &resize;
    }
    return NULL;
}

static const LV2UI_Descriptor descriptors[] = {
    {PLUGIN_UI_URI,instantiate,cleanup,port_event,extension_data},
#ifdef PLUGIN_UI_URI2
    {PLUGIN_UI_URI2,instantiate,cleanup,port_event,extension_data},
#endif
};

LV2_SYMBOL_EXPORT
const LV2UI_Descriptor* lv2ui_descriptor(uint32_t index) {
    if (index >= sizeof(descriptors) / sizeof(descriptors[0])) {
        return NULL;
    }
    return descriptors + index;
}

