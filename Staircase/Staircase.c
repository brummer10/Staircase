
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define HIDE_NAME 

#define CONTROLS 5

#define GUI_ELEMENTS 0

#define TAB_ELEMENTS 0


#define PLUGIN_UI_URI "urn:brummer:stair_ui"


#include "lv2_plugin.h"

#include "lv2_plugin.cc"


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
    ui->widget[0] = add_lv2_toggle_button (ui->widget[0], ui->win, 2, "Enable", ui, 40,  10, 60, 30);

    ui->widget[4] = add_lv2_knob (ui->widget[4], ui->win, 3, "LowCut", ui, 40,  49, 60, 80);
    set_adjustment(ui->widget[4]->adj, 220.0, 220.0, 20.0, 2200.0, 0.01, CL_LOGARITHMIC);

    ui->widget[1] = add_lv2_knob (ui->widget[1], ui->win, 4, "Drive", ui, 140,  49, 60, 80);
    set_adjustment(ui->widget[1]->adj, 1.2, 1.2, 0.1, 4.0, 0.01, CL_CONTINUOS);

    ui->widget[2] = add_lv2_knob (ui->widget[2], ui->win, 5, "Amount", ui, 240,  49, 60, 80);
    set_adjustment(ui->widget[2]->adj, 0.75, 0.75, 0.1, 1.0, 0.01, CL_CONTINUOS);

    ui->widget[3] = add_lv2_knob (ui->widget[3], ui->win, 6, "HighCut", ui, 340,  49, 60, 80);
    set_adjustment(ui->widget[3]->adj, 2100.0, 2100.0, 40.0, 22000.0, 0.01, CL_LOGARITHMIC);

}

void plugin_cleanup(X11_UI *ui) {
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
                    ui->asize = (vector_data->size - sizeof(LV2_Atom_Vector_Body)) / vec->atom.size;
                    const float* audio = (float*) LV2_ATOM_BODY(&vec->atom);
                    memcpy(ui->abuffer,audio, ui->asize * sizeof(float));
                    expose_widget(ui->win);
                }
            }
        }
    }
}

