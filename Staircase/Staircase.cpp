
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <cstring>
#include <unistd.h>

#include <lv2/core/lv2.h>
#include <lv2/state/state.h>
#include <lv2/worker/worker.h>
#include <lv2/atom/atom.h>
#include <lv2/options/options.h>

#include <lv2/atom/util.h>
#include <lv2/atom/forge.h>
#include <lv2/midi/midi.h>
#include <lv2/urid/urid.h>
#include <lv2/patch/patch.h>

#include "Staircase.h"
#include "StreamingResampler.h"
#include "ParallelThread.h"
#include "FFTAnalyzer.h"

///////////////////////// MACRO SUPPORT ////////////////////////////////

#define PLUGIN_URI "urn:brummer:staircase"

using std::min;
using std::max;

typedef int PortIndex;

typedef struct {
    LV2_URID atom_Object;
    LV2_URID atom_Float;
    LV2_URID atom_Vector;
    LV2_URID atom_URID;
    LV2_URID atom_eventTransfer;
} URIs;

static inline void map_osclv2_uris(LV2_URID_Map* map, URIs* uris) {
    uris->atom_Object             = map->map(map->handle, LV2_ATOM__Object);
    uris->atom_Float              = map->map(map->handle, LV2_ATOM__Float);
    uris->atom_Vector             = map->map(map->handle, LV2_ATOM__Vector);
    uris->atom_URID               = map->map(map->handle, LV2_ATOM__URID);
    uris->atom_eventTransfer      = map->map(map->handle, LV2_ATOM__eventTransfer);
}

////////////////////////////// PLUG-IN CLASS ///////////////////////////

namespace staircase {

class Xstaircase
{
private:
    LM_EII12 stair;
    StreamingResampler resUp;
    StreamingResampler resDown;
    ParallelThread     xrworker;
    FFTAnalyzer        ana;
    
    const LV2_Atom_Sequence* control;
    LV2_Atom_Sequence* notify;
    LV2_URID_Map* map;

    LV2_Atom_Forge forge;
    LV2_Atom_Sequence* notify_port;
    LV2_Atom_Forge_Frame notify_frame;
    URIs uris;
    float* input0;
    float* output0;
    float* abuffer;
    float bypass_;
    int frames;
    // bypass ramping
    bool needs_ramp_down;
    bool needs_ramp_up;
    float ramp_down;
    float ramp_up;
    float ramp_up_step;
    float ramp_down_step;
    bool bypassed;

    // private functions
    inline void run_dsp_(uint32_t n_samples);
    inline void connect_(uint32_t port,void* data);
    inline void init_dsp_(uint32_t rate);
    inline void connect_all__ports(uint32_t port, void* data);
    inline void activate_f();
    inline void clean_up();
    inline void deactivate_f();
    void analyse();
public:
    // LV2 Descriptor
    static const LV2_Descriptor descriptor;
    // static wrapper to private functions
    static void deactivate(LV2_Handle instance);
    static void cleanup(LV2_Handle instance);
    static void run(LV2_Handle instance, uint32_t n_samples);
    static void activate(LV2_Handle instance);
    static void connect_port(LV2_Handle instance, uint32_t port, void* data);
    static LV2_Handle instantiate(const LV2_Descriptor* descriptor,
                                double rate, const char* bundle_path,
                                const LV2_Feature* const* features);
    Xstaircase();
    ~Xstaircase();
};

// constructor
Xstaircase::Xstaircase() :
    xrworker(),
    input0(NULL),
    output0(NULL),
    abuffer(NULL),
    bypass_(2),
    needs_ramp_down(false),
    needs_ramp_up(false),
    bypassed(false) {
        xrworker.start();
    };

// destructor
Xstaircase::~Xstaircase() {
    xrworker.stop();
    delete[] abuffer;
};

///////////////////////// PRIVATE CLASS  FUNCTIONS /////////////////////

void Xstaircase::init_dsp_(uint32_t rate)
{
    abuffer = new float[8192];
    memset(abuffer, 0, 8192 * sizeof(float));
    ana.init(2048, (float)rate);
    // set values for internal ramping
    ramp_down_step = 32 * (256 * rate) / 48000; 
    ramp_up_step = ramp_down_step;
    ramp_down = ramp_down_step;
    ramp_up = 0.0;
    frames = 0;
    stair.setSampleRate(2*rate);
    resUp.setup(1, 8192, rate, 2*rate);
    resDown.setup(1, 8192, 2*rate, rate);

    xrworker.setThreadName("Worker");
    xrworker.set<Xstaircase, &Xstaircase::analyse>(this);
    xrworker.runProcess();
}

// connect the Ports used by the plug-in class
void Xstaircase::connect_(uint32_t port,void* data)
{
    switch ((PortIndex)port)
    {
        case 0:
            input0 = static_cast<float*>(data);
            break;
        case 1:
            output0 = static_cast<float*>(data);
            break;
        case 2:
            stair.onOff = static_cast<float*>(data);
            break;
        case 3:
            stair.lowcut = static_cast<float*>(data);
            break;
        case 4:
            stair.drive = static_cast<float*>(data);
            break;
        case 5:
            stair.amount = static_cast<float*>(data);
            break;
        case 6:
            stair.highcut = static_cast<float*>(data);
            break;
        case 7:
            notify = (LV2_Atom_Sequence*)data;
            break;
        default:
            break;
    }
}

void Xstaircase::activate_f()
{
    // allocate the internal DSP mem
}

void Xstaircase::clean_up()
{
    // delete the internal DSP mem
}

void Xstaircase::deactivate_f()
{
    // delete the internal DSP mem
}

void Xstaircase::analyse() {
    if (!frames) return;
    ana.processBlock(abuffer, frames);
}

void Xstaircase::run_dsp_(uint32_t n_samples)
{
    if(n_samples<1) return;
    URIs* uris = &this->uris;
    const uint32_t notify_capacity = this->notify->atom.size;
    lv2_atom_forge_set_buffer(&forge, (uint8_t*)notify, notify_capacity);
    lv2_atom_forge_sequence_head(&forge, &notify_frame, 0);
    if (notify_capacity<n_samples) return;


    // do inplace processing on default
    if(output0 != input0)
        memcpy(output0, input0, n_samples*sizeof(float));

    float buf0[n_samples];
    // check if bypass is pressed
    if (bypass_ != static_cast<uint32_t>((*stair.onOff))) {
        bypass_ = static_cast<uint32_t>((*stair.onOff));
        if (!bypass_) {
            needs_ramp_down = true;
            needs_ramp_up = false;
        } else {
            needs_ramp_down = false;
            needs_ramp_up = true;
            bypassed = false;
        }
    }

    if (needs_ramp_down || needs_ramp_up) {
         memcpy(buf0, input0, n_samples*sizeof(float));
    }
    if (!bypassed) {
        int r = resUp.getOutSize(n_samples);
        float buf[r];
        resUp.resample(output0, buf, n_samples);
        stair.process(buf, r);
        resDown.resample(buf, output0, r);
    }

    // check if ramping is needed
    if (needs_ramp_down) {
        float fade = 0;
        for (uint32_t i=0; i<n_samples; i++) {
            if (ramp_down >= 0.0) {
                --ramp_down; 
            }
            fade = max(0.0f,ramp_down) /ramp_down_step ;
            output0[i] = output0[i] * fade + buf0[i] * (1.0 - fade);
        }
        if (ramp_down <= 0.0) {
            // when ramped down, clear buffer from dsp
            needs_ramp_down = false;
            bypassed = true;
            ramp_down = ramp_down_step;
            ramp_up = 0.0;
        } else {
            ramp_up = ramp_down;
        }
    } else if (needs_ramp_up) {
        float fade = 0;
        for (uint32_t i=0; i<n_samples; i++) {
            if (ramp_up < ramp_up_step) {
                ++ramp_up ;
            }
            fade = min(ramp_up_step,ramp_up) /ramp_up_step ;
            output0[i] = output0[i] * fade + buf0[i] * (1.0 - fade);
        }
        if (ramp_up >= ramp_up_step) {
            needs_ramp_up = false;
            ramp_up = 0.0;
            ramp_down = ramp_down_step;
        } else {
            ramp_down = ramp_up;
        }
    }
    memcpy(abuffer, output0, n_samples * sizeof(float));
    frames = n_samples;
    xrworker.runProcess();

    if (ana.hasNewData()) {
        LV2_Atom_Forge_Frame frame;
        lv2_atom_forge_frame_time(&this->forge, 0);
        lv2_atom_forge_object(&this->forge, &frame, 1, uris->atom_Float);
        lv2_atom_forge_property_head(&this->forge, uris->atom_Vector,0);
        lv2_atom_forge_vector(&this->forge, sizeof(float), uris->atom_Float, ana.getBins(), (void*)ana.getMagnitudes());
        lv2_atom_forge_pop(&this->forge, &frame);
        ana.clearFlag();
    }
}

void Xstaircase::connect_all__ports(uint32_t port, void* data)
{
    // connect the Ports used by the plug-in class
    connect_(port,data); 
}

////////////////////// STATIC CLASS  FUNCTIONS  ////////////////////////

LV2_Handle 
Xstaircase::instantiate(const LV2_Descriptor* descriptor,
                            double rate, const char* bundle_path,
                            const LV2_Feature* const* features)
{
    LV2_URID_Map* map = NULL;
    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_URID__map)) {
            map = (LV2_URID_Map*)features[i]->data;
        }
    }
    if (!map) {
        return NULL;
    }
    // init the plug-in class
    Xstaircase *self = new Xstaircase();
    if (!self) {
        return NULL;
    }

    map_osclv2_uris(map, &self->uris);
    lv2_atom_forge_init(&self->forge, map);

    self->map = map;

    self->init_dsp_((uint32_t)rate);
    return (LV2_Handle)self;
}

void Xstaircase::connect_port(LV2_Handle instance, 
                                   uint32_t port, void* data)
{
    // connect all ports
    static_cast<Xstaircase*>(instance)->connect_all__ports(port, data);
}

void Xstaircase::activate(LV2_Handle instance)
{
    // allocate needed mem
    static_cast<Xstaircase*>(instance)->activate_f();
}

void Xstaircase::run(LV2_Handle instance, uint32_t n_samples)
{
    // run dsp
    static_cast<Xstaircase*>(instance)->run_dsp_(n_samples);
}

void Xstaircase::deactivate(LV2_Handle instance)
{
    // free allocated mem
    static_cast<Xstaircase*>(instance)->deactivate_f();
}

void Xstaircase::cleanup(LV2_Handle instance)
{
    // well, clean up after us
    Xstaircase* self = static_cast<Xstaircase*>(instance);
    self->clean_up();
    delete self;
}

const LV2_Descriptor Xstaircase::descriptor =
{
    PLUGIN_URI ,
    Xstaircase::instantiate,
    Xstaircase::connect_port,
    Xstaircase::activate,
    Xstaircase::run,
    Xstaircase::deactivate,
    Xstaircase::cleanup,
    NULL
};

} // end namespace staircase

////////////////////////// LV2 SYMBOL EXPORT ///////////////////////////

LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
    switch (index)
    {
        case 0:
            return &staircase::Xstaircase::descriptor;
        default:
            return NULL;
    }
}

///////////////////////////// FIN //////////////////////////////////////
