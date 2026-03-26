
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <cstring>
#include <unistd.h>

#include <lv2/core/lv2.h>
#include "Staircase.h"
#include "StreamingResampler.h"

///////////////////////// MACRO SUPPORT ////////////////////////////////

#define PLUGIN_URI "urn:brummer:staircase"

using std::min;
using std::max;

typedef int PortIndex;

////////////////////////////// PLUG-IN CLASS ///////////////////////////

namespace staircase {

class Xstaircase
{
private:
    LM_EII12 stair;
    StreamingResampler resUp;
    StreamingResampler resDown;
    float* input0;
    float* output0;
    float bypass_;
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

    input0(NULL),
    output0(NULL),
    bypass_(2),
    needs_ramp_down(false),
    needs_ramp_up(false),
    bypassed(false) {};

// destructor
Xstaircase::~Xstaircase() {};

///////////////////////// PRIVATE CLASS  FUNCTIONS /////////////////////

void Xstaircase::init_dsp_(uint32_t rate)
{
    // set values for internal ramping
    ramp_down_step = 32 * (256 * rate) / 48000; 
    ramp_up_step = ramp_down_step;
    ramp_down = ramp_down_step;
    ramp_up = 0.0;
    stair.setSampleRate(2*rate);
    resUp.setup(1, 8192, rate, 2*rate);
    resDown.setup(1, 8192, 2*rate, rate);
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
            stair.drive = static_cast<float*>(data);
            break;
        case 4:
            stair.amount = static_cast<float*>(data);
            break;
        case 5:
            stair.cutoff = static_cast<float*>(data);
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

void Xstaircase::run_dsp_(uint32_t n_samples)
{
    if(n_samples<1) return;


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
    // init the plug-in class
    Xstaircase *self = new Xstaircase();
    if (!self) {
        return NULL;
    }
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
