# Staircase

**Staircase** is a lightweight distortion plugin with integrated filtering and real-time spectrum visualization.
It is designed for efficient DSP performance and a clean, responsive UI.

<p align="center">
    <img src="https://github.com/brummer10/Staircase/blob/main/Staircase.png?raw=true" />
</p>

---

##  Features

*  Distortion with controllable drive and output
*  Integrated Lowcut / Highcut filtering
*  Real-time FFT spectrum display
*  Low CPU usage, optimized for realtime processing
*  Clean and minimal UI (libxputty / Cairo)

---

##  Controls

* **LowCut** – Removes low frequencies before distortion
* **Drive** – Controls distortion intensity
* **Amount** – Output level / wet amount
* **HighCut** – Smooths high frequencies after distortion
* **Enable** – Bypass processing

---

##  Technical Notes

* Filters are implemented as cascaded one-pole filters
  stable under fast parameter modulation
* Spectrum analyzer uses FFT-based magnitude display
* Logarithmic frequency scaling (20 Hz – 20 kHz)
* Designed to avoid denormals and minimize DSP load

---

##  Build

Requirements:

* libcairo2-dev
* libx11-dev
* libfftw3-dev
* liblv2-dev

```bash
 git submodule init
 git submodule update
 make
 make install # will install into ~/.lv2 ... AND/OR....
 sudo make install # will install into /usr/lib/lv2
```

---

##  Status

Work in progress – actively developed and optimized.

---

##  License

BSD-3-Clause

---

