# WM9M3 Path Tracer

C++ coursework project extending the course RTBase renderer. This repository
contains the program only; the coursework report is not included.

## Run

Download or clone the entire repository, then double-click `Run_Demo.cmd`.
Choose Cornell path tracing, MaterialsScene with guided denoising, Light
Tracing, or Instant Radiosity. Outputs are saved under `Outputs/`. The app
closes automatically after the requested sample count; the launcher displays
the exit code and output location. Demo settings are short previews, not the
report's equal-time benchmarks.

The launcher sets the working directory to `RTBase/`. Do not move the EXE
alone: its OIDN/TBB DLLs must remain beside it, and scene assets must be kept.

Requires Windows x64, an AVX2-capable CPU, and the Microsoft Visual C++ v14 x64
runtime. If MSVCP140 or VCRUNTIME140 is missing, install the Microsoft runtime.

## Build

Open `RTBase.sln` in Visual Studio with Desktop development with C++, MSVC
v143, and a Windows SDK. Select **Release x64** and build. Dependencies use
relative paths and are included; no additional download is needed. The output
is `x64/Release/RTBase.exe`. Win32 is not supported by the bundled OIDN build.

## Features and Controls

GGX reflection, ideal glass, SAH BVH, tile-based threading, latitude-longitude
environment sampling with MIS, first-hit AOVs, CPU OIDN denoising, Light
Tracing, and Instant Radiosity.

Escape quits. W/A/S/D moves the camera; Q/E moves down/up. P saves HDR; L saves
PNG. Input is handled between complete rendering passes. Console timings are
per-pass seconds, not total end-to-end time.

From `RTBase/`, for example:

```bat
..\x64\Release\RTBase.exe -scene cornell-box -integrator path -SPP 64 -threads 0 -outputFilename ..\cornell.hdr
```

`-integrator` accepts `path`, `light` or `ir`. `-SPP` means camera samples per
pixel for path/IR; for LT it means rounds of one emitted path per pixel.
`-threads 0` selects available processors. `-vplPaths 64` sets IR emitted paths.
Path tracing supports `-denoise off|color|guided|both` and `-aovPrefix`.
Raw path tracing saves HDR automatically; denoising also saves raw/filtered
PNG files. LT/IR save HDR and PNG; IR additionally saves VPL data.

## Scope and Limitations

LT/IR require area lights and do not support MaterialsScene environment
emission. LT does not cover ideal camera-side specular chains. IR uses fixed,
unclamped VPL sets, which can cause bright spots and dark bands. OrenNayar and
Plastic remain course placeholders. First-hit AOVs omit geometry behind glass.

`ThirdParty/oidn-2.5.1.x64.windows/` is a CPU-only subset of the existing OIDN
2.5.1 distribution: headers, import library, CPU DLLs and original notices.
GPU backends and SDK utilities are omitted. Licence notices remain in `doc/`
and `x64/Release/oidn-licenses/`; original course/source comments are preserved.
