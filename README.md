# RealSense SDK addon for openframeworks 0.10.x
## Under Construction

Currently supported:
* RGB Streaming
* Depth Streaming (Raw Depth and Depth as Color)
* Point Cloud

## Cameras

Tested:
* Intel® RealSense™ Depth Cameras D435

Untested:
* Intel® RealSense™ Depth Cameras D415
* Intel® RealSense™ Depth Modules D400, D410, D420, D430
* Intel® RealSense™ Vision Processor D4m
* Intel® RealSense™ Tracking Module (limited support)

## Supported Platforms

Tested:
* Windows 10 (Build 1803 or later)

Untested:
* Ubuntu 16.04/18.04 LTS (Linux Kernels 4.4, 4.8 ,4.10, 4.13 and 4.15)
* Windows 8.1 *
* Mac OS* (High Sierra 10.13.2)

****hardware frame synchronization is not available for the D400 series

### Dependecies

* Microsoft Visual Studio Community edition 2017 https://visualstudio.microsoft.com/de/downloads/
* Openframeworks release 0.10.0 [download page](http://openframeworks.cc/download).
* Openframeworks addon [ofxGuiExtended](https://github.com/maybites/ofxGuiExtended)(use my fork)

### Instructions

drop this repositoriy into the \<openframeworksfolder>/apps/\<yourappfolder>

drop the addons into the \<openframeworksfolder>/addons/ folder

#### Visual Studio
Examles have a realtive path to the RealSenseSDK 2.19.1 inside the ofxRSSDK/libs folder.

if otherwise, change the following:

* Menu > Project > Properties > C/C++ > General > Additional Include directories > (Edit...) > (RSSKD_Dir)\include
* Menu > Project > Properties > Linker > General > Additional Library directories > (Edit...) > (RSSKD_Dir)\lib\x64

## credits

Martin Froehlich

this addon is based on https://github.com/SethGibson/ofxRSSDK, however it has been heavily altered on order to make it compatible with the current RSSDK.

contains the relevant libraries/includes from Intel® RealSense™ SDK 2.0 (build 2.19.1) https://github.com/IntelRealSense/librealsense
