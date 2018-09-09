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
* Intel® RealSense™ SDK 2.0 (build 2.16.0) https://github.com/IntelRealSense/librealsense
* Openframeworks release 0.10.0 [download page](http://openframeworks.cc/download).
* Openframeworks addon [ofxRSSDK](https://github.com/tecartlab/ofxRSSDK)
* Openframeworks addon [ofxGuiExtended](https://github.com/frauzufall/ofxGuiExtended)

### Instructions

drop this repositoriy into the \<openframeworksfolder>/apps/\<yourappfolder>

drop the addons into the \<openframeworksfolder>/addons/ folder

#### Visual Studio
Examles require to be linked to the installed RealSense SDK. All the examples assume the SDK is installed under C:\Program Files(x86)\Intel RealSense SDK 2.0

if otherwise, change the following:

* Menu > Project > Properties > C/C++ > General > Additional Include directories > (Edit...) > (RSSKD_Dir)\include
* Menu > Project > Properties > Linker > General > Additional Library directories > (Edit...) > (RSSKD_Dir)\lib\x64

## credits

Martin Froehlich

this addon is based on https://github.com/SethGibson/ofxRSSDK, however it has been heavily altered on order to make it compatible with the current RSSDK.
