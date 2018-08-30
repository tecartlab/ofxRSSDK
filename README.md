# RealSense SDK addon for openframeworks 0.9.x
__Under Construction__

Currently supported:
* RGB Streaming
* Depth Streaming (Raw Depth and Depth as Color)
* Point Cloud

__Cameras__

Tested:
    Intel® RealSense™ Depth Cameras D435

Untested:
    Intel® RealSense™ Depth Cameras D415
    Intel® RealSense™ Depth Modules D400, D410, D420, D430
    Intel® RealSense™ Vision Processor D4m
    Intel® RealSense™ Tracking Module (limited support)

__Supported Platforms__

Tested:
    Windows 10 (Build 1803 or later)

Untested:
        Ubuntu 16.04/18.04 LTS (Linux Kernels 4.4, 4.8 ,4.10, 4.13 and 4.15)
        Windows 8.1 *
        Mac OS* (High Sierra 10.13.2)

****hardware frame synchronization is not available for the D400 series

__Dependecies__

Intel® RealSense™ SDK 2.0 (build 2.16.0) https://github.com/IntelRealSense/librealsense

__credits__

Martin Froehlich

this addon is based on https://github.com/SethGibson/ofxRSSDK, however it has been heavily altered on order to make it compatible with the current RSSDK.
