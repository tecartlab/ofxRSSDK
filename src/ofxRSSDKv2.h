#ifndef __OFX_RSSDK_H__
#define __OFX_RSSDK_H__
#ifdef _DEBUG
#pragma comment(lib, "realsense2.lib")
#else
#pragma comment(lib, "realsense2.lib")
#endif
#include <memory>
#include "ofMain.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API 
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rsutil.h> // Include RealSense Cross Platform API 
// configure in windows -> project properties -> C/C++ -> General -> additional include directories -> SDK include folder

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

#include "Helper.cpp"

#ifndef STBIW_MALLOC
#define STBIW_MALLOC(sz)        malloc(sz)
#define STBIW_REALLOC(p,newsz)  realloc(p,newsz)
#define STBIW_FREE(p)           free(p)
#endif

#ifndef STBIW_REALLOC_SIZED
#define STBIW_REALLOC_SIZED(p,oldsz,newsz) STBIW_REALLOC(p,newsz)
#endif


#ifndef STBIW_MEMMOVE
#define STBIW_MEMMOVE(a,b,sz) memmove(a,b,sz)
#endif


#ifndef STBIW_ASSERT
#include <assert.h>
#define STBIW_ASSERT(x) assert(x)
#endif

#define STBIW_UCHAR(x) (unsigned char) ((x) & 0xff)

using namespace std;

namespace ofxRSSDK
{
	class RSDevice;
	typedef unique_ptr<RSDevice> RSDevicePtr;
	typedef shared_ptr<RSDevice> RSDeviceRef;

	enum DepthRes
	{
		R200_SD,	// 480x360
		R200_VGA,	// 628x468
		F200_VGA,	// 640x480 
		QVGA		// 320x240
	};

	enum RGBRes
	{
		VGA,
		HD720,
		HD1080
	};

	enum AlignMode
	{
		ALIGN_FRAME,
		ALIGN_UVS_ONLY
	};

	enum CloudRes
	{
		FULL_RES=1,
		HALF_RES=2,
		Q_RES=4
	};

	enum PointCloud
	{
		DEPTH = 0,
		VIDEO = 1,
		INFRALEFT = 2
	};

	enum CaptureMode
	{
		Capture = 0,
		Recording = 1,
		Playback = 2
	};

	enum FilterPersistency
	{
		DISABLED			= 0,	//Persistency filter is not activated and no hole filling occurs.
		VALID_IN_8_OF_8		= 1,	//Persistency activated if the pixel was valid in 8 out of the last 8 frames
		VALID_IN_2_OF_LAST_3 = 2,	//Activated if the pixel was valid in two out of the last 3 frames
		VALID_IN_2_OF_LAST_4 = 3,	//Activated if the pixel was valid in two out of the last 4 frames
		VALID_IN_2_OF_8		= 4,	//Activated if the pixel was valid in two out of the last 8 frames
		VALID_IN_1_OF_LAST_2 = 5,	//Activated if the pixel was valid in one of the last two frames
		VALID_IN_1_OF_LAST_5 = 6,	//Activated if the pixel was valid in one out of the last 5 frames
		VALID_IN_1_OF_LAST_8 = 7,	//Activated if the pixel was valid in one out of the last 8 frames
		INDEFINITLY			= 8		//Persistency will be imposed regardless of the stored history(most aggressive filtering)
	};

	class RSDevice
	{
	protected:
		RSDevice();
	public:
		~RSDevice();
		static RSDevicePtr createUniquePtr() { return RSDevicePtr(new RSDevice()); }
		static RSDeviceRef createSharedPtr() { return RSDeviceRef(new RSDevice()); }
		
		/**
		Checks if at least one device is connected. If none is connected it will pop up
		an alertbox until a device is connected...
		*/
		void checkConnectedDialog();

		void hardwareReset();

		void setPointCloudRange(float pMin, float pMax);

		/**
		Starts the device with these parameters
		@param _captureMode CaptureMode::xxx capture mode
		@param device serial
		@return false if no device is attached
		*/
		bool start(int _captureMode, const std::string &serial);
		/**
		Starts the first device it finds.
		@param _captureMode CaptureMode::xxx capture mode
		@return false if no device is attached
		*/
		bool start(int _captureMode);

		/**
		Starts the first device it finds and records the data
		@return false if no device is attached
		*/
		bool record();

		/**
		Starts the first device it find
		@return false if no device is attached
		*/
		bool capture();

		/**
		Starts the first device it find
		@return false if no device is attached
		*/
		bool playback();

		/**
		update point cloud. 
		@param color choose texture to update with
		*/
		bool update(int color); 

		bool stop();

		bool draw();
		bool drawVideoStream(const ofRectangle & rect); // draw the video stream
		bool drawDepthStream(const ofRectangle & rect); // draw the false color depth stream
		bool drawInfraLeftStream(const ofRectangle & rect); // draw the left infrared stream

		bool isRunning();

		public: ofParameter<string> param_recordingPath; //recording file path  (default= ofFilePath::getCurrentExeDir() + "data/pointRecording.bag")

		/**
		Global setting to enable/disable postprocessing
		This has to be set to true if any filter are meant to
		do their job....
		*/
		public: void usePostProcessing(bool const & enable);
		private: void usePostProcessing_p(bool & enable);
		public: ofParameter<bool> param_usePostProcessing; //post processing parameter for use with ofxGUI

		/** Uses the decimation filter
		Effectively reduces the depth scene complexity. 
		The filter run on kernel sizes [2x2] to [8x8] pixels. 
		For patches sized 2 and 3 the median depth value is selected. 
		For larger kernels, 4-8 pixels, the mean depth is used due to performance considerations.

		The image size is scaled down proportionally in both dimensions to preserve the aspect ratio. 
		Internally, the filter imposes 4-pixel block alignment for the output frame size width and height. 
		E.g. for input size (1280X720) and scale factor 3 the output size calculation is:

		[1280,720]/3 -> [426.6666667, 240] -> [428,240]
		*The padded rows/columns are zero-filled.

		After the resulted frame is produced, the frame intrinsic parameters 
		are recalculated to compensate for the resolution changes.
		The filter also provides some hole filling capability, 
		as the filter uses valid (non-zero) pixels only.		
		*/
		public: void filterDecimation(bool const & enable);
		private: void filterDecimation_p(bool & enable);
		public: ofParameter<bool> param_filterDecimation; //Uses the decimation filter parameter for use with ofxGUI

		/** setting of decimation filter
		@param magnitude The decimation linear scale factor in Discrete steps in [2-8] range
		@return void
		*/
		public: void filterDecimation_mag(int const & magnitude);
		private: void filterDecimation_mag_p(int & magnitude);
		public: ofParameter<int> param_filterDecimation_mag; //magnitudeof  decimation filter parameter for use with ofxGUI

		/** Uses the SpatialEdgePreserve filter
		*The implementation is based on paper (http://inf.ufrgs.br/%7Eeslgastal/DomainTransform/)
		by Eduardo S. L. Gastal and Manuel M. Oliveira.

		Key characteristics:

		1D edge-preserving spatial filter using high-order domain transform.
		Linear-time compute, not affected by the choice of parameters.

		The filter performs a series of 1D horizontal and vertical passes or iterations, 
		to enhance the smoothness of the reconstructed data.	

		NOTICE: disparity filter has be enabled to make this filter work!
		*/
		public: void filterSpatial(bool const & enable);
		private: void filterSpatial_p(bool & enable);
		public: ofParameter<bool> param_filterSpatial; //Uses the SpatialEdgePreserve filter parameter for use with ofxGUI

		/** setting the SpatialEdgePreserve filter
		@param magnitude: The decimation linear scale factor, Discrete steps in [2-5] (2)
		*/
		public: void filterSpatial_mag(int const & magnitude);
		private: void filterSpatial_mag_p(int & magnitude);
		public: ofParameter<int> param_filterSpatial_mag; //magnitude for SpatialEdgePreserve filter parameter for use with ofxGUI
		/** setting the SpatialEdgePreserve filter
		@param smoothAlpha: The Alpha factor, Range [0.25-1] (0.5)
		*/
		public:	void filterSpatial_smoothAlpha(float const & smoothAlpha);
		private: void filterSpatial_smoothAlpha_p(float & smoothAlpha);
		public: ofParameter<float> param_filterSpatial_smoothAlpha; //smoothAlpha for SpatialEdgePreserve filter parameter for use with ofxGUI
		/** setting the SpatialEdgePreserve filter
		@param smoothDelta: Step-size boundary, Discrete [1-50] (20)
		*/
		public: void filterSpatial_smoothDelta(int const & smoothDelta);
		private: void filterSpatial_smoothDelta_p(int & smoothDelta);
		public: ofParameter<int> param_filterSpatial_smoothDelta; //smoothDelta for SpatialEdgePreserve filter parameter for use with ofxGUI
		/** setting the SpatialEdgePreserve filter
		@param holeFilling: hole-filling mode, 0-5] range mapped to [none,2,4,8,16,unlimited] pixels. (0)
		*/
		public: void filterSpatial_holeFilling(int const & holeFilling);
		private: void filterSpatial_holeFilling_p(int & holeFilling);
		public: ofParameter<int> param_filterSpatial_holeFilling; //holeFilling for SpatialEdgePreserve filter parameter for use with ofxGUI

		/**
		The temporal filter is intended to improve the depth data persistency by 
		manipulating per-pixel values based on previous frames. The filter 
		performs a single pass on the data, adjusting the depth values while 
		also updating the tracking history. In cases where the pixel data is 
		missing or invalid the filter uses a user-defined persistency mode to 
		decide whether the missing value should be rectified with stored data. 
		
		Note that due to its reliance on historic data the filter may 
		introduce visible blurring/smearing artefacts, and therefore 
		is best-suited for static scenes.

		NOTICE: disparity filter has be enabled to make this filter work!
		*/
		public: void filterTemporal(bool const & enable);
		private: void filterTemporal_p(bool & enable);
		public: ofParameter<bool> param_filterTemporal; //Uses the temporal filter parameter for use with ofxGUI

		/** Setting temporal filter
		@param smoothAlpha: The Alpha factor in an exponential moving average | range [0-1] (0.4)
		*/
		public: void filterTemporal_smoothAlpha(float const & smoothAlpha);
		private: void filterTemporal_smoothAlpha_p(float & smoothAlpha);
		public: ofParameter<float> param_filterTemporal_smoothAlpha; //Setting smoothAlpha temporal filter parameter for use with ofxGUI
		/** Setting temporal filter
		@param smoothDelta: Step-size boundary | range [0-100] (20)
		*/
		public: void filterTemporal_smoothDelta(int const & smoothDelta);
		private: void filterTemporal_smoothDelta_p(int & smoothDelta);
		public: ofParameter<int> param_filterTemporal_smoothDelta; //Setting smoothDelta temporal filter parameter for use with ofxGUI
		/** Setting temporal filter
		@param persistency:				| range [0-8](3)
				A set of predefined rules (masks) that govern when missing pixels will be replace with the last valid value so that the data will remain persistent over time:
				Disabled - Persistency filter is not activated and no hole filling occurs.
				Valid in 8/8 - Persistency activated if the pixel was valid in 8 out of the last 8 frames
				Valid in 2/last 3 - Activated if the pixel was valid in two out of the last 3 frames
				Valid in 2/last 4 - Activated if the pixel was valid in two out of the last 4 frames
				Valid in 2/8 - Activated if the pixel was valid in two out of the last 8 frames
				Valid in 1/last 2 - Activated if the pixel was valid in one of the last two frames
				Valid in 1/last 5 - Activated if the pixel was valid in one out of the last 5 frames
				Valid in 1/last 8 - Activated if the pixel was valid in one out of the last 8 frames
				Persist Indefinitely - Persistency will be imposed regardless of the stored history (most aggressive filtering)

		*/
		public: void filterTemporal_persistency(int const & persistency);
		private: void filterTemporal_persistency_p(int & persistency);
		public: ofParameter<int> param_filterTemporal_persistency; //Setting persistency temporal filter parameter for use with ofxGUI

		/**
		This filter has to be enabled in order to have the temporal and spatial filter work
		*/
		public: void filterDisparities(bool const & enable);
		private: void filterDisparities_p(bool & enable);
		public: ofParameter<bool> param_filterDisparities; //Uses the disparity filters parameter for use with ofxGUI

		/**
		enable the laser projektor power
		*/
		public: void deviceLaser(bool const & enable);
		private: void deviceLaser_p(bool & enable);
		public: ofParameter<bool> param_deviceLaser; //device laser parameter for use with ofxGUI
		
		/**
		set the laser projektor power
		*/
		public: void deviceLaser_mag(float const & magnitude);
		private: void deviceLaser_mag_p(float & magnitude);
		public: ofParameter<float> param_deviceLaser_mag; //device laser parameter for use with ofxGUI

		/**
		set auto exposure
		*/
		public: void deviceAutoExposure(bool const & enable);
		private: void deviceAutoExposure_p(bool & enable);
		public: ofParameter<bool> param_deviceAutoExposure; //exposure parameter for use with ofxGUI

		/**
		set exposure value
		@param exposure in microseconds [20 - 166000]
		*/
		public: void deviceExposure_mag(int const & magnitude);
		private: void deviceExposure_mag_p(int & magnitude);
		public: ofParameter<int> param_deviceExposure_mag; //exposure parameter for use with ofxGUI
		
		/**
		frame que size
		Max number of frames you can hold at a given time. 
		Increasing this number will reduce frame drops but increase latency, and vice versa
		@param frame queue size  [0 - 32]
		*/
		public: void deviceFrameQueSize_mag(int const & magnitude);
		private: void deviceFrameQueSize_mag_p(int & magnitude);
		public: ofParameter<int> param_deviceFrameQueSize_mag; //exposure parameter for use with ofxGUI

		/**
		Gain
		UVC image gain
		@param gain min=16| max = 248
		*/
		public: void deviceGain_mag(int const & magnitude);
		private: void deviceGain_mag_p(int & magnitude);
		public: ofParameter<int> param_deviceGain_mag; //gain parameter for use with ofxGUI

		/**
		Current Asic Temperature (degree celsius)
		@return temperature min=-40| max = 125
		*/
		public: int get_deviceAsicTemperature();
		/**
		read only temperature parameter for display with ofxGUI
		this paramter will be updated with each call of the update() function
		*/
		public: ofParameter<string> param_deviceAsicTemparature;

		/**
		Current Projector Temperature (degree celsius)
		@return temperature min=-40| max = 125
		*/
		public: int get_deviceProjectorTemperature();
		/**
		read only temperature parameter for display with ofxGUI
		this paramter will be updated with each call of the update() function
		*/
		public: ofParameter<string> param_deviceProjectorTemparature;

		const ofPixels&	getVideoFrame(); 
		const ofPixels&	getDepthFrame();
		const ofPixels&	getInfraLeftFrame();

		ofMesh getPointCloud();
		vector<glm::vec3> & getPointCloudVertices();

		//Nomenclature Notes:
		//	"Space" denotes a 3d coordinate
		//	"Image" denotes an image space point ((0, width), (0,height), (image depth))
		//	"Coords" denotes texture space (U,V) coordinates
		//  "Frame" denotes a full Surface


		/**
		Get a local space point of the specified depth frame coordinate
		*/
		glm::vec3 getSpacePointFromDepthFrameCoord(glm::vec2 depthCoordinate);

		/**
		Get a local space point of the specified video frame coordinate
		*/
		glm::vec3 getSpacePointFromVideoFrameCoord(glm::vec2 videoCoordinate);

		/**
		Get a local space point of the specified infrared frame coordinate
		*/
		glm::vec3 getSpacePointFromInfraLeftFrameCoord(glm::vec2 infraCoordinate);

		/**
		Get the distance from the device to the specified depth coordinate
		*/
		float getSpaceDistanceFromDepthFrame(glm::vec2 depthCoordinate);

		/**
		Sets the device with these parameters
		(424 x 240, 480 x 270, 640 x 360, 640 x 400, 640 x 480, [848 x 480], 1280 x 720, 1280 x 800)
		NOTICE1: if the device has already started, you need to restart the device to make this settings active
		@param width of depth and infrared resolution
		@param hight of depth and infrared resolution
		*/
		void				setDepthSize(int width, int height);

		// returns the point cloud resolution after all postprocessing filters have been applied
		const glm::vec2&	getDepthSize() { return mDepthStreamSize;  } 
		const int			getDepthWidth() { return mDepthStreamSize.x;  }
		const int			getDepthHeight() { return mDepthStreamSize.y; }

		/**
		Sets the device with these parameters
		(320 x 180, 320 x 240, 424 x 240, 480 x 270, 640 x 360, 640 x 480, 848 x 480, 960 x 540, [1280 x 720], 1920 1080)
		NOTICE: if the device has already started, you need to restart the device to make this settings active
		@param width of video resolution
		@param hight of depth resolution 
		*/
		void				setVideoSize(int width, int height);

		// returns the video stream resolution
		const glm::vec2&	getVideoSize() { return mVideoStreamSize; }
		const int			getVideoWidth() { return mVideoStreamSize.x; }
		const int			getVideoHeight() { return mVideoStreamSize.y; }

		/**
		return the number of valid device attached
		*/
		int					countDevicesAttached(); 

		/**
		Get the SerialNumber of the chosen device.
		if index is set to -1, it will get the serial from the currently running device, 
		if no device is running it will attempt do get the serial from the first device it finds.
		NOTICE: if no device is attached, this method will wait until a device is attached.
		*/
		const std::string	getSerialNumber(int index); // get the serialnumber of the running device.

		/**
		Print the info of the currently running device or the first device detected
		*/
		void printDeviceInfo();

	private:
		float RSDevice::get_depth_scale(rs2::device dev);

		int depthWidth, depthHeight, videoWidth, videoHeight;

		bool
			mIsInit,
			mIsPaused,
			mIsRunning,
			mIsCapturing,
			mIsPlayback,
			mIsRecording;
		bool
			isUsingPostProcessing,
			isUsingFilterDec, 
			isUsingFilterSpat, 
			isUsingFilterTemp, 
			isUsingFilterDisparity;

		AlignMode		mAlignMode;
		CloudRes		mCloudRes;

		glm::vec2			mPointCloudRange;

		glm::vec2			mDepthStreamSize; // size of the depth stream after post processing (if applied)
		glm::vec2			mVideoStreamSize;
		glm::vec2			mInfraredStreamSize;

		ofPixels		mVideoFrame;
		ofPixels		mDepthFrame;
		ofPixels		mInfraLeftFrame;

		// Declare depth colorizer for pretty visualization of depth data
		rs2::colorizer rs2Color_map;

		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud rs2PointCloud;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		rs2::points rs2Points;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		std::shared_ptr<rs2::pipeline> rs2Pipe;

		// Profile
		rs2::pipeline_profile rs2PipeLineProfile;

		// Device
		rs2::device rs2Device;

		// current frame
		rs2::frameset rs2FrameSet;

		// current Depth frame
		rs2::frame rs2Depth;

		struct rs2_intrinsics rs2DepthIntrinsics;
		struct rs2_intrinsics rs2VideoIntrinsics;
		struct rs2_intrinsics rsInfraLeftIntrinsics;

		// Declare filters
		rs2::decimation_filter rs2Filter_dec;
		rs2::spatial_filter rs2Filter_spat;
		rs2::temporal_filter rs2Filter_temp;

		rs2::disparity_transform rs2Filter_DispIn;
		rs2::disparity_transform rs2Filter_DispOut;

		ofMesh mPointCloud;

		uint16_t				*mRawDepth;
	};
};
#endif
