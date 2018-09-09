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
// configure in windows -> project properties -> C/C++ -> General -> additional include directories -> SDK include folder

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams


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

	/**
	Class to encapsulate a filter alongside its options
	*/
	class Disparity
	{
	protected:
		Disparity();
	public:
		~Disparity();
		rs2::process_interface& filter_in;           //The filter in use
		rs2::process_interface& filter_out;           //The filter in use
	};

	class RSDevice
	{
	protected:
		RSDevice();
	public:
		~RSDevice();
		static RSDevicePtr createUniquePtr() { return RSDevicePtr(new RSDevice()); }
		static RSDeviceRef createSharedPtr() { return RSDeviceRef(new RSDevice()); }
		
		void enableAlignedImages(bool pState = true, AlignMode pMode = AlignMode::ALIGN_UVS_ONLY) { mShouldAlign = pState; mAlignMode = pMode; }
		void enablePointCloud(CloudRes pCloudRes, float pMinDepth, float pMaxDepth) { mCloudRes=pCloudRes; mShouldGetPointCloud=true; mPointCloudRange = ofVec2f(pMinDepth,pMaxDepth);}
		void setPointCloudRange(float pMin, float pMax);

		bool start();
		bool update();
		bool stop();

		bool draw();
		bool drawColor(const ofRectangle & rect);
		bool drawDepth(const ofRectangle & rect);

		bool isRunning();

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
		
		@param magnitude The decimation linear scale factor in Discrete steps in [2-8] range
		@return void
		*/
		void useFilterDecimation(int magnitude);
		
		/** Uses the SpatialEdgePreserve filter
		*The implementation is based on paper (http://inf.ufrgs.br/%7Eeslgastal/DomainTransform/)
		by Eduardo S. L. Gastal and Manuel M. Oliveira.

		Key characteristics:

		1D edge-preserving spatial filter using high-order domain transform.
		Linear-time compute, not affected by the choice of parameters.

		The filter performs a series of 1D horizontal and vertical passes or iterations, 
		to enhance the smoothness of the reconstructed data.		
		
		@param magnitude: The decimation linear scale factor, Discrete steps in [2-5] (2)
		@param smoothAlpha: The Alpha factor, Range [0.25-1] (0.5)
		@param smoothDelta: Step-size boundary, Discrete [1-50] (20)
		@param holeFilling: hole-filling mode, 0-5] range mapped to [none,2,4,8,16,unlimited] pixels. (0)
		@return void
		*/
		void useFilterSpatialEdgePreserve(int magnitude, float smoothAlpha, int smoothDelta, int holeFilling);

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

		@param smoothAlpha: The Alpha factor in an exponential moving average | range [0-1] (0.4)
		@param smoothDelta: Step-size boundary | range [0-100] (20)
		@param persitency:				| range [0-8](3)
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
		void useFilterTemporal(float smoothAlpha, int smoothDelta, int persitency);

		void useFilterDisparity();

		const ofPixels&	getRgbFrame();
		const ofPixels&	getDepthFrame();

		const ofPixels&	getColorMappedToDepthFrame();
		const ofPixels&	getDepthMappedToColorFrame();

		ofMesh getPointCloud();
		vector<glm::vec3> & getPointCloudVertices();

		//Nomenclature Notes:
		//	"Space" denotes a 3d coordinate
		//	"Image" denotes an image space point ((0, width), (0,height), (image depth))
		//	"Coords" denotes texture space (U,V) coordinates
		//  "Frame" denotes a full Surface

		//get a camera space point from a depth image point
		const ofPoint		getDepthSpacePoint(float pImageX, float pImageY, float pImageZ);
		const ofPoint		getDepthSpacePoint(int pImageX, int pImageY, uint16_t pImageZ);
		const ofPoint		getDepthSpacePoint(ofPoint pImageCoords);

		//get a ofColor object from a depth image point
		const ofColor		getColorFromDepthImage(float pImageX, float pImageY, float pImageZ);
		const ofColor		getColorFromDepthImage(int pImageX, int pImageY, uint16_t pImageZ);
		const ofColor		getColorFromDepthImage(ofPoint pImageCoords);

		//get a ofColor object from a depth camera space point
		const ofColor		getColorFromDepthSpace(float pCameraX, float pCameraY, float pCameraZ);
		const ofColor		getColorFromDepthSpace(ofPoint pCameraPoint);

		//get ofColor space UVs from a depth image point
		const glm::vec2		getColorCoordsFromDepthImage(float pImageX, float pImageY, float pImageZ);
		const glm::vec2		getColorCoordsFromDepthImage(int pImageX, int pImageY, uint16_t pImageZ);
		const glm::vec2		getColorCoordsFromDepthImage(ofPoint pImageCoords);

		//get ofColor space UVs from a depth space point
		const glm::vec2		getColorCoordsFromDepthSpace(float pCameraX, float pCameraY, float pCameraZ);
		const glm::vec2		getColorCoordsFromDepthSpace(ofPoint pCameraPoint);

		const glm::vec2&	getDepthSize() { return mDepthSize;  }
		const int		getDepthWidth() { return mDepthSize.x;  }
		const int		getDepthHeight() { return mDepthSize.y; }

		const glm::vec2&	getRgbSize() { return mRgbSize; }
		const int		getRgbWidth() { return mRgbSize.x; }
		const int		getRgbHeight() { return mRgbSize.y; }

		void			updatePointCloud();
		void			updatePointCloud(ofPixels colors);

		bool isUsingFilterDec, isUsingFilterSpat, isUsingFilterTemp, isUsingFilterDisparity;

	private:

		bool			
			mIsInit,
			mIsRunning,
			mHasRgb,
			mHasDepth,
			mShouldAlign,
			mShouldGetDepthAsColor,
			mShouldGetPointCloud,
			mHasChangedResolution;

		AlignMode		mAlignMode;
		CloudRes		mCloudRes;

		glm::vec2			mPointCloudRange;
		glm::vec2			mDepthSize;
		glm::vec2			mRgbSize;

		ofPixels		mRgbFrame;
		ofPixels		mDepthFrame;

		ofPixels		mColorToDepthFrame;
		ofPixels		mDepthToColorFrame;

		// Declare depth colorizer for pretty visualization of depth data
		rs2::colorizer rs2Color_map;

		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud rs2PointCloud;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		rs2::points rs2Points;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline rs2Pipe;

		// Profile
		rs2::pipeline_profile rs2PipeLineProfile;

		// current frame
		rs2::frameset rs2FrameSet;

		// current Depth frame
		rs2::frame rs2Depth;

		// Declare filters
		rs2::decimation_filter rs2Filter_dec;
		rs2::spatial_filter rs2Filter_spat;
		rs2::temporal_filter rs2Filter_temp;

		Disparity rs2Filter_disparity;

		ofMesh mPointCloud;

		uint16_t				*mRawDepth;
	};
};
#endif
