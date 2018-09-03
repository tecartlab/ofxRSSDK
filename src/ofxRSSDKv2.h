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

	private:

		bool			mIsInit,
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

		ofMesh mPointCloud;

		uint16_t				*mRawDepth;
	};
};
#endif
