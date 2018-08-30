#include <exception>
#include "ofxRSSDKv2.h"
#include "ofMain.h"

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

namespace ofxRSSDK
{
	RSDevice::~RSDevice(){}

	RSDevice::RSDevice(){
		mIsInit = false;
		mIsRunning = false;
		mHasRgb = false;
		mHasDepth = false;
		mShouldAlign = false;
		mShouldGetDepthAsColor = false;
		mShouldGetPointCloud = false;
		mPointCloudRange = ofVec2f(0,3000);
		mCloudRes = CloudRes::FULL_RES;
	}

#pragma region Init

#pragma endregion

	void RSDevice::setPointCloudRange(float pMin=100.0f, float pMax=1500.0f)
	{
		mPointCloudRange = ofVec2f(pMin,pMax);
	}

	bool RSDevice::start()
	{
		rs2PipeLineProfile = rs2Pipe.start();

		auto depth_stream = rs2PipeLineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		auto color_sream = rs2PipeLineProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

		mRgbSize.x = color_sream.width();
		mRgbSize.y = color_sream.height();
		mDepthSize.x = depth_stream.width();
		mDepthSize.y = depth_stream.height();

		/**
		auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
		auto i = depth_stream.get_intrinsics();
		auto principal_point = std::make_pair(i.ppx, i.ppy);
		auto focal_length = std::make_pair(i.fx, i.fy);
		rs2_distortion model = i.model;
		**/

			if (mShouldAlign)
			{
				mColorToDepthFrame.allocate(mRgbSize.x, mRgbSize.y, ofPixelFormat::OF_PIXELS_RGBA);
				mDepthToColorFrame.allocate(mRgbSize.x, mRgbSize.y, ofPixelFormat::OF_PIXELS_RGBA);
			}
			mIsRunning = true;
			return true;

		// Capture 30 frames to give autoexposure, etc. a chance to settle
		for (auto i = 0; i < 30; ++i) rs2Pipe.wait_for_frames();

	}

	bool RSDevice::update()
	{
		if (rs2Pipe.poll_for_frames(&rs2FrameSet))
		{
			auto rs2DepthFrame = rs2FrameSet.first(RS2_STREAM_DEPTH);
			// Generate the pointcloud and texture mapping	s
			rs2Points = rs2PointCloud.calculate(rs2DepthFrame);
			updatePointCloud();

			auto rs2VideoFrame = rs2FrameSet.first(RS2_STREAM_COLOR).as<rs2::video_frame>();

			// We can only save video frames as pngs, so we skip the rest
			if (rs2VideoFrame)
			{
				// Use the colorizer to get an rgb image for the depth stream
				if (rs2VideoFrame.is<rs2::depth_frame>()) rs2VideoFrame = rs2Color_map(rs2FrameSet);

				mRgbFrame.setFromExternalPixels((unsigned char *)rs2VideoFrame.get_data(), rs2VideoFrame.get_width(), rs2VideoFrame.get_height(), 3);

				//copyFrame(&rs2VideoFrame, &mRgbFrame);

				// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
//					if (!rs2VideoFrame)
//						rs2VideoFrame = rs2FrameSet.first(RS2_STREAM_INFRARED);

				cout << "copied video frame " << endl;
				//cout << " source h=" << rs2VideoFrame.get_height() << " w = " << rs2VideoFrame.get_width() << endl;
				//cout << " target h=" << mRgbFrame.getHeight() << " w = " << mRgbFrame.getWidth() << endl;

				

				//mRgbFrame.setFromExternalPixels(rs2VideoFrame.get_data(), mRgbSize.x, mRgbSize.y, 4);

			}
			else {
				cout << "no video frame frame" << endl;
			}


			return true;
		}
		/**
	
			if (mHasDepth)
			{
				if (!mCurrentSample->depth)
					return false;
				PXCImage *cDepthImage = mCurrentSample->depth;
				PXCImage::ImageData cDepthData;
				cStatus = cDepthImage->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &cDepthData);
				
				if (cStatus < PXC_STATUS_NO_ERROR)
				{
					cDepthImage->ReleaseAccess(&cDepthData);
					return false;
				}
				mDepthFrame.setFromExternalPixels(reinterpret_cast<uint16_t *>(cDepthData.planes[0]), mDepthSize.x, mDepthSize.y, 1);
				memcpy(mRawDepth, reinterpret_cast<uint16_t *>(cDepthData.planes[0]), (size_t)((int)mDepthSize.x*(int)mDepthSize.y*sizeof(uint16_t)));			
				cDepthImage->ReleaseAccess(&cDepthData);

				if (mShouldGetDepthAsColor)
				{
					PXCImage::ImageData cDepth8uData;
					cStatus = cDepthImage->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &cDepth8uData);
					if (cStatus < PXC_STATUS_NO_ERROR)
					{
						cDepthImage->ReleaseAccess(&cDepth8uData);
						return false;
					}
					mDepth8uFrame.setFromExternalPixels(reinterpret_cast<uint8_t *>(cDepth8uData.planes[0]), mDepthSize.x, mDepthSize.y, 4);
					cDepthImage->ReleaseAccess(&cDepth8uData);
				}

				if(mShouldGetPointCloud)
				{
					updatePointCloud();
				}

				if (!mHasRgb)
				{
					mSenseMgr->ReleaseFrame();
					return true;
				}
			}

			if (mHasDepth&&mHasRgb&&mShouldAlign&&mAlignMode==AlignMode::ALIGN_FRAME)
			{
				PXCImage *cMappedColor = mCoordinateMapper->CreateColorImageMappedToDepth(mCurrentSample->depth, mCurrentSample->color);
				PXCImage *cMappedDepth = mCoordinateMapper->CreateDepthImageMappedToColor(mCurrentSample->color, mCurrentSample->depth);

				if (!cMappedColor || !cMappedDepth)
					return false;

				PXCImage::ImageData cMappedColorData;
				if (cMappedColor->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &cMappedColorData) >= PXC_STATUS_NO_ERROR)
				{
					mColorToDepthFrame.setFromExternalPixels(reinterpret_cast<uint8_t *>(cMappedColorData.planes[0]), mRgbSize.x, mRgbSize.y, 4);
					cMappedColor->ReleaseAccess(&cMappedColorData);
				}

				PXCImage::ImageData cMappedDepthData;
				if (cMappedDepth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &cMappedDepthData) >= PXC_STATUS_NO_ERROR)
				{
					mDepthToColorFrame.setFromExternalPixels(reinterpret_cast<uint8_t *>(cMappedDepthData.planes[0]), mRgbSize.x, mRgbSize.y, 4);
					cMappedDepth->ReleaseAccess(&cMappedDepthData);
				}

				try
				{
					cMappedColor->Release();
					cMappedDepth->Release();
				}
				catch (const exception &e)
				{
					ofLog(ofLogLevel::OF_LOG_WARNING, "Release check error: ");
					ofLog(ofLogLevel::OF_LOG_WARNING, e.what());
				}
			}
			mSenseMgr->ReleaseFrame();
			return true;
		}
		**/

		return false;
	}

	bool RSDevice::stop()
	{
		rs2Pipe.stop();
		return true;
		
	}

	bool RSDevice::copyFrame(rs2::video_frame *source, ofPixels *target) {
		int pWidth = source->get_width();
		int pHeight = source->get_height();
		int stride_bytes = source->get_stride_in_bytes();
		int bytesPerPixel = source->get_bytes_per_pixel();

		unsigned char *data = (unsigned char *) source->get_data();

		target->setFromExternalPixels(data, pWidth, pHeight, 3);
		return true;
	}

#pragma region Enable

	/*
	unsigned char *RSDevice::stbi_write_png_to_mem(unsigned char *data, int stride_bytes, int pWidth, int pHeight, int bytesPerPixel, int *out_len)
	{
		unsigned char *out, *filt;
		signed char *line_buffer;
		int i, j, k, p, zlen;

		if (stride_bytes == 0)
			stride_bytes = pWidth * bytesPerPixel;


		filt = (unsigned char *)STBIW_MALLOC((pWidth * bytesPerPixel + 1) * pHeight); if (!filt) return 0;
		line_buffer = (signed char *)STBIW_MALLOC(pWidth * bytesPerPixel); if (!line_buffer) { STBIW_FREE(filt); return 0; }
		for (j = 0; j < pHeight; ++j) {
			static int mapping[] = { 0,1,2,3,4 };
			static int firstmap[] = { 0,1,0,5,6 };
			int *mymap = j ? mapping : firstmap;
			int best = 0, bestval = 0x7fffffff;
			for (p = 0; p < 2; ++p) {
				for (k = p ? best : 0; k < 5; ++k) {
					int type = mymap[k], est = 0;
					unsigned char *z = data + stride_bytes*j;
					for (i = 0; i < bytesPerPixel; ++i)
						switch (type) {
						case 0: line_buffer[i] = z[i]; break;
						case 1: line_buffer[i] = z[i]; break;
						case 2: line_buffer[i] = z[i] - z[i - stride_bytes]; break;
						case 3: line_buffer[i] = z[i] - (z[i - stride_bytes] >> 1); break;
						case 4: line_buffer[i] = (signed char)(z[i] - stbiw__paeth(0, z[i - stride_bytes], 0)); break;
						case 5: line_buffer[i] = z[i]; break;
						case 6: line_buffer[i] = z[i]; break;
						}
					for (i = bytesPerPixel; i < pWidth*bytesPerPixel; ++i) {
						switch (type) {
						case 0: line_buffer[i] = z[i]; break;
						case 1: line_buffer[i] = z[i] - z[i - bytesPerPixel]; break;
						case 2: line_buffer[i] = z[i] - z[i - stride_bytes]; break;
						case 3: line_buffer[i] = z[i] - ((z[i - bytesPerPixel] + z[i - stride_bytes]) >> 1); break;
						case 4: line_buffer[i] = z[i] - stbiw__paeth(z[i - bytesPerPixel], z[i - stride_bytes], z[i - stride_bytes - bytesPerPixel]); break;
						case 5: line_buffer[i] = z[i] - (z[i - bytesPerPixel] >> 1); break;
						case 6: line_buffer[i] = z[i] - stbiw__paeth(z[i - bytesPerPixel], 0, 0); break;
						}
					}
					if (p) break;
					for (i = 0; i < pWidth*bytesPerPixel; ++i)
						est += abs((signed char)line_buffer[i]);
					if (est < bestval) { bestval = est; best = k; }
				}
			}
			// when we get here, best contains the filter type, and line_buffer contains the data
			filt[j*(pWidth*bytesPerPixel + 1)] = (unsigned char)best;
			STBIW_MEMMOVE(filt + j*(pWidth*bytesPerPixel + 1) + 1, line_buffer, pWidth*bytesPerPixel);
		}
		STBIW_FREE(line_buffer);
		STBIW_FREE(filt);
	}

	static int RSDevice::stbi_write_png(char const *filename, int x, int y, int comp, const void *data, int stride_bytes)
	{
		FILE *f;
		int len;
		unsigned char *png = stbi_write_png_to_mem((unsigned char *)data, stride_bytes, x, y, comp, &len);
		if (png == NULL) return 0;
		f = fopen(filename, "wb");
		if (!f) { STBIW_FREE(png); return 0; }
		fwrite(png, 1, len, f);
		fclose(f);
		STBIW_FREE(png);
		return 1;
	}

	static unsigned char RSDevice::stbiw__paeth(int a, int b, int c)
	{
		int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
		if (pa <= pb && pa <= pc) return STBIW_UCHAR(a);
		if (pb <= pc) return STBIW_UCHAR(b);
		return STBIW_UCHAR(c);
	}
	*/

#pragma endregion

#pragma region Update
	void RSDevice::updatePointCloud()
	{
		int width = (int)mDepthSize.x;
		int height = (int)mDepthSize.y;
		int step = (int)mCloudRes;
		mPointCloud.clear();

		vector<rs2::vertex> depthPoints, worldPoints;

		auto vertices = rs2Points.get_vertices();              // get vertices

		/**
		for (int i = 0; i < rs2Points.size(); i++)
		{
			if (vertices[i].z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3fv(vertices[i]);
			}
		}
		**/

		for (int dy = 0; dy < height;dy+=step)
		{
			for (int dx = 0; dx < width; dx+=step)
			{
				rs2::vertex cPoint;
				cPoint.x = dx; 
				cPoint.y = dy; 
				cPoint.z = (float)vertices[dy*width + dx].z;
				if(cPoint.z > mPointCloudRange.x && cPoint.z < mPointCloudRange.y)
					depthPoints.push_back(cPoint);
			}
		}

		worldPoints.resize(depthPoints.size());

		for (int i = 0; i < depthPoints.size();++i)
		{
			rs2::vertex p = worldPoints[i];
			mPointCloud.push_back(ofVec3f(p.x, p.y, p.z));
		}
	}

	bool RSDevice::draw()
	{
		return false;
	}
#pragma endregion

#pragma region Getters
	const ofPixels& RSDevice::getRgbFrame()
	{
		return mRgbFrame;
	}

	const ofShortPixels& RSDevice::getDepthFrame()
	{
		return mDepthFrame;
	}

	const ofPixels& RSDevice::getDepth8uFrame()
	{
		return mDepth8uFrame;
	}

	const ofPixels& RSDevice::getColorMappedToDepthFrame()
	{
		return mColorToDepthFrame;
	}

	const ofPixels& RSDevice::getDepthMappedToColorFrame()
	{
		return mDepthToColorFrame;
	}

	vector<ofVec3f> RSDevice::getPointCloud()
	{
		return mPointCloud;
	}

	//Nomenclature Notes:
	//	"Space" denotes a 3d coordinate
	//	"Image" denotes an image space point ((0, width), (0,height), (image depth))
	//	"Coords" denotes texture space (U,V) coordinates
	//  "Frame" denotes a full Surface

	//get a camera space point from a depth image point
	const ofPoint RSDevice::getDepthSpacePoint(float pImageX, float pImageY, float pImageZ)
	{
		/**
		if (mCoordinateMapper)
		{
			PXCPoint3DF32 cPoint;
			cPoint.x = pImageX;
			cPoint.y = pImageY;
			cPoint.z = pImageZ;

			mInPoints3D.clear();
			mInPoints3D.push_back(cPoint);
			mOutPoints3D.clear();
			mOutPoints3D.resize(2);
			mCoordinateMapper->ProjectDepthToCamera(1, &mInPoints3D[0], &mOutPoints3D[0]);
			return ofPoint(mOutPoints3D[0].x, mOutPoints3D[0].y, mOutPoints3D[0].z);
		}
		**/
		return ofPoint(0);
	}

	const ofPoint RSDevice::getDepthSpacePoint(int pImageX, int pImageY, uint16_t pImageZ)
	{
		return getDepthSpacePoint(static_cast<float>(pImageX), static_cast<float>(pImageY), static_cast<float>(pImageZ));
	}

	const ofPoint RSDevice::getDepthSpacePoint(ofPoint pImageCoords)
	{
		return getDepthSpacePoint(pImageCoords.x, pImageCoords.y, pImageCoords.z);
	}

	//get a Color object from a depth image point
	const ofColor RSDevice::getColorFromDepthImage(float pImageX, float pImageY, float pImageZ)
	{
		/**
		if (mCoordinateMapper)
		{
			PXCPoint3DF32 cPoint;
			cPoint.x = pImageX;
			cPoint.y = pImageY;
			cPoint.z = pImageZ;
			PXCPoint3DF32 *cInPoint = new PXCPoint3DF32[1];
			cInPoint[0] = cPoint;
			PXCPointF32 *cOutPoints = new PXCPointF32[1];
			mCoordinateMapper->MapDepthToColor(1, cInPoint, cOutPoints);

			float cColorX = cOutPoints[0].x;
			float cColorY = cOutPoints[0].y;

			delete cInPoint;
			delete cOutPoints;
			if (cColorX >= 0 && cColorX < mRgbSize.x&&cColorY >= 0 && cColorY < mRgbSize.y)
			{
				return mRgbFrame.getColor(cColorX, cColorY);
			}
		}
		**/
		return ofColor::black;
	}

	const ofColor RSDevice::getColorFromDepthImage(int pImageX, int pImageY, uint16_t pImageZ)
	{
		/**
		if (mCoordinateMapper)
			return getColorFromDepthImage(static_cast<float>(pImageX),static_cast<float>(pImageY),static_cast<float>(pImageZ));
		**/
		return ofColor::black;
	}

	const ofColor RSDevice::getColorFromDepthImage(ofPoint pImageCoords)
	{
		/**
		if (mCoordinateMapper)
			return getColorFromDepthImage(pImageCoords.x, pImageCoords.y, pImageCoords.z);
			**/
		return ofColor::black;
	}


		//get a ofColor object from a depth camera space point
	const ofColor RSDevice::getColorFromDepthSpace(float pCameraX, float pCameraY, float pCameraZ)
	{
		/**
		if (mCoordinateMapper)
		{
			PXCPoint3DF32 cPoint;
			cPoint.x = pCameraX; cPoint.y = pCameraY; cPoint.z = pCameraZ;

			mInPoints3D.clear();
			mInPoints3D.push_back(cPoint);
			mOutPoints2D.clear();
			mOutPoints2D.resize(2);
			mCoordinateMapper->ProjectCameraToColor(1, &mInPoints3D[0], &mOutPoints2D[0]);

			int imageX = static_cast<int>(mOutPoints2D[0].x);
			int imageY = static_cast<int>(mOutPoints2D[0].y);
			if( (imageX>=0&&imageX<mRgbSize.x)  &&(imageY>=0&&imageY<mRgbSize.y))
				return mRgbFrame.getColor(imageX, imageY);
			return ofColor::black;
		}
		**/
		return ofColor::black;
	}

	const ofColor RSDevice::getColorFromDepthSpace(ofPoint pCameraPoint)
	{
		/**
		if (mCoordinateMapper)
			return getColorFromDepthSpace(pCameraPoint.x, pCameraPoint.y, pCameraPoint.z);
		**/
		return ofColor::black;
	}

		//get ofColor space UVs from a depth image point
	const ofVec2f RSDevice::getColorCoordsFromDepthImage(float pImageX, float pImageY, float pImageZ)
	{
		/**
		if (mCoordinateMapper)
		{
			PXCPoint3DF32 cPoint;
			cPoint.x = pImageX;
			cPoint.y = pImageY;
			cPoint.z = pImageZ;

			PXCPoint3DF32 *cInPoint = new PXCPoint3DF32[1];
			cInPoint[0] = cPoint;
			PXCPointF32 *cOutPoints = new PXCPointF32[1];
			mCoordinateMapper->MapDepthToColor(1, cInPoint, cOutPoints);

			float cColorX = cOutPoints[0].x;
			float cColorY = cOutPoints[0].y;

			delete cInPoint;
			delete cOutPoints;
			return ofVec2f(cColorX / (float)mRgbSize.x, cColorY / (float)mRgbSize.y);
		}
		**/
		return ofVec2f(0);
	}

	const ofVec2f RSDevice::getColorCoordsFromDepthImage(int pImageX, int pImageY, uint16_t pImageZ)
	{
		return getColorCoordsFromDepthImage(static_cast<float>(pImageX), static_cast<float>(pImageY), static_cast<float>(pImageZ));
	}

	const ofVec2f RSDevice::getColorCoordsFromDepthImage(ofPoint pImageCoords)
	{
		return getColorCoordsFromDepthImage(pImageCoords.x, pImageCoords.y, pImageCoords.z);
	}

		//get ofColor space UVs from a depth space point
	const ofVec2f RSDevice::getColorCoordsFromDepthSpace(float pCameraX, float pCameraY, float pCameraZ)
	{
		/**
		if (mCoordinateMapper)
		{
			PXCPoint3DF32 cPoint;
			cPoint.x = pCameraX; cPoint.y = pCameraY; cPoint.z = pCameraZ;

			PXCPoint3DF32 *cInPoint = new PXCPoint3DF32[1];
			cInPoint[0] = cPoint;
			PXCPointF32 *cOutPoint = new PXCPointF32[1];
			mCoordinateMapper->ProjectCameraToColor(1, cInPoint, cOutPoint);

			ofVec2f cRetPt(cOutPoint[0].x / static_cast<float>(mRgbSize.x), cOutPoint[0].y / static_cast<float>(mRgbSize.y));
			delete cInPoint;
			delete cOutPoint;
			return cRetPt;
		}
		**/
		return ofVec2f(0);
	}

	const ofVec2f RSDevice::getColorCoordsFromDepthSpace(ofPoint pCameraPoint)
	{
		return getColorCoordsFromDepthSpace(pCameraPoint.x, pCameraPoint.y, pCameraPoint.z);
	}
}
#pragma endregion