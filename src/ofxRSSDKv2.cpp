#include <exception>
#include "ofxRSSDKv2.h"
#include "ofMain.h"


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

		// Capture 30 frames to give autoexposure, etc. a chance to settle
		for (auto i = 0; i < 30; ++i) rs2Pipe.wait_for_frames();

		return true;
	}

	bool RSDevice::update()
	{
		if (rs2Pipe.poll_for_frames(&rs2FrameSet))
		{
			auto rs2DepthFrame = rs2FrameSet.first(RS2_STREAM_DEPTH);
	
			if (rs2DepthFrame)
			{
				// Use the colorizer to get an rgb image for the depth stream
				auto rs2DepthVideoFrame = rs2Color_map(rs2DepthFrame);

				mDepthFrame.setFromExternalPixels((unsigned char *)rs2DepthVideoFrame.get_data(), rs2DepthVideoFrame.get_width(), rs2DepthVideoFrame.get_height(), 3);
			}

			auto rs2VideoFrame = rs2FrameSet.first(RS2_STREAM_COLOR).as<rs2::video_frame>();

			if (rs2VideoFrame)
			{
				mRgbFrame.setFromExternalPixels((unsigned char *)rs2VideoFrame.get_data(), rs2VideoFrame.get_width(), rs2VideoFrame.get_height(), 3);
			}

			// Generate the pointcloud and texture mapping	s
			updatePointCloud(rs2DepthFrame, rs2VideoFrame, mRgbFrame);

			return true;
		}
	
		return false;
	}

	bool RSDevice::stop()
	{
		rs2Pipe.stop();
		return true;
		
	}

#pragma region Enable

#pragma endregion

#pragma region Update
	void RSDevice::updatePointCloud(rs2::frame depthFrame, rs2::video_frame texture, ofPixels colors)
	{

		cout << "calculate pc: " << endl;

		// Generate the pointcloud and texture mapping	s
		rs2Points = rs2PointCloud.calculate(depthFrame);

		rs2PointCloud.map_to(texture);

		int width = (int)mDepthSize.x;
		int height = (int)mDepthSize.y;
		int step = (int)mCloudRes;
		
		cout << "depth point cloud w: " << width << " h: " << height << endl;

		vector<rs2::vertex> depthPoints, worldPoints;

		auto vertices = rs2Points.get_vertices();              // get vertices
		auto textureUVs = rs2Points.get_texture_coordinates();

		if (mPointCloudVertices.size() == 0) {
			cout << "create new vector: " << endl;
			mPointCloudVertices.resize(height * width / (step * step));
			std::fill(mPointCloudVertices.begin(), mPointCloudVertices.end(), glm::vec3(0, 0, 0));
		}
		
		float firstTime = ofGetElapsedTimef();  

		glm::vec3 vec = glm::vec3(0, 0, 0);

		glm::vec3* p = mPointCloudVertices.data();

		int indexA, indexB;
		for (int dy = 0; dy < height; dy+=step)
		{
			for (int dx = 0; dx < width; dx+=step)
			{

				indexA = dy * width + dx;
				indexB = dy * width / (step * step) + dx / step;
				//indexB = indexA / width;

				p[indexB].x = vertices[indexA].x;
				p[indexB].y = vertices[indexA].y;
				p[indexB].z = vertices[indexA].z;
				
				
				/*
				mPointCloudVertices[indexB] = glm::vec3(vertices[indexA].x, vertices[indexA].y, vertices[indexA].z);
				mPointCloudVertices[indexB].set(vertices[indexA].x, vertices[indexA].y, vertices[indexA].z);
				mPointCloudVertices[indexB].x = vertices[indexA].x;
				mPointCloudVertices[indexB].y = vertices[indexA].y;
				mPointCloudVertices[indexB].z = vertices[indexA].z;
				*/

				//cPoint.z = (float)vertices[dy*width + dx].z;
				//if (cPoint.z > mPointCloudRange.x && cPoint.z < mPointCloudRange.y) {
				//	depthPoints.push_back(cPoint);
				//}
			}
		}

		float lastTime = ofGetElapsedTimef();

		cout << "elapsed time " << lastTime - firstTime  << endl;

		/*
		worldPoints.resize(depthPoints.size());

		for (int i = 0; i < depthPoints.size();++i)
		{
			rs2::vertex p = worldPoints[i];
			mPointCloudVertices.push_back(ofVec3f(p.x, p.y, p.z));
		}
		*/

		cout << "final depth point cloud size: " << mPointCloudVertices.size() << endl;
	}

	bool RSDevice::draw(float width, float height, float offset, float pitch, float yaw, ofTexture texture)
	{
		if (!rs2Points)
			return false;

		cout << "drawing pointcloud: " << rs2Points.get_frame_number() << endl;

		// OpenGL commands that prep screen for the pointcloud
		glPopMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
		glClear(GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		gluPerspective(60, width / height, 0.01f, 10.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

		glTranslatef(0, 0, +0.5f + offset*0.05f);
		glRotated(pitch, 1, 0, 0);
		glRotated(yaw, 0, 1, 0);
		glTranslatef(0, 0, -0.5f);

		glPointSize(width / 640);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_TEXTURE_2D);

		float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
		glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
		glBegin(GL_POINTS);

		/* this segment actually prints the pointcloud */
		auto vertices = rs2Points.get_vertices();              // get vertices
		auto tex_coords = rs2Points.get_texture_coordinates(); // and texture coordinates
		for (int i = 0; i < rs2Points.size(); i++)
		{
			if (vertices[i].z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3fv(vertices[i]);
				glColor3f(1., 0., 0.);
				//glTexCoord2fv(tex_coords[i]);
			}
		}

		// OpenGL cleanup
		glEnd();
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glPushMatrix();

		return true;
	}
#pragma endregion

#pragma region Getters
	const ofPixels& RSDevice::getRgbFrame()
	{
		return mRgbFrame;
	}

	const ofPixels& RSDevice::getDepthFrame()
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

	vector<glm::vec3> RSDevice::getPointCloud()
	{
		return mPointCloudVertices;
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
	const glm::vec2 RSDevice::getColorCoordsFromDepthImage(float pImageX, float pImageY, float pImageZ)
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

	const glm::vec2 RSDevice::getColorCoordsFromDepthImage(int pImageX, int pImageY, uint16_t pImageZ)
	{
		return getColorCoordsFromDepthImage(static_cast<float>(pImageX), static_cast<float>(pImageY), static_cast<float>(pImageZ));
	}

	const glm::vec2 RSDevice::getColorCoordsFromDepthImage(ofPoint pImageCoords)
	{
		return getColorCoordsFromDepthImage(pImageCoords.x, pImageCoords.y, pImageCoords.z);
	}

		//get ofColor space UVs from a depth space point
	const glm::vec2 RSDevice::getColorCoordsFromDepthSpace(float pCameraX, float pCameraY, float pCameraZ)
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
		return glm::vec2();
	}

	const glm::vec2 RSDevice::getColorCoordsFromDepthSpace(ofPoint pCameraPoint)
	{
		return getColorCoordsFromDepthSpace(pCameraPoint.x, pCameraPoint.y, pCameraPoint.z);
	}
}
#pragma endregion