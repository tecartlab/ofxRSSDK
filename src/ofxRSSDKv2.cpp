#include <exception>
#include "ofxRSSDKv2.h"
#include "ofMain.h"

//sdjk https://github.com/IntelRealSense/librealsense
//extrinsic data: https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/examples/sensor-control/api_how_to.h#L209
// projection: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
// howtos: https://github.com/IntelRealSense/librealsense/wiki/API-How-To#get-depth-units

namespace ofxRSSDK
{	
	Disparity::~Disparity()
	{
		delete filter_in;
		delete filter_out;
	}

	Disparity::Disparity()
	{
		filter_in = new rs2::disparity_transform(true);
		filter_out = new rs2::disparity_transform(false);
	}

	RSDevice::~RSDevice(){
	}

	RSDevice::RSDevice(){
		mIsInit = false;
		mIsRunning = false;
		mStreamsVideo = false;
		mStreamsDepth = false;
		mShouldAlign = false;
		mShouldGetDepthAsColor = false;
		mShouldGetPointCloud = false;
		mPointCloudRange = ofVec2f(0,3000);
		mCloudRes = CloudRes::FULL_RES;
		isUsingFilterDec = false;
		isUsingFilterSpat = false;
		isUsingFilterTemp = false;
		isUsingFilterDisparity = false;
		isUsingPostProcessing = false;

		// setting up ofParameters
		param_usePostProcessing.set("use PostProcessing", false);
		param_usePostProcessing.addListener(this, &RSDevice::usePostProcessing_p);

		param_filterDecimation.set("use decimation filter", false);
		param_filterDecimation.addListener(this, &RSDevice::filterDecimation_p);

		param_filterDecimation_mag.set("decimation magnitude", 2, 2, 8);
		param_filterDecimation_mag.addListener(this, &RSDevice::filterDecimation_mag_p);

		param_filterSpatial.set("use spatial filter", false);
		param_filterSpatial.addListener(this, &RSDevice::filterSpatial_p);

		param_filterSpatial_mag.set("spatial magnitude", 2, 2, 5);
		param_filterSpatial_mag.addListener(this, &RSDevice::filterSpatial_mag_p);

		param_filterSpatial_smoothAlpha.set("spatial smoothAlpha", 0.5, 0.25, 1.0);
		param_filterSpatial_smoothAlpha.addListener(this, &RSDevice::filterSpatial_smoothAlpha_p);

		param_filterSpatial_smoothDelta.set("spatial smoothDelta", 20, 1, 50);
		param_filterSpatial_smoothDelta.addListener(this, &RSDevice::filterSpatial_smoothDelta_p);

		param_filterTemporal.set("use temporal Filter", false);
		param_filterTemporal.addListener(this, &RSDevice::filterTemporal_p);

		param_filterTemporal_smoothAlpha.set("temporal smoothAlpha", 0.4, 0.0, 1.0);
		param_filterTemporal_smoothAlpha.addListener(this, &RSDevice::filterTemporal_smoothAlpha_p);

		param_filterTemporal_smoothDelta.set("temporal smoothDelta", 20, 0, 100);
		param_filterTemporal_smoothDelta.addListener(this, &RSDevice::filterTemporal_smoothDelta_p);

		param_filterTemporal_persistency.set("temporal persistency", 3, 0, 8);
		param_filterTemporal_persistency.addListener(this, &RSDevice::filterTemporal_persistency_p);

		param_filterDisparities.set("use disparity filters", false);
		param_filterDisparities.addListener(this, &RSDevice::filterDisparities_p);

		param_deviceLaser.set("laser power", 0.5);
		param_deviceLaser.addListener(this, &RSDevice::deviceLaser_p);

	}

#pragma region Init

#pragma endregion

	void RSDevice::setPointCloudRange(float pMin=100.0f, float pMax=1500.0f)
	{
		mPointCloudRange = ofVec2f(pMin,pMax);
	}

	bool RSDevice::start(bool useDepth, bool useVideo, bool useInfrared, int depthWidth, int depthHeight, int videoWidth, int videoHeight)
	{
		mPointCloud.clear();
		mPointCloud.setMode(OF_PRIMITIVE_POINTS);
		mPointCloud.enableColors();

		mVideoStreamSize.x = videoWidth;
		mVideoStreamSize.y = videoHeight;

		mInfraredStreamSize.x = depthWidth;
		mInfraredStreamSize.y = depthHeight;

		//Create a configuration for configuring the pipeline with a non default profile
		rs2::config cfg;

		//Add desired streams to configuration
		//cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16); // Enable default depth
											 // For the color stream, set format to RGBA
											 // To allow blending of the color frame on top of the depth frame
		//cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
		if (useDepth) {
			cfg.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16, 30);
			mStreamsDepth = true;
		}
		if (useVideo) {
			cfg.enable_stream(RS2_STREAM_COLOR, videoWidth, videoHeight, RS2_FORMAT_RGB8, 30);
			mStreamsVideo = true;
		}
		if (useInfrared) {
			cfg.enable_stream(RS2_STREAM_INFRARED, 1);
			mStreamsIR = true;
		}

		rs2PipeLineProfile = rs2Pipe.start(cfg);
		
		rs2Device = rs2PipeLineProfile.get_device();

		// Capture 30 frames to give autoexposure, etc. a chance to settle
		for (auto i = 0; i < 30; ++i) rs2Pipe.wait_for_frames();

		mIsRunning = true;
		mHasChangedResolution = true;

		return true;
	}

	bool RSDevice::isRunning() {
		return mIsRunning;
	}

	bool RSDevice::update()
	{
		if (rs2Pipe.poll_for_frames(&rs2FrameSet))
		{
			if (mStreamsDepth) {
				rs2Depth = rs2FrameSet.first(RS2_STREAM_DEPTH);

				if (rs2Depth)
				{
					if (isUsingPostProcessing) {
						if (isUsingFilterDec) {
							rs2Depth = rs2Filter_dec.process(rs2Depth);
						}
						if (isUsingFilterDisparity) {
							rs2Depth = rs2Filter_disparity.filter_in->process(rs2Depth);
						}
						if (isUsingFilterSpat) {
							rs2Depth = rs2Filter_spat.process(rs2Depth);
						}
						if (isUsingFilterTemp) {
							rs2Depth = rs2Filter_temp.process(rs2Depth);
						}
						if (isUsingFilterDisparity) {
							rs2Depth = rs2Filter_disparity.filter_out->process(rs2Depth);
						}
					}

					// Use the colorizer to get an rgb image for the depth stream
					auto rs2DepthVideoFrame = rs2Color_map.colorize(rs2Depth);

					// set the new resolutions for the depth stream
					mDepthStreamSize.x = rs2DepthVideoFrame.get_width();
					mDepthStreamSize.y = rs2DepthVideoFrame.get_height();

					mDepthFrame.setFromExternalPixels((unsigned char *)rs2DepthVideoFrame.get_data(), mDepthStreamSize.x, mDepthStreamSize.y, 3);
				}
			}

			if (mStreamsVideo) {
				auto rs2VideoFrame = rs2FrameSet.first(RS2_STREAM_COLOR).as<rs2::video_frame>();

				if (rs2VideoFrame)
				{
					mVideoFrame.setFromExternalPixels((unsigned char *)rs2VideoFrame.get_data(), rs2VideoFrame.get_width(), rs2VideoFrame.get_height(), 3);
				}
			}

			if (mStreamsIR) {
				auto rs2IRFrame = rs2FrameSet.first(RS2_STREAM_INFRARED).as<rs2::video_frame>();

				if (rs2IRFrame)
				{
					mInfraLeftFrame.setFromExternalPixels((unsigned char *)rs2IRFrame.get_data(), rs2IRFrame.get_width(), rs2IRFrame.get_height(), 1);
				}
			}
			return true;
		}
	
		return false;
	}

	bool RSDevice::stop()
	{
		rs2Pipe.stop();
		mIsRunning = false;
		return true;
		
	}

#pragma region Enable

#pragma endregion

#pragma region Update
	void RSDevice::updatePointCloud()
	{
		if (mVideoFrame.size()) {
			updatePointCloud(mVideoFrame);
		}
		else {
			updatePointCloud(mDepthFrame);
		}
	}

	void RSDevice::updatePointCloud(ofPixels colors)
	{
		// Generate the pointcloud and texture mapping	s
		rs2Points = rs2PointCloud.calculate(rs2Depth);

		int dWidth = (int)mDepthStreamSize.x;
		int dHeight = (int)mDepthStreamSize.y;
		int cWidth = colors.getWidth();
		int cHeight = colors.getHeight();

		int step = (int)mCloudRes;

		int length = dHeight * dWidth / (step * step);

		if (length != mPointCloud.getVertices().size()) {
			mPointCloud.clear();
			for (int i = 0; i < length; i++) {
				mPointCloud.addVertex(glm::vec3(0, 0, 0));
				mPointCloud.addColor(ofDefaultColorType());
			}
			cout << "created new depth point cloud w: " << dWidth << " h: " << dHeight << endl;
			//cout << "created new mesh: " << dHeight << "/" << dWidth << endl;
		}
		
		//float firstTime = ofGetElapsedTimef();  

		auto vertices = rs2Points.get_vertices();              // get vertices

		glm::vec3* pVertices = mPointCloud.getVerticesPointer();
		ofDefaultColorType* pColors = mPointCloud.getColorsPointer();

		int i_dOrig, i_dTarget;
		float relHeight = (float)cHeight / (float)dHeight;
		float relWidth = (float)cWidth / (float)dWidth;

		//cout << "relHeight: " << relHeight << " relWidth: " << relWidth << endl;

		for (int dy = 0; dy < dHeight; dy+=step)
		{
			int cy = dy * relHeight;
			auto pxlLine = colors.getLine(cy);

			for (int dx = 0; dx < dWidth; dx+=step)
			{
				int cx = dx * relHeight;
				auto pxl = pxlLine.getPixel(cx);

				i_dOrig = dy * dWidth + dx;

				i_dTarget = dy * dWidth / (step * step) + dx / step;

				pVertices[i_dTarget].x = vertices[i_dOrig].x;
				pVertices[i_dTarget].y = vertices[i_dOrig].y;
				pVertices[i_dTarget].z = vertices[i_dOrig].z;
				
				pColors[i_dTarget].r = pxl[0] / 255.;
				pColors[i_dTarget].g = pxl[1] / 255.;
				pColors[i_dTarget].b = pxl[2] / 255.;
			}
		}

		/*
		float lastTime = ofGetElapsedTimef();
		cout << "final depth point cloud size: " << mPointCloud.getVertices().size() << endl;
		cout << "elapsed time " << lastTime - firstTime  << endl;
		*/

	}

	bool RSDevice::draw()
	{
		mPointCloud.setMode(OF_PRIMITIVE_POINTS);
		mPointCloud.enableColors();
		mPointCloud.draw();
		return true;
	}

	bool RSDevice::drawVideoStream(const ofRectangle & rect)
	{
		if (mVideoFrame.getWidth() > 0) {
			ofTexture texRGB;
			texRGB.loadData(mVideoFrame);
			texRGB.draw(rect.x, rect.y, rect.width, rect.height);
			return true;
		}
		return false;
	}

	bool RSDevice::drawDepthStream(const ofRectangle & rect)
	{
		if (mDepthFrame.getWidth() > 0) {
			ofTexture texRGB;
			texRGB.loadData(mDepthFrame);
			texRGB.draw(rect.x, rect.y, rect.width, rect.height);
			return true;
		}
		return false;
	}

	bool RSDevice::drawInfraLeftStream(const ofRectangle & rect)
	{
		if (mInfraLeftFrame.getWidth() > 0) {
			ofTexture texRGB;
			texRGB.loadData(mInfraLeftFrame);
			texRGB.draw(rect.x, rect.y, rect.width, rect.height);
			return true;
		}
		return false;
	}

#pragma endregion

#pragma region Getters
	const ofPixels& RSDevice::getVideoFrame()
	{
		return mVideoFrame;
	}

	const ofPixels& RSDevice::getInfraLeftFrame()
	{
		return mInfraLeftFrame;
	}

	const ofPixels& RSDevice::getDepthFrame()
	{
		return mDepthFrame;
	}

	const ofPixels& RSDevice::getColorMappedToDepthFrame()
	{
		return mColorToDepthFrame;
	}

	const ofPixels& RSDevice::getDepthMappedToColorFrame()
	{
		return mDepthToColorFrame;
	}

	ofMesh RSDevice::getPointCloud()
	{
		return mPointCloud;
	}

	vector<glm::vec3> & RSDevice::getPointCloudVertices()
	{
		return mPointCloud.getVertices();
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

	void RSDevice::checkConnectedDialog() {
		rs2::context ctx;
		auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
		while (list.size() == 0) {
			ofSystemAlertDialog("No Device found. Have you attached a RealSense D400 camera?");
			list = ctx.query_devices();
		}
	}

	const glm::vec2 RSDevice::getColorCoordsFromDepthSpace(ofPoint pCameraPoint)
	{
		return getColorCoordsFromDepthSpace(pCameraPoint.x, pCameraPoint.y, pCameraPoint.z);
	}

	void RSDevice::usePostProcessing_p(bool & enable) {
		usePostProcessing(enable);
	}
	void RSDevice::usePostProcessing(bool const & enable) {
		isUsingPostProcessing = enable;
	}

	void RSDevice::filterDecimation_p(bool & enable) {
		filterDecimation(enable);
	}
	void RSDevice::filterDecimation(bool const & enable) {
		isUsingFilterDec = enable;
	}

	void RSDevice::filterDecimation_mag_p(int & magnitude) {
		filterDecimation_mag(magnitude);
	}
	void RSDevice::filterDecimation_mag(int const & magnitude) {
		rs2Filter_dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, magnitude);
	}

	void RSDevice::filterSpatial_p(bool & enable) {
		filterSpatial(enable);
	}
	void RSDevice::filterSpatial(bool const & enable) {
		isUsingFilterSpat = enable;
	}

	void RSDevice::filterSpatial_mag_p(int & magnitude) {
		filterSpatial_mag(magnitude);
	}
	void RSDevice::filterSpatial_mag(int const & magnitude) {
		rs2Filter_spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, magnitude);
	}

	void RSDevice::filterSpatial_smoothAlpha_p(float & smoothAlpha) {
		filterSpatial_smoothAlpha(smoothAlpha);
	}
	void RSDevice::filterSpatial_smoothAlpha(float const & smoothAlpha) {
		rs2Filter_temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, smoothAlpha);
	}

	void RSDevice::filterSpatial_smoothDelta_p(int & smoothDelta) {
		filterSpatial_smoothDelta(smoothDelta);
	}
	void RSDevice::filterSpatial_smoothDelta(int const & smoothDelta) {
		rs2Filter_spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, smoothDelta);
	}

	void RSDevice::filterSpatial_holeFilling_p(int & holeFilling) {
		filterSpatial_holeFilling(holeFilling);
	}
	void RSDevice::filterSpatial_holeFilling(int const & holeFilling) {
		rs2Filter_spat.set_option(RS2_OPTION_HOLES_FILL, holeFilling);
	}

	void RSDevice::filterTemporal_p(bool & enable) {
		filterTemporal(enable);
	}
	void RSDevice::filterTemporal(bool const & enable) {
		isUsingFilterTemp = enable;
	}

	void RSDevice::filterTemporal_smoothAlpha_p(float & smoothAlpha) {
		filterTemporal_smoothAlpha(smoothAlpha);
	}
	void RSDevice::filterTemporal_smoothAlpha(float const & smoothAlpha) {
		rs2Filter_temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, smoothAlpha);
	}

	void RSDevice::filterTemporal_smoothDelta_p(int & smoothDelta) {
		filterTemporal_smoothDelta(smoothDelta);
	}
	void RSDevice::filterTemporal_smoothDelta(int const & smoothDelta) {
		rs2Filter_temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, smoothDelta);
	}

	void RSDevice::filterTemporal_persistency_p(int & persitency) {
		filterTemporal_persistency(persitency);
	}
	void RSDevice::filterTemporal_persistency(int const & persitency) {
		rs2Filter_temp.set_option(RS2_OPTION_HOLES_FILL, persitency);
	}

	void RSDevice::filterDisparities_p(bool & enable) {
		filterDisparities(enable);
	}
	void RSDevice::filterDisparities(bool const & enable) {
		isUsingFilterDisparity = enable;
	}

	void RSDevice::deviceLaser(float const & magnitude) {
		auto rs2DepthSensor = rs2Device.first<rs2::depth_sensor>();

		if (rs2DepthSensor.supports(RS2_OPTION_EMITTER_ENABLED) && magnitude > 0.0)
		{
			rs2DepthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
			if (rs2DepthSensor.supports(RS2_OPTION_LASER_POWER))
			{
				// Query min and max values:
				auto range = rs2DepthSensor.get_option_range(RS2_OPTION_LASER_POWER);
				rs2DepthSensor.set_option(RS2_OPTION_LASER_POWER, range.max * magnitude); // Set max power
			}
		}
		else {
			rs2DepthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
		}
	}
	void RSDevice::deviceLaser_p(float & magnitude) {
		deviceLaser(magnitude);
	}

}
#pragma endregion