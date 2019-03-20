#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetWindowShape(1280, 960);

	realSense = RSDevice::createUniquePtr();

	realSense->checkConnectedDialog();

	//realSense->hardwareReset();

	ofLogNotice("Device detected..");
	gui_post.setup("PostProcessing", "postprocessingSetup", 0, 0); // most of the time you don't need a name but don't forget to call setup
	gui_post.add(realSense->param_usePostProcessing);
	gui_post.add(realSense->param_filterDecimation);
	gui_post.add(realSense->param_filterDecimation_mag);
	gui_post.add(realSense->param_filterDisparities);
	gui_post.add(realSense->param_filterSpatial);
	gui_post.add(realSense->param_filterSpatial_smoothAlpha);
	gui_post.add(realSense->param_filterSpatial_smoothDelta);
	gui_post.add(realSense->param_filterSpatial_mag);
	gui_post.add(realSense->param_filterTemporal);
	gui_post.add(realSense->param_filterTemporal_smoothAlpha);
	gui_post.add(realSense->param_filterTemporal_smoothDelta);
	gui_post.add(realSense->param_filterTemporal_persistency);

	//realSense->enablePointCloud(CloudRes::FULL_RES);
	realSense->setPointCloudRange(100.0f,1000.0f);

	gui_device.setup("Device", "deviceSettings", 200, 0);

	if (realSense->capture()) {
		// the device settings should be loaded/set after the start()
		gui_device.add(realSense->param_deviceLaser);
		gui_device.add(realSense->param_deviceLaser_mag);
		gui_device.add(realSense->param_deviceAutoExposure);
		gui_device.add(realSense->param_deviceExposure_mag);
		gui_device.add(realSense->param_deviceGain_mag);
		gui_device.add(realSense->param_deviceFrameQueSize_mag);
		gui_device.add(realSense->param_deviceAsicTemparature);
		gui_device.add(realSense->param_deviceProjectorTemparature);
	}

	realSense->printDeviceInfo();

	setupCamera();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (realSense->update(ofxRSSDK::PointCloud::INFRALEFT)) {
		// if a frame has been updated, the code continues in here,
		// in case you need to do something special...
		/*
		glm::vec2 col = glm::vec2(realSense->getDepthWidth() / 2, realSense->getDepthHeight() / 2);
		glm::vec3 d_pt = realSense->getSpacePointFromDepthFrameCoord(col);
		cout << "color pixel x=" << col.x << ", y=" << col.y << endl;
		cout << "depth pixel x=" << d_pt.x << ", y=" << d_pt.y << ", z =" << d_pt.z << endl;
		float distance = realSense->getSpaceDistanceFromDepthFrame(col);
		cout << "distance =" << distance << endl;
		*/
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofClear(ofColor::black);
	ofSetColor(ofColor::white);

	//realSense->drawVideoStream(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));
	realSense->drawVideoStream(ofRectangle(0, 0, ofGetWidth() / 2., ofGetHeight() / 2.));
	realSense->drawDepthStream(ofRectangle(ofGetWidth() / 2., 0, ofGetWidth() / 2., ofGetHeight() / 2.));
	realSense->drawInfraLeftStream(ofRectangle(0, ofGetHeight() / 2., ofGetWidth() / 2., ofGetHeight() / 2.));

	mCamera.begin(ofRectangle(ofGetWidth() / 2., ofGetHeight() / 2., ofGetWidth() / 2., ofGetHeight() / 2.));

	ofPushMatrix();
	ofScale(100, 100, 100);
	realSense->draw();
	ofPopMatrix();

	mCamera.end();

	ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), ofGetWidth() - 200, 10);

	gui_post.draw();
	gui_device.draw();
}


void ofApp::exit()
{
	realSense->stop();
}

void ofApp::setupCamera()
{
	mCamera.setFov(45.0f);
	mCamera.setAspectRatio(ofGetWindowWidth() / (float)ofGetWindowHeight());
	mCamera.setNearClip(100);
	mCamera.setFarClip(5000);

	mCamera.setGlobalPosition(ofVec3f(0, 0, 0));
	mCamera.lookAt(ofVec3f(0, 0, 100), ofVec3f(0, 1, 0));
	mCamera.setAutoDistance(true);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
