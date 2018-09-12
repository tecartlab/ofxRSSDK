#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetWindowShape(1280, 960);

	mRSSDK = RSDevice::createUniquePtr();

	mRSSDK->checkConnectedDialog();
	ofLogNotice("Device detected..");

	gui_post.setup("PostProcessing", "postprocessingSetup", 0, 0); // most of the time you don't need a name but don't forget to call setup
	gui_post.add(mRSSDK->param_usePostProcessing);
	gui_post.add(mRSSDK->param_filterDecimation);
	gui_post.add(mRSSDK->param_filterDecimation_mag);
	gui_post.add(mRSSDK->param_filterDisparities);
	gui_post.add(mRSSDK->param_filterSpatial);
	gui_post.add(mRSSDK->param_filterSpatial_smoothAlpha);
	gui_post.add(mRSSDK->param_filterSpatial_smoothDelta);
	gui_post.add(mRSSDK->param_filterSpatial_mag);
	gui_post.add(mRSSDK->param_filterTemporal);
	gui_post.add(mRSSDK->param_filterTemporal_smoothAlpha);
	gui_post.add(mRSSDK->param_filterTemporal_smoothDelta);
	gui_post.add(mRSSDK->param_filterTemporal_persistency);



	//mRSSDK->enablePointCloud(CloudRes::FULL_RES);
	mRSSDK->setPointCloudRange(100.0f,1000.0f);

	mRSSDK->start();

	// the device settings should be loaded/set after the start()
	gui_device.setup("Device", "deviceSettings", 200, 0);
	gui_device.add(mRSSDK->param_deviceLaser);
	gui_device.add(mRSSDK->param_deviceLaser_mag);

	setupCamera();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (mRSSDK->update(ofxRSSDK::PointCloud::INFRALEFT)) {
		// if a frame has been updated, the code continues in here,
		// in case you need to do something special...
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofClear(ofColor::black);
	ofSetColor(ofColor::white);

	//mRSSDK->drawVideoStream(ofRectangle(0, 0, ofGetWidth() / 2., ofGetHeight() / 2.));
	//mRSSDK->drawDepthStream(ofRectangle(ofGetWidth() / 2., 0, ofGetWidth() / 2., ofGetHeight() / 2.));
	//mRSSDK->drawInfraLeftStream(ofRectangle(0, ofGetHeight() / 2., ofGetWidth() / 2., ofGetHeight() / 2.));

	mCamera.begin(); 

	ofPushMatrix();
	ofScale(100, 100, 100);
	mRSSDK->draw();
	ofPopMatrix();

	mCamera.end();

	ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), ofGetWidth() - 200, 10);

	gui_post.draw();
	gui_device.draw();
}


void ofApp::exit()
{
	mRSSDK->stop();
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
