#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetWindowShape(1280, 960);
	mRSSDK = RSDevice::createUniquePtr();

	//mRSSDK->enablePointCloud(CloudRes::FULL_RES);
	mRSSDK->setPointCloudRange(100.0f,1000.0f);

	mRSSDK->useFilterDecimation(8);
	mRSSDK->useFilterSpatialEdgePreserve(2, 0.5, 20, 0);
	mRSSDK->useFilterTemporal(0.4, 20, 3);
	mRSSDK->useFilterDisparity();

	mRSSDK->start();
	setupCamera();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (mRSSDK->update()) {
		// Generate the pointcloud with the default colors
		//mRSSDK->updatePointCloud();
		//Generate the pointcloud with specified ofPixels objects
		mRSSDK->updatePointCloud(mRSSDK->getDepthFrame());
		//mRSSDK->updatePointCloud(mRSSDK->getRgbFrame());
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofClear(ofColor::black);
	ofSetColor(ofColor::white);

	//mRSSDK->drawColor(ofRectangle(0, 0, ofGetWidth() / 2., ofGetHeight() / 2.));
	//mRSSDK->drawDepth(ofRectangle(ofGetWidth() / 2., ofGetHeight() / 2., ofGetWidth() / 2., ofGetHeight() / 2.));

	mCamera.begin(); 

	ofPushMatrix();
	ofScale(100, 100, 100);
	mRSSDK->draw();
	ofPopMatrix();

	mCamera.end();

	ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), ofGetWidth() - 200, 10);

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
