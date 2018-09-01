#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetWindowShape(1280, 720);
	mRSSDK = RSDevice::createUniquePtr();

	//mRSSDK->enablePointCloud(CloudRes::FULL_RES);
	mRSSDK->setPointCloudRange(100.0f,1000.0f);

	mRSSDK->start();
	setupCamera();
}

//--------------------------------------------------------------
void ofApp::update()
{
	mRSSDK->update();

	mCloudMesh.clear();
	mCloudMesh.setMode(OF_PRIMITIVE_POINTS);
	mCloudMesh.enableColors();

	vector<glm::vec3> pointCloud = mRSSDK->getPointCloud();

	cout << "point cloud size: " << pointCloud.size() << endl;

	float firstTime = ofGetElapsedTimef();

	//mCloudMesh.addVertices(pointCloud);

	// copies pointCloud into vertices (there must be a faster way...)
	mCloudMesh.getVertices() = pointCloud;
	
	float lastTime = ofGetElapsedTimef();

	cout << "create mesh elapsed time " << lastTime - firstTime << endl;

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofClear(ofColor::black);
	ofSetColor(ofColor::white);

	//ofTexture tex;
	//tex.loadData(mRSSDK->getRgbFrame());
	//tex.loadData(mRSSDK->getDepthFrame());
	//tex.draw(0, 0);

	//mRSSDK->draw(1280, 720, 10., 0., 0., tex);


	mCamera.begin(); 

	ofPushMatrix();
	ofScale(100, 100, 100);
	mCloudMesh.draw();
	ofPopMatrix();

	mCamera.end();
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
