#ifndef __OFAPP_H__
#define __OFAPP_H__
#include <vector>
#include "ofMain.h"
#include "ofxRSSDKv2.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API 

using namespace std;
using namespace ofxRSSDK;

class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
	void draw();
	void exit();

	void ofApp::draw_pointcloud(float width, float height, rs2::points& points);

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

private:
	void setupCamera();
	
	RSDevicePtr mRSSDK;	
	ofMesh			mCloudMesh;
	ofEasyCam		mCamera;
};

#endif
