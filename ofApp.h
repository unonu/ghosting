#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

class ofApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	// void drawPointCloud();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	ofxKinect kinect;

	ofImage fgCol; // foreground image
	ofxCvColorImage plate; // background plate
	ofxCvGrayscaleImage fgClip;
	ofxCvGrayscaleImage bgClip;

	ofFbo ghostBuffer;

	bool first;
	bool drawGhost;
	bool updateGhost;
	bool drawPlates;
	bool goodBackground;
	bool drawTrails;

	unsigned char threshold;
	int angle;

	char** averagePoints;

	// used for viewing the point cloud
	ofEasyCam easyCam;
};
