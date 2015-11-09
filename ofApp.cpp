#include "ofApp.h"
#include <math.h>

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetLogLevel(OF_LOG_SILENT);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();

	kinect.open(); // opens first available kinect

	fgCol.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR_ALPHA);
	plate.allocate(kinect.width, kinect.height);
	fgClip.allocate(kinect.width, kinect.height);
	bgClip.allocate(kinect.width, kinect.height);

	threshold = 164;
	first = true;
	drawGhost = true;
	updateGhost = true;
	drawPlates = false;
	goodBackground = true;
	drawTrails = false;

	ofSetFrameRate(60);

	kinect.update();

	plate.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
	fgClip.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	bgClip.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

	plate.flagImageChanged();
	fgClip.flagImageChanged();
	bgClip.flagImageChanged();

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(0);
	ofSleepMillis(1000);
}

//--------------------------------------------------------------
void ofApp::update()
{

	ofBackground(0);

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		ofxCvColorImage temp_plate = ofxCvColorImage();
		ofxCvGrayscaleImage temp_bgClip = ofxCvGrayscaleImage();

		temp_plate.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
		temp_bgClip.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		// or we do it ourselves - show people how they can work with the pixels
		unsigned char * bg_pix = bgClip.getPixels();
		unsigned char * fg_pix = fgClip.getPixels();
		unsigned char * bg_col = plate.getPixels();
		unsigned char * fg_col = fgCol.getPixels();
		unsigned char * tm_col = temp_plate.getPixels();
		unsigned char * tm_pix = temp_bgClip.getPixels();

		int numPixels = temp_bgClip.getWidth() * temp_bgClip.getHeight();
		for(int i = 0; i < numPixels; i++) {
			// on the first run, just set it all to the image
			if (first) {
				bg_pix[i] = tm_pix[i];
				bg_col[(i*3) + 0] = tm_col[(i*3) + 0];
				bg_col[(i*3) + 1] = tm_col[(i*3) + 1];
				bg_col[(i*3) + 2] = tm_col[(i*3) + 2];
				continue;
			}

			// peaks and out-of-range
			if (!tm_pix[i])
				tm_pix[i] = threshold;

			// handle the foreground plate
			if (updateGhost) {
			fg_pix[i] = 0;
			if (tm_pix[i] > threshold) {
				// make visible the foreground parts
				fg_col[(i*4) + 0] = tm_col[(i*3) + 0];
				fg_col[(i*4) + 1] = tm_col[(i*3) + 1];
				fg_col[(i*4) + 2] = tm_col[(i*3) + 2];
				fg_col[(i*4) + 3] = 
					(unsigned char) (((float) (tm_pix[i] - threshold) / (255. - (float) threshold)) * 255);
				fg_pix[i] = 255;
			} else {
				// make invisible the background parts
				if (i > 1 && i > kinect.width && i < numPixels - kinect.width &&
				   (tm_pix[i+1] > threshold ||
					tm_pix[i-1] > threshold ||
					tm_pix[i+kinect.width] > threshold ||
					tm_pix[i-kinect.width] > threshold ||
					tm_pix[i-kinect.width-1] > threshold ||
					tm_pix[i-kinect.width+1] > threshold ||
					tm_pix[i+kinect.width-1] > threshold ||
					tm_pix[i+kinect.width+1] > threshold) ) {
					fg_col[(i*4) + 0] = tm_col[(i*3) + 0];
					fg_col[(i*4) + 1] = tm_col[(i*3) + 1];
					fg_col[(i*4) + 2] = tm_col[(i*3) + 2];
					fg_col[(i*4) + 3] = 20;
				} else if (drawTrails) {
					fg_col[(i*4) + 0] = max(0, fg_col[(i*4) + 0] - 2);
					fg_col[(i*4) + 1] = max(0, fg_col[(i*4) + 1] - 2);
					fg_col[(i*4) + 2] = max(0, fg_col[(i*4) + 2] - 2);
					fg_col[(i*4) + 3] = max(0, fg_col[(i*4) + 3] - 2);
				} else {
					fg_col[(i*4) + 0] = 0;
					fg_col[(i*4) + 1] = 0;
					fg_col[(i*4) + 2] = 0;
					fg_col[(i*4) + 3] = 0;
				}
			}
			}

			// handle the background plate
			if ((tm_pix[i] < bg_pix[i] && bg_pix[i] - tm_pix[i] > 24) ||
				!bg_pix[i]) {
				bg_col[(i*3) + 0] = tm_col[(i*3) + 0];
				bg_col[(i*3) + 1] = tm_col[(i*3) + 1];
				bg_col[(i*3) + 2] = tm_col[(i*3) + 2];
				
				if (bg_pix && bg_pix[i] - tm_pix[i] < 2) {
					int x,y;
					x = i % kinect.width;
					y = i / kinect.height;
					if (x > 1 && x < kinect.width - 2) {
						bg_pix[i] = (bg_pix[i-1] +
									 bg_pix[i-2] +
									 bg_pix[i+1] +
									 bg_pix[i+2] +
									(4 * tm_pix[i])) / 8;
					}
					if (y > 1 && y < kinect.height - 2) {
						bg_pix[i] = (bg_pix[i-kinect.width] +
									 bg_pix[i-kinect.width*2] +
									 bg_pix[i+kinect.width] +
									 bg_pix[i+kinect.width*2] +
									(4 * tm_pix[i])) / 8;
					}
				} else {
					bg_pix[i] = tm_pix[i];
				}
			} else {
				if (goodBackground) {
					if (!fg_pix[i]) { 
						bg_col[(i*3) + 0] = tm_col[(i*3) + 0];
						bg_col[(i*3) + 1] = tm_col[(i*3) + 1];
						bg_col[(i*3) + 2] = tm_col[(i*3) + 2];
					}
				} else {
					if (((abs(bg_col[(i*3) + 0] - tm_col[(i*3) + 0]) > 24 ||
						abs(bg_col[(i*3) + 1] - tm_col[(i*3) + 1]) > 24 ||
						abs(bg_col[(i*3) + 2] - tm_col[(i*3) + 2]) > 24   ) &&
						(tm_pix[i] < threshold/2 && tm_pix[i] - bg_pix[i] <= 24 &&
						!fg_pix[i]) ) || (abs(tm_pix[i] - bg_pix[i]) < 4)) { 
						bg_col[(i*3) + 0] = tm_col[(i*3) + 0];
						bg_col[(i*3) + 1] = tm_col[(i*3) + 1];
						bg_col[(i*3) + 2] = tm_col[(i*3) + 2];
					}

				}
			}
		}

		// update the cv images
		fgCol.update();
		plate.flagImageChanged();
		fgClip.flagImageChanged();
		bgClip.flagImageChanged();
		if (first)
			first = false;
		// updateGhost = !updateGhost;
		updateGhost = true;
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{

	ofSetColor(255);
	plate.draw(940, -63, -1024, 768);

if (drawGhost) {
	ofSetColor(0,200,255,128);	
	fgCol.draw(940, -63, -1024, 768);		
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	fgCol.draw(940, -63, -1024, 768);		
	ofEnableBlendMode(OF_BLENDMODE_ALPHA);
}

if (drawPlates) {
	ofSetColor(255);
	bgClip.draw(0,0,200,150);
	plate.draw(200,0,200,150);
	fgClip.draw(0,150,200,150);
	fgCol.draw(200,150,200,150);
}

	ofSetColor(255);
	ofDrawBitmapString("g - toggle foreground | p - toggle plates |  b - switch plate method | t - draw trails", 20, 690);

}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key)
{
	if (key == 'g') drawGhost = !drawGhost;
	else if (key == 'p') drawPlates = !drawPlates;
	else if (key == 'b') goodBackground = !goodBackground;
	else if (key == 't') drawTrails = !drawTrails;
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
