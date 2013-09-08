#pragma once

#include "ofMain.h"
#include "ofx3DModelLoader.h"
#include "ofxGstVideoRecorder.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxV4LControls.h"
#include "ofxSimpleGuiToo.h"
#include "ofxKinectBlobFinder.h"
#include "ofxKinectBlobTracker.h"
#include "ofxIpCamera.h"
#include "ofxWidowX.h"
#include "GL/glut.h"
#include "./BGS/AdvancedBGS.hpp"

#define _OFXGRABCAM

#ifdef _OFXGRABCAM
    #include "ofxGrabCam.h"
#endif

#define _LINUX

//#define _SERVOS
//#define _WIDOWX
//#define _VIVOTEK
//#define _VIDEO



//class testApp : public ofBaseApp {
class testApp : public ofBaseApp, public ofxKinectBlobListener {
	public:
		void setup();
		void update();
		void draw();
		void exit();
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void setupGui();

		void drawPointCloud();
		void draw2Dgui();
        void drawReport();
        void draw3Dgui();
        void drawRoomModel();
        void drawBlobs();
        void drawTracking();

        void auto3Dflight();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
        void calcAvgFPS();
        ofVec3f getKinectRot();
        void updateBGS();
        void updateServos(int pan, int tilt);
     //   void updatePC();

		void drawGlWireBox(ofVec3f minPoint, ofVec3f maxPoint, bool centerCross);
		void drawGlBox(float width, float height, float depth, char solid);
		void setGLlighting(bool enable);
        ofVec3f calcAvgAccel();
        void drawRect(float * dest, float * src2dest, int dw, int dh, int dx, int dy, int side2, float value, int srcIdx  );
        void draw3Daxis(ofPoint tran, ofPoint rot, float axisLen);
        void moveServo(int id, int Ms);
        void updatePanTilt();
		void blobOn( ofVec3f centroid, int id, int order );
        void blobMoved( ofVec3f centroid, int id, int order);
        void blobOff( ofVec3f centroid, int id, int order );
        ofVec3f calcDirection(int id, vector <ofVec3f> &trajectory);
        void moveVivotekXY(float pan, float tilt);
        void setupRecording();

#ifdef _SERVOS
        ofSerial	    serial;
#elif defined _WIDOWX
		ofxWidowX       turret;
#elif defined _VIVOTEK
        ofxIpCamera	    vivotek;
#endif

		ofx3DModelLoader roomModel;

        ofTrueTypeFont font;

        int kw, kh;
		ofxKinect kinect;

        ofVideoGrabber 		vidGrabber;
        ofxGstVideoRecorder     recorder;

#ifdef _OFXGRABCAM
        ofxGrabCam          virtualCamera;
#else
        ofEasyCam          virtualCamera;
#endif
        ofCamera            ofCam;

        // for advanced bgs
        RgbImage                rgbBgImage;
        // for opencv bgs
        ofxCvGrayscaleImage 	depthImage;
		ofxCvGrayscaleImage 	depthThresh;
		ofxCvGrayscaleImage 	depthThreshFar;
		ofxCvGrayscaleImage     bgImage;
        ofxCvGrayscaleImage 	grayDiff;
        // for KinectBlobTracker
        ofImage                 grayDiffOfImage;
        ofImage                 window_img;
        ofImage                 imgLogos;

        ofVec3f             *p3DCloud;

        int                 *kinect2proj;

		//ofxCvContourFinder 	contourFinder;


		bool				bAuto3DFlightMode;
		bool				bDrawPointCloud;
		bool                bDraw3D;
		bool				bLearnBakground;
		bool                bUpdateControls;
		bool                bLoadFromXML;
		bool                bSetDefaults;
		bool                bUpsideDown;

		bool                bRecording;
        bool                bShowVideo;
		bool                bMoveTurret;
		bool                bFullScreen;
		bool                bSerialInited;
		bool                bUpdateKinectRotTran;
		bool                bAdvancedBGS;

		float 				nearThreshold;
		float			    farThreshold;
		float               bgsThreshold;

		ofVec3f             threshMinXYZ;
        ofVec3f             threshMaxXYZ;

        int minBlobPoints;
        int winnerBlob;


		int					kMotorAngle;
		float               trnX,trnY,trnZ;
        float screenFPS;

	//	ofRectangle         centroids[MAX_BLOBS];

//        blob3D *            blobs3D;

//        vector<blobMesh*>    my_blobs;

        ofxKinectBlobFinder  blobFinder;
        ofxKinectBlobTracker blobTracker;


#ifdef _LINUX
        //stuff for the linux camera controls
        ControlList             ctls;
        int hdevice;
#endif



		Algorithms::BackgroundSubtraction::ZivkovicParams Z_params;
		Algorithms::BackgroundSubtraction::ZivkovicAGMM Z_bgs;

		bgs_struct         bgs;//bgsArray[7];
		int                 currFrameNum;




};
