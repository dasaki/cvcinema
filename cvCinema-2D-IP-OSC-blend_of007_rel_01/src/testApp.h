#pragma once
#include "ofMain.h"


#include "./BGS/AdvancedBGS.hpp"
#include "ofxOpenCv.h"
#include "ofxGstVideoRecorder.h"
//#include "ofxSimpleGuiToo.h"
#include "ofxGui.h"
#include "ofxV4L2Settings.h"
#include "ofxXmlSettings.h"

#define OFX_OSC
#ifdef OFX_OSC
    #include "ofxOsc.h"
#endif

#define OFX_FENSTER
#ifdef OFX_FENSTER
   #include "ofxFensterManager.h"
#endif

#define OFX_IP_VIDEO_GRABBER
#ifdef OFX_IP_VIDEO_GRABBER
    #include "ofxIpVideoGrabber.h"
#else
    #include "ofxIpCamera.h"
#endif

// listen on port 12345
#define PORT 12345
#define NUM_MSG_STRINGS 20
// ip cameras stuff
#define NUM_IP_CAMERAS 3
#define NUM_CAMERAS 4
#define NUM_ROWS 2
#define NUM_COLS 2

#ifdef OFX_FENSTER
class outputWindowListener: public ofxFensterListener {
public:
	~outputWindowListener() {
		cout << "Render screen destroyed" << endl;
	}
	void setup() {
//	    fbo.allocate(1024,768);
	}
	void draw(ofxFenster* f) {
        int winWidth = f->getWidth();
        //int winHeight = f->getHeight();
        if (img) img->draw(0,0,winWidth, (float)winWidth/imgAspect);
	}

	void keyReleased(int key, ofxFenster* window) {
		switch (key) {
            case ' ': ofxFensterManager::get()->deleteFenster(window); break;
            case 'f':  window->toggleFullscreen();
                    break;
            default: break;
        }
	}

    void setImage(ofxCvColorImage *newImage) {
        img = newImage;
        imgAspect = img->getWidth()/img->getHeight();
	}

	ofxCvColorImage *img;
	float            imgAspect;
};
#endif

class IPCameraDef {
public:
    IPCameraDef() {    };
    IPCameraDef(string _uri) { url = _uri; }
    IPCameraDef(string _name, string _uri, string _username, string _password) {
        name = _name;
        url = _uri;
        username = _username;
        password = _password;
    }

    string name;
    string url;
    string username;
    string password;
};

enum outputSelection { OUTPUT_IMAGE=0, ANALYSIS_WINDOW=1, BG_IMAGE=2,
                       FOUR_WINDOWS = 3, INPUT_IMAGE = 4 };

class testApp : public ofBaseApp{

	public:
		struct bgs_struct {
            Algorithms::BackgroundSubtraction::Bgs          *bgsPtr;
            Algorithms::BackgroundSubtraction::BgsParams    *paramsPtr;
            int                                             On;
            string                                          name;
            BwImage                                         low;
            BwImage                                         high;
        };

        void setup();
		void update();
		void draw();
        void exit();

        void multiC();
		void monoCmonoB();
		void monoCmultiB();

        void copyRegion(ofxCvColorImage &src, int &sx, int &sy, int &sw, int &sh,
                         ofxCvColorImage &dst, int &dx, int &dy, int &dw, int &dh);

        void DrawIplImage(IplImage *image, int x, int y, GLfloat xZoom, GLfloat yZoom);
        void DrawBGR(int algNum,bgs_struct *bgs, int x, int y);

		void setupGui();
		void setupAnalisys(int width, int height);
		bool setupLiveVideo();
		void loadMovie();

		void setupOSC();
        void updateOSC();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawReport(int xPos, int yPos, int lineH);
		void drawMainScreen();
		void drawFullScreen();
		void drawBlending(int xpos, int ypos, int w, int h);
		void drawBlendSources(int xpos, int ypos, int w, int h);
		void updateBlendImage();
		void updateIpImages();
		bool cvFilterCartoon(ofxCvColorImage &src, ofxCvColorImage &dst, int w, int h);
		void setupRecording();
		void updateCvCinema();
		void openOutWin();
		void closeOutWin();

        #ifdef OFX_FENSTER
        outputWindowListener    outWinListener;
        ofxFenster*             outWin;
        #endif

        ofVideoGrabber 		vidGrabber;
        ofVideoPlayer 		vidPlayer;

        ofxPanel gui;
        ofxV4L2Settings settings;
        ofxGstVideoRecorder     recorder;

        ofxCvColorImage         inputImg;
        ofxCvColorImage         outputImg;
        ofxCvColorImage         blendImg;
        ofxCvColorImage         ipImg[NUM_IP_CAMERAS];
        ofxCvColorImage         fullFrame;
        ofxCvColorImage		    colorImg;
        ofxCvGrayscaleImage 	grayImage;
        ofxCvColorImage 	    lastFrame;
		ofxCvGrayscaleImage 	lastFrameGray;
		ofxCvColorImage 	    bgImage;
		ofxCvGrayscaleImage 	grayDiff;
        ofxCvGrayscaleImage 	grayTemp;
        ofxCvGrayscaleImage 	grayBgImage;
        ofxCvColorImage 	    colorDiff;

        ofImage                 img;
        ofImage                 imgLogos;

        ofxCvContourFinder 	contourFinder;

		int 				threshold;
		bool				bLearnBakground;
		bool                saveFrame;
		bool                bRecording;
		bool                bFullScreen;
		bool                bShowGui;
		bool                bMovieAudio;
		bool                bLiveVideo;
		bool                bLoadDefaults;

		int                 algorithm;
		char                cropping;
		bool                bZoomTarget;

        int                 inW;
        int                 inH;
        float               inAspect;
        float               outAspect;
        int                 out_H_in_aspect;
        float               in_out_scale;
        float               in_draw_scale;
        float               analysis_out_scale;
        float               in_analysis_scale;

        int                 analysisW;
        int                 analysisH;
        int                 cropW;
        int                 cropH;
        int                 in_H_gap;
        int                 out_H_gap;
        int                 draw_H_gap;

        int                 left;
        int                 top;
        int                 targW;
        int                 targH;
        int                 cols;
		int                 rows;

		outputSelection     selOutput;

		int                 currFrameNum;
		float               videoFPS;
		int                 lastTime;
		vector<ofxCvBlob>   lastBlobs;

		Algorithms::BackgroundSubtraction::ZivkovicParams Z_params;
		Algorithms::BackgroundSubtraction::ZivkovicAGMM Z_bgs;

		bgs_struct         bgs;//bgsArray[7];


        // IP CAMERAS STUFF
        void loadCameras();
        IPCameraDef& getNextCamera();
        vector<IPCameraDef> ipcams; // a list of IPCameras
        int nextCamera;

#ifdef OFX_IP_VIDEO_GRABBER
		// This message occurs when the incoming video stream image size changes.
        // This can happen if the IPCamera has a single broadcast state (some cheaper IPCams do this)
        // and that broadcast size is changed by another user.
        void videoResized(const void * sender, ofResizeEventArgs& arg);
        vector< ofxSharedIpVideoGrabber > ipGrabber;
#else
        typedef ofPtr< ofxIpCamera > ofxSharedIpCamera;
        vector< ofxSharedIpCamera > ipGrabber;
#endif
        // OSC / blend messages stuff
        ofBlendMode     blendMode;
        char          blendString[255];
        int           alphaValues[NUM_CAMERAS];
#ifdef OFX_OSC
        ofxOscReceiver receiver;
#endif
};

