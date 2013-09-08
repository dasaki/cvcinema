#include "testApp.h"

#define FPS_MEAN        30
#define MAX_DEPTH       5.1
#define K_VFOV          43
#define K_VFOV_2_RAD    0.375245789f
#define K_HFOV          57
#define K_HFOV_2_RAD    0.497418837f
#define ACC_HIS_LEN     100

#define FHH             farThreshold/cosf(K_HFOV_2_RAD)
#define FVH             farThreshold/cosf(K_VFOV_2_RAD)
#define K_FAR           ofVec3f(FHH*sinf(K_HFOV_2_RAD),farThreshold,FVH*sinf(K_VFOV_2_RAD))
#define NHH             nearThreshold/cosf(K_HFOV_2_RAD)
#define NVH             nearThreshold/cosf(K_VFOV_2_RAD)
#define K_NEAR          ofVec3f(NHH*sinf(K_HFOV_2_RAD),nearThreshold,NVH*sinf(K_VFOV_2_RAD))

#define BG_POINT        ofVec3f(-100000,-100000,-100000)

#define KW              640
#define KH              480
#define SW              160
#define SH              120
#define S_LEN           SW*SH
#define DOWNSAMPLING    KW/SW

#define ERODE           4
#define DILATE          2


#define DRAW_W          100
#define DRAW_H          DRAW_W*SH/SW
#define DRAW_GAP        10
#define SCALE_3D_VIEW   100.0
#define AXIS_LEN        0.5

#define KIN_3D_W        0.05
#define KIN_3D_H        0.05
#define KIN_3D_L        0.3
#define CAM_3D_SPH_RAD  0.035
#define CAM_3D_BASE     0.1
#define CAM_3D_CONE_H   0.15
#define CAM_3D_CONE_BASE 0.1

#define MAX_BLOBS       10

#define VIVOTEK_IMAGE_W     720
#define VIVOTEK_IMAGE_H     576
//-------- auto 3D flight stuff ----------
#define VIRT_CAM_MIN_DIST    1.0
#define VIRT_CAM_DIST_FACTOR 1.0
#define VIRT_CAM_A_RATE      0.2 //radians per second
#define VIRT_CAM_B_RATE      0.1 //radians per second
#define VIRT_CAM_Z_OFFSET    -0.3 //radians per second


#define FONT_COLOR          0xffffff


float virtCamAlpha = 0.0;
float virtCamBeta = 0.0;

//----------------------------------------
unsigned char contact;
unsigned int MsX = 1500;
unsigned int MsY = 1500;

int numBlobs = 0;
int recorderFileNumber = 0;

ofVec3f kinectAvgAccel;
ofVec3f kinectAccelHistory[ACC_HIS_LEN];
ofVec3f kRot       = ofPoint(0,0,0);
ofVec3f kTran      = ofPoint(0,0,2.025f);//ofPoint(-0.85f,1.55f,-3.025f);
ofVec3f ptzRot     = ofPoint(0,0,0);;
ofVec3f ptzTran    = ofPoint(0.6,0.4,1.0f);
ofVec3f ptzPointer = ofPoint(0,0,0);

float tiltAngle, panAngle;

float kFPS = 0;
float avgkFPS = 0;
int frameCount = 0;
int lastMillis = 0;
int timeLapse = 0;
int numPixels = 0;
int lastReqTime =  0;

string devfile = "/dev/video1";

float prevX = 0;

/************************************************************
                            SETUP
*************************************************************/
void testApp::setup()
{
    imgLogos.loadImage("faldon-logos.bmp");

//    ofSetFrameRate(60);
    ofSetVerticalSync(true);
        //-------------- 3D model loader
    roomModel.loadModel("sala_malaga.3ds", 0.0254);
    roomModel.setRotation(0, 90, 0, 0, 1);
  //  roomModel.setRotation(1, 270, 0, 0, 1);
   // roomModel.setScale(0.9, 0.9, 0.9);
    roomModel.setPosition(0,0, 0);

    ofSetVerticalSync(true);
    ofSetFrameRate(30);


    ofCam.setFarClip(1000000.0);
    ofCam.setNearClip(0.0);


#ifdef _OFXGRABCAM
    virtualCamera.setFixUpwards(false);
#endif

    //------------------------------------
#ifdef _VIDEO

    #ifdef _VIVOTEK
        vivotek.setVerbose(true);
        vivotek.init(VIVOTEK_IMAGE_W, VIVOTEK_IMAGE_H);
        vivotek.setUri("http://192.168.1.30:80/video2.mjpg");
        vivotek.setCredentials("", "");
    #else
        #ifdef _LINUX
                hdevice = OpenVideoDevice(devfile.c_str());
        #endif
        vidGrabber.listDevices();
        vidGrabber.setVerbose(true);
        vidGrabber.setDeviceID(1);
        vidGrabber.setDesiredFrameRate(30);
        vidGrabber.initGrabber(640,480);
    #endif
#endif

	// enable depth->rgb image calibration
	kinect.setRegistration(true);
    //
    /// initialize resources, must be called before open()
	/// infrared controls whether the video image is rgb or IR
	/// set video to false to disable video image grabbing (saves bandwidth)
	/// set texture to false if you don't need to use the internal textures
	///
	/// naturally, if you disable the video image the video pixels and
	/// RGB color will be 0
	/// bool init(bool infrared=false, bool video=true, bool texture=true);


	bgsThreshold = 1;
	nearThreshold = 0.5f;
    farThreshold  = 12.00f; // meters

    kinect.init();
    kinect.setDepthClipping(nearThreshold*1000,farThreshold*1000);
	//kinect.setVerbose(true);
	kinect.open();

    bUpsideDown = false;

    // bind our kinect to the blob finder
    blobFinder.init(&kinect, true, bUpsideDown); // standarized coordinate system: z in the direction of gravity
    blobFinder.setResolution(BF_MEDIUM_RES);//BF_LOW_RES);
    blobFinder.setRotation(kRot);
    blobFinder.setTranslation(kTran);
    blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters

    blobTracker.loadFont("PerfectDOSVGA437.ttf");

    float sqrResolution = blobFinder.getResolution();
    sqrResolution *= sqrResolution;
    minBlobPoints = (int)(0.002*(float)numPixels/sqrResolution);


	depthImage.allocate(KW, KH);
	depthThresh.allocate(SW, SH);
	depthThreshFar.allocate(SW, SH);
	bgImage.allocate(KW, KH);
    grayDiff.allocate(KW, KH);
    grayDiffOfImage.allocate(KW, KH, OF_IMAGE_GRAYSCALE);

    rgbBgImage = cvCreateImage(cvSize(SW,SH), IPL_DEPTH_8U, 3 );

    //----------- advanced BGS
    // For ZivkovicAGMM ----------------------------
        bgs.bgsPtr = &Z_bgs;
        bgs.paramsPtr = &Z_params;
        Z_params.SetFrameSize(SW, SH);
        Z_params.LowThreshold() = 40;// default 5.0f*5.0f;
        Z_params.HighThreshold() = 2*Z_params.LowThreshold();
        Z_params.Alpha() = 0.001f;// default 0.001f;
        Z_params.MaxModes() = 3;// default 3
        Z_bgs.Initalize(Z_params);
        bgs.low = cvCreateImage(cvSize(SW, SH),IPL_DEPTH_8U,1);
        bgs.high = cvCreateImage(cvSize(SW, SH),IPL_DEPTH_8U,1);
//----------- advanced BGS
    // For ZivkovicAGMM ----------------------------

    threshMinXYZ = ofVec3f(-5, 0, 0);
    threshMaxXYZ = ofVec3f( 5, 5, kTran.z);

    numPixels = KW*KH;

    bRecording = false;
    bAuto3DFlightMode = false;
    bFullScreen = false;
	bLearnBakground = true;
	bShowVideo = false;
	bUpdateKinectRotTran = true;
	bMoveTurret = false;
    bDraw3D = true;
    bDrawPointCloud = true;
    bLoadFromXML = true;
    bUpdateControls = false;
    bSetDefaults = false;
    bAdvancedBGS = false;
    winnerBlob = -1;
    numBlobs = 0;
	kMotorAngle = 0;
	currFrameNum = 0;
	kMotorAngle = kinect.getTargetCameraTiltAngle();


#ifdef _SERVOS
	serial.enumerateDevices();
	bSerialInited = serial.setup("/dev/ttyUSB0",115200);
    contact = ' ';
#elif defined _WIDOWX
    turret.setup("/dev/ttyUSB0");
#elif defined _VIVOTEK
    vivotek.init(VIVOTEK_IMAGE_W, VIVOTEK_IMAGE_H);
    vivotek.setUri("http://192.168.1.30:80/video2.mjpg");
    vivotek.setCredentials("", "");
#endif
	font.loadFont("PerfectDOSVGA437.ttf",15);

    setupGui();
    bUpdateControls = true;



    timeLapse = 3000+ofGetElapsedTimeMillis();

    blobTracker.setListener( this );


}
/*********************************************************************

                               SETUP GUI

*********************************************************************/
void testApp::setupGui() {
        gui.setDraw(false);
    gui.setAutoSave(false);
    // 'gui' is a global variable declared in ofxSimpleGuiToo.h
    gui.addTitle("KINECT");
    // SETTINGS GUI
    gui.addSlider("Kinect Rot X", kRot.x, -180, 180);
    gui.addSlider("Kinect Rot Y", kRot.y, -180, 180);
    gui.addSlider("Kinect Rot Z", kRot.z, -180, 180);

	gui.addSlider("Kinect Tran X", kTran.x, -20, 20);
    gui.addSlider("Kinect Tran Y", kTran.y, -20, 20);
    gui.addSlider("Kinect Tran Z", kTran.z, -20, 20);

	gui.addSlider("PTZ Tran X", ptzTran.x, -20, 20);
    gui.addSlider("PTZ Tran Y", ptzTran.y, -20, 20);
    gui.addSlider("PTZ Tran Z", ptzTran.z, -20, 20);

    #ifdef _VIDEO
        gui.addPage("CAMERA");
        gui.addTitle("CAMERA");
        #ifdef _LINUX
         //***********************
            // Setup linux camera controls on gui. See:
            // http://v4l2spec.bytesex.org/spec/r10386.htm#CTRL-CLASS
            //************************
        if (hdevice < 0) fprintf( stderr, "Could not open device: %s\n",
                                  devfile.c_str());
         else {
             // get the controls list from the camera device, so we can add it to our control gui
            ListControls(hdevice, ctls);
            for(unsigned int i = 0; i < ctls.size(); i++)
            {
                 switch ( ctls[i].control.type ) {
                       case V4L2_CTRL_TYPE_INTEGER :  {
                           int*  temp = (int*)(&(ctls[i].value));
                           gui.addSlider( (char *)(ctls[i].control.name),
                                          *temp, ctls[i].control.minimum, ctls[i].control.maximum);
                              } break;
                       case V4L2_CTRL_TYPE_BOOLEAN :  {
                           bool*  temp = (bool*)(&(ctls[i].value));
                           gui.addToggle((char *)(ctls[i].control.name), *temp);
                            }break;
                       case V4L2_CTRL_TYPE_MENU : {
                            int len = ctls[i].control.maximum-ctls[i].control.minimum+1;
                            string titleArray[len];
                            string tempStr;
                            for ( int j = 0; j < len; j++) {
                                tempStr = (char *)(ctls[i].menu[j].name);
                                titleArray[j] = tempStr;
                            }
                            int*  temp = (int*)(&(ctls[i].value));
                            gui.addComboBox( (char *)(ctls[i].control.name), *temp, len,  titleArray);

                            }break;
                        case V4L2_CTRL_TYPE_BUTTON : {
                            bool*  temp = (bool*)(&(ctls[i].value));
                            gui.addButton((char *)(ctls[i].control.name), *temp);
                            }break;
                        case V4L2_CTRL_TYPE_INTEGER64 :
                            break;
                        case V4L2_CTRL_TYPE_CTRL_CLASS :
                            break;
                        default : ;
                 }
            }
        }
        #endif
        gui.addButton("Update Controls", bUpdateControls);
        gui.addButton("Set Defaults", bSetDefaults);
        gui.addContent("Camera feed", vidGrabber);
    #else
//        gui.addContent("Video feed", vidPlayer);
    #endif


}
/************************************************************
                            UPDATE
*************************************************************/
void testApp::update()
{
    if (bLoadFromXML) {
	    gui.loadFromXML();
	    bUpdateControls = true;
        bLoadFromXML = false;
	}
    kinect.update();
    if (kinect.isFrameNew()) {
        // set current frame number
        calcAvgFPS();
        // if no preset loaded from xml the get kinect's
        // rotation from its internal accelerometers
        if (kRot.length() == 0) {
            kRot = getKinectRot();
       //     bUpdateKinectRotTran = true;
        }
     //   if (bUpdateKinectRotTran)  {
            blobFinder.setRotation(kRot);
            blobFinder.setTranslation(kTran);
 //           bUpdateKinectRotTran = false;
    //    }

        updateBGS();

        grayDiffOfImage.setFromPixels(grayDiff.getPixels(), KW, KH, OF_IMAGE_GRAYSCALE);

        blobFinder.findBlobs( &grayDiffOfImage,
                           ofVec3f(-100.0, -100.0, -100.0), ofVec3f(100.0, 100.0, 100.0),
                           ofPoint(0.2,0.2,0.3), 1,
                           0.075f, 2.0f, 50, MAX_BLOBS);
    blobTracker.trackBlobs( blobFinder.blobs );

        updatePanTilt();
    }

#ifdef _VIDEO
    #ifdef _VIVOTEK
        vivotek.update();
    #else
        vidGrabber.update();
    #endif
	#ifdef _LINUX
	if (bSetDefaults) {
        for(unsigned int i = 0; i < ctls.size(); i++) {
            ctls[i].value = ctls[i].control.default_value;
        }
        bSetDefaults = false;
        bUpdateControls = true;
	}
	#endif
    if (bUpdateControls) {
        // camera controls
        #ifdef _LINUX
        if (hdevice < 0) {
            fprintf(stderr, "Could not open device: %s\n", devfile.c_str());
        }
        else {
            SetControlsList(hdevice, ctls);
        }
        #endif
        bUpdateControls = false;
	}
#endif
}
/************************************************************
                CALCULATE AVERAGE FPS
*************************************************************/
void testApp::calcAvgFPS() {
    int currMillis = ofGetElapsedTimeMillis();
    avgkFPS += (1000.0/(currMillis-lastMillis))/FPS_MEAN;
    lastMillis = currMillis;
    frameCount++;
    if (frameCount >= FPS_MEAN) {
        kFPS = avgkFPS;
        avgkFPS = frameCount =  0;
    }
}
/************************************************************
                    UPDATE KINECT ANGLE
*************************************************************/
ofVec3f testApp::getKinectRot() {
    ofVec3f accAngle = kinect.getMksAccel();

   /* kRot = ofPoint( ofRadToDeg(asin(accAngle.z/9.8)),
                    0,//ofRadToDeg(asin(accAngle.x/9.8)),
                    0);//ofRadToDeg(asin(kinect.getMksAccel().y/9.8))
  */
    ofVec3f tempRot = ofPoint( ofRadToDeg(asin(accAngle.z/9.8)),
                    kRot.y,//ofRadToDeg(asin(accAngle.x/9.8)),
                    kRot.z);//ofRadToDeg(asin(kinect.getMksAccel().y/9.8))
    return tempRot;
    /*kRot = ofPoint( accAngle.angle( ofVec3f(0,  accAngle.y,  accAngle.z)),
                    accAngle.angle( ofVec3f( accAngle.x, 0,  accAngle.z)),
                    accAngle.angle( ofVec3f( accAngle.x,  accAngle.y, 0)) );
    */
}

/// ************************************************************
///                UPDATE BACKGROUND SUBTRACTION
/// ************************************************************
void testApp::updateBGS() {


    // for the advancedBGS
 /*   currFrameNum++;
    unsigned char * rgbPix = (unsigned char *)rgbBgImage.Ptr()->imageData;
    unsigned char * gPix = (unsigned char*)depthImage.getCvImage()->imageData;
    unsigned char * dPix  = (unsigned char*)kinect.getDepthPixels();

    int row_incr = KW*(DOWNSAMPLING-1);
    for (int j = 0; j < KH; j+=DOWNSAMPLING) {
        for (int i = 0; i < KW; i+=DOWNSAMPLING) {
            *gPix = *dPix;
            gPix++;
            dPix += DOWNSAMPLING;
        }
        dPix += row_incr;
    }
    depthImage.flagImageChanged();

    for (int i = 0; i < S_LEN; i++) {
        *rgbPix = *gPix;
        rgbPix++;
        *rgbPix = *gPix;
        rgbPix++;
        *rgbPix = *gPix;
        rgbPix++;
        gPix++;
    }
*/

    depthImage.setFromPixels(kinect.getDepthPixels(), KW,KH);
    if (bLearnBakground){
        if (ofGetElapsedTimeMillis() >= timeLapse) {
            bgImage = depthImage;// let this frame be the background image from now on
            bgs.bgsPtr->InitModel(rgbBgImage);
            bLearnBakground = false;
        }
    }
    else {
        // Subtract the current frame from the background model and produce a binary foreground mask using
        // both a low and high threshold value.
        bgs.bgsPtr->Subtract(currFrameNum, rgbBgImage, bgs.low, bgs.high);
        // Update the background model. Only pixels set to background in update_mask are updated.
        bgs.bgsPtr->Update(currFrameNum, rgbBgImage, bgs.low );
    }

    if (bAdvancedBGS) {
        bgImage.setFromPixels((unsigned char *)(bgs.bgsPtr->Background()->Ptr()->imageData),SW,SH);
        grayDiff.setFromPixels((unsigned char *)bgs.low.Ptr()->imageData,SW,SH);
    }
    else {
        cvAbsDiff(bgImage.getCvImage(), depthImage.getCvImage(), grayDiff.getCvImage());
    }

    cvErode(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, ERODE);
    cvDilate(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, DILATE);
    // threshold ignoring little differences
    cvThreshold(grayDiff.getCvImage(), grayDiff.getCvImage(), 3, 255, CV_THRESH_BINARY);
    grayDiff.flagImageChanged();
}

/************************************************************
                    UPDATE PAN TILT ANGLES
*************************************************************/
void testApp::updatePanTilt()
{
     if (blobFinder.blobs.size() > 0) {
         winnerBlob = 0;
        float minDist = 10000000; // any high number
        for (int i = 0; i < numBlobs; i++) {
            ofVec3f blobMaxZ = blobFinder.blobs[i].maxZ;
            if ( ptzTran.distance(blobMaxZ) < minDist) {
                minDist = ptzTran.distance(blobMaxZ);
                winnerBlob = i;
            }
        }
        ptzPointer = blobFinder.blobs[winnerBlob].maxZ - ptzTran;

        ofVec3f yAxis = ofVec3f(0,1,0);
        //pan
        panAngle = yAxis.angle(ofVec3f(ptzPointer.x, ptzPointer.y, 0));
        if (ptzPointer.x >= 0) panAngle = -panAngle;
        //tilt
        ofVec3f ptzProjYZ = ptzPointer;
        ptzProjYZ.rotate(0,0, -panAngle);
        tiltAngle = yAxis.angle(ptzProjYZ);
        if (ptzPointer.z <= 0) tiltAngle = -tiltAngle;

        //printf("pan: %.2f    tilt: %.2f \n",panAngle, tiltAngle);
        //----------------------------
        if (bMoveTurret) {
#ifdef _SERVOS
            updateServos(panAngle, tiltAngle);
#elif defined _WIDOWX
            turret.setPanTiltDeg(-panAngle, -tiltAngle);
#elif defined _VIVOTEK
            moveVivotekXY(panAngle, tiltAngle);
#endif
        }
	}
}
/************************************************************
                UPDATE SERVO TURRET
*************************************************************/
void testApp::updateServos(int pan, int tilt) {
#ifdef _SERVOS
    if ((contact != 'A') && (bSerialInited) && (serial.available() > 0)) {
        serial.readBytes(&contact, 1);
    }
    else if (bMoveTurret) {
        if ((tilt < 90) && (tilt > -90) && (pan < 180) && (pan > -180)) {
            MsX = (int) ofMap(pan, 180, -180, 2000, 1000, true); // pan
            MsY = (int) ofMap(tilt, 90,-90,  1000, 2000, true); // tilt
        }
        moveServo(2,MsY);
        moveServo(0,MsX);
    }
#endif
}
/************************************************************
                                DRAW
*************************************************************/
void testApp::draw()
{
    //if (gui.isOn()) gui.draw();
   // else {
        ofBackground(0, 0, 0);
        ofSetFullscreen(bFullScreen);
       // if (!bFullScreen) {
            if(bDraw3D){
                ofPushMatrix();
                ofScale(SCALE_3D_VIEW,SCALE_3D_VIEW,SCALE_3D_VIEW);
                glEnable(GL_DEPTH_TEST);
                virtualCamera.begin();
                if (bAuto3DFlightMode) auto3Dflight();
//                drawRoomModel();

                if (bDrawPointCloud) drawPointCloud();
               // draw3Dgui();
                //glPointSize(3);
                //blobFinder.drawCloud();
               // blobFinder.draw(OF_MESH_POINTS, true, true);
              //  blobTracker.draw();
                glDisable(GL_DEPTH_TEST);
                virtualCamera.end();
                ofPopMatrix();
            }
            if (gui.isOn()) gui.draw();
           // else draw2Dgui();
       /* }
        else { // fullscreen
            ofSetHexColor(0xffffff);
    #ifdef _VIDEO
          #ifdef _VIVOTEK
            vivotek.draw(0,0,ofGetWidth(),ofGetHeight());
          #else
            vidGrabber.draw(0,0,ofGetWidth(),ofGetHeight());
          #endif
    #endif
        }*/
      //  drawReport();
   // }

   int yDim = (int)(((float)imgLogos.height/(float)imgLogos.width)*(float)ofGetWidth());
   int yPos = ofGetHeight()-yDim;
    imgLogos.draw(0, yPos, ofGetWidth(), yDim);
//   window_img.grabScreen(0,0,ofGetWidth(),ofGetHeight());
}
/************************************************************
                    AUTO 3D FLIGHT
*************************************************************/
void testApp::auto3Dflight() {
        virtCamAlpha += VIRT_CAM_A_RATE/ofGetFrameRate();
        if (virtCamAlpha > TWO_PI) virtCamAlpha = 0.0;
        virtCamBeta += VIRT_CAM_B_RATE/ofGetFrameRate();
        if (virtCamBeta > TWO_PI) virtCamBeta = 0.0;
        float virtCamDist = VIRT_CAM_MIN_DIST+VIRT_CAM_DIST_FACTOR*(1+sin(virtCamBeta));

        float virtCamPosX = virtCamDist*cos(virtCamAlpha);
        float virtCamPosY = virtCamDist*sin(virtCamAlpha);

        float virtCamZoffset = VIRT_CAM_Z_OFFSET*virtCamDist/VIRT_CAM_MIN_DIST;
        virtualCamera.setPosition( blobFinder.blobs[winnerBlob].maxZ +
                                   ofVec3f(virtCamPosX,virtCamPosY,virtCamZoffset/2.0));
        virtualCamera.lookAt( blobFinder.blobs[winnerBlob].maxZ +
                              ofVec3f(0,0,virtCamZoffset), ofVec3f(0,0,1));
}
/************************************************************
                    DRAW 2D GUI
*************************************************************/
void testApp::draw2Dgui() {
    ofSetColor(255, 255, 255);
    int ypos = DRAW_GAP;
    int xpos = DRAW_GAP;

    //-------------------------------------
    ofDrawBitmapString("DEPTH",xpos,ypos);
    depthImage.draw(DRAW_GAP,ypos, DRAW_W, DRAW_H);
    //-------------------------------------
    ypos += DRAW_GAP + DRAW_H;
    ofDrawBitmapString("BGS",xpos,ypos);
    grayDiff.draw(DRAW_GAP,ypos, DRAW_W,DRAW_H);
    //----------------------------------------
#ifdef _VIDEO
    if (bShowVideo) {
        int vidW = DRAW_W;
        int vidH = DRAW_H;
        if (bDraw3D) {
                ypos += DRAW_GAP + DRAW_H;
        }
        else {
            xpos = DRAW_W+2*DRAW_GAP;
            ypos = DRAW_GAP;
            vidW = ofGetWidth()-xpos-DRAW_GAP;
            #ifdef _VIVOTEK
                vidH = VIVOTEK_IMAGE_H*vidW/VIVOTEK_IMAGE_W;
            #else
                vidH = vidGrabber.height*vidW/vidGrabber.width;
            #endif
        }

        #ifdef _VIVOTEK
            vivotek.draw(xpos,ypos, vidW,vidH);
        #else
            vidGrabber.draw(xpos,ypos, vidW,vidH);
        #endif
        ofLine(xpos+vidW/2, ypos, xpos+vidW/2,ypos+vidH);
        ofLine(xpos,ypos+vidH/2, xpos+vidW,ypos+vidH/2);
    }
#endif
    ypos += DRAW_GAP + DRAW_H;
    xpos = DRAW_GAP;
    ofSetColor(255, 255, 255);
    /*ofDrawBitmapString("accel is: " + ofToString(kinectAvgAccel.x, 2) + " / "
                                    + ofToString(kinectAvgAccel.y, 2) + " / "
                                    + ofToString(kinectAvgAccel.z, 2), 20, ypos );
    ypos += 20;
    */
    char reportStr[2024];
    sprintf(reportStr, "BLOBS: %i, FPS: %.2f\n"
                       "Motor tilt angle: %i\n\n"
                       "{ } NEAR thr: %.2f \n"
                       "[ ] FAR thr: %.2f\n"
                       " p  Draw POINTCLOUD: %s\n"
                       " 3  Draw 3D WORLD: %s\n"
                       "' ' Move TURRET: %s\n"
                       " v  Show _VIDEO: %s\n"
                       " F1 Show settings\n"
                       " F2 toggle ABGS: %s\n"
                       " F3 toggle fullscreen: %s",
                        numBlobs, kFPS, kMotorAngle, nearThreshold, farThreshold,
                        bDrawPointCloud == true? "TRUE" : "FALSE",
                        bDraw3D == true? "TRUE" : "FALSE",
                        bMoveTurret == true? "TRUE" : "FALSE",
                        bShowVideo == true? "TRUE" : "FALSE",
                        bAdvancedBGS == true? "TRUE" : "FALSE",
                        bFullScreen == true? "TRUE" : "FALSE");
    ofSetHexColor(0x000000);
    ofDrawBitmapString(reportStr, xpos+1, ypos+1);
    ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString(reportStr, xpos, ypos);

    ofSetHexColor(0x000000);
    ofDrawBitmapString(" r  Record: ", xpos+1, 1+(ypos+=170));
    ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString(" r  Record: ", xpos, ypos);
    int r = 255;
    if (bRecording) {
        r = (ofGetElapsedTimeMillis() % 1000) < 500? 255:0;
        sprintf(reportStr,"RECORDING");
    }
    else sprintf(reportStr,"NOT RECORDING");

    xpos+=100;
    ofSetHexColor(0x000000);
    ofDrawBitmapString(reportStr, xpos+1, ypos+1);
    ofSetColor(255,r,r);
    ofDrawBitmapString(reportStr, xpos, ypos);

    if (bLearnBakground) {
        ofSetHexColor(0xffffff);
        sprintf( reportStr, "%i", timeLapse-ofGetElapsedTimeMillis() );
        font.drawString(reportStr, ofGetWidth()/2, (ofGetHeight()-font.getLineHeight())/2);
    }

}
/************************************************************
                    DRAW REPORT
*************************************************************/
void testApp::drawReport() {
    char reportStr[1024];
    sprintf( reportStr, "X %4.2f\nY %4.2f\nZ %4.2f\n"
                        "Pan %4.2f\nTilt %4.2f",
                        ptzPointer.x, ptzPointer.y, ptzPointer.z, panAngle,tiltAngle);
    ofSetHexColor(0x000000);
    font.drawString(reportStr, ofGetWidth()-200, font.getLineHeight()+2);
    ofSetLineWidth(2);
    ofSetHexColor(0xffffff);
    font.drawString(reportStr, ofGetWidth()-202, font.getLineHeight());
}
/************************************************************
                    DRAW 3D AXIS
*************************************************************/
void testApp::draw3Daxis(ofPoint tran, ofPoint rot, float axisLen) {
    ofPushMatrix();
    ofTranslate(tran.x, tran.y, tran.z);

    ofRotateX(rot.x);
    ofRotateY(rot.y);
    ofRotateZ(rot.z);

    glBegin(GL_LINES);
    glColor3ub((unsigned char)255,(unsigned char)0,(unsigned char)0);
	glVertex3f(0, 0, 0);
    glVertex3f(axisLen, 0, 0);
    glColor3ub((unsigned char)0,(unsigned char)255,(unsigned char)0);
	glVertex3f(0, 0, 0);
    glVertex3f(0, axisLen, 0);
    glColor3ub((unsigned char)0,(unsigned char)0,(unsigned char)255);
	glVertex3f(0, 0, 0);
    glVertex3f(0, 0, axisLen);
    glEnd();
    ofPopMatrix();
}
/************************************************************
                    DRAW 3D GUI
*************************************************************/
void testApp::draw3Dgui() {
    ofPushMatrix();
    glPushMatrix();
    // draw room box
    glColor3ub((unsigned char)255,(unsigned char)128,(unsigned char)128);
 /*   ofVec3f roomMin = ofPoint(-5,0,0);
	ofVec3f roomMax = ofPoint(5,10,3);
	drawGlWireBox(roomMin, roomMax, false);
*/
	// draw coordinate system
    draw3Daxis(ofPoint(0,0,0),ofPoint(0,0,0), AXIS_LEN);
    // draw kinect potition
    ofPushMatrix();
	ofTranslate(kTran.x, kTran.y, kTran.z);
    // the order of rotation matters!!! 1st Z, 2nd X, 3rd Y
	ofRotateZ(kRot.z);
	ofRotateX(kRot.x);
	ofRotateY(kRot.y);

    // draw kinect box
    glColor3ub((unsigned char)0,(unsigned char)0,(unsigned char)0);
    setGLlighting(true);
    drawGlBox(KIN_3D_L, KIN_3D_W,KIN_3D_H, true);
    setGLlighting(false);
    // kinect axis
    draw3Daxis(ofPoint(0,0,0),ofPoint(0,0,0), AXIS_LEN);
    // kinect camera ranges and FOV
    glBegin(GL_LINES);
    glColor3ub((unsigned char)128,(unsigned char)128,(unsigned char)255);
    glVertex3f(0, 0, 0);
    glVertex3f(K_FAR.x, K_FAR.y, K_FAR.z);
    glVertex3f(0, 0, 0);
    glVertex3f(-K_FAR.x, K_FAR.y, K_FAR.z);
    glVertex3f(0, 0, 0);
    glVertex3f(K_FAR.x, K_FAR.y, -K_FAR.z);
    glVertex3f(0, 0, 0);
    glVertex3f(-K_FAR.x, K_FAR.y, -K_FAR.z);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(K_FAR.x, K_FAR.y, K_FAR.z);
    glVertex3f(K_FAR.x, K_FAR.y, -K_FAR.z);
    glVertex3f(-K_FAR.x, K_FAR.y, -K_FAR.z);
    glVertex3f(-K_FAR.x, K_FAR.y, K_FAR.z);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(K_NEAR.x, K_NEAR.y, K_NEAR.z);
    glVertex3f(K_NEAR.x, K_NEAR.y, -K_NEAR.z);
    glVertex3f(-K_NEAR.x, K_NEAR.y, -K_NEAR.z);
    glVertex3f(-K_NEAR.x, K_NEAR.y, K_NEAR.z);
    glEnd();
    ofPopMatrix();
    // kinect xyz range box
  //  glColor3ub((unsigned char)128,(unsigned char)128,(unsigned char)128);
 //  drawGlWireBox(threshMinXYZ, threshMaxXYZ, false);
/*
    // kinect acceleration / gravity vector
    glBegin(GL_LINES);
    glColor3ub((unsigned char)128,(unsigned char)255,(unsigned char)128);
    glVertex3f(0, 0, 0);
    glVertex3f(-kinectAccel.x, -kinectAccel.z, -kinectAccel.y);
    glEnd();
*/
    // draw ptz
    ofPushMatrix();
    ofRotateX(ptzRot.x); ofRotateY(ptzRot.y); ofRotateZ(ptzRot.z);
    ofTranslate(ptzTran.x, ptzTran.y, ptzTran.z);
    // draw ptz axis
    draw3Daxis(ofPoint(0,0,0),ofPoint(0,0,0), 0.15);
    //draw ptz pointer
    glBegin(GL_LINES);
    glColor3ub(128,255,128);
    glVertex3f(0, 0, 0);
    glVertex3f(ptzPointer.x, ptzPointer.y, ptzPointer.z);
    glEnd();
    // turn on lighting fot solid objects
    setGLlighting(true);
    // draw ptz camera sphere & cone

    ofPushMatrix();
    ofRotateZ(panAngle);
    ofRotateX(tiltAngle+90);

    glColor3ub(200,150,100);
    glutSolidSphere(CAM_3D_SPH_RAD,6,6);
    glColor3ub(0,0,0);
    glutWireSphere(CAM_3D_SPH_RAD,6,6);


    ofRotateZ(45);
    ofTranslate(0, 0, -CAM_3D_CONE_H);
    glColor4ub(200,150,100,50);
    glutSolidCone(CAM_3D_CONE_BASE,CAM_3D_CONE_H,4,1);
    glColor3ub(0,0,0);
    glutWireCone(CAM_3D_CONE_BASE,CAM_3D_CONE_H,4,1);

    ofPopMatrix();
    // draw ptz basebox below the camera sphere
    ofTranslate(0, 0, -CAM_3D_BASE);
    ofRotateZ(45);
    glColor3ub(200,150,100);
    glutSolidCone(CAM_3D_BASE,CAM_3D_BASE,4,1);
    glColor3ub(0,0,0);
    glutWireCone(CAM_3D_BASE,CAM_3D_BASE,4,1);
    // turn on lighting fot solid objects
    setGLlighting(false);

    ofPopMatrix();


    glPopMatrix();
    ofPopMatrix();
}
/************************************************************
                    DRAW 3D POINT CLOUD
*************************************************************/

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;


	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
		    float distat = kinect.getDistanceAt(x, y);
			if (( distat > 0)  && (distat < farThreshold*1000)) {
				mesh.addColor(kinect.getColorAt(x,y));
				ofVec3f tempPoint = kinect.getWorldCoordinateAt(x, y);
				//if (bUpsideDown) tempPoint = ofVec3f(-tempPoint.x, tempPoint.y, -tempPoint.z);
                mesh.addVertex(tempPoint);
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	ofTranslate(kTran.x, kTran.y, kTran.z);
	ofRotateZ(kRot.z);
    ofRotateY(kRot.y);
	ofRotateX(kRot.x+90);
	if (bUpsideDown) ofRotateZ(180);
	/*ofRotateZ(-rot.y);
	ofRotateX(rot.x+90);
	ofRotateY(rot.z);
	*/
	ofScale(0.001, -0.001, -0.001);
	mesh.drawVertices();
	ofPopMatrix();
}
/************************************************************
                    DRAW ROOM MODEL
*************************************************************/

void testApp::drawRoomModel() {
	ofPushMatrix();
    glColor3ub((unsigned char)255,(unsigned char)0,(unsigned char)0);
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    roomModel.draw();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    setGLlighting(true);
    roomModel.draw();
    setGLlighting(false);
    ofPopMatrix();
}
/************************************************************
                    SET GL LIGHTING
*************************************************************/
void testApp::setGLlighting(bool enable) {
    if (enable) {
        glShadeModel(GL_FLAT);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_NORMALIZE);
        // enable color tracking
        glEnable(GL_COLOR_MATERIAL);
        // GLOBAL AMBIENT
        //GLfloat globalAmbient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
        //glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
        // AMBIENT
        GLfloat ambient[] = {0.1f, 0.1f, 0.1f, 1.0f};
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
        // DIFUSE
        GLfloat diffuse[] = {0.5f, 0.5f, 0.5f, 1.0f};
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
        // SPECULAR
        GLfloat specular[] = {1.0f, 1.0f,  1.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    }
    else {
        glDisable(GL_LIGHTING); glDisable(GL_LIGHT0);
        glDisable(GL_NORMALIZE);
        glDisable(GL_COLOR_MATERIAL);
    }
}
/************************************************************
                    DRAW BLOBS
*************************************************************/
void testApp::drawBlobs() {
    glPushMatrix();
    // draw spheres on blob's centroids
    setGLlighting(true);
    glColor4ub(255,0,0,100);
    for (unsigned int i=0; i < blobFinder.blobs.size(); i++) {
        blobFinder.blobs[i].draw();
    }
    setGLlighting(false);
    // draw blob's 3D bounding box with center cross
    glColor3ub((unsigned char)255,(unsigned char)255,(unsigned char)255);
	for (unsigned int i = 0; i < blobFinder.blobs.size(); i++) {
        //drawGlWireBox(blobs[i]->min, blobs[i]->max, true);
	}
	glPopMatrix();
}
/****************************************************************************

                    CALC DIRECTION

*****************************************************************************/

ofVec3f testApp::calcDirection(int id, vector <ofVec3f> &trajectory) {
    int filterDepth = 3;
    int filterDist = 3;

   /* if (abs(robotRad[robot]) > 45) {
        if (abs(robotSpeed[robot]) < 200) {
            filterDepth = 2;
            filterDist = 5;
        }
        else {
            filterDepth = 6;
            filterDist = 8;
        }
    }
    else {
        filterDepth = 8;
        filterDist = 10;
    }*/
    ofVec3f filtered = ofVec3f(0,0,0);

    for (int j = trajectory.size()-filterDist-1, k = filterDepth; j >0, k > 0; j--, k--) {
            filtered.x += (trajectory[j+filterDist].x-trajectory[j].x);
            filtered.y += (trajectory[j+filterDist].y-trajectory[j].y);
            filtered.z += (trajectory[j+filterDist].z-trajectory[j].z);
    }
    filtered.x /= filterDepth;
    filtered.y /= filterDepth;
    filtered.z /= filterDepth;

    return filtered;
}
/************************************************************
                    DRAW TRACKING
*************************************************************/
void testApp::drawTracking() {
    ofTrueTypeFont testFont;
    testFont.loadFont("PerfectDOSVGA437.ttf", 16, true, true, true);

    for( unsigned int i=0; i<blobTracker.blobs.size(); i++ ) {
        ofPushMatrix();
        ofTranslate(blobTracker.blobs[i].massCenter.x, blobTracker.blobs[i].massCenter.y, blobTracker.blobs[i].maxZ.z);
        ofRotateX(-90);
        ofScale(0.01f, 0.01f, 0.01f);
        testFont.drawStringAsShapes(ofToString(blobTracker.blobs[i].id), 0, 0);
        ofPopMatrix();
        // trajectory
        vector <ofVec3f> trajectory;
        blobTracker.getTrajectoryById(blobTracker.blobs[i].id, trajectory);

        unsigned int trjSize = trajectory.size();
        if (trjSize > 1) {
            glPushMatrix();
            glBegin(GL_LINE);

            for (unsigned int j = 0; j < trjSize; j++) {
                glVertex3f( trajectory[j].x, trajectory[j].y, trajectory[j].z );
            }
            glEnd();
            glPopMatrix();

            // velocity
            ofPushMatrix();
            ofSetLineWidth(5);
            ofSetColor(255,128,128);
            ofVec3f velocity = calcDirection(blobTracker.blobs[i].id, trajectory);//trajectory[trjSize-1] - trajectory[trjSize-2];
            velocity *= 10;
            ofTranslate(blobTracker.blobs[i].massCenter);
            ofLine(ofVec3f(0,0,0), velocity);
            ofPopMatrix();
            if (velocity.length() > 0.05) printf("velocity X: %.3f    Y: %.3f    Z: %.3f   \n", velocity.x, velocity.y, velocity.z);
            trajectory.clear();
        }

	}

}
/************************************************************
                    DRAW GL WIRE BOX
*************************************************************/
void testApp::drawGlWireBox(ofVec3f minPoint, ofVec3f maxPoint, bool centerCross)
{
    // draw bounding rectangular parallelepiped
    glBegin(GL_LINE_LOOP);
    glVertex3f(minPoint.x, minPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(minPoint.x, minPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
    glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
    if (centerCross) {
        //draw crossing lines marking centroid
        glVertex3f(minPoint.x, minPoint.y, minPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
    }
    glEnd();
}

/************************************************************
                    DRAW GL WIRE BOX
*************************************************************/
void testApp::drawGlBox(float width, float height, float depth, char solid)
{
  char i, j = 0;
  float x = width / 2.0, y = height / 2.0, z = depth / 2.0;
  glPushMatrix();
  for (i = 0; i < 4; i++) {
    glRotatef(90.0, 0.0, 0.0, 1.0);
    if (j) {
      if (!solid)
        glBegin(GL_LINE_LOOP);
      else
        glBegin(GL_QUADS);
      glNormal3f(-1.0, 0.0, 0.0);
      glVertex3f(-x, y, z);
      glVertex3f(-x, -y, z);
      glVertex3f(-x, -y, -z);
      glVertex3f(-x, y, -z);
      glEnd();
      if (solid) {
        glBegin(GL_TRIANGLES);
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, z);
        glVertex3f(-x, y, z);
        glVertex3f(-x, -y, z);
        glNormal3f(0.0, 0.0, -1.0);
        glVertex3f(0.0, 0.0, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, y, -z);
        glEnd();
      }
      j = 0;
    } else {
      if (!solid)
        glBegin(GL_LINE_LOOP);
      else
        glBegin(GL_QUADS);
      glNormal3f(-1.0, 0.0, 0.0);
      glVertex3f(-y, x, z);
      glVertex3f(-y, -x, z);
      glVertex3f(-y, -x, -z);
      glVertex3f(-y, x, -z);
      glEnd();
      if (solid) {
        glBegin(GL_TRIANGLES);
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, z);
        glVertex3f(-y, x, z);
        glVertex3f(-y, -x, z);
        glNormal3f(0.0, 0.0, -1.0);
        glVertex3f(0.0, 0.0, -z);
        glVertex3f(-y, -x, -z);
        glVertex3f(-y, x, -z);
        glEnd();
      }
      j = 1;
    }
  }
  glPopMatrix();
}

/************************************************************
                          CALC AVG ACCEL
*************************************************************/
ofVec3f testApp::calcAvgAccel(){
    ofVec3f tempAvg = ofPoint(0,0,0);

    for (int i = 1; i < ACC_HIS_LEN; i++) {
        kinectAccelHistory[i] = kinectAccelHistory[i-1];
        tempAvg += kinectAccelHistory[i];
    }
    kinectAccelHistory[0] = kinect.getMksAccel();
    tempAvg += kinectAccelHistory[0];
    tempAvg /= ACC_HIS_LEN;
    return tempAvg;
}

/************************************************************
                          MOVE SERVO
*************************************************************/
void testApp::moveServo(int id, int Ms){
#ifdef _SERVOS
    serial.writeByte('s');
    serial.writeByte((unsigned char)(id));

    serial.writeByte((unsigned char)(Ms));
    Ms >>= 8;
    serial.writeByte((unsigned char)(Ms));

/*    unsigned char buffer[] = {'s', (unsigned char)id, (unsigned char)(Ms  & 0xFF), (unsigned char)((Ms >> 8) & 0xFF)};
    serial.writeBytes(buffer, 4);
*/
#endif
}

/************************************************************
                          MOVE VIVOTEK X Y
*************************************************************/
void testApp::moveVivotekXY(float pan, float tilt) {


#ifdef _VIVOTEK
    int elapsedNow = ofGetElapsedTimeMillis();
    int timePassed = elapsedNow - lastReqTime;
    if (timePassed > 800) {
          lastReqTime = elapsedNow;
      }
      else if (timePassed > 400) {
         int vivoPan = round(ofMap(pan, -180, 180, 1,1600));
         int vivoTilt = round(ofMap(tilt, -90, 90, -378, 445 ));//67, 445));

        char theReqChar[1024];

        sprintf(theReqChar, "/cgi-bin/viewer/camctrl.cgi?setpan=%i&settilt=%i", vivoPan, vivoTilt);
        cout << theReqChar << "\n";
       //http://192.168.1.30/cgi-bin/viewer/camctrl.cgi?getpan&gettilt&getzoom

         //string response;
        vivotek.sendRequest(theReqChar);
        //cout << response << "\n";
        lastReqTime = elapsedNow;
      }


#endif
}
/// ****************************************************
///
///                     SETUP RECORDING
///
/// ****************************************************
void testApp::setupRecording() {
    char fileName[1024];
    bool fileExists = true;
    do {
        recorderFileNumber++;
        sprintf( fileName, "cvcinema_kinect_3d_%i.avi",
                 recorderFileNumber);
        ofFile file(fileName);
        fileExists = file.exists();
    } while (fileExists);
    cout << "START RECORDING to file: " << fileName << endl;

    recorder.setup(window_img,24, ofToDataPath(fileName),ofxGstVideoRecorder::H264,25);
    //recorder.setupRecordWindow(0,0,ofGetWidth(),ofGetHeight(),24,ofToDataPath(fileName),ofxGstVideoRecorder::H264,25);
}
/************************************************************
                        KEYPRESSED
*************************************************************/
void testApp::keyPressed (int key)
{
	switch (key)
	{
		case OF_KEY_F1: gui.toggleDraw(); break;
		case OF_KEY_F2: bAdvancedBGS = !bAdvancedBGS; break;
		case OF_KEY_F3: bFullScreen = !bFullScreen; break;
		case OF_KEY_F4: blobFinder.setResolution(BF_HIGH_RES); break;
		case OF_KEY_F5: blobFinder.setRotation(kRot);
                        blobFinder.setTranslation(kTran);
                        break;
		case OF_KEY_F6: bAuto3DFlightMode = !bAuto3DFlightMode; break;
		case ' ': bMoveTurret = !bMoveTurret; break;
		case 'b':
            bLearnBakground = true;
            timeLapse = 3000+ofGetElapsedTimeMillis();
            break;
        case 'v':   bShowVideo = !bShowVideo;
                    #ifdef _VIVOTEK
                        if (bShowVideo) vivotek.open();
                        else vivotek.close();
                    #endif
                    break;
		case'p': bDrawPointCloud = !bDrawPointCloud; break;
        case'3': bDraw3D = !bDraw3D; break;
        case ']':
            farThreshold  += 0.01;
        //    if (farThreshold > MAX_DEPTH) farThreshold = MAX_DEPTH;
            kinect.setDepthClipping(nearThreshold*1000,farThreshold*1000);

            //threshMaxXYZ.z += 0.01;
			//if (threshMaxXYZ.z > 10) threshMaxXYZ.z = 10;
			break;
		case '[':
		    farThreshold  -= 0.01;
            if (farThreshold < nearThreshold) farThreshold = nearThreshold;
            kinect.setDepthClipping(nearThreshold*1000,farThreshold*1000);

			//threshMaxXYZ.z -= 0.01;
			//if (threshMaxXYZ.z < threshMinXYZ.z) threshMaxXYZ.z = threshMinXYZ.z;
			break;
        case '}':
			nearThreshold += 0.01;
			if (nearThreshold > farThreshold) nearThreshold = farThreshold;
			kinect.setDepthClipping(nearThreshold*1000,farThreshold*1000);

			break;
		case '{':
			nearThreshold -= 0.01;
			if (nearThreshold < 0) nearThreshold = 0;
			kinect.setDepthClipping(nearThreshold*1000,farThreshold*1000);

			break;
        case ')':
			threshMinXYZ.z += 0.01;
			if (threshMinXYZ.z > threshMaxXYZ.z) threshMinXYZ.z = threshMaxXYZ.z;
			break;
		case '(':
		case '8':
			threshMinXYZ.z -= 0.01;
			if (threshMinXYZ.z < -10) threshMinXYZ.z = -10;
			break;
		case OF_KEY_UP:
        case 'x' :
			if(kMotorAngle++ > 30) kMotorAngle=30;
			kinect.setCameraTiltAngle(kMotorAngle);
			bUpdateKinectRotTran = true;
			break;
		case OF_KEY_DOWN:
        case 'z' :
			if(kMotorAngle-- < -30) kMotorAngle=-30;
			kinect.setCameraTiltAngle(kMotorAngle);
			bUpdateKinectRotTran = true;
			break;
        /*case OF_KEY_UP: ptzTran.z -=0.1; break;
        case OF_KEY_DOWN: ptzTran.z +=0.1; break;
        case OF_KEY_LEFT: ptzTran.x -=0.1; break;
        case OF_KEY_RIGHT: ptzTran.y +=0.1; break;*/
        case 'j': kRot.z--; bUpdateKinectRotTran = true; break;
        case 'l': kRot.z++; bUpdateKinectRotTran = true; break;
        case 'i': kRot.x ++; bUpdateKinectRotTran = true; break;
        case 'k': kRot.x --; bUpdateKinectRotTran = true; break;
        case 'm': kRot.y ++; bUpdateKinectRotTran = true; break;
        case 'n': kRot.y --; bUpdateKinectRotTran = true; break;
        case 'w': kTran.y+=0.05; bUpdateKinectRotTran = true; break;
        case 's': kTran.y-=0.05; bUpdateKinectRotTran = true; break;
        case 'd': kTran.x+=0.05; bUpdateKinectRotTran = true; break;
        case 'a': kTran.x-=0.05; bUpdateKinectRotTran = true; break;
        case 'e': kTran.z+=0.05; bUpdateKinectRotTran = true; break;
        case 'q': kTran.z-=0.05; bUpdateKinectRotTran = true; break;
        case 't': ptzTran.y+=0.05; break;
        case 'g': ptzTran.y-=0.05; break;
        case ',': ptzTran.x+=0.05; break;
        case '.': ptzTran.x-=0.05; break;
        case 'h': ptzTran.z+=0.05; break;
        case 'f': ptzTran.z-=0.05; break;
        case 'r':   if (!bRecording) {
                setupRecording();
            }
            else  {
                recorder.stop();
                cout << "STOP RECORDING"<< endl;
            }
            bRecording = ! bRecording;
        break;
    }
}
//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{

}
//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}
//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}
//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}
//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){
}
//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){


}


/*
 *
 *	blob section
 *
 *	from here on in it's blobs
 *	thanks to stefanix and the opencv library :)
 *if (bUpsideDown) thePos = ofVec3f(-thePos.x,thePos.y,-thePos.z);

 */

//--------------------------------------------------
void testApp::blobOn( ofVec3f centroid, int id, int order ) {
   // cout << "blobOn() - id:" << id << " order:" << order << endl;
}

void testApp::blobMoved( ofVec3f centroid, int id, int order) {
  //  cout << "blobMoved() - id:" << id << " order:" << order << endl;

    // full access to blob object ( get a reference)
  //  ofxKinectTrackedBlob blob = blobTracker.getById( id );
   // cout << "volume: " << blob.volume << endl;
}

void testApp::blobOff( ofVec3f centroid, int id, int order ) {
   // cout << "blobOff() - id:" << id << " order:" << order << endl;if (bUpsideDown) thePos = ofVec3f(-thePos.x,thePos.y,-thePos.z);

}

/************************************************************
                            EXIT
*************************************************************/
void testApp::exit(){
    #ifdef _VIVOTEK
        vivotek.close();
    #endif
	kinect.close();
}
