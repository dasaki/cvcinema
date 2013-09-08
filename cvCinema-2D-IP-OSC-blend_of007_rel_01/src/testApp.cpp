#include "testApp.h"

#define IP_CAMERA_W         320
#define IP_CAMERA_H         240
#define FIXED_ZOOM          6
#define OUT_ASPECT          1.33333 // (4.0/3.0)
#define LETTERBOX           false
#define DRAW_W              320
#define DRAW_H              240
#define DRAW_SPACE          10
#define FONT_COLOR          0xffffff
#define FONT_SEL_COLOR      0xaa0000
#define MAX_ANALYSIS_W      320
#define VIDEO_DEVICE_ID     0
#define MAX_BLOBS           6
#define HFLIPX(x,w)         ( (hFlip)? (x+w) : (x) )
#define HFLIPW(w)           ( (hFlip)? (-w) : (w) )
#define VFLIPY(y,h)         ( (vFlip)? (y+h) : (y) )
#define VFLIPH(h)           ( (vFlip)? (-h) : (h) )

#ifndef MIN
	#define MIN(x,y)        (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
	#define MAX(x,y)        (((x) > (y)) ? (x) : (y))
#endif


int grabberResLen = 6;
int grabberRes[][2] = {{1920,1080},{1280,720},{1024,768},{800,600},{640,480},{320,240}};
// 2592x1944
// 2048x1536
// 1600x1200
// 1792x1008
// 1280x960
// 1712x960
// 352x288
// 160x120
int grabberW, grabberH;
int outW, outH;
bool bIsVideoGrabbing = false;
bool vFlip = false;
bool hFlip = false;
bool updateControls = true;
bool setDefaults = true;
bool bNewFrame = false;
bool updatingCtls = false;
string  movieFile = "";
int recorderFileNumber = 0;
bool bAnalisysAllSetup = false;
///*****************************************************************
///
///                        SETUP
///
///*****************************************************************
void testApp::setup(){
   // ofSetLogLevel(OF_LOG_NOTICE);
    bLiveVideo = true;
    bShowGui = false;
    selOutput = OUTPUT_IMAGE;
    bRecording = false;
    bMovieAudio = false;
    bFullScreen = false;
	saveFrame = false;
	threshold = 50;
    // if we set BG subtraction as default (no. 2) there is a segmentation fault because the background image is not set for the first frame
	algorithm = 1;
	cropping = '6';
	bZoomTarget = false;
	cols = 1;
	rows = 1;
	lastTime = ofGetElapsedTimeMillis();
	currFrameNum = 0;
	videoFPS = 0;
	imgLogos.loadImage("faldon-logos.bmp");
	imgLogos.resize(ofGetWidth(), round(imgLogos.height*((float)ofGetWidth()/(float)imgLogos.width)));
	//ofEnableAlphaBlending();
    ofSetVerticalSync(true);
    ofSetFrameRate(30);
    string devfile = "/dev/video"+ofToString(VIDEO_DEVICE_ID);
    settings.setup(devfile);

    vidGrabber.setVerbose(true);
    vidGrabber.setDeviceID(VIDEO_DEVICE_ID);
    vidGrabber.setDesiredFrameRate(30);
    if (bLiveVideo) setupLiveVideo();
    else loadMovie();
    setupGui();
#ifdef OFX_FENSTER
	outWinListener.setup();
	outWinListener.setImage(&blendImg);
#endif
    //recorder.setupRecordWindow(0,0,1024,576,24,"capture.mp4",ofxGstVideoRecorder::H264,30);
    //recorder.setupRecordWindow((DRAW_SPACE*2+320),(DRAW_SPACE*2+240+out_H_gap),analysisW*2,analysisH,24,"bg_sub_vs_bg&fd_01.mp4",ofxGstVideoRecorder::H264,30);

    setupOSC();
    blendMode = OF_BLENDMODE_ADD;//OF_BLENDMODE_ALPHA;
    alphaValues[0] = alphaValues[1] = alphaValues[2] = alphaValues[3] = 64;
    loadCameras();
}
/// ****************************************************
///
///                     SETUP GUI
///
/// ****************************************************
void testApp::setupGui() {
    gui.setup("panel","settings.xml",0,0);
	map<string,ofxV4L2Settings::Control>::iterator it;
	for(it=settings.controls.begin();it!=settings.controls.end();it++){
		cout << it->first << " " << it->second.parameter << endl;
		gui.add(new ofxIntSlider(it->second.parameter.getName(),it->second.parameter,it->second.minimum,it->second.maximum));
	}
    gui.add(new ofxButton("LoadDefaults"));
    //gui.add(new ofxToggle("LoadDefaults", bLoadDefaults));
}

///*****************************************************************
///                        SETUP LIVE VIDEO
///*****************************************************************
bool testApp::setupLiveVideo() {
    int i = 0;
    cout << "Setting up maximum available resolution" << endl;
    for (; i < grabberResLen; i++) {
        bIsVideoGrabbing = vidGrabber.initGrabber(grabberRes[i][0],grabberRes[i][1]);
        if (bIsVideoGrabbing) break;
        cout << grabberRes[i][0] <<"x" <<grabberRes[i][1]<< " failed  " << endl;
    }


    if (bIsVideoGrabbing) {
        grabberW = grabberRes[i][0];
        grabberH = grabberRes[i][1];
        cout << "Grabbing at " << grabberW << "x"<< grabberH << endl;
        setupAnalisys(grabberW,grabberH);
    }
    return bIsVideoGrabbing;
}
///*****************************************************************
///                        SETUP ANALYSIS
///*****************************************************************
void testApp::setupAnalisys(int inWidth, int inHeight) {
    bAnalisysAllSetup = false;
    inW = inWidth;
    inH = inHeight;

    outW = round((float)inW/(float)FIXED_ZOOM);
    outH = round((float)outW/(float)OUT_ASPECT);

    in_out_scale = (float)inWidth/(float)outW;
    in_draw_scale =  (float)inWidth/(float)DRAW_W;
    analysis_out_scale = (float)analysisW/(float)outW;
    inAspect = (float)inWidth/(float)inHeight;
    outAspect = (float)outW/(float)outH;
    if (LETTERBOX) out_H_in_aspect = round((float)outW/(float)inAspect);
    else out_H_in_aspect = outH;
    cropW = outW;
    cropH = out_H_in_aspect;

    if (inWidth > MAX_ANALYSIS_W) {
        analysisW = MAX_ANALYSIS_W;
        analysisH = round((float)MAX_ANALYSIS_W/(float)inAspect);
    }
    in_analysis_scale = (float)inWidth/(float)analysisW;

    out_H_gap = round((float)(outH-out_H_in_aspect)/2.0);
    draw_H_gap = round((float)(DRAW_H-DRAW_W/inAspect)/2.0);
    cout << "outW: " << outW << "outH: " << outH <<
    "out_H_in_aspect" << out_H_in_aspect <<" out gap: " << out_H_gap << endl;
    if (fullFrame.bAllocated) fullFrame.resize(inWidth, inHeight);
    else fullFrame.allocate(inWidth, inHeight);
    if (outputImg.bAllocated) outputImg.resize(outW, outH);
    else outputImg.allocate(outW, outH);
    if (blendImg.bAllocated) blendImg.resize(outW, outH);
    else blendImg.allocate(outW, outH);


    if (colorImg.bAllocated) colorImg.resize(analysisW, analysisH);
    else colorImg.allocate(analysisW, analysisH);
    if (grayImage.bAllocated) grayImage.resize(analysisW, analysisH);
    else grayImage.allocate(analysisW, analysisH);
    if (lastFrame.bAllocated) lastFrame.resize(analysisW, analysisH);
    else lastFrame.allocate(analysisW, analysisH);
    if (lastFrameGray.bAllocated) lastFrameGray.resize(analysisW, analysisH);
    else lastFrameGray.allocate(analysisW, analysisH);
    if (bgImage.bAllocated) bgImage.resize(analysisW, analysisH);
    else bgImage.allocate(analysisW, analysisH);
    if (colorDiff.bAllocated) colorDiff.resize(analysisW, analysisH);
    else colorDiff.allocate(analysisW, analysisH);
    if (grayDiff.bAllocated) grayDiff.resize(analysisW, analysisH);
    else grayDiff.allocate(analysisW, analysisH);
    if (grayTemp.bAllocated) grayTemp.resize(analysisW, analysisH);
    else grayTemp.allocate(analysisW, analysisH);
    if (grayBgImage.bAllocated) grayBgImage.resize(analysisW, analysisH);
    else grayBgImage.allocate(analysisW, analysisH);
    // For ZivkovicAGMM ----------------------------
    if (bgs.low.Ptr() != NULL) bgs.low.Release();
    if (bgs.high.Ptr() != NULL) bgs.high.Release();
    bgs.bgsPtr = &Z_bgs;
    bgs.paramsPtr = &Z_params;
    Z_params.SetFrameSize(analysisW, analysisH);
    Z_params.LowThreshold() = 40;// default 5.0f*5.0f;
    Z_params.HighThreshold() = 2*Z_params.LowThreshold();
    Z_params.Alpha() = 0.001f;// default 0.001f;
    Z_params.MaxModes() = 3;// default 3
    Z_bgs.Initalize(Z_params);
    bgs.low = cvCreateImage(cvSize(analysisW,analysisH),IPL_DEPTH_8U,1);
    bgs.high = cvCreateImage(cvSize(analysisW,analysisH),IPL_DEPTH_8U,1);
//  ----------------------------
    bLearnBakground = true;
    bAnalisysAllSetup = true;
    // Analysis setup report
  /*  printf( "Input      W: %i   H: %i    Aspect: %.3f\n\n", inWidth, inHeight, inAspect);
    printf( "Analysis   W: %i   H: %i    Aspect: %.3f\n\n", analysisW, analysisH, inAspect);
    printf( "Output     W: %i   H: %i    Aspect: %.3f\n\n", outW, outH, OUT_ASPECT);
    printf( "In/Out scale: %.2f\n\n", in_out_scale);
    printf( "Crop       W: %i   H: %i    Aspect: %.3f\n\n",
            outW, out_H_in_aspect, (float)outW/(float)out_H_in_aspect);
*/
}
///*****************************************************************
///                             SETUP OSC
///*****************************************************************
void testApp::setupOSC() {
#ifdef OFX_OSC
	// listen on the given port
	cout << "listening for osc messages on port " << PORT << "\n";
	receiver.setup(PORT);
#endif
}
///*****************************************************************
///                         GET NEXT IP CAMERA
///*****************************************************************
IPCameraDef& testApp::getNextCamera() {
    nextCamera = (nextCamera + 1) % ipcams.size();
    return ipcams[nextCamera];
}
///*****************************************************************
///                LOAD IP CAMERAS CONNECTION DATA
///*****************************************************************
void testApp::loadCameras() {

    // all of these cameras were found using this google query
    // http://www.google.com/search?q=inurl%3A%22axis-cgi%2Fmjpg%22
    // some of the cameras below may no longer be valid.

    // to define a camera with a username / password
    //ipcams.push_back(IPCameraDef("http://148.61.142.228/axis-cgi/mjpg/video.cgi", "username", "password"));

	ofLog(OF_LOG_NOTICE, "---------------Loading Streams---------------");

	ofxXmlSettings XML;

	if( XML.loadFile("streams.xml") ){

        XML.pushTag("streams");
		string tag = "stream";

		int nCams = XML.getNumTags(tag);

		for(int n = 0; n < nCams; n++) {

            IPCameraDef def;

			def.name = XML.getAttribute(tag, "name", "", n);
			def.url = XML.getAttribute(tag, "url", "", n);
			def.username = XML.getAttribute(tag, "username", "", n);
			def.password = XML.getAttribute(tag, "password", "", n);

			string logMessage = "STREAM LOADED: " + def.name +
			" url: " +  def.url +
			" username: " + def.username +
			" password: " + def.password;

            ofLog(OF_LOG_NOTICE, logMessage);

            ipcams.push_back(def);

		}

		XML.popTag();



	} else {
		ofLog(OF_LOG_ERROR, "Unable to load streams.xml.");
	}
	ofLog(OF_LOG_NOTICE, "-----------Loading Streams Complete----------");


    nextCamera = ipcams.size();

    // initialize connection
    for(int i = 0; i < NUM_IP_CAMERAS; i++) {
        IPCameraDef& cam = getNextCamera();
        #ifdef OFX_IP_VIDEO_GRABBER
            ofxSharedIpVideoGrabber c( new ofxIpVideoGrabber());
            // if your camera uses standard web-based authentication, use this
            c->setUsername(cam.username);
            c->setPassword(cam.password);
            // if your camera uses cookies for authentication, use something like this:
            // c->setCookie("user", cam.username);
            // c->setCookie("password", cam.password);
            c->setCameraName(cam.name);
            c->setURI(cam.url);
            c->connect(); // connect immediately
            // if desired, set up a video resize listener
            ofAddListener(c->videoResized, this, &testApp::videoResized);
        #else
            ofxSharedIpCamera c( new ofxIpCamera());
            c->setVerbose(true);
            c->init(IP_CAMERA_W, IP_CAMERA_H);
            c->setUri(cam.url);
            c->setCredentials(cam.username, cam.password);

        #endif
        ipGrabber.push_back(c);
    }
}
///*****************************************************************
///                      IP CAMERA VIDEO RESIZED
///*****************************************************************
#ifdef OFX_IP_VIDEO_GRABBER
void testApp::videoResized(const void * sender, ofResizeEventArgs& arg) {
    // find the camera that sent the resize event changed
    for(int i = 0; i < NUM_IP_CAMERAS; i++) {
        if(sender == &ipGrabber[i]) {
            stringstream ss;
            ss << "videoResized: ";
            ss << "Camera connected to: " << ipGrabber[i]->getURI() + " ";
            ss << "New DIM = " << arg.width << "/" << arg.height;
            ofLogVerbose("testApp") << ss.str();
        }
    }
}
#endif

///*****************************************************************
///                            LOAD MOVIE
///*****************************************************************
void testApp::loadMovie() {
    ofFileDialogResult fileDialogResult = ofSystemLoadDialog("Open video file", false);
    if (fileDialogResult.bSuccess) {
        currFrameNum = 0;
   	    string fPath = fileDialogResult.getPath();
        vidPlayer.closeMovie();
        vidPlayer.loadMovie(fPath);
        analysisW = inW = vidPlayer.width;
        analysisH = inH = vidPlayer.height;
     //   printf("movie %s loaded: %s\nfull: %ix%i\n", fPath.c_str(), bIsMovieFileLoaded? "TRUE" : "FALSE",inW, inH);
        vidPlayer.setLoopState(OF_LOOP_NORMAL);
        if (!bMovieAudio) vidPlayer.setVolume(0);
        vidPlayer.play();
        setupAnalisys(inW, inH);
    }
}
///*****************************************************************
///                      CREATE OUPUT WINDOW
///*****************************************************************
void testApp::openOutWin(){
#ifdef OFX_FENSTER
    outWin=ofxFensterManager::get()->createFenster(400, 300, 400, 300, OF_WINDOW);
    outWin->addListener(&outWinListener);
    outWin->setWindowTitle("output");
    outWin->hideCursor();
#endif
}
///*****************************************************************
///                      DESTROY OUPUT WINDOW
///*****************************************************************
void testApp::closeOutWin(){
#ifdef OFX_FENSTER
    outWin->destroy();
#endif
}
///*****************************************************************
///                            UPDATE
///*****************************************************************
void testApp::update(){
    // update the IP cameras
    updateIpImages();
    // update the OSC-dependant data
    updateOSC();
    //
    if (bLoadDefaults) {
        bLoadDefaults = false;
        settings.setDefaults();
    }
    if (bLiveVideo) {
        if (bIsVideoGrabbing) {
            vidGrabber.update();
            bNewFrame = vidGrabber.isFrameNew();
        }
        else bNewFrame = false;
    }
    else {
      if (!vidPlayer.isLoaded()) return;
      vidPlayer.idleMovie();
      bNewFrame = vidPlayer.isFrameNew();
    }
	if (bNewFrame) {
	    updateCvCinema();

	    updateBlendImage();
	}
}
///*****************************************************************
///                            UPDATE OSC
///*****************************************************************
void testApp::updateOSC() {
#ifdef OFX_OSC
    // check for waiting messages
	//while(receiver.hasWaitingMessages()){
	if (receiver.hasWaitingMessages()){
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(&m);

		// check for blend message
		if(m.getAddress() == "/blend/values"){
			// the arguments are int32's
			// an alpha value for each image to blend
			// the sum of all four should be <= 255
			for (int i = 0; i < NUM_CAMERAS; i++) {
                alphaValues[i] = m.getArgAsInt32(i);
            //    printf("va1_%i: %i   ", i, alphaValues[i]);
			}
			//printf("\n");
		}
	}
#endif
}
///*****************************************************************
///                            UPDATE CVCINEMA
///*****************************************************************
void testApp::updateCvCinema() {
  // Calculate video framerate
    int currTime = ofGetElapsedTimeMillis();
    videoFPS = 1000.0/(currTime-lastTime);
    lastTime = currTime;
    if (bLiveVideo) {
        // set current frame number
        currFrameNum++;
        fullFrame.setRoiFromPixels(vidGrabber.getPixels(), inW, inH);
    }
    else {
        // get current frame number
        currFrameNum = vidPlayer.getCurrentFrame();
        fullFrame.setRoiFromPixels(vidPlayer.getPixels(), inW, inH);
    }
    // save last frame & update current frame
    lastFrame = colorImg;
    colorImg.scaleIntoMe(fullFrame, CV_INTER_NN);
    // for the advancedBGS
    RgbImage rgbImage = cvCloneImage(colorImg.getCvImage());
    if (bLearnBakground) {
        bgImage = colorImg;		// let this frame be the background image from now on
        bgs.bgsPtr->InitModel(rgbImage);
        bLearnBakground = false;
    }
    else {
        // Subtract the current frame from the background model and produce a binary foreground mask using
        // both a low and high threshold value.
        bgs.bgsPtr->Subtract(currFrameNum, rgbImage, bgs.low, bgs.high);
        // Update the background model. Only pixels set to background in update_mask are updated.
        bgs.bgsPtr->Update(currFrameNum, rgbImage, bgs.low );
    }
    switch (algorithm) {
       case 0:     // color frame differencing
                // take the abs value of the difference between last and current frame
                cvAbsDiff(colorImg.getCvImage(), lastFrame.getCvImage(), colorDiff.getCvImage());
                grayDiff  = colorDiff;
                // apply dilate to reduce pixel noise interference
                grayDiff.dilate_3x3();
                // apply the threshold
                grayDiff.threshold(threshold);
                break;
        case 1:     // grayscale frame differencing
                lastFrameGray = lastFrame;
                grayImage = colorImg;
                // take the abs value of the difference between last and current frame
                grayDiff.absDiff(lastFrameGray, grayImage);
                // apply dilate to reduce pixel noise interference
                grayDiff.dilate_3x3();
                // apply the threshold
                grayDiff.threshold(threshold);
                break;
        case 2:     //  backbround subtraction
                grayImage = colorImg;
                grayBgImage = bgImage;
                //grayImage.dilate_3x3();
                //grayBgImage.dilate_3x3();
                // take the abs value of the difference between background and incoming and then threshold:
                grayDiff.absDiff(grayBgImage, grayImage);
                // apply erode & dilate to reduce pixel noise interference
                grayDiff.erode_3x3();
                grayDiff.dilate_3x3();
                // apply the threshold
                grayDiff.threshold(threshold);
                break;
        case 3:     // brightness tracking
                grayDiff = colorImg;
                // get the brightest pixel and set the threshold to its brightness
                // NOTE: surprisingly, initializing the variable (with this sentence: int max_brightness = 0;)
                // would interfere with the "case" statement
                int max_brightness;
                max_brightness = 0;
                // Same NOTE here about the variable initialization
                unsigned char * gray_pixels;
                gray_pixels = grayDiff.getPixels();
                for (int i = 0; i < analysisW*analysisH; i++) {
                   if (gray_pixels[i] > max_brightness) max_brightness = gray_pixels[i];
                }
                // apply erode & dilate to reduce pixel noise interference
                grayDiff.erode_3x3();
                grayDiff.dilate_3x3();
                // apply the threshold
                grayDiff.threshold(max_brightness-threshold);
                break;
        case 4:     //  backbround subtraction + frame differencing
                // grayscale frame differencing
                lastFrameGray = lastFrame;
                grayImage = colorImg;
                // take the abs value of the difference between last and current frame
                grayTemp.absDiff(lastFrameGray, grayImage);
                // apply dilate to reduce pixel noise interference
                //grayTemp.dilate_3x3();
                // background subtraction
                grayImage = colorImg;
                grayBgImage = bgImage;
                // take the abs value of the difference between background and incoming and then threshold:
                grayDiff.absDiff(grayBgImage, grayImage);
                // apply erode & dilate to reduce pixel noise interference
                grayDiff.erode_3x3();
                grayDiff.dilate_3x3();
                // add: BGS + FD
                grayDiff += grayTemp;
                // apply the threshold
                grayDiff.threshold(threshold);
                break;
        case 5:     //  Adaptive GMM BG subtraction
                bgImage.setFromPixels((unsigned char *)(bgs.bgsPtr->Background()->Ptr()->imageData),analysisW,analysisH);
                grayDiff.setFromPixels((unsigned char *)bgs.low.Ptr()->imageData,analysisW,analysisH);
                break;
    }

    // the contourfinder raises an exception depending on the preprocessing
    // if there is not much difference within the thresholded image
    try { // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        contourFinder.findContours(grayDiff, 20, (analysisW*analysisH)/3, MAX_BLOBS, false);	// find holes = false
    }
    catch( char * str ) { cout << "Exception raised: " << str << '\n'; }
    // if we find blobs we use them, if not we use the previous ones
    if (contourFinder.nBlobs != 0 ) lastBlobs = contourFinder.blobs;
    // Set to black the output pixels background
    memset( outputImg.getCvImage()->imageData, 0, outW*outH*3 );
    // -----------------  GENERATE OUTPUT IMAGE ----------------
    switch (cropping) {
        case '6': monoCmonoB(); break;
        case '7': monoCmultiB(); break;
        case '8': if (lastBlobs.size() > 1) multiC();
                  else monoCmonoB();
                  break;
    }
    if ((hFlip) || (vFlip)) {
        int flipMode;
        if ((hFlip) && (vFlip)) flipMode = -1;
        else if (hFlip) flipMode = 1;
        else if (vFlip) flipMode = 0;
        cvFlip( outputImg.getCvImage(), NULL, flipMode);
    }
}
/// ****************************************************
///
///             UPDATE BLEND IMAGE
///
/// ****************************************************
void testApp::updateBlendImage() {
  // Temporary storage.
  IplImage* s = cvCreateImage( cvSize(outW,outH), IPL_DEPTH_8U, 3 );
  cvSet(s, cvScalar(0,0,0));

    ofxCvColorImage tempImg;
    tempImg.allocate(outW, outH);

    for(size_t i = 0; i < ipGrabber.size(); i++) {
        //ofSetColor(255, 255, 255,alphaValues[i]);
        //ipGrabber[i]->draw(xpos,ypos,w,h);
        //ipImg[i].setFromPixels(ipGrabber[i]->getPixels(), outW, outH);
        int ipWidth = ipGrabber[i]->getWidth();
        int ipHeight = ipGrabber[i]->getHeight();
        try {
            if ((ipWidth != outW) || (ipHeight != outH)) {
                ofxCvColorImage tempIpImg;
                tempIpImg.allocate(ipWidth, ipHeight);

                    tempIpImg.setFromPixels(ipGrabber[i]->getPixels(), ipWidth, ipHeight);
                    tempImg.scaleIntoMe(tempIpImg, OF_INTERPOLATE_NEAREST_NEIGHBOR);
                    cvAddWeighted( s, 1.0,
                                   tempImg.getCvImage(), (float)alphaValues[i]/255.0,
                                   0.0, s);
            }
            else {
                tempImg.setFromPixels(ipGrabber[i]->getPixels(), outW, outH);
                cvAddWeighted( s, 1.0,
                               tempImg.getCvImage(), (float)alphaValues[i]/255.0,
                               0.0, s);
            }
         }
        catch (Exception& e) {
            ofLogError("testApp") << "Exception : " <<  e.displayText();
        }
    }
  cvAddWeighted( s, 1.0,
                outputImg.getCvImage(), (float)alphaValues[NUM_CAMERAS-1]/255.0,
                0.0, (unsigned char *)s);
  blendImg.setFromPixels((unsigned char *)s->imageData,outW,outH );
  cvReleaseImage( &s );
}

/// ****************************************************
///
///             UPDATE IP IMAGES
///
/// ****************************************************
void testApp::updateIpImages() {
    int ipWidth = 0;
    int ipHeight = 0;
    for(size_t i = 0; i < ipGrabber.size(); i++) {
        ipWidth = 320;
        ipHeight = 240;
        if (ipImg[i].bAllocated) ipImg[i].resize(ipWidth, ipHeight);
        else ipImg[i].allocate(ipWidth, ipHeight);
        ipGrabber[i]->update();
        /*ipWidth = ipGrabber[i]->getWidth();
        ipHeight = ipGrabber[i]->getHeight();
        if ((ipWidth > 0) && (ipHeight >0) ) {
           if ((ipWidth != ipImg[i].getWidth()) || (ipHeight != ipImg[i].getHeight())) {
                if (ipImg[i].bAllocated) ipImg[i].resize(ipWidth, ipHeight);
                else ipImg[i].allocate(ipWidth, ipHeight);
           }
            if ((ipWidth != outW) || (ipHeight != outH)) {
                ofxCvColorImage tempIpImg;
                tempIpImg.allocate(ipWidth, ipHeight);
                tempIpImg.setFromPixels(ipGrabber[i]->getPixels(), ipWidth, ipHeight);
                ipImg[i].scaleIntoMe(tempIpImg, OF_INTERPOLATE_NEAREST_NEIGHBOR);
            }
            else {
                ipImg[i].setFromPixels(ipGrabber[i]->getPixels(), ipWidth, ipHeight);
            }
        }*/
    }
}
///*****************************************************************
///                            MONOCHANNEL MONOBLOB
///******************************************************************/
void testApp::monoCmonoB(){
    int numBlobs = lastBlobs.size();
    if (numBlobs > 0) {
        if (bZoomTarget) {
            left = lastBlobs[0].boundingRect.x * in_analysis_scale;
            top = lastBlobs[0].boundingRect.y  * in_analysis_scale;
            targW = lastBlobs[0].boundingRect.width * in_analysis_scale;
            targH = lastBlobs[0].boundingRect.height * in_analysis_scale;
            // adjust to mantain inAspect ratio
            int targW_inAspect = targH*inAspect;
            if (targW < targW_inAspect) {
                left -= (targW_inAspect-targW)/2;
                targW = targW_inAspect;
            }
            else {
                int targH_inAspect = targW/inAspect;
                top -= (targH_inAspect-targH)/2;
                targH = targH_inAspect;
            }
        }
        else {
            targW = cropW;
            targH = cropH;
            top = lastBlobs[0].centroid.y*in_analysis_scale-targH/2;
            left = lastBlobs[0].centroid.x*in_analysis_scale-targW/2;
        }
        // copyRegion needs variables as argumets
        int out_left = 0;

        copyRegion( fullFrame, left, top, targW, targH,
                    outputImg, out_left, out_H_gap, outW, out_H_in_aspect);
    }
}
///*****************************************************************
///                            MONOCHANNEL MULTIBLOB
///******************************************************************/
void testApp::monoCmultiB(){
    int numBlobs = lastBlobs.size();
    if (numBlobs == 1) monoCmonoB();
    else if (numBlobs > 1) {
        int max_x = 0;
        int max_y = 0;
        for (unsigned int i = 0; i < lastBlobs.size(); i++) {

            left = MIN( left, lastBlobs[i].boundingRect.x) ;
            top  = MIN( top,  lastBlobs[i].boundingRect.y) ;
            max_x = MAX( max_x, lastBlobs[i].boundingRect.x+
                                lastBlobs[i].boundingRect.width );
            max_y = MAX( max_y, lastBlobs[i].boundingRect.y+
                                lastBlobs[i].boundingRect.height );
        }
        left *= in_analysis_scale;
        top *= in_analysis_scale;
        max_x *= in_analysis_scale;
        max_y *= in_analysis_scale;
        if (bZoomTarget) {
            targW = (max_x-left);
            targH = (max_y-top);
            // adjust to mantain inAspect ratio
            int targW_inAspect = targH*inAspect;
            if (targW < targW_inAspect) {
                left -= (targW_inAspect-targW)/2;
                targW = targW_inAspect;
            }
            else {
                int targH_inAspect = targW/inAspect;
                top -= (targH_inAspect-targH)/2;
                targH = targH_inAspect;
            }
        }
        else {
            targW = cropW;
            targH = cropH;
            // centroid of all blobs
            top = (float)(top+max_y)/2;
            top -= ((float)targH/2);
            left = (float)(left+max_x)/2;
            left -= ((float)targW/2);
        }
        int out_left = 0;
//        int out_H = outH;
        copyRegion( fullFrame, left, top, targW, targH,
                    outputImg, out_left, out_H_gap, outW, out_H_in_aspect);
    }
}

///*****************************************************************
///                            MULTICHANNEL
///******************************************************************/
void testApp::multiC(){
    int numBlobs = lastBlobs.size();
     if (numBlobs > 0) {
    // calculate number of rows & columns needeed
        float sqroot = sqrtf(numBlobs);

        float trun = truncf(sqroot);
        float rnd = roundf(sqroot);

        if (trun == rnd) rnd +=0.5;
        else trun += 0.5;

        int rows = (int)roundf(trun);
        if (rows <= 0) rows = 1;
        int cols = (int)roundf(rnd);
        if (cols <= 0) cols = 1;

    // calculate channel width and height
        int channelW = (float)outW/(float)cols;
        int channelH = channelW/outAspect;
        int comonGap = (outH-channelH*rows)/2;
        int channelGap = out_H_gap*((float)channelH/(float)outH); // letterbox black zones height
        // draw channels to output buffer image
        for (int i =0; i < numBlobs;i++) {
           int dx = (i%cols)*channelW;
           int dy = comonGap+(i/cols)*(channelH+channelGap);
            targW = cropW;
            targH = cropH;
            top = 0;
            left = 0;

            if (bZoomTarget) {
                 top = lastBlobs[i].boundingRect.y;
                 left = lastBlobs[i].boundingRect.x;
                 targW = lastBlobs[i].boundingRect.width;
                 targH = lastBlobs[i].boundingRect.height;
                 targW = MAX(targW, targH/inAspect);
                 targH = MAX(targH, targW*inAspect);

            }
            else {
                top = lastBlobs[i].centroid.y;
                left = lastBlobs[i].centroid.x;
            }
        // whithout the (float) casting a segmentation fault occurs
        top = in_out_scale*(float)top;
        top -= ((float)targH/2);
        left = in_out_scale*(float)left;
        left -= ((float)targW/2);

         copyRegion( fullFrame, left, top, targW, targH,
                     outputImg, dx, dy, channelW, channelH);
        }
     }
}
///*****************************************************************
///                            COPY REGION
///******************************************************************/
void testApp::copyRegion(ofxCvColorImage &src, int &sx, int &sy, int &sw, int &sh,
                         ofxCvColorImage &dst, int &dx, int &dy, int &dw, int &dh){
    if (sy < 0) sy = 0;
    else if (sy > src.height-sh) sy = src.height-sh;
    if (sx < 0) sx = 0;
    else if (sx > src.width-sw) sx = src.width-sw;

    src.setROI(sx, sy, sw, sh );
    dst.setROI(dx,dy,dw, dh);
    dst.scaleIntoMe(src, CV_INTER_NN);
    dst.resetROI();
    src.resetROI();
}
/*
void testApp::copyRegion(ofxCvColorImage &src, int &sx, int &sy, int &sw, int &sh,
                         ofxCvColorImage &dst, int &dx, int &dy, int &dw, int &dh){
    if (sy < 0) sy = 0;
    else if (sy > (*src).height-sh) sy = (*src).height-sh;
    if (sx < 0) sx = 0;
    else if (sx > (*src).width-sw) sx = (*src).width-sw;

    (*src).setROI(sx, sy, sw, sh );
    (*dst).setROI(dx,dy,dw, dh);
    (*dst).scaleIntoMe((*src), CV_INTER_NN);
    (*dst).resetROI();
    (*src).resetROI();
}
*/
///*****************************************************************
///
///                            DRAW
///
///******************************************************************
void testApp::draw(){
   ofBackground(128);
   if (!bAnalisysAllSetup) return;
   ofSetFullscreen(bFullScreen);
   if (!bFullScreen) drawMainScreen();
   else drawFullScreen();
   if (bShowGui) gui.draw();
}
/// ****************************************************
///
///                     DRAW MAIN SCREEN
///
///
/// ****************************************************
void testApp::drawMainScreen() {
    int yPos = DRAW_SPACE;
    int xPos = DRAW_SPACE;
    int draw_H_in_aspect = round((float)DRAW_W/(float)inAspect);
    int lineH = 12;

    // draw the incoming, the grayscale, the bg and the thresholded difference
    //-------------------  INPUT
    ofFill();
    ofSetHexColor(0x000000);
    ofRect(xPos,yPos,DRAW_W,DRAW_H);
    ofSetHexColor(0xffffff);
    colorImg.draw(xPos,yPos+draw_H_gap,DRAW_W,draw_H_in_aspect);
    ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString(ofToString(currFrameNum), DRAW_SPACE, yPos + DRAW_H-lineH);
    // red rectangle showing selected area
    ofPushMatrix();
    ofSetHexColor(0xff0000);
    ofNoFill();
    ofSetLineWidth(2);
    int t = round((float)top/in_draw_scale);
    int l = round((float)left/in_draw_scale);
    int tW = round((float)targW/in_draw_scale);
    int tH = round((float)targH/in_draw_scale);
    ofRect( DRAW_SPACE+l, DRAW_SPACE+draw_H_gap+t,tW, tH);
    ofPopMatrix();
    //-------------------  OUTPUT
    ofSetHexColor(0x000000);
    ofRect(xPos+DRAW_W+DRAW_SPACE,yPos,DRAW_W,DRAW_H);
    ofSetHexColor(0xffffff);

    outputImg.draw(xPos+DRAW_W+DRAW_SPACE,yPos,DRAW_W,DRAW_H);
    //------------------- BACKGROUND
    yPos += DRAW_H+DRAW_SPACE;
 /*   ofSetHexColor(0x000000);
    ofRect(xPos,yPos,DRAW_W,DRAW_H);
    ofSetHexColor(0xffffff);
    bgImage.draw(xPos,yPos+draw_H_gap,DRAW_W,draw_H_in_aspect);
*/
    drawBlendSources(xPos,yPos,DRAW_W,DRAW_H);
    //-------------------- THRESHOLDED DIFFERENCE
    xPos += DRAW_W+DRAW_SPACE;
   /* ofSetHexColor(0x000000);
    ofRect(xPos,yPos,DRAW_W,DRAW_H);
    ofSetHexColor(0xffffff);
    yPos+=draw_H_gap;
    grayDiff.draw(xPos,yPos,DRAW_W,draw_H_in_aspect);
*/
    // we could draw the whole contour finder
   // contourFinder.draw(xPos,yPos,DRAW_W,draw_H_in_aspect);
    //drawBlending(xPos,yPos,DRAW_W,DRAW_H);
    ofSetHexColor(0x000000);
    ofRect(xPos,yPos,DRAW_W,DRAW_H);
    ofSetHexColor(0xffffff);
    blendImg.draw(xPos,yPos,DRAW_W,DRAW_H);

    drawReport(xPos, yPos, lineH);

    // LOGOS
    yPos = 2*(DRAW_H+DRAW_SPACE);
    xPos = DRAW_SPACE;

    yPos = ofGetHeight()-imgLogos.height;
    ofSetHexColor(0xffffff);
    imgLogos.draw(0, yPos);

    // REPORT / MENU


}
/// ****************************************************
///
///                     DRAW REPORT
///
///
/// ****************************************************
void testApp::drawReport(int xPos, int yPos, int lineH) {
    xPos += DRAW_W+DRAW_SPACE;
    yPos = DRAW_SPACE+lineH;
    ofPushMatrix();
    ofSetHexColor(FONT_COLOR);
    char reportStr[1024];
    sprintf( reportStr, "CVCINEMA SYSTEM 1.0\n"
                       "Copyright (c) 2012 UPV\n"
                       "www.cvcinema.com\n\n"
                       "' ' -> full screen output\n"
                       "'l' -> live video\n"
                       "'o' -> video file\n"
                       "'h' -> hFlip\n"
                       "'v' -> vFlip\n"
                       "threshold %i (press: +/-)\n\n"
                       "num blobs found %i\n"
                       "screen fps: %.0f\n"
                       "video fps: %.0f",
            threshold, contourFinder.nBlobs, ofGetFrameRate(), videoFPS);


    ofDrawBitmapString(reportStr, xPos, yPos );

    yPos += lineH*16;
    (algorithm == 1)? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("1 -> Frame differencing", xPos, yPos+=lineH);

    (algorithm == 2)? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("2 -> Background subtraction", xPos, yPos+=lineH);

    (algorithm == 3)? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("3 -> Brightness thresholding", xPos, yPos+=lineH);

    (algorithm == 4)? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("4 -> BG subtraction + F differencing", xPos, yPos+=lineH);

    (algorithm == 5)? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("5 -> Adaptive GMM BG subtraction", xPos, yPos+=lineH);

    yPos+=lineH ;
    (cropping == '6')? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("6 -> Monochannel, centroid of MS_Blob", xPos, yPos+=lineH);

    (cropping == '7')? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("7 -> Monochannel, centroid of All_Blobs", xPos, yPos+=lineH);

   (cropping == '8')? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("8 -> Multichannel", xPos, yPos+=lineH);

    yPos+=lineH ;
    (bZoomTarget)? ofSetHexColor(FONT_SEL_COLOR) : ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("z -> toggle ZOOM", xPos, yPos+=lineH);

    yPos+= DRAW_SPACE ;
    ofSetHexColor(FONT_COLOR);
    ofDrawBitmapString("b -> Learn background", xPos, yPos+=lineH);

    yPos+= DRAW_SPACE ;

    sprintf(reportStr,"rows: %i  cols: %i",rows, cols);
    ofDrawBitmapString(reportStr, xPos, yPos+=lineH);

    sprintf(reportStr,"r -> Record (%s): ",(selOutput == OUTPUT_IMAGE)? "output":"analysis");
    ofDrawBitmapString(reportStr, xPos, yPos+=lineH);
    if (bRecording) {
        int r = (ofGetElapsedTimeMillis() % 1000) < 500? 255:0;
        ofSetColor(255,r,r);
        sprintf(reportStr,"RECORDING");
        ofDrawBitmapString(reportStr, xPos+175, yPos);
    }
    else {
        ofSetHexColor(FONT_COLOR);
        sprintf(reportStr,"NOT RECORDING");
        ofDrawBitmapString(reportStr, xPos+175, yPos);
    }
    ofPopMatrix();
}
/// ****************************************************
///
///             DRAW BLEND SOURCES
///
/// ****************************************************
void testApp::drawBlendSources(int xpos, int ypos, int w, int h) {

    ofSetHexColor(0xffffff);

    int row = 0;
    int col = 0;

    int x = xpos;
    int y = ypos;

    int colw = w / NUM_COLS;
    int rowh = h / NUM_ROWS;

    ofPushMatrix();
    ofFill();
    ofSetHexColor(0x000000);
    ofRect(xpos,ypos,w,h);

    for(size_t i = 0; i < ipGrabber.size(); i++) {
        x = xpos+col * colw;
        y = ypos+row * rowh;

        // draw in a grid
        row = (row + 1) % NUM_ROWS;
        if(row == 0) {
            col = (col + 1) % NUM_COLS;
        }
        ofPushMatrix();
        ofTranslate(x,y);
        ofSetColor(255,255,255,255);
        ipGrabber[i]->draw(0,0,colw,rowh); // draw the camera
  //      ipImg[i].draw(0,0,colw,rowh); // draw the camera

#ifdef OFX_IP_VIDEO_GRABBER
        float kbps = ipGrabber[i]->getBitRate() / 1000.0f; // kilobits / second, not kibibits / second
        float fps = ipGrabber[i]->getFrameRate();

        stringstream ss;
        // ofToString formatting available in 0072+
        ss << "NAME: " << ipGrabber[i]->getCameraName() << endl;
        //ss << "HOST: " << ipGrabber[i]->getHost() << endl;
        ss << " FPS: " << ofToString(fps,  2/*,13,' '*/) << endl;
        ss << "Kb/S: " << ofToString(kbps, 2/*,13,' '*/) << endl;
        //ss << " #Bytes Recv'd: " << ofToString(ipGrabber[i]->getNumBytesReceived(),  0/*,10,' '*/) << endl;
        //ss << "#Frames Recv'd: " << ofToString(ipGrabber[i]->getNumFramesReceived(), 0/*,10,' '*/) << endl;
        //ss << "Auto Reconnect: " << (ipGrabber[i]->getAutoReconnect() ? "YES" : "NO") << endl;
        //ss << " Needs Connect: " << (ipGrabber[i]->getNeedsReconnect() ? "YES" : "NO") << endl;
        //ss << "Time Till Next: " << ipGrabber[i]->getTimeTillNextAutoRetry() << " ms" << endl;
        ///ss << "Num Reconnects: " << ofToString(ipGrabber[i]->getReconnectCount()) << endl;
        //ss << "Max Reconnects: " << ofToString(ipGrabber[i]->getMaxReconnects()) << endl;
        //ss << "  Connect Fail: " << (ipGrabber[i]->hasConnectionFailed() ? "YES" : "NO");
        ofSetColor(0);
        ofDrawBitmapString(ss.str(), 10+1, 10+12+1);
        ofSetColor(255);
        ofDrawBitmapString(ss.str(), 10, 10+12);
#endif
        ofPopMatrix();
    }
    ofSetColor(255,255,255,255);

    outputImg.draw(xpos+colw,ypos+rowh,colw,rowh);
    ofPopMatrix();
}
/// ****************************************************
///
///             DRAW BLEND SOURCES
///
/// ****************************************************
void testApp::drawBlending(int xpos, int ypos, int w, int h) {
    ofPushMatrix();
    ofFill();
    ofSetHexColor(0x000000);
    ofRect(xpos,ypos,w,h);
    ofEnableBlendMode(blendMode);
    for(size_t i = 0; i < ipGrabber.size(); i++) {
        ofSetColor(255, 255, 255,alphaValues[i]);
        ipGrabber[i]->draw(xpos,ypos,w,h);
    }
    ofSetColor(255, 255, 255,alphaValues[NUM_CAMERAS-1]);
    outputImg.draw(xpos,ypos,w,h);
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofDisableBlendMode();
    ofPopMatrix();
}

/// ****************************************************
///
///                     DRAW FULL SCREEN
///
///
/// ****************************************************
void testApp::drawFullScreen() {
        ofSetHexColor(0x000000);
        ofRect(0,0,ofGetWidth(),ofGetHeight());
        ofSetHexColor(0xffffff);
//        float screenAspect = (float)ofGetWidth()/(float)ofGetHeight();
        int fullScreenH = round((float)ofGetWidth()/(float)outAspect);
        int aspect_H_offset = round((float)(fullScreenH-ofGetHeight())/2.0);
  //      cout << fullScreenH << endl;
        /*switch(selOutput) {
            case OUTPUT_IMAGE:
          */

          drawBlending(0,-aspect_H_offset,ofGetWidth(),fullScreenH);
          //outputImg.draw(0,-aspect_H_offset,ofGetWidth(),fullScreenH);

           /* break;
            case ANALYSIS_WINDOW:
                grayDiff.draw(0,-aspect_H_offset,ofGetWidth(),fullScreenH);
            break;
            case BG_IMAGE:
                bgImage.draw(0,-aspect_H_offset,ofGetWidth(),fullScreenH);
            break;
            case FOUR_WINDOWS:
                //colorImg
                //outputImg.draw(0,-out_H_gap*ofGetWidth()/outW,ofGetWidth(),ofGetWidth()/outAspect);
                //bgImage.draw(0,-out_H_gap*ofGetWidth()/outW,ofGetWidth(),ofGetWidth()/outAspect);
                //grayDiff.draw(0,-out_H_gap*ofGetWidth()/outW,ofGetWidth(),ofGetWidth()/outAspect);
            break;
            case INPUT_IMAGE:
                fullFrame.draw(0,-aspect_H_offset,ofGetWidth(),fullScreenH);
            break;
            default:
            break;
        }*/
}

/// ****************************************************
///
///                     CARTOON FILTER
///
/// ****************************************************
bool testApp::cvFilterCartoon(ofxCvColorImage &src, ofxCvColorImage &dst, int w, int h)
{
    //CvtColor(src, dst, code)
    //cv::cvtColor(inputFrame, bgr, CV_BGRA2BGR);
    //  cv::pyrMeanShiftFiltering(bgr.clone(), bgr, sp, sr);
    // PyrMeanShiftFiltering(src, dst, sp, sr, max_level=1, termcrit=(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 5, 1))


    // Temporary storage.
    IplImage* pyr = cvCreateImage( cvSize(w,h), IPL_DEPTH_8U, 3 );
    IplImage* edges = cvCreateImage( cvSize(w,h), IPL_DEPTH_8U, 1 );
    IplImage* edgesRgb = cvCreateImage( cvSize(w,h), IPL_DEPTH_8U, 3 );
    //cvSet(s, cvScalar(0,0,0));


    ofxCvGrayscaleImage tempGrayImg;

    tempGrayImg.allocate(w, h);


    tempGrayImg.setFromColorImage(src);


    //------------------------------
    cvPyrMeanShiftFiltering(src.getCvImage(), pyr, 10, 10);

    //  cv::Canny(gray, edges, 150, 150);
    cvCanny(tempGrayImg.getCvImage(), edges, 150,150);
    cvCvtColor(edges, edgesRgb, CV_GRAY2RGB);
    cvAbsDiff(pyr, edgesRgb, pyr);
    //cvAbsDiff(colorImg.getCvImage(), lastFrame.getCvImage(), colorDiff.getCvImage());
    dst.setFromPixels((unsigned char *)pyr->imageData, w, h);
    return true;
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
        sprintf( fileName, "output_%i___thresh-%i_alg-%i_chan-%i_zoom-%s.avi",
                 recorderFileNumber, threshold,algorithm,cropping, (bZoomTarget? "zoom":"noZoom"));
        ofFile file(fileName);
        fileExists = file.exists();
    } while (fileExists);
    cout << "START RECORDING to file: " << fileName << endl;

 recorder.setup(blendImg,24, ofToDataPath(fileName),ofxGstVideoRecorder::H264,25);

  /*  switch (selOutput) {

        case OUTPUT_IMAGE:
            recorder.setup(outputImg,24, ofToDataPath(fileName),ofxGstVideoRecorder::H264,25);

           // recorder.setup(outputImg,24, fileName,ofxGstVideoRecorder::H264,25);
        break;
        case INPUT_IMAGE:


            recorder.setup(fullFrame,24, ofToDataPath(fileName),ofxGstVideoRecorder::H264,25);
          //recorder.setup(fullFrame,24, "./data/output_"+ofToString(recorderFileNumber)+"___thresh-"+ofToString(threshold)+"_alg-"+ofToString(algorithm)+
            //                             "_chan-"+cropping+"_zoom-"+(bZoomTarget? "zoom":"noZoom")+".mp4",ofxGstVideoRecorder::H264,25);
       break;

        case ANALYSIS_WINDOW:
            recorder.setupRecordWindow( DRAW_W+2*DRAW_SPACE, DRAW_H+2*DRAW_SPACE,
                                        DRAW_W, DRAW_H, 24,
                                        ofToDataPath(fileName),ofxGstVideoRecorder::H264,25);

        break;


        default:
        break;
    }*/
}
//--------------------------------------------------------------
void testApp::keyPressed  (int key){

	switch (key){
		case ' ':
			bFullScreen = !bFullScreen;
			break;
		case 'b':
			bLearnBakground = true;
			break;
		case '+':
			threshold ++;
			if (threshold > 255) threshold = 255;
			break;
		case '-':
			threshold --;
			if (threshold < 0) threshold = 0;
			break;
        case '3':   threshold = 10;
                    algorithm = key-48; // character '1' is dec number 49
                    break;
        case '0':
		case '1':
		case '2':
		case '4':
		case '5':
                    threshold = 80;
                    algorithm = key-48; // character '1' is dec number 49
                    break;
        case '6':
        case '7':
        case '8':   cropping = key;
                    break;
        case 'Z':
        case 'z':   bZoomTarget = !bZoomTarget;
                    break;
        case 's':   saveFrame = true;
                    break;

        case 'd':   settings.setDefaults();
                    break;
        case 'l':   bLiveVideo = !bLiveVideo;
                    if (bLiveVideo) {
                        vidPlayer.close();
                        setupLiveVideo();
                    }
                    else {
                        vidGrabber.close();
                        loadMovie();
                    }
                    ofSleepMillis(500);
                    break;

        case 'a':   bMovieAudio = !bMovieAudio;
                    if (!bMovieAudio) vidPlayer.setVolume(0);
                    else vidPlayer.setVolume(1);
                    break;
        case 'h':   hFlip = !hFlip; break;
        case 'v':   vFlip = !vFlip; break;

        case 'r':   if (!bRecording) {
                        setupRecording();
                    }
                    else  {
                        recorder.stop();
                        cout << "STOP RECORDING"<< endl;
                    }
                    bRecording = ! bRecording;
                    break;

        case 'g': bShowGui = !bShowGui; break;
        case 'o': bLiveVideo = false;
                  vidGrabber.close();
                  loadMovie();
                  break;
        case 'w': openOutWin(); break;
        case 'i': // initialize connection
                  /*for(int i = 0; i < NUM_CAMERAS; i++) {
                        ofRemoveListener(ipGrabber[i]->videoResized, this, &testApp::videoResized);
                        ofxSharedIpVideoGrabber c( new ofxIpVideoGrabber());
                        IPCameraDef& cam = getNextCamera();
                        c->setUsername(cam.username);
                        c->setPassword(cam.password);
                        URI uri(cam.url);
                        c->setURI(uri);
                        c->connect();
                            ipGrabber[i] = c;
                  }*/
            break;
        case OF_KEY_F1: selOutput =  OUTPUT_IMAGE;
            break;
        case OF_KEY_F2: selOutput =  ANALYSIS_WINDOW;
            break;
        case OF_KEY_F3: selOutput =  INPUT_IMAGE;
            break;
        case OF_KEY_F4: selOutput =  FOUR_WINDOWS;
            break;
        case OF_KEY_F5: selOutput =  BG_IMAGE;
            break;

        default:
            break;
     }
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
 updateControls  = true;
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}


//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){

}

//--------------------------------------------------------------
void testApp::exit(){
        vidPlayer.closeMovie();
}

