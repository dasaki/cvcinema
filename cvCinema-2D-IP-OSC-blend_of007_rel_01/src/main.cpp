#include "ofMain.h"
#include "testApp.h"
#ifdef OFX_FENSTER
#include "ofxFensterManager.h"
#endif
#include "ofAppGlutWindow.h"

//========================================================================
int main( ){
#ifdef OFX_FENSTER
    ofSetupOpenGL(ofxFensterManager::get(), 1024, 600, OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunFensterApp(new testApp());

#else
	    ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024,600, OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp( new testApp());
#endif
}
