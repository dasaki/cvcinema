#include "ofMain.h"
#include "ofxThread.h"

class ofxThreadedImageSaver : public ofxThread, public ofImage {
public:
   string fileName;
   ofImage myImage;
   void threadedFunction() {
      if(lock()) {
         myImage.saveImage(fileName);
         unlock();
      } else {
         printf("ofxThreadedImageSaver - cannot save %s cos I'm locked", fileName.c_str());
      }
      stopThread();
   }
   void saveThreaded(string fileName, ofImage imageToSave) {
      this->fileName = fileName;
      this->myImage = imageToSave;
      startThread(false, false);
   }
};
