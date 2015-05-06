#ifndef CANONCAMERA_H
#define CANONCAMERA_H

#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "EDSDKErrors.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

typedef enum{
    CAMERA_UNKNOWN,
    CAMERA_READY,
    CAMERA_OPEN,
    CAMERA_CLOSED,
}cameraState;

static int sdkRef = 0;
static void easyRelease(EdsBaseRef &ref){
    if(ref != NULL){
        EdsRelease(ref);
        ref = NULL;
    }
}

class CanonCamera
{
public:
    CanonCamera();
    virtual ~CanonCamera();
    //---------------------------------------------------------------------
     //  SDK AND SESSION MANAGEMENT
     //---------------------------------------------------------------------
    bool setup(int cameraID);   //You must call this to init the canon sdk
    void destroy();             //To clean up - also called by destructor

    bool openSession();         //Begins communication with camera
    bool closeSession();        //Ends communication with camera.
                                //Note on sessions: Commands like takePicture
                                //will open a session if none exists. This
                                //is slower though so consider calling it
                                //once at the begining of your app.
    //---------------------------------------------------------------------
    //  CONFIG
    //---------------------------------------------------------------------
   // void setDeleteFromCameraAfterDownload(bool deleteAfter);
    void setDownloadPath(std::string downloadPathStr);
    void enableDownloadOnTrigger();     //Trigger meaning takePicture
    void disableDownloadOnTrigger();    //Trigger meaning takePicture

    //---------------------------------------------------------------------
    //  ACTIONS
    //---------------------------------------------------------------------
    bool takePicture();   //Takes a picture. If enabled it will also download
                          //the image to the folder set by the download path.

    bool sendCommand( EdsCameraCommand inCommand,  EdsInt32 inParam = 0);

    //---------------------------------------------------------------------
    //  MISC EXTRA STUFF
    //---------------------------------------------------------------------

     std::string getLastImageName();  //The full path of the last downloaded image
     std::string getLastImagePath();  //The name of the last downloaded image

     //This doesn't work perfectly - for some reason it can be one image behind
     //something about how often the camera updates the SDK.
     //Having the on picture event registered seems to help.
     //But downloading via event is much more reliable at the moment.

     //WARNING - If you are not taking pictures and you have bDeleteAfterDownload set to true
     //you will be deleting the files that are on the camera.
     //Simplified: be careful about calling this when you haven't just taken a photo.
     bool downloadLastImage();

     //Hmm - might be needed for threading - currently doesn't work
     //bool isTransfering();
     EdsError getState(){  return  state; }
protected:

    bool downloadImage(EdsDirectoryItemRef directoryItem);



   // void registerCallback();
    bool preCommand();
    void postCommand();



private:
    std::string lastImageName;
    std::string lastImagePath;
    std::string downloadPath;


    bool downloadEnabled;
    bool needToOpen;

    cameraState state;
    EdsCameraRef        theCamera ;
    EdsCameraListRef	theCameraList;
};

#endif // CANONCAMERA_H
