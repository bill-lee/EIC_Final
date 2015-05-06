#include "CanonCamera.h"

using namespace std;

CanonCamera::CanonCamera()
{

    downloadPath="D:/";
    theCamera= NULL;
    needToOpen= false;
    downloadEnabled= true;
    state = CAMERA_UNKNOWN;
}
CanonCamera::~CanonCamera()
{
    this->destroy();
}
bool CanonCamera::setup(int cameraID)
{
   /*
    if( theCamera != NULL || theCameraList != NULL){
           destroy();
       }

       */

    EdsError err = EDS_ERR_OK;
    EdsUInt32 cameraCount = 0 ;

        err = EdsInitializeSDK();

    if(err != EDS_ERR_OK){
         cout<<"Couldn't open sdk!"<<endl;
          return false;
      }else{
          cout<<"Opening the sdk"<<endl;
          sdkRef++;
      }

    Sleep(3000);

    // Initialize
    // Get the camera list
    err = EdsGetCameraList(&theCameraList);

    // Get the number of cameras.
    if( err == EDS_ERR_OK ){
        err = EdsGetChildCount( theCameraList, &cameraCount );
        if ( cameraCount == 0 ){
            err = EDS_ERR_DEVICE_NOT_FOUND;
            cout<<"No devices found!"<<endl;
            return false;
        }
    }


    // Get the camera
    if ( err == EDS_ERR_OK ){
        if (cameraID >= cameraCount){
            cout<<"No camera of id "<<cameraID<<" exists - number of cameras is "<<cameraCount<<endl;
            return false;
        }

        cout<<"We are opening camera:"<<cameraID<<endl;

        err = EdsGetChildAtIndex( theCameraList , cameraID , &theCamera );


        //Release camera list
        if(theCameraList != NULL){
            EdsRelease(theCameraList);
        }

        if(err == EDS_ERR_OK)
        {
            cout<<"We are connected!"<<endl;
            state = CAMERA_READY;
            //return true;
        }else
        {
            cout<<"We are not connected!"<<endl;
            state = CAMERA_UNKNOWN;
            return false;
        }

        cout<<cameraCount<<endl;





        return true;
    }
}

void CanonCamera::destroy()
{

    /*
    if( getLiveViewActive() ){
        endLiveView();
    }

    */

    if( theCamera != NULL){
        closeSession();
    }

    easyRelease(theCamera);
    easyRelease(theCameraList);

    if( sdkRef > 0 ){
        sdkRef--;
        if( sdkRef == 0 ){
            EdsTerminateSDK();
            cout<<"Terminating the sdk"<<endl;
        }
    }





}

bool CanonCamera::openSession()
{
    EdsError err = EDS_ERR_OK;

     err = EdsOpenSession( theCamera );
     if(err == EDS_ERR_OK){
         cout<<"We are opening session!"<<endl;
         state = CAMERA_OPEN;
         return true;
     }
     else{
         cout<<"We failed at opening session!"<<endl;
     }
     return false;

}

bool CanonCamera::closeSession()
{

    if( state == CAMERA_CLOSED)
        return false;
    EdsError err = EDS_ERR_OK;

    err = EdsCloseSession( theCamera );
    if(err == EDS_ERR_OK){
        cout<<"We are closing session!"<<endl;
        state = CAMERA_CLOSED;
        return true;
    }
    else{
        cout<<"We failed at closing session!"<<endl;
    }
    return false;

}

void CanonCamera::setDownloadPath(std::string downloadPathStr)
{
    downloadPath = downloadPathStr;
    if( downloadPath != "" ){
          if(downloadPath[ downloadPath.length()-1 ] != '/' ){
              downloadPath  = downloadPath + "/";
          }
    }
}

bool CanonCamera::takePicture()
{
    return sendCommand(kEdsCameraCommand_TakePicture, 0);
}

bool CanonCamera::sendCommand(EdsCameraCommand inCommand, EdsInt32 inParam)
{
    EdsError err = EDS_ERR_OK;

    if( preCommand() ){
        err = EdsSendCommand(theCamera, inCommand, inParam);

        postCommand();

        if(err == EDS_ERR_OK){
            return true;
        }

        if(err == EDS_ERR_DEVICE_BUSY){
            cout<<"sendCommand - EDS_ERR_DEVICE_BUSY"<<endl;
            return false;
        }

    }

    return false;

}

bool CanonCamera::preCommand()
{

    cout<<"pre command!"<<endl;

    if( state > CAMERA_UNKNOWN ){
        needToOpen = false;
        bool readyToGo  = false;

        if( state != CAMERA_OPEN ){
            needToOpen = true;
        }else{
            readyToGo = true;
            return true;
        }

        if( needToOpen ){
            readyToGo = openSession();
        }

        return readyToGo;

    }else{
        return false;
    }
}

void CanonCamera::postCommand()
{

    cout<<"post command "<<endl;

    if(state == CAMERA_OPEN && needToOpen){
        cout<<"postCommand - closing session"<<endl;
        closeSession();
    }
}

void CanonCamera::enableDownloadOnTrigger()
{
    downloadEnabled = true;
}

void CanonCamera::disableDownloadOnTrigger()
{
    downloadEnabled = true;
}

bool CanonCamera::downloadImage(EdsDirectoryItemRef directoryItem)
{

      if( !downloadEnabled )
          return false;



      EdsError err = EDS_ERR_OK;
              EdsStreamRef stream = NULL;
              EdsDirectoryItemInfo dirItemInfo;

              bool success = false;
              string imageName;
              string imagePath;
     // int timeStart = ofGetElapsedTimeMillis();

      err = EdsGetDirectoryItemInfo(directoryItem, &dirItemInfo);

      if(err == EDS_ERR_OK){

          imageName = dirItemInfo.szFileName;
          imagePath = downloadPath + imageName;

          cout<<"Downloading image"<<imageName.c_str()<<" to "<<imagePath.c_str()<<endl;
          err = EdsCreateFileStream( imagePath.c_str(), kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream);
      }

      if(err == EDS_ERR_OK){
             err = EdsDownload( directoryItem, dirItemInfo.size, stream);
         }

      if(err == EDS_ERR_OK){

                 lastImageName = imageName;
                 lastImagePath = imagePath;

             //    printf("Image downloaded in %ims\n", ofGetElapsedTimeMillis()-timeStart);

                 err = EdsDownloadComplete(directoryItem);

                 /*
                 if( bDeleteAfterDownload ){
                     printf("Image deleted\n");
                     EdsDeleteDirectoryItem(directoryItem);
                 }

                 */
                 success = true;
             }

             easyRelease(stream);
             return success;
}

bool CanonCamera::downloadLastImage()  //將記憶卡圖片下載至電腦
{

    preCommand();

    EdsVolumeRef 		theVolumeRef	    = NULL ;
    EdsDirectoryItemRef	dirItemRef_DCIM	    = NULL;
    EdsDirectoryItemRef	dirItemRef_Sub	    = NULL;
    EdsDirectoryItemRef	dirItemRef_Image    = NULL;


    EdsDirectoryItemInfo dirItemInfo_Image;

     EdsError err    = EDS_ERR_OK;
     EdsUInt32 Count = 0;
     bool success    = false;

     //get the number of memory devices


     err = EdsGetChildCount( theCamera, &Count );
     if( Count == 0 )
     {
         cout<<"Memory device not found"<<endl;
         err = EDS_ERR_DEVICE_NOT_FOUND;
         return false;
    }
    cout<<"num00:"<<Count<<endl;

     // Download Card No.0 contents
     err = EdsGetChildAtIndex( theCamera, 0, &theVolumeRef );

             if ( err == EDS_ERR_OK ){
                 //printf("getting volume info\n");

                 EdsVolumeInfo    volumeInfo;
                 err = EdsGetVolumeInfo( theVolumeRef, &volumeInfo ) ;

                 cout<<"!!"<<volumeInfo.szVolumeLabel<<endl;
             }

     //Now lets find out how many Folders the volume has
     if ( err == EDS_ERR_OK )
     {
         err = EdsGetChildCount( theVolumeRef, &Count );


         if ( err == EDS_ERR_OK ){
             //Lets find the folder called DCIM
             bool bFoundDCIM = false;

             for(int i = 0; i < Count; i++){
                 err = EdsGetChildAtIndex( theVolumeRef, i, &dirItemRef_DCIM ) ;
                 if ( err == EDS_ERR_OK ){
                     EdsDirectoryItemInfo dirItemInfo;
                     err = EdsGetDirectoryItemInfo( dirItemRef_DCIM, &dirItemInfo );
                     if( err == EDS_ERR_OK){
                         string folderName = dirItemInfo.szFileName;
                         if( folderName == "DCIM" ){
                             bFoundDCIM = true;
                             cout<<"Found the DCIM folder at index"<<i<<endl;
                             break;
                         }
                     }
                 }
                 //we want to release the directories that don't match
                 easyRelease(dirItemRef_DCIM);
             }
             ///////////////////////////////////////////////////
             if( bFoundDCIM && dirItemRef_DCIM != NULL){
                  //now we are going to look for the last folder in DCIM
                  Count = 0;
                  err = EdsGetChildCount(dirItemRef_DCIM, &Count);

                  bool foundLastFolder = false;
                  if( Count > 0 ){
                      int lastIndex = Count-1;

                      EdsDirectoryItemInfo dirItemInfo_Sub;

                      err = EdsGetChildAtIndex( dirItemRef_DCIM, lastIndex, &dirItemRef_Sub ) ;
                      err = EdsGetDirectoryItemInfo( dirItemRef_Sub, &dirItemInfo_Sub);

                      cout<<"Last Folder is "<<dirItemInfo_Sub.szFileName<<endl;

                      EdsUInt32 jpgCount = 0;
                      err = EdsGetChildCount(dirItemRef_Sub, &jpgCount );

                      if( jpgCount > 0 ){
                          int latestJpg = jpgCount-1;

                          err = EdsGetChildAtIndex(dirItemRef_Sub, latestJpg, &dirItemRef_Image ) ;
                          err = EdsGetDirectoryItemInfo(dirItemRef_Image, &dirItemInfo_Image);

                          cout<<"Latest image is "<<dirItemInfo_Image.szFileName<<endl;
                          success = true;
                      }else{
                          cout<<"Error - No jpegs inside"<<dirItemInfo_Image.szFileName<<endl;
                      }
                  }else{
                      cout<<"Error - No subfolders inside DCIM!"<<endl;
                  }
              }

         }
     }



     if( success ){
         success = downloadImage(dirItemRef_Image);
     }

     easyRelease(theVolumeRef);
     easyRelease(dirItemRef_DCIM);
     easyRelease(dirItemRef_Sub);
     easyRelease(dirItemRef_Image);

     postCommand();

     return success;



}
