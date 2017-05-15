/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include "aruco.h"
#include "highlyreliablemarkers.h"
#include "cvdrawingutils.h"
#include "chromaticmask.h"

using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
string TheBoardConfigFile;
string TheDictionaryFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
VideoCapture TheVideoCapturer;
Mat TheDistInputImage, TheInputImage,TheInputImageCopy, TheInputImageH;
CameraParameters TheDistCameraParameters, TheCameraParameters;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
ChromaticMask TheChromaticMask;
bool chromatic=false;

string TheOutVideoFilePath;
cv::VideoWriter VWriter;

void cvTackBarEvents(int pos,void*);
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=10;


/************************************
 *
 *
 *
 *
 ************************************/
void getHueImg(const cv::Mat &rgbImg, cv::Mat &hImg)
{
  cv::Mat hsvImg;
  vector<cv::Mat> hsvImg_channels;
  cvtColor(rgbImg, hsvImg, CV_BGR2HSV);
  cv::split(hsvImg, hsvImg_channels);
  hsvImg_channels[0].copyTo(hImg);
}


/************************************
 *
 *
 *
 *
 ************************************/

bool readArguments ( int argc,char **argv )
{

    if (argc<3) {
      cerr<<"Invalid number of arguments"<<endl;
	
      cerr << "Usage: (in.avi|live) dictionary.yml boardConfig.yml [intrinsics.yml] [size] [chromatic] [out.avi] \n \
	in.avi/live: open videofile or connect to a camera using the OpenCV library \n \
	dictionary.yml: input marker dictionary used for detection \n \
	boardConfig.yml: input board configuration file \n \
	intrinsics.yml: input camera parameters (in OpenCV format) to allow camera pose estimation \n \
	size: markers size in meters to allow camera pose estimation \n \
	chromatic: 0 for black&white markers, 1 for green&blue chromatic markers \n \
	out: save video output to videofile" << endl;      	
        return false;
    }
    
    TheInputVideo=argv[1];
    TheDictionaryFile=argv[2];
    TheBoardConfigFile=argv[3];
    if (argc>=5)
        TheIntrinsicFile=argv[4];
    if (argc>=6)
        TheMarkerSize=atof(argv[5]);
    if (argc>=7)
        chromatic = (argv[6][0]=='1');
    if (argc>=8)
        TheOutVideoFilePath=argv[7];

    if (argc==4)
        cerr<<"NOTE: You need makersize to see 3d info!!!!"<<endl;

    return true;
}

void processKey(char k) {
    switch (k) {
    case 's':
        if (waitTime==0) waitTime=10;
        else waitTime=0;
        break;

    // calibrate mask
    case 'm':
	if(!chromatic) return;
	float prob = (float)TheBoardDetector.getDetectedBoard().size() / (float)TheBoardDetector.getDetectedBoard().conf.size();
	//TheChromaticMask.detectBoard( TheInputImageH );
	if(prob>0.2) TheChromaticMask.train(TheInputImageH, TheBoardDetector.getDetectedBoard());
// 	if(prob>0.2) TheVisibilityMask.calibrate(TheBoardDetector.getDetectedBoard(), TheInputImageH, TheCameraParameters, TheMarkerSize);
	break;
	
    }
}

pair<double,double> AvrgTime(0,0) ;

/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc,char **argv)
{
    try
    {
        if (  readArguments (argc,argv)==false) return 0;
	//parse arguments
	
        TheBoardConfig.readFromFile(TheBoardConfigFile);
        //read from camera or from  file
        if (TheInputVideo=="live") {
            TheVideoCapturer.open(0);
            waitTime=10;
        }
        else TheVideoCapturer.open(TheInputVideo);
        //check video is open
        if (!TheVideoCapturer.isOpened()) {
            cerr<<"Could not open video"<<endl;
            return -1;

        }

        // read dictionary
        aruco::Dictionary D;
        if (!D.fromFile(TheDictionaryFile)) {
            cerr<<"Could not open dictionary file"<<endl;
            return -1;
        }     
        HighlyReliableMarkers::loadDictionary(D);

	if(chromatic)
	  std::cout << "Press 'm' key when board is not occluded to calibrate chromatic mask" << std::endl;
	
        //read first image to get the dimensions
        TheVideoCapturer>>TheDistInputImage;

        //Open outputvideo
        if ( TheOutVideoFilePath!="")
            VWriter.open(TheOutVideoFilePath,CV_FOURCC('M','J','P','G'),30,TheDistInputImage.size());

        //read camera parameters if passed
        if (TheIntrinsicFile!="") {
            TheDistCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheDistCameraParameters.resize(TheDistInputImage.size());
	    TheCameraParameters=TheDistCameraParameters;
	    TheCameraParameters.Distorsion.setTo(cv::Scalar::all(0));
        }

        if(chromatic) {
	  vector<cv::Point3f> corners;
// 	  corners.push_back( cv::Point3f(  0.08,  0.12, 0.) );
// 	  corners.push_back( cv::Point3f(  0.08, -0.12, 0.) );
// 	  corners.push_back( cv::Point3f( -0.08, -0.12, 0.) );
// 	  corners.push_back( cv::Point3f( -0.08,  0.12, 0.) );	  
	  TheChromaticMask.setParams(5,5,0.0001,TheCameraParameters,TheBoardConfig,TheMarkerSize);
	}
        
        //Create gui

        cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
	
	TheBoardDetector.setYPerperdicular(false); 
	TheBoardDetector.setParams(TheBoardConfig,TheCameraParameters,TheMarkerSize);
	TheBoardDetector.getMarkerDetector().setThresholdParams( 21,7); // for blue-green markers, the window size has to be larger
	TheBoardDetector.getMarkerDetector().getThresholdParams( ThresParam1,ThresParam2);
	TheBoardDetector.getMarkerDetector().setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
	TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::LINES);
	TheBoardDetector.getMarkerDetector().setWarpSize((D[0].n()+2)*8);
	TheBoardDetector.getMarkerDetector().setMinMaxSize(0.005, 0.5);	

        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 31, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
        char key=0;
        int index=0;
        //capture
	do
        {
            TheVideoCapturer.retrieve( TheDistInputImage);
	    //remove distortion
	    if (TheCameraParameters.CameraMatrix.total()!=0) 
	      cv::undistort(TheDistInputImage,TheInputImage, TheDistCameraParameters.CameraMatrix, TheDistCameraParameters.Distorsion);
	    else TheInputImage=TheDistInputImage;

// 	    cv::cvtColor(TheInputImage,TheInputImage,CV_BayerBG2BGR);
	    	    
	    TheInputImage.copyTo(TheInputImageCopy);
	    if(chromatic) getHueImg(TheInputImage, TheInputImageH);
	    	    
            index++; //number of images captured

            //Detection of the board
            float probDetect=0.;
            if(chromatic) probDetect=TheBoardDetector.detect(TheInputImageH);
	    else probDetect=TheBoardDetector.detect(TheInputImage);
	    
            //print marker borders
            for (unsigned int i=0;i<TheBoardDetector.getDetectedBoard().size();i++)
                TheBoardDetector.getDetectedBoard()[i].draw(TheInputImageCopy,Scalar(0,0,255),1);

            //print board
             if (TheCameraParameters.isValid()) {
                if ( probDetect>0.2)   {
                    CvDrawingUtils::draw3dAxis( TheInputImageCopy,TheBoardDetector.getDetectedBoard(),TheCameraParameters);
                }
            }

            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
            cv::imshow("thres",TheBoardDetector.getMarkerDetector().getThresholdedImage());
	    
	    // create mask
	    if(chromatic && TheChromaticMask.isValid()) {



	      
	      if(probDetect>0.2) {
		
 	    double tick = (double)getTickCount();		
		
		TheChromaticMask.classify2(TheInputImageH,TheBoardDetector.getDetectedBoard());
		
             AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency(); AvrgTime.second++;
             cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;		
		
		if(index%200==0) {
		  cout << "Updating Mask" << endl;
		  TheChromaticMask.update(TheInputImageH);
		}
	      }
	      else TheChromaticMask.resetMask();
// 	      TheVisibilityMask.createMask(TheInputImageH);
     
	      cv::imshow("mask",TheChromaticMask.getMask()*255);
// 	      cv::imshow("maskcells",TheChromaticMask.getCellMap()*10);

	    }
	    
            //write to video if required
            if (  TheOutVideoFilePath!="") {
                //create a beautiful compiosed image showing the mask
                //first create a small version of the mask image
                if(TheChromaticMask.isValid()) {
		  cv::Mat smallMask;
		  cv::resize( TheChromaticMask.getMask()*255,smallMask,cvSize(TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
		  cv::Mat small3C;
		  cv::cvtColor(smallMask,small3C,CV_GRAY2BGR);
		  cv::Mat roi=TheInputImageCopy(cv::Rect(0,0,TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
		  small3C.copyTo(roi);
		}
                VWriter<<TheInputImageCopy;
            }

            key=cv::waitKey(waitTime);//wait for key to be pressed
            processKey(key);
	    
        }while ( key!=27 && TheVideoCapturer.grab());


    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

}
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
     TheBoardDetector.getMarkerDetector().setThresholdParams(ThresParam1,ThresParam2);
    //recompute
    //Detection of the board
    float probDetect=TheBoardDetector.detect( TheInputImage);
    TheInputImage.copyTo(TheInputImageCopy);
    if (TheCameraParameters.isValid() && probDetect>0.2)
        aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheBoardDetector.getDetectedBoard(),TheCameraParameters);

    
    cv::imshow("in",TheInputImageCopy);
    cv::imshow("thres",TheBoardDetector.getMarkerDetector().getThresholdedImage());
}





