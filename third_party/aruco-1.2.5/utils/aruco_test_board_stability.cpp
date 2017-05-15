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
#include <opencv2/opencv.hpp>
#include "aruco.h"

#define TOTAL_METHODS 4

using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
string TheBoardConfigFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
MarkerDetector MDetector[TOTAL_METHODS];
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
string TheOutVideoFilePath;
cv::VideoWriter VWriter;

void cvTackBarEvents(int pos,void*);
pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;
int selectedView=0;



class StabilityChecker
{
public:
  
  StabilityChecker()
  {
    rsum=cv::Mat::zeros(3,1,CV_64F);
    tsum=cv::Mat::zeros(3,1,CV_64F);
    rsum2=cv::Mat::zeros(3,1,CV_64F);
    tsum2=cv::Mat::zeros(3,1,CV_64F);
    nTimesR=0;
    maxDif=-1;
  }  
  
  void process(cv::Mat Rvec, cv::Mat Tvec)
  {
    
	cv::Mat r64,t64;
	Rvec.convertTo(r64,CV_64F);
	Tvec.convertTo(t64,CV_64F);
	rsum+=r64;
	tsum+=t64;
	for(int j=0;j<3;j++){
	  rsum2.at<double>(0,j)+=Rvec.at<float>(0,j)*Rvec.at<float>(0,j);
	  tsum2.at<double>(0,j)+=Tvec.at<float>(0,j)*Tvec.at<float>(0,j);
	}
	nTimesR++;
	rmean=rsum*(1./double(nTimesR));
	tmean=tsum*(1./double(nTimesR));
	rmean2=rmean.clone();
	tmean2=tmean.clone();
	for(int j=0;j<3;j++){
	  rmean2.at<double>(0,j)*=rmean2.at<double>(0,j);
	  tmean2.at<double>(0,j)*=tmean2.at<double>(0,j);
	}
	rdev=(rsum2*(1./double(nTimesR))) -rmean2;
	tdev=(tsum2*(1./double(nTimesR))) - tmean2;
	  for(int j=0;j<3;j++){
	  rdev.at<double>(0,j)=sqrt(rdev.at<double>(0,j));
	  tdev.at<double>(0,j)=sqrt(tdev.at<double>(0,j));
	}
	if (nTimesR>10){
	  double dif=cv::norm(rdev);
	  if (dif>maxDif) maxDif=dif;
	}  
  }  
  
  void print()
  {
    std::cout<< "mean= "<< rmean<<" "<< tmean<<std::endl;
    std::cout<< "var= "<< rdev<<" "<< tdev<<std::endl;
    std::cout<< "dev= "<< rdev<<" "<< tdev<<std::endl;
    std::cout<< "maxdif= "<< maxDif<< std::endl;
  }  
  
private:
  
  cv::Mat rsum, tsum, rsum2, tsum2;
  cv::Mat rmean, tmean, rmean2, tmean2, rdev, tdev;
  int nTimesR;
  double maxDif;
  
};


StabilityChecker SC[TOTAL_METHODS];


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
        cerr<<"Usage: (in.avi|live) boardConfig.abc [intrinsics.yml] [size] [out]"<<endl;
        return false;
    }
    TheInputVideo=argv[1];
    TheBoardConfigFile=argv[2];
    if (argc>=4)
        TheIntrinsicFile=argv[3];
    if (argc>=5)
        TheMarkerSize=atof(argv[4]);
    if (argc>=6)
        TheOutVideoFilePath=argv[5];


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

    case 'p':
        selectedView = (selectedView+1)%TOTAL_METHODS;
        break;
    }
}

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

        //read first image to get the dimensions
        TheVideoCapturer>>TheInputImage;

        //Open outputvideo
        if ( TheOutVideoFilePath!="")
            VWriter.open(TheOutVideoFilePath,CV_FOURCC('M','J','P','G'),15,TheInputImage.size());

        //read camera parameters if passed
        if (TheIntrinsicFile!="") {
            TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheCameraParameters.resize(TheInputImage.size());
        }
        
        //Create gui

        cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
	for(unsigned int i=0; i<TOTAL_METHODS; i++) {
	  MDetector[i].getThresholdParams( ThresParam1,ThresParam2);
	  MDetector[i].enableErosion(false);
	  MDetector[i].setCornerRefinementMethod((MarkerDetector::CornerRefinementMethod)i);
	}
// 	MDetector[0].setCornerRefinementMethod(MarkerDetector::NONE);
// 	MDetector[1].setCornerRefinementMethod(MarkerDetector::HARRIS);
// 	MDetector[2].setCornerRefinementMethod(MarkerDetector::SUBPIX);
// 	MDetector[3].setCornerRefinementMethod(MarkerDetector::LINES);
	
        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
	cv::createTrackbar("View", "in",&selectedView, 3);
        char key=0;
        int index=0;
        //capture until press ESC or until the end of the video
        while ( key!=27 && TheVideoCapturer.grab())
        {
            TheVideoCapturer.retrieve( TheInputImage);
            TheInputImage.copyTo(TheInputImageCopy);
            index++; //number of images captured
            double tick = (double)getTickCount();//for checking the speed
            //Detection of markers in the image passed
	    for(unsigned int i=0; i<TOTAL_METHODS; i++) {
	      MDetector[i].detect(TheInputImage,TheMarkers,TheCameraParameters);
	      float probDetect=TheBoardDetector.detect( TheMarkers, TheBoardConfig,TheBoardDetected, TheCameraParameters,TheMarkerSize);
	      if(probDetect>0) SC[i].process(TheBoardDetected.Rvec, TheBoardDetected.Tvec);
	      
	      switch(i) {
		case 0:
		  cout << "NONE: " << endl;
		  break;
		case 1:
		  cout << "HARRIS: " << endl;
		  break;
		case 2:
		  cout << "SUBPIX: " << endl;
		  break;
		case 3:
		  cout << "LINES: " << endl;
		  break;		  
	      }
	      SC[i].print();
	      cout << endl;
	      
	      
	      if (TheCameraParameters.isValid() && i==selectedView) {
		  if ( probDetect>0.)   {
		      CvDrawingUtils::draw3dAxis( TheInputImageCopy,TheBoardDetected,TheCameraParameters);
		      //draw3dBoardCube( TheInputImageCopy,TheBoardDetected,TheIntriscCameraMatrix,TheDistorsionCameraParams);
		  }
	      }
	      
	    }
            //Detection of the board
            
            //chekc the speed by calculating the mean speed of all iterations
            AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
            AvrgTime.second++;
            cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;
            //print marker borders
//             for (unsigned int i=0;i<TheMarkers.size();i++)
//                 TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);

            //print board

            //DONE! Easy, right?

            cout<<endl<<endl<<endl;
            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
            cv::imshow("thres",MDetector[0].getThresholdedImage());
            //write to video if required
            if (  TheOutVideoFilePath!="") {
                //create a beautiful compiosed image showing the thresholded
                //first create a small version of the thresholded image
                cv::Mat smallThres;
                cv::resize( MDetector[0].getThresholdedImage(),smallThres,cvSize(TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
                cv::Mat small3C;
                cv::cvtColor(smallThres,small3C,CV_GRAY2BGR);
                cv::Mat roi=TheInputImageCopy(cv::Rect(0,0,TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
                small3C.copyTo(roi);
                VWriter<<TheInputImageCopy;
// 			 cv::imshow("TheInputImageCopy",TheInputImageCopy);

            }

            key=cv::waitKey(waitTime);//wait for key to be pressed
            processKey(key);
        }


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
    for(unsigned int i=0; i<TOTAL_METHODS; i++)
      MDetector[i].setThresholdParams(ThresParam1,ThresParam2);
//recompute
      for(unsigned int i=0; i<TOTAL_METHODS; i++)
	MDetector[i].detect(TheInputImage,TheMarkers,TheCameraParameters ,TheMarkerSize);
//Detection of the board
    float probDetect=TheBoardDetector.detect( TheMarkers, TheBoardConfig,TheBoardDetected, TheCameraParameters);
    if (TheCameraParameters.isValid() && probDetect>0.2)
        aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheBoardDetected,TheCameraParameters);

    cv::imshow("in",TheInputImageCopy);
    cv::imshow("thres",MDetector[0].getThresholdedImage());
}



