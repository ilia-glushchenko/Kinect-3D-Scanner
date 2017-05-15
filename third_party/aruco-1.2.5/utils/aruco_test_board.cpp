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
#include <opencv2/imgproc/imgproc.hpp>
#include "aruco.h"
using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
string TheBoardConfigFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
VideoCapture TheVideoCapturer;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;

string TheOutVideoFilePath;
cv::VideoWriter VWriter;

void cvTackBarEvents(int pos,void*);
pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;




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
        cerr<<"Usage: (in.avi|live) boardConfig.yml [intrinsics.yml] [size] [out]"<<endl;
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

/*    case 'p':
        if (MDetector.getCornerRefinementMethod()==MarkerDetector::SUBPIX)
            MDetector.setCornerRefinementMethod(MarkerDetector::NONE);
        else
            MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
        break;*/
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
	TheBoardDetector.setParams(TheBoardConfig,TheCameraParameters,TheMarkerSize);
	TheBoardDetector.getMarkerDetector().getThresholdParams( ThresParam1,ThresParam2);
	TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(MarkerDetector::HARRIS);
	TheBoardDetector.set_repj_err_thres(1.5);
// 	TheBoardDetector.getMarkerDetector().enableErosion(true);//for chessboards
        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
        char key=0;
        int index=0;
        //capture until press ESC or until the end of the video
        do
        {
            TheVideoCapturer.retrieve( TheInputImage);
            TheInputImage.copyTo(TheInputImageCopy);
            index++; //number of images captured
            double tick = (double)getTickCount();//for checking the speed
            //Detection of the board
            float probDetect=TheBoardDetector.detect(TheInputImage);
            //chekc the speed by calculating the mean speed of all iterations
            AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
            AvrgTime.second++;
            cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;
            //print marker borders
            for (unsigned int i=0;i<TheBoardDetector.getDetectedMarkers().size();i++)
                TheBoardDetector.getDetectedMarkers()[i].draw(TheInputImageCopy,Scalar(0,0,255),1);

            //print board
             if (TheCameraParameters.isValid()) {
                if ( probDetect>0.2)   {
                    CvDrawingUtils::draw3dAxis( TheInputImageCopy,TheBoardDetector.getDetectedBoard(),TheCameraParameters);
// 		    CvDrawingUtils::draw3dCube(TheInputImageCopy, TheBoardDetector.getDetectedBoard(),TheCameraParameters);
                    //draw3dBoardCube( TheInputImageCopy,TheBoardDetected,TheIntriscCameraMatrix,TheDistorsionCameraParams);
                }
            }
            //DONE! Easy, right?

            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
            cv::imshow("thres",TheBoardDetector.getMarkerDetector().getThresholdedImage());
            //write to video if required
            if (  TheOutVideoFilePath!="") {
                //create a beautiful compiosed image showing the thresholded
                //first create a small version of the thresholded image
                cv::Mat smallThres;
                cv::resize( TheBoardDetector.getMarkerDetector().getThresholdedImage(),smallThres,cvSize(TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
                cv::Mat small3C;
                cv::cvtColor(smallThres,small3C,CV_GRAY2BGR);
                cv::Mat roi=TheInputImageCopy(cv::Rect(0,0,TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
                small3C.copyTo(roi);
                VWriter<<TheInputImageCopy;
// 			 cv::imshow("TheInputImageCopy",TheInputImageCopy);

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



