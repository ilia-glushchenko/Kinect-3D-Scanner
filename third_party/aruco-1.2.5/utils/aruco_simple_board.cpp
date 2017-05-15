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
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "aruco.h"
#include "boarddetector.h"
#include "cvdrawingutils.h"
using namespace cv;
using namespace aruco; 
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
		if(argc<3) {cerr<<"Usage: image  boardConfig.yml [cameraParams.yml] [markerSize]  [outImage]"<<endl;exit(0);}
		aruco::CameraParameters CamParam;
		MarkerDetector MDetector;
		vector<Marker> Markers;
		float MarkerSize=-1;
		BoardConfiguration TheBoardConfig;
		BoardDetector TheBoardDetector;
		Board TheBoardDetected;
		
		cv::Mat InImage=cv::imread(argv[1]);
		TheBoardConfig.readFromFile(argv[2]);
		if (argc>=4) {
		  CamParam.readFromXMLFile(argv[3]);
		  //resizes the parameters to fit the size of the input image
		  CamParam.resize( InImage.size());
		}

		if (argc>=5) 
		  MarkerSize=atof(argv[4]);
		
		cv::namedWindow("in",1);
		MDetector.detect(InImage,Markers);//detect markers without computing R and T information
		//Detection of the board
		float probDetect=TheBoardDetector.detect( Markers, TheBoardConfig,TheBoardDetected, CamParam,MarkerSize);
		
		//for each marker, draw info and its boundaries in the image
		for(unsigned int i=0;i<Markers.size();i++){
			cout<<Markers[i]<<endl;
			Markers[i].draw(InImage,Scalar(0,0,255),2);
		}

		//draw a 3d cube in each marker if there is 3d info
		if (  CamParam.isValid()){
 		  for(unsigned int i=0;i<Markers.size();i++){
 		    CvDrawingUtils::draw3dCube(InImage,Markers[i],CamParam);
 		    CvDrawingUtils::draw3dAxis(InImage,Markers[i],CamParam);
 		  }
		  CvDrawingUtils::draw3dAxis(InImage,TheBoardDetected,CamParam);
		  cout<<TheBoardDetected.Rvec<<" "<<TheBoardDetected.Tvec<<endl;
		}
		//draw board axis
		
		//show input with augmented information
		cv::imshow("in",InImage);
		cv::waitKey(0);//wait for key to be pressed
		if(argc>=6) cv::imwrite(argv[5],InImage);
		
	}catch(std::exception &ex)

	{
		cout<<"Exception :"<<ex.what()<<endl;
	}

}