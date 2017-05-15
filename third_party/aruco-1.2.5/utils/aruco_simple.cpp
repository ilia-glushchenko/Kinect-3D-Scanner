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
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace aruco;
int main(int argc,char **argv)
{
    try
    {
        if (argc<2) {
            cerr<<"Usage: (in.jpg|in.avi) [cameraParams.yml] [markerSize] [outImage]"<<endl;
            exit(0);
        }


        aruco::CameraParameters CamParam;
        MarkerDetector MDetector;
        vector<Marker> Markers;
        float MarkerSize=-1;
        //read the input image
        cv::Mat InImage;
        //try opening first as video
        VideoCapture vreader(argv[1]);
        if (vreader.isOpened()) {
            vreader.grab();
            vreader.retrieve(InImage);
        }
        else {
            InImage=cv::imread(argv[1]);
        }
        //at this point, we should have the image in InImage
        //if empty, exit
        if (InImage.total()==0) {
            cerr<<"Could not open input"<<endl;
            return 0;
        }

	//read camera parameters if specifed
        if (argc>=3) {
            CamParam.readFromXMLFile(argv[2]);
            //resizes the parameters to fit the size of the input image
            CamParam.resize( InImage.size());
        }
        //read marker size if specified
        if (argc>=4) MarkerSize=atof(argv[3]);
        cv::namedWindow("in",1);

	
	//Ok, let's detect
        MDetector.detect(InImage,Markers,CamParam,MarkerSize);
        //for each marker, draw info and its boundaries in the image
        for (unsigned int i=0;i<Markers.size();i++) {
            cout<<Markers[i]<<endl;
            Markers[i].draw(InImage,Scalar(0,0,255),2);
        }
        //draw a 3d cube in each marker if there is 3d info
        if (  CamParam.isValid() && MarkerSize!=-1)
            for (unsigned int i=0;i<Markers.size();i++) {
                CvDrawingUtils::draw3dCube(InImage,Markers[i],CamParam);
            }
        //show input with augmented information
        cv::imshow("in",InImage);
	//show also the internal image resulting from the threshold operation
        cv::imshow("thes", MDetector.getThresholdedImage() );
        cv::waitKey(0);//wait for key to be pressed


        if (argc>=5) cv::imwrite(argv[4],InImage);
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
