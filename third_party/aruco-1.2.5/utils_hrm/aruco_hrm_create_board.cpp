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

#include "highlyreliablemarkers.h"
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "board.h"

int main(int argc,char **argv)
{
   if(argc < 6) {
     cerr<<"Invalid number of arguments"<<endl;     
     cerr << "Usage: dictionary.yml outputboard.yml outputimage.png height width [chromatic=0] [outdictionary.yml] \n \
      dictionary.yml: input dictionary from where markers are taken to create the board \n \
      outputboard.yml: output board configuration file in aruco format \n \
      outputimage.png: output image for the created board \n \
      height: height of the board (num of markers) \n \
      width: width of the board (num of markers) \n \
      chromatic: 0 for black&white markers, 1 for green&blue chromatic markers \n \
      outdictionary.yml: output dictionary with only the markers included in the board" << endl;     
     exit(-1);
   }  
  
    // read parameters
    std::string dictionaryfile = argv[1];
    std::string outboard = argv[2];
    std::string outimg = argv[3];
    cv::Size gridSize;
    gridSize.height = atoi(argv[4]);
    gridSize.width = atoi(argv[5]);     
    bool chromatic = false;
    if(argc>=7) chromatic = (argv[6][0]=='1');
   
    aruco::Dictionary D;
    D.fromFile(dictionaryfile);
    if(D.size()==0) {
     std::cerr << "Error: Dictionary is empty" << std::endl;
     exit(-1);      
    }
    
    int nMarkers=gridSize.height*gridSize.width;
    unsigned int MarkerSize = (D[0].n()+2)*20;
    unsigned int MarkerDistance = MarkerSize/5;

    int sizeY=gridSize.height*MarkerSize+(gridSize.height-1)*MarkerDistance;
    int sizeX=gridSize.width*MarkerSize+(gridSize.width-1)*MarkerDistance;
    //find the center so that the ref systeem is in it
    float centerX=sizeX/2.;
    float centerY=sizeY/2.;

    aruco::Dictionary outD;
    
    aruco::BoardConfiguration BC;
    BC.mInfoType = aruco::BoardConfiguration::PIX;
        
    //indicate the data is expressed in pixels
    cv::Mat tableImage(sizeY,sizeX,CV_8UC1);
    tableImage.setTo(cv::Scalar(255));
    int idp=0;
    for (int y=0;y<gridSize.height;y++)
        for (int x=0;x<gridSize.width;x++,idp+=1) {
	    // create image
            cv::Mat subrect(tableImage,cv::Rect( x*(MarkerDistance+MarkerSize),y*(MarkerDistance+MarkerSize),MarkerSize,MarkerSize));
            cv::Mat marker=D[idp].getImg(MarkerSize);
            marker.copyTo(subrect);
	    outD.push_back(D[idp]);
	    
	    // add to board configuration
	    aruco::MarkerInfo MI;
	    MI.resize(4);
	    MI.id = D[idp].getId();
        for(unsigned int i=0; i<4; i++) MI[i].z = 0;
	    MI[0].x = x*(MarkerDistance+MarkerSize) - centerX;
	    MI[0].y = y*(MarkerDistance+MarkerSize) - centerY;
	    MI[1].x = x*(MarkerDistance+MarkerSize)+MarkerSize - centerX;
	    MI[1].y = y*(MarkerDistance+MarkerSize) - centerY;
	    MI[2].x = x*(MarkerDistance+MarkerSize)+MarkerSize - centerX;
	    MI[2].y = y*(MarkerDistance+MarkerSize)+MarkerSize - centerY;
	    MI[3].x = x*(MarkerDistance+MarkerSize) - centerX;
	    MI[3].y = y*(MarkerDistance+MarkerSize)+MarkerSize - centerY;
	    // makes y negative so z axis is pointing up
	    MI[0].y *= -1;
	    MI[1].y *= -1;
	    MI[2].y *= -1;
	    MI[3].y *= -1;
	    BC.push_back(MI);
	     
        }

    BC.saveToFile(outboard); // save board configuration
    if(argc>=8) outD.toFile(argv[7]); // save new dictionary just with the used markers, if desired
    
    if(chromatic) {
      cv::Scalar color1 = cv::Scalar(250,134,4);
      //   cv::Scalar color2 = cv::Scalar(0,255,0);
      cv::Vec3b color2Vec3b = cv::Vec3b(0,255,0); // store as a Vec3b to assign easily to the image
      
      // create new image with border and with color 1
      cv::Mat chromaticImg(tableImage.rows+2*MarkerDistance, tableImage.cols+2*MarkerDistance, CV_8UC3, color1);
      
      // now use color2 in black pixels 
      for(unsigned int i=0; i<tableImage.rows; i++) {
    for(unsigned int j=0; j<tableImage.cols; j++) {
	  if(tableImage.at<uchar>(i,j)==0)
	    chromaticImg.at<cv::Vec3b>(MarkerDistance+i, MarkerDistance+j) = color2Vec3b;
	}
      }
      tableImage = chromaticImg;
    }
    
      
    cv::imshow("Board", tableImage);
    cv::waitKey(0);
    
    cv::imwrite(outimg, tableImage); // save output image
    
    
    

    
}
