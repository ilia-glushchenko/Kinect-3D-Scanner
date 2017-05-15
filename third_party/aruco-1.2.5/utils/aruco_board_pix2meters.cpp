/*****************************
Copyright 2012 Rafael Muñoz Salinas. All rights reserved.

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

//This program converts a boardconfiguration file expressed in pixel to another one expressed in meters
#include <iostream>
#include "board.h"
using namespace std;
using namespace aruco;
int main(int argc,char **argv){
 try{
    
  if (argc<4){
    cerr<<"Usage:  in_boardConfiguration.yml markerSize_meters out_boardConfiguration.yml"<<endl;
    return -1;
  }
 aruco::BoardConfiguration BInfo;
 BInfo.readFromFile(argv[1]);
 if (BInfo.size()==0){cerr<<"Invalid bord with no markers"<<endl;return -1;}
 if (!BInfo.isExpressedInPixels()){cerr<<"The board is not expressed in pixels"<<endl;return -1;}
 //first, we are assuming all markers are equally sized. So, lets get the size in pixels
 
 int markerSizePix=cv::norm(BInfo[0][0]-BInfo[0][1]);
 BInfo.mInfoType=BoardConfiguration::METERS;
 //now, get the size of a pixel, and change scale
 float markerSize_meters=atof(argv[2]);
 float pixSize= markerSize_meters/float(markerSizePix);
 cout<<markerSize_meters<<" "<<float(markerSizePix)<<" "<<pixSize<<endl;
 for(size_t i=0;i<BInfo.size();i++)
   for(int c=0;c<4;c++){
	BInfo[i][c]*=pixSize; 
   }
   //save to file
  BInfo.saveToFile(argv[3]);
 }catch(std::exception &ex){
    cout<<ex.what()<<endl;
 }
}

