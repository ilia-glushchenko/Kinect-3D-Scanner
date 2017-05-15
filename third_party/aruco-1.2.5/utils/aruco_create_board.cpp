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
#include "board.h"
#include <cstdio>
#include <opencv2/highgui/highgui.hpp>
#include "arucofidmarkers.h"
using namespace std;
using namespace cv;
int main(int argc,char **argv)
{
    try {
        if (argc<4) {
            cerr<<"Usage: X:Y boardImage.png boardConfiguration.yml [pixSize] [Type(0: panel,1: chessboard, 2: frame)] [interMarkerDistance(0,1)]"<<endl;
            return -1;
        }
        int XSize,YSize;
        if (sscanf(argv[1],"%d:%d",&XSize,&YSize)!=2) {
            cerr<<"Incorrect X:Y specification"<<endl;
            return -1;
        }
        int pixSize=100;
        float interMarkerDistance=0.2;
        bool isChessBoard=false;
	int typeBoard=0;
        if (argc>=5) pixSize=atoi(argv[4]);
        if (argc>=6) typeBoard=atoi(argv[5]);
        if (argc>=7) interMarkerDistance=atoi(argv[6]);
        aruco::BoardConfiguration BInfo;
        Mat BoardImage;
        if (typeBoard==0)
            BoardImage=aruco::FiducidalMarkers::createBoardImage(Size(XSize,YSize), pixSize,pixSize*interMarkerDistance,BInfo);
        else if (typeBoard==1)
            BoardImage=aruco::FiducidalMarkers::createBoardImage_ChessBoard(Size(XSize,YSize), pixSize,BInfo);
        else if (typeBoard==2)
            BoardImage=aruco::FiducidalMarkers::createBoardImage_Frame(Size(XSize,YSize), pixSize,pixSize*interMarkerDistance,BInfo);
	  
	  else {cerr<<"Incorrect board type"<<typeBoard<<endl;return -1;}
	  
        imwrite(argv[2],BoardImage);
        BInfo.saveToFile(argv[3]);

    }
    catch (std::exception &ex)
    {
        cout<<ex.what()<<endl;
    }

}

