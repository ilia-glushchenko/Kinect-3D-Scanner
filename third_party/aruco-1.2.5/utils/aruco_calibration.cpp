//Automatic camera calibration using an aruco board only
//The board must be expressed in meters (use aruco_board_pix2meters)

//You must move the camera around the board and the program automatically detects it and recalibrate camera
//Not all he detections are employed for calibration, we create store viewpoints at a given distance between thems

#include "boarddetector.h"
#include "board.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

cv::VideoCapture TheVideoCapturer;

aruco::BoardConfiguration TheBoardConfig;
aruco::BoardDetector TheBoardDetector;
cv::Mat TheInputImage;
aruco::CameraParameters TheCamParams;
std::vector<cv::Mat> The_rvecs,The_tvecs;//locations at which calibration is done
std::vector<vector<cv::Point2f> > TheimagePointsV;
std::vector<vector<cv::Point3f> > TheobjPointsV;  //image and obj points at which calibration is done
float ViewPointDistance=0.025;
using namespace std;


//returns an initial dummy camera parameters
aruco::CameraParameters getDummyInitialCameraParams(cv::Size imageSize) {
    aruco::CameraParameters Cp;
    cv::Mat CamMatrix=cv::Mat::eye(3,3,CV_32F);
    CamMatrix.at<float>(0,0)=500;
    CamMatrix.at<float>(1,1)=500;
    CamMatrix.at<float>(0,2)=imageSize.width/2;
    CamMatrix.at<float>(1,2)=imageSize.height/2;
    Cp.setParams(CamMatrix,cv::Mat::zeros(1,5,CV_32F) ,imageSize);
    return Cp;

}


//computes  minimum distance of T to elements in The_tvecs
float minDistanceToStoredLocations(cv::Mat T) {
    //normalize R
    float minDist=std::numeric_limits< float >::max();
    for (size_t i=0;i<The_tvecs.size();i++) {
        //first, normalize the vector of R

        float norm=cv::norm(T-The_tvecs[i]);
        if (norm<minDist) minDist=norm;
        //now, check norm
    }

    return minDist;
}
// inclusion of new points to compute
void setCurrentViewPoint(vector<cv::Point3f> &objPoints,vector<cv::Point2f> & imagePoints) {
    cv::Mat Rvec,Tvec;
    //get the current location given the points
    cv::solvePnP(objPoints,imagePoints, TheCamParams.CameraMatrix,TheCamParams.Distorsion,Rvec,Tvec);
    //check the minimum distance to the rest of valid locations
//   cerr<<"Tvec="<<Tvec<<" "<<minDistanceToStoredLocations(Rvec,Tvec) <<endl;
    if ( minDistanceToStoredLocations(Tvec) > ViewPointDistance ) {
        cerr<<"Adding new view point"<<endl;
        //addToPool(imagePoints,objPoints);
        //add the points to calculate the camera params
        TheimagePointsV.push_back(imagePoints);
        TheobjPointsV.push_back(objPoints);
        The_tvecs.push_back(Tvec);
        The_rvecs.push_back(Rvec);
//       char c;cin>>c;
        if (TheobjPointsV.size()>=3 && TheobjPointsV.size()<8) {
            //now, calibrate camera
            float repro=cv::calibrateCamera(TheobjPointsV,TheimagePointsV,TheCamParams.CamSize, TheCamParams.CameraMatrix, TheCamParams.Distorsion,The_rvecs,The_tvecs);
            cerr<<"Recalibared: "<<repro<<endl;
            cerr<<TheCamParams.CameraMatrix<<" "<<endl<<TheCamParams.Distorsion<<endl;
        }


    }

}
// void getObjectAndImagePoints(aruco::Board &B, cv::Mat &objPoints,cv::Mat &imagePoints) {
//     //composes the matrices
//     int nPoints=B.size()*4;
// 
//     imagePoints.create(nPoints,1,CV_32FC2);
//     objPoints.create(nPoints,1,CV_32FC3);
//     int cIdx=0;
//     for (size_t i=0;i<B.size();i++) {
//         const aruco::MarkerInfo  & mInfo=B.conf.getMarkerInfo(B[i].id);
//         for (int j=0;j<4;j++,cIdx++) {
//             imagePoints.ptr<cv::Point2f>(0)[cIdx]= B[i][j];
//             objPoints.ptr<cv::Point3f>(0)[cIdx]= mInfo[j];
//         }
//     }
// 
// }
void getObjectAndImagePoints(aruco::Board &B, vector<cv::Point3f> &objPoints,vector<cv::Point2f> &imagePoints) {
    //composes the matrices
    int nPoints=B.size()*4;

    int cIdx=0;
    for (size_t i=0;i<B.size();i++) {
        const aruco::MarkerInfo  & mInfo=B.conf.getMarkerInfo(B[i].id);
        for (int j=0;j<4;j++,cIdx++) {
            imagePoints.push_back(B[i][j]);
            objPoints.push_back(  mInfo[j]);
        }
    }

}
int main(int argc,char **argv) {


    try {
        if (argc<4) {
            cerr<<"Usage : (in.avi|live) board_in_meters.yml out_camera_params.yml  [viewPointDist (default 0.025)]";
            return -1;
        }

        //extra param?
        if (argc>=5) {
            ViewPointDistance=atof(argv[4]);
            cerr<<"Using view point distance="<<ViewPointDistance<<endl;
        }

        //Open video stream
        if (string(argv[1])=="live")
            TheVideoCapturer.open(0);
        else TheVideoCapturer.open(argv[1]);
        if (!TheVideoCapturer.isOpened()) {
            cerr<<"Could not open video"<<endl;
            return -1;
        }
        //assume an initial camera params
        //capture first image to do so and get the image size
        TheVideoCapturer.grab();
        TheVideoCapturer.retrieve(TheInputImage);
        TheCamParams=getDummyInitialCameraParams(TheInputImage.size());
//         TheCalibrator.setParams(TheInputImage.size());
        //load config file
        TheBoardConfig.readFromFile(argv[2]);
        TheBoardDetector.setParams(TheBoardConfig);
        TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
        //let's go! until wnd of file or ESC pressed
        char key=0;

        while (TheVideoCapturer.grab() && key!=27) {//
            TheVideoCapturer.retrieve(TheInputImage);
            //detect
            float prob=TheBoardDetector.detect(TheInputImage);
            for (size_t i=0;i<TheBoardDetector.getDetectedBoard().size();i++)
                TheBoardDetector.getDetectedBoard()[i].draw(TheInputImage,cv::Scalar(0,0,255));
            cv::imshow("image",TheInputImage);
            key=cv::waitKey(10);
            if (prob>0.2) {
                //lets add the detection
                vector<cv::Point3f> objPoints;
		vector<cv::Point2f> imgPoints;
                getObjectAndImagePoints(TheBoardDetector.getDetectedBoard(),objPoints,imgPoints);
                setCurrentViewPoint(objPoints,imgPoints);
            }

        }

//do the final calibration with all images
        if (TheobjPointsV.size()>=3) {
            cerr<<"Performing final calibration with the "<< TheobjPointsV.size()<<" images selected. It migth take a little while"<<endl;
            float repro=cv::calibrateCamera(TheobjPointsV,TheimagePointsV,TheCamParams.CamSize, TheCamParams.CameraMatrix, TheCamParams.Distorsion,The_rvecs,The_tvecs);
            cerr<<"Recalibared: "<<repro<<endl;
            cerr<<TheCamParams.CameraMatrix<<" "<<endl<<TheCamParams.Distorsion<<endl;
	    //do a refinement using these with low reprjection error
	    vector<cv::Point2f> reprj;
	    int nToRemove=0,nTotal=0;
	    vector<vector<bool> > toRemove(TheobjPointsV.size());
	    for(size_t i=0;i<TheobjPointsV.size();i++){
	      cv::projectPoints(TheobjPointsV[i],The_rvecs[i],The_tvecs[i],TheCamParams.CameraMatrix, TheCamParams.Distorsion,reprj);
	      toRemove[i].resize(TheobjPointsV[i].size(),false);
	      for(size_t j=0;j<TheimagePointsV.size();j++,nTotal++){
		if (cv::norm(reprj[j]-TheimagePointsV[i][j])>0.99){
		  toRemove[i][j]=true;
		  nToRemove++;
		}
	      }
	    }
	    cout<<"Total o "<<nToRemove<<" outliers of "<<nTotal<<" total points"<<endl;
	    //now, copy only the good ones
	    vector<vector<cv::Point3f> > ob3d(TheobjPointsV.size());
	    vector<vector<cv::Point2f> > ob2d(TheobjPointsV.size());
	    for(size_t i=0;i<TheobjPointsV.size();i++){
	      	      for(size_t j=0;j<TheimagePointsV[i].size();j++){
			    if (!toRemove[i][j]){
			      ob3d[i].push_back(TheobjPointsV[i][j]);
			      ob2d[i].push_back(TheimagePointsV[i][j]);
			    }
		      }
	    }
	    
	    //repeating calibration without outliers
	          cerr<<"Performing final calibration with the "<< TheobjPointsV.size()<<" images selected. It migth take a little while"<<endl;
              repro=cv::calibrateCamera(ob3d,ob2d,TheCamParams.CamSize, TheCamParams.CameraMatrix, TheCamParams.Distorsion,The_rvecs,The_tvecs);
            cerr<<"Recalibared: "<<repro<<endl;
            cerr<<TheCamParams.CameraMatrix<<" "<<endl<<TheCamParams.Distorsion<<endl;
    

            TheCamParams.saveToFile(argv[3]);
        }



    } catch (std::exception &ex) {
        cout<<ex.what()<<endl;
    }
}
