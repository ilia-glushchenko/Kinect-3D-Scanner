#ifndef SURFKEYPOINTDETECTOR_H
#define SURFKEYPOINTDETECTOR_H

#include <QObject>
#include <QSettings>
#include <QDebug>

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/stitching/stitcher.hpp"

#include "types.h"

class SurfKeypointDetector : public QObject
{
	Q_OBJECT

public:
	SurfKeypointDetector(
		QObject *parent,
		PcdPtr cloud_ptr1, PcdPtr cloud_ptr2,
		cv::Mat  img1, cv::Mat  img2,
		PcdPtr keypoint_cloud_ptr1, PcdPtr keypoint_cloud_ptr2
	);

	void detect();
	void getMatchImagesVector(std::vector<cv::Mat>* matchImagesVector);

private:
	QSettings* settings;

	PcdPtr _point_cloud_ptr1;
	PcdPtr _point_cloud_ptr2;
	cv::Mat  image1;
	cv::Mat  image2;
	PcdPtr keypoint_point_cloud_ptr1;
	PcdPtr keypoint_point_cloud_ptr2;

	std::vector<cv::Mat> afterThreshNanMatchesImagesVector;

	//-------------------------------------------------------

	bool flat_area_keypoints_filter_threshold_check(
		PcdPtr   point_cloud_ptr,
		int x, int y,
		int radius,
		double threshold
	);
	void flat_area_keypoints_filter(
		std::vector<cv::Point2f> keypoints1,
		std::vector<cv::Point2f> keypoints2,
		std::vector<cv::DMatch>  matches,
		std::vector<cv::Point2f> &after_thresh_keypoints1,
		std::vector<cv::Point2f> &after_thresh_keypoints2,
		std::vector<cv::DMatch>  &matches_after_thresh
	);

	//-------------------------------------------------------
	
	void perform_detection();

	void surf_detect_keypoints(
		std::vector<cv::Point2f>  &result_keypoints1,
		std::vector<cv::Point2f>  &result_keypoints2,
		std::vector<cv::KeyPoint> &_keypoints1,
		std::vector<cv::KeyPoint> &_keypoints2,
		std::vector<cv::DMatch>   &result_matches
	);

	void surf_reject_keypoints(
		std::vector<cv::Point2f> in_keypoints1,
		std::vector<cv::Point2f> in_keypoints2,
		std::vector<cv::DMatch>  matches,
		std::vector<cv::Point2f>& out_keypoints1,
		std::vector<cv::Point2f>& out_keypoints2,
		std::vector<cv::DMatch>&  out_matches
	);

	void surf_remove_nan_from_keypoints(
		std::vector<cv::Point2f> keypoints1,
		std::vector<cv::Point2f> keypoints2,
		std::vector<cv::DMatch>  matches,
		std::vector<cv::Point2f> &result_keypoints1,
		std::vector<cv::Point2f> &result_keypoints2,
		std::vector<cv::DMatch>  &result_matches
	);

	void surf_fill_keypoint_clouds(
		std::vector<cv::Point2f> keypoints1,
		std::vector<cv::Point2f> keypoints2
	);

	//---------------------------------------------------------

	void surf_grid_keypoints_matcher(
		std::vector<cv::Point2f> in_keypoints1,
		std::vector<cv::Point2f> in_keypoints2,
		std::vector<cv::DMatch>  in_matches,
		std::vector<cv::Point2f>& out_keypoints1,
		std::vector<cv::Point2f>& out_keypoints2,
		std::vector<cv::DMatch>&  out_matches
	);

	void fill_first_grid(
		cv::Point2i p1, cv::Point2i p2,
		std::vector<cv::Point2f>& out_keypoints1,
		std::vector<cv::DMatch>&  out_matches
	);

	void fill_second_grid(
		cv::Point2i p1, cv::Point2i p2,
		cv::Point2i p1_, cv::Point2i p2_,
		std::vector<cv::Point2f>& out_keypoints1,
		std::vector<cv::Point2f>& out_keypoints2,
		std::vector<cv::DMatch>&  out_matches
	);
};

#endif // SURFKEYPOINTDETECTOR_H
