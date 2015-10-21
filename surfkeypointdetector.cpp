#include "surfkeypointdetector.h"

#define WIDTH  640
#define HEIGHT 480

SurfKeypointDetector::SurfKeypointDetector(
	QObject *parent,
	QSettings* parent_settings,
	PcdPtr cloud_ptr1,
	PcdPtr cloud_ptr2,
	cv::Mat  img1,
	cv::Mat  img2,
	PcdPtr keypoint_cloud_ptr1,
	PcdPtr keypoint_cloud_ptr2
	) 
	: ScannerBase(parent, parent_settings)
{
	setParent(parent);
	_point_cloud_ptr1 = cloud_ptr1;
	_point_cloud_ptr2 = cloud_ptr2;
	image1 = img1;
	image2 = img2;
	keypoint_point_cloud_ptr1 = keypoint_cloud_ptr1;
	keypoint_point_cloud_ptr2 = keypoint_cloud_ptr2;
}

void SurfKeypointDetector::detect()
{
	perform_detection();
}

void SurfKeypointDetector::getMatchImagesVector(std::vector<cv::Mat>* matchImagesVector)
{
	for (int i = 0; i < afterThreshNanMatchesImagesVector.size(); i++)
		matchImagesVector->push_back(afterThreshNanMatchesImagesVector[i]);
}

//-------------------------------------------------------

bool SurfKeypointDetector::flat_area_keypoints_filter_threshold_check(
	PcdPtr point_cloud_ptr,
	int x, int y,
	int radius,
	double threshold
	)
{
	int left_x  = x - radius, left_y  = y - radius;
	int right_x = x + radius, right_y = y + radius;
	if (left_x < 0) left_x = 0;
	if (left_y < 0) left_y = 0;
	if (right_x >= WIDTH)  right_x = WIDTH;
	if (right_y >= HEIGHT) right_y = HEIGHT;

	double current = point_cloud_ptr->at(x, y).z;
	double averege = 0.0f;
	double counter = 0.0f;

	for (int y = left_y; y < right_y; y++)
		for (int x = left_x; x < right_x; x++)
			if (isnan<float>(point_cloud_ptr->at(x, y).z) == false)
			{
				averege += point_cloud_ptr->at(x, y).z;
				counter++;
			}

	averege /= counter;
	double value = abs(averege - current);

	if (value < threshold)
		return true;

	return false;
}

void SurfKeypointDetector::flat_area_keypoints_filter(
	std::vector<cv::Point2f> keypoints1,
	std::vector<cv::Point2f> keypoints2,
	std::vector<cv::DMatch>  matches,
	std::vector<cv::Point2f> &after_thresh_keypoints1,
	std::vector<cv::Point2f> &after_thresh_keypoints2,
	std::vector<cv::DMatch>  &matches_after_thresh
	)
{
	using namespace cv;

	int iter_counter = 0;

	int radius =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/FLAT_KEYPOINT_FILTER_SQUARE_SIDE_LENGTH"
		).toInt();
	int min_radius =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/FLAT_KEYPOINT_FILTER_SQUARE_SIDE_MIN_LENGTH"
		).toInt();
	double threshold =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/FLAT_KEYPOINT_FILTER_THRESHOLD"
		).toDouble();
	int max_iter =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/FLAT_KEYPOINT_FILTER_MAX_ITERATIONS"
		).toInt();
	int min_match =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/FLAT_KEYPOINT_FILTER_MIN_MATCHES"
		).toInt();

	while (matches_after_thresh.size() < min_match)
	{
		matches_after_thresh.clear();
		after_thresh_keypoints1.clear();
		after_thresh_keypoints2.clear();

		for (int i = 0; i < keypoints1.size(); i++)
		{
			if (iter_counter < min_radius)
				radius -= iter_counter;

			if (iter_counter >= min_radius)
				threshold += (double)iter_counter / 1000.0f - (double)min_radius / 1000.0f;

			int x1 = int(round(keypoints1[i].x));
			int y1 = int(round(keypoints1[i].y));
			bool pass_threshold1 = flat_area_keypoints_filter_threshold_check(
				_point_cloud_ptr1, x1, y1, radius, threshold
			);

			int x2 = int(round(keypoints2[i].x));
			int y2 = int(round(keypoints2[i].y));
			bool pass_threshold2 = flat_area_keypoints_filter_threshold_check(
				_point_cloud_ptr2, x2, y2, radius, threshold
			);

			if (pass_threshold1 && pass_threshold2)
			{
				matches_after_thresh.push_back(matches[i]);
				after_thresh_keypoints1.push_back(keypoints1[i]);
				after_thresh_keypoints2.push_back(keypoints2[i]);
			}
		}

		iter_counter++;
		if (iter_counter > max_iter)
		{
			qWarning() << 
				QString(
				"############################\nFLAT FILTER WARNING!\n%1 from %2 matches passed! Unfiltered matches used!\nRelax your thresholds!"
				).arg(matches_after_thresh.size()).arg(matches.size()).toStdString().c_str();
	

			after_thresh_keypoints1 = keypoints1;
			after_thresh_keypoints2 = keypoints2;
			matches_after_thresh = matches;
			break;
		}
			
	}
}

//-------------------------------------------------------

void SurfKeypointDetector::perform_detection()
{
	afterThreshNanMatchesImagesVector.clear();

	std::vector<cv::Point2f>  keypoints1, keypoints2;
	std::vector<cv::KeyPoint> _keypoints1, _keypoints2;
	std::vector<cv::DMatch> matches;
	surf_detect_keypoints(
		keypoints1, keypoints2,
		_keypoints1, _keypoints2,
		matches
	);

	std::vector<cv::Point2f> inlier_keypoints1, inlier_keypoints2;
	std::vector<cv::DMatch>  inlier_matches;
	surf_reject_keypoints(
		keypoints1, keypoints2, matches,
		inlier_keypoints1, inlier_keypoints2, inlier_matches
	);

	if (settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/GRID_KEYPOINT_FILTER_ENABLE").toBool())
	{
		std::vector<cv::Point2f> grid_keypoints1, grid_keypoints2;
		std::vector<cv::DMatch>  grid_matches;
		surf_grid_keypoints_matcher(
			inlier_keypoints1, inlier_keypoints2, inlier_matches,
			grid_keypoints1, grid_keypoints2, grid_matches
		);

		std::vector<cv::Point2f> grid_inlier_keypoints1, grid_inlier_keypoints2;
		std::vector<cv::DMatch>  grid_inlier_matches;
		surf_reject_keypoints(
			grid_keypoints1, grid_keypoints2, grid_matches,
			grid_inlier_keypoints1, grid_inlier_keypoints2, grid_inlier_matches
		);

		surf_fill_keypoint_clouds(
			grid_inlier_keypoints1, grid_inlier_keypoints2
		);
	}
	else
	{
		surf_fill_keypoint_clouds(
			inlier_keypoints1, inlier_keypoints2
			);
	}


	if (settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/DRAW_GOOD_FILTERED_MATCHES"
		).toBool())
	{
		cv::Mat img_matches;
		drawMatches(image1, _keypoints1,
			image2, _keypoints2,
			inlier_matches, img_matches,
			cv::Scalar::all(-1),
			cv::Scalar::all(-1),
			std::vector<char>());
		afterThreshNanMatchesImagesVector.push_back(img_matches);
	}
}

void SurfKeypointDetector::surf_detect_keypoints(
	std::vector<cv::Point2f>  &result_keypoints1,
	std::vector<cv::Point2f>  &result_keypoints2,
	std::vector<cv::KeyPoint> &_keypoints1,
	std::vector<cv::KeyPoint> &_keypoints2,
	std::vector<cv::DMatch>   &result_matches
	)
{
	using namespace cv;
	Mat descriptors1, descriptors2;

	SurfFeatureDetector	detector(
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/MIN_HISS"
		).toInt()
	);
	SurfDescriptorExtractor extractor;
	vector<DMatch> matches;
	
	detector.detect(image1, _keypoints1);
	detector.detect(image2, _keypoints2);

	extractor.compute(image1, _keypoints1, descriptors1);
	extractor.compute(image2, _keypoints2, descriptors2);

	if (settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/BF_MATCHER").toBool())
	{
		BFMatcher matcher(NORM_L2);
		matcher.match(descriptors1, descriptors2, matches);
	}
	else if (settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/FLANN_MATCHER").toBool())
	{
		FlannBasedMatcher matcher;
		matcher.match(descriptors1, descriptors2, matches);
	}		
	else
	{
		FlannBasedMatcher matcher;
		matcher.match(descriptors1, descriptors2, matches);
	}
		
	int y_threshold =
		settings->value(
			"OPENCV_KEYPOINT_DETECTION_SETTINGS/Y_AMPLITUDE_KEYPOINTS_THRESHOLD"
		).toInt();

	vector<DMatch> y_thresh_matches;
	for (int i = 0; i < matches.size(); i++)
	{
		cv::Point2f kp1 = _keypoints1[matches[i].queryIdx].pt;
		cv::Point2f kp2 = _keypoints2[matches[i].trainIdx].pt;
		if (abs(kp1.y - kp2.y) < y_threshold)
			y_thresh_matches.push_back(matches[i]);
	}

	double min_dist =
		settings->value(
			"OPENCV_KEYPOINT_DETECTION_SETTINGS/MIN_DIST_INIT"
		).toDouble();

	for (int i = 0; i < y_thresh_matches.size(); i++)
	{
		double dist = y_thresh_matches[i].distance;
		if (dist < min_dist) min_dist = dist;
	}

	double multy =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/GOOD_KEYPOINTS_DIST_COEF"
		).toDouble();
	double abs_min_dist =
		settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/MINIMAL_GOOD_KEYPOINTS_DIST"
		).toDouble();


	for (int i = 0; i < y_thresh_matches.size(); i++)
		if (y_thresh_matches[i].distance <= max(multy * min_dist, abs_min_dist))
			result_matches.push_back(y_thresh_matches[i]);

	for (int i = 0; i < result_matches.size(); i++)
	{
		result_keypoints1.push_back(_keypoints1[result_matches[i].queryIdx].pt);
		result_keypoints2.push_back(_keypoints2[result_matches[i].trainIdx].pt);
	}
}

void SurfKeypointDetector::surf_reject_keypoints(
	std::vector<cv::Point2f> in_keypoints1, 
	std::vector<cv::Point2f> in_keypoints2,
	std::vector<cv::DMatch>  matches,
	std::vector<cv::Point2f>& out_keypoints1,
	std::vector<cv::Point2f>& out_keypoints2,
	std::vector<cv::DMatch>&  out_matches
	)
{
	std::vector<cv::Point2f> no_nan_good_keypoints1;
	std::vector<cv::Point2f> no_nan_good_keypoints2;
	std::vector<cv::DMatch>  good_matches_after_nan;
	surf_remove_nan_from_keypoints(
		in_keypoints1, in_keypoints2,
		matches,
		no_nan_good_keypoints1, no_nan_good_keypoints2,
		good_matches_after_nan
	);

	std::vector<cv::Point2f> no_nan_after_thresh_good_keypoints1;
	std::vector<cv::Point2f> no_nan_after_thresh_good_keypoints2;
	std::vector<cv::DMatch>  good_matches_after_thresh_nan;
	if (settings->value(
		"OPENCV_KEYPOINT_DETECTION_SETTINGS/FLAT_KEYPOINT_FILTER_ENABLE"
		).toBool())
	{
		flat_area_keypoints_filter(
			no_nan_good_keypoints1, no_nan_good_keypoints2,
			good_matches_after_nan,
			no_nan_after_thresh_good_keypoints1,
			no_nan_after_thresh_good_keypoints2,
			good_matches_after_thresh_nan
		);
	}
	else
	{
		no_nan_after_thresh_good_keypoints1 = no_nan_good_keypoints1;
		no_nan_after_thresh_good_keypoints2 = no_nan_good_keypoints2;
		good_matches_after_thresh_nan = good_matches_after_nan;
	}

	qDebug() << "  SURF Keypoints found:" << good_matches_after_thresh_nan.size();

	out_keypoints1 = no_nan_after_thresh_good_keypoints1;
	out_keypoints2 = no_nan_after_thresh_good_keypoints2;
	out_matches = good_matches_after_thresh_nan;
}

void SurfKeypointDetector::surf_remove_nan_from_keypoints(
	std::vector<cv::Point2f> keypoints1,
	std::vector<cv::Point2f> keypoints2,
	std::vector<cv::DMatch>  matches,
	std::vector<cv::Point2f> &result_keypoints1,
	std::vector<cv::Point2f> &result_keypoints2,
	std::vector<cv::DMatch>  &result_matches
	)
{
	using namespace cv;

	for (int i = 0; i < keypoints1.size(); i++)
	{
		int x1 = int(round(keypoints1[i].x));
		int y1 = int(round(keypoints1[i].y));

		int x2 = int(round(keypoints2[i].x));
		int y2 = int(round(keypoints2[i].y));

		if (pcl_isnan(_point_cloud_ptr1->at(x1, y1).z) == false &&
			pcl_isnan(_point_cloud_ptr2->at(x2, y2).z) == false)
		{
			result_matches.push_back(matches[i]);

			keypoints1[i].x = x1;
			keypoints1[i].y = y1;

			keypoints2[i].x = x2;
			keypoints2[i].y = y2;

			result_keypoints1.push_back(keypoints1[i]);
			result_keypoints2.push_back(keypoints2[i]);
		}
	}
}

void SurfKeypointDetector::surf_fill_keypoint_clouds(
	std::vector<cv::Point2f> keypoints1,
	std::vector<cv::Point2f> keypoints2
	)
{
	for (int i = 0; i < keypoints1.size(); i++)
	{
		if (pcl_isnan(_point_cloud_ptr1->at(keypoints1[i].x, keypoints1[i].y).z) == false &&
			pcl_isnan(_point_cloud_ptr2->at(keypoints2[i].x, keypoints2[i].y).z) == false)
		{
			keypoint_point_cloud_ptr1->push_back(
				_point_cloud_ptr1->at(keypoints1[i].x, keypoints1[i].y)
			);

			keypoint_point_cloud_ptr2->push_back(
				_point_cloud_ptr2->at(keypoints2[i].x, keypoints2[i].y)
			);
		}
	}
}

//---------------------------------------------------------

void SurfKeypointDetector::surf_grid_keypoints_matcher(
	std::vector<cv::Point2f> in_keypoints1,
	std::vector<cv::Point2f> in_keypoints2,
	std::vector<cv::DMatch>  in_matches,
	std::vector<cv::Point2f>& out_keypoints1,
	std::vector<cv::Point2f>& out_keypoints2,
	std::vector<cv::DMatch>&  out_matches
	)
{
	int match_index1 = 0;
	int match_index2 = 0;

	float max_area = 0.0f;
	for (size_t i = 0; i < in_keypoints1.size(); i++)
	{
		cv::Point2i p1, p2, p3;
		float area = 0;
		p1.x = int(round(in_keypoints1[i].x));
		p1.y = int(round(in_keypoints1[i].y));

		for (size_t j = 0; j < in_keypoints1.size(); j++)
		{
			p2.x = int(round(in_keypoints1[j].x));
			p2.y = int(round(in_keypoints1[j].y));

			if (p2.x > p1.x) {
				p3.x = p2.x;
				p3.y = p1.y;
			}
			else {
				p3.x = p1.x;
				p3.y = p2.y;
			}

			area = sqrtf(powf(p1.x - p3.x, 2.0f) + powf(p1.y - p3.y, 2.0f)) *
				   sqrtf(powf(p2.x - p3.x, 2.0f) + powf(p2.y - p3.y, 2.0f));
			if (area > max_area)
			{
				max_area = area;
				match_index1 = i;
				match_index2 = j;
			}
		}
	}

	cv::Point2i p1;
	p1.x = int(round(in_keypoints1[match_index1].x));
	p1.y = int(round(in_keypoints1[match_index1].y));
	cv::Point2i p2;
	p2.x = int(round(in_keypoints1[match_index2].x));
	p2.y = int(round(in_keypoints1[match_index2].y));
	if (p1.x > p2.x) {
		std::swap(p1.x, p2.x);
	}
	if (p1.y > p2.y) {
		std::swap(p1.y, p2.y);
	}

	cv::Point2i p1_;
	p1_.x = int(round(in_keypoints2[match_index1].x));
	p1_.y = int(round(in_keypoints2[match_index1].y));
	cv::Point2i p2_;
	p2_.x = int(round(in_keypoints2[match_index2].x));
	p2_.y = int(round(in_keypoints2[match_index2].y));
	if (p1_.x > p2_.x) {
		std::swap(p1_.x, p2_.x);
	}
	if (p1_.y > p2_.y) {
		std::swap(p1_.y, p2_.y);
	}

	int length1 = (p2.x  - p1.x  + 1) * (p2.y  - p1.y  + 1);
	int length2 = (p2_.y - p1_.y + 1) * (p2_.x - p1_.x + 1);
	if (length1 >= length2)
	{
		fill_first_grid(p1, p2, out_keypoints1, out_matches);
		fill_second_grid(p1, p2, p1_, p2_, out_keypoints1, out_keypoints2, out_matches);
	}
	else
	{
		fill_first_grid(p1_, p2_, out_keypoints2, out_matches);
		fill_second_grid(p1_, p2_, p1, p2, out_keypoints2, out_keypoints1, out_matches);
	}
}

void SurfKeypointDetector::fill_first_grid(
	cv::Point2i p1, cv::Point2i p2,
	std::vector<cv::Point2f>& out_keypoints1,
	std::vector<cv::DMatch>&  out_matches
	)
{
	float vertical_grid_step   
		= settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/GRID_KEYPOINT_FILTER_VERT_RES").toInt();
	float horizontal_frid_step
		= settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/GRID_KEYPOINT_FILTER_HOR_RES").toInt();

	int index = 0;
	for (int y = p1.y; y <= p2.y; y += vertical_grid_step)
		for (int x = p1.x; x <= p2.x; x += horizontal_frid_step)
		{
			out_keypoints1.push_back(cv::Point2f(x, y));
			out_matches.push_back(cv::DMatch(index, index, 0));
			index++;
		}
}

void SurfKeypointDetector::fill_second_grid(
	cv::Point2i p1, cv::Point2i p2,
	cv::Point2i p1_, cv::Point2i p2_,
	std::vector<cv::Point2f>& out_keypoints1,
	std::vector<cv::Point2f>& out_keypoints2,
	std::vector<cv::DMatch>&  out_matches
	)
{
	float vertical_grid_step
		= settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/GRID_KEYPOINT_FILTER_VERT_RES").toInt();
	float horizontal_frid_step
		= settings->value("OPENCV_KEYPOINT_DETECTION_SETTINGS/GRID_KEYPOINT_FILTER_HOR_RES").toInt();

	float y_expander_coef = 1.0f;
	float x_expander_coef = 1.0f;

	if (p2_.x - p1_.x != p2.x - p1.x)
		x_expander_coef = ((float)p2_.x - p1_.x) / ((float)p2.x - p1.x);

	if (p2_.y - p1_.y != p2.y - p1.y)
		y_expander_coef = ((float)p2_.y - p1_.y) / ((float)p2.y - p1.y);

	int index = 0;
	for (float y = p1_.y; y <= p2_.y; y += vertical_grid_step * y_expander_coef)
		for (float x = p1_.x; x <= p2_.x; x += horizontal_frid_step * x_expander_coef)
		{
			out_keypoints2.push_back(cv::Point2f(int(round(x)), int(round(y))));
			out_matches[index].distance = sqrtf(
				powf(out_keypoints1[index].x - out_keypoints2[index].x, 2) +
				powf(out_keypoints1[index].y - out_keypoints2[index].y, 2)
			);
			index++;
		}

}

//---------------------------------------------------------