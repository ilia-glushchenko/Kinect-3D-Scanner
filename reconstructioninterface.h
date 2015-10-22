#ifndef RECONSTRUCTIONINTERFACE_H
#define RECONSTRUCTIONINTERFACE_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <pcl/registration/lum.h>
#include <pcl/registration/elch.h>

//##################################################
//-----------------------STL------------------------
#include <cstdlib> //rand
#include <utility> //pair
#include <vector> 
#include <chrono>  //pause
#include <thread>  //pause

//##################################################
//--------------------------------------------------
#include "imagesviewerwidget.h"
#include "calibrationinterface.h"
#include "pclio.h"

#include "pcdfilters.h"

#include "surfkeypointdetector.h"
#include "arucokeypointdetector.h"

#include "keypointsrejection.h"

#include "sacregistration.h"
#include "icpregistration.h"

#include "volumereconstruction.h"
#include "pcdvizualizer.h"

#include "types.h"

class ReconstructionInterface : public ScannerBase
{
	Q_OBJECT

public:
	ReconstructionInterface(QObject *parent, QSettings* parent_settings);

	void set_use_reconstruction(bool);
	void set_use_undistortion(bool);
	void set_use_bilateral_filter(bool);
	void set_use_statistical_outlier_removal_filter(bool);
	void set_use_moving_least_squares_filter(bool);	

private:
	bool use_reconstruction;
	bool use_undistortion;
	bool use_bilateral_filter;
	bool use_statistical_outlier_removal_filter;
	bool use_moving_least_squares_filter;

	VolumeReconstruction* volumeReconstruction;
	PcdVizualizer*		  pcdVizualizer;

	//#######################################################
	//------------------------SAVE READ----------------------
	void read_data(int from, int to, int step, Frames& frames);

	//#######################################################
	//----------------------PCL Part-------------------------

	//#-------------------Filters BEGIN----------------------
	void filter_point_cloud(
		Frame& frame
	);
	void filter_all_point_clouds(
		Frames& frames
	);
	void remove_nan_from_all_clouds(
		Frames& frames
	);
	void reorganize_all_point_clouds(
		Frames& frames
	);
	//#--------------------Filters END-----------------------

	//#------------------Keypoints BEGIN---------------------
	void calculate_keypoint_clouds(
		Frame& frame1, Frame& frame2,
		KeypointsFrame& keypoints_frame,
		std::vector<cv::Mat>& matches_images_vector
	);
	void calculate_all_keypoint_pairs(
		Frames& frames,
		KeypointsFrames& keypointsFrames
	);
	void calculate_all_keypoint_pairs_rejection(
		KeypointsFrames& keypointsFrames_in, KeypointsFrames& keypointsFrames_out
	);
	void calculate_middle_based_all_keypoint_pairs(
		Frames& frames,
		KeypointsFrames& keypointsFrames,
		int middle_index
	);
	//#------------------Keypoints END------------------------

	//-----------------Registration BEGIN---------------------

	void perform_sac_registration(
		Frames&			 frames,
		KeypointsFrames& keypointsFrames,
		Eigen::Matrix4f& intial_transformation_matrix,
		Matrix4fVector&  sac_translation_matrix_vector,
		int middle_index = -1
	);
	void perform_icp_registration(
		Frames&			 frames, 
		KeypointsFrames& keypointsFrames,
		Matrix4fVector&  icp_translation_matrix_vector,
		int middle_index = -1
	);

	void calculate_final_transformation_matrix_vector(
		Matrix4fVector& sac_translation_matrix_vector,
		Matrix4fVector& icp_translation_matrix_vector,
		Matrix4fVector& final_translation_matrix_vector,
		int final_translation_matrix_vector_size
	);

	void perform_tsdf_integration(
		Frames& frames,
		Matrix4fVector& final_translation_matrix_vector
	);

	void perform_tsdf_meshing(
		Matrix4fVector& final_translation_matrix_vector
	);

	//------------------Registration END----------------------

	void perform_reconstruction(
		Frames& frames_in,
		Eigen::Matrix4f& intial_transformation_matrix,
		KeypointsFrames& keypoints_frames_out,
		Matrix4fVector& final_translation_matrix_vector_out,
		int middle_index = -1
	);
	void perform_reconstruction();
	void perform_iterative_reconstruction();
	void perform_partition_recursive_reconstruction();

	//LUM based reconstruction
	void mergeKeypointsFrames(
		KeypointsFrames& keypoints_frames_in,
		KeypointsFrames& loop_ends_keypoint_frames_in,
		Matrix4fVector& final_translation_matrix_vector_in,
		CorrespondencesVector& correspondences_vector_out,
		PcdPtrVector& merged_keypoints_vector_out
	);
	void perform_elch_correction(
		PcdPtrVector& merged_keypoints_vector_in,
		CorrespondencesVector& merged_correspondences_vector_in,
		Matrix4fVector& final_translation_matrix_vector_out
	);
	void perform_lum_correction(
		PcdPtrVector& merged_keypoints_vector_in,
		CorrespondencesVector& merged_correspondences_vector_in,
		Matrix4fVector& final_translation_matrix_vector_out
	);
	void perform_lum_reconstruction();

public slots:
	void slot_perform_reconstruction();
	void slot_change_pair(int);

	void slot_set_use_reconstruction  (int);
	void slot_set_use_undistortion	  (int);
	void slot_set_use_bilateral_filter(int);
	void slot_set_use_statistical_outlier_removal_filter(int);
	void slot_set_use_moving_least_squares_filter		(int);
};

#endif // RECONSTRUCTIONINTERFACE_H
