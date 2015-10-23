#ifndef PCL_OPENNI_QT_KINECT_3D_SCANNER_H
#define PCL_OPENNI_QT_KINECT_3D_SCANNER_H

//##################################################
//-----------------------Qt-------------------------
#include <QIODevice>
#include <QByteArray>
#include <QFile>
#include <QDataStream>
#include <QTextStream>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <QtWidgets/QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QCheckBox>
#include <QStatusBar>

//##################################################
//----------------------Boost-----------------------
#include <boost/thread/thread.hpp>

//##################################################
//-----------------------PCL------------------------
#include <pcl/common/common_headers.h>

#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include <pcl/registration/icp_nl.h>
#include <pcl/correspondence.h>

#include <Eigen/StdVector>

//##################################################
//----------------------OpenCV----------------------
#include <opencv2/opencv.hpp>

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
#include "surfkeypointdetector.h"
#include "arucokeypointdetector.h"
#include "pclio.h"
#include "openniinterface.h"
#include "calibrationinterface.h"
#include "volumereconstruction.h"
#include "sacregistration.h"

class PCL_OpenNI_Qt_Kinect_3d_Scanner : public QMainWindow
{
	Q_OBJECT

	typedef std::vector<int> DepthMap;

	typedef pcl::PointXYZRGB		   PointType;
	typedef pcl::PointCloud<PointType> Pcd;
	typedef boost::shared_ptr<Pcd>	   PcdPtr;
	typedef std::vector<PcdPtr>		   PcdPtrVector;

	typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> Matrix4fVector;
	typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>	Affine3dVector;

public:
	PCL_OpenNI_Qt_Kinect_3d_Scanner(QWidget *parent = 0);

private:
	//#######################################################
	//--------------------Qt GUI Part------------------------
	QSettings*   settings;

	QWidget*	 widget;
	QStatusBar*  statusBar;
	QVBoxLayout* vBoxLayout;

	QPushButton* initButton;
	QCheckBox*	 recCheck;
	QCheckBox*	 streamFromCheck;
	QCheckBox*	 recToPclDataCheck;
	QCheckBox*	 undistCheck;
	QCheckBox*	 bilateralCheck;	

	QPushButton* takeImagesButton;
	QPushButton* takeOpImagesButton;
	QPushButton* takeOneOpImageButton;
 
	QPushButton* drawScene3dModelButton;
	QCheckBox*   undistrtionCheck;
	QCheckBox*	 bilateralFilterCheck;
	QCheckBox*	 removeNanCheck;
	QCheckBox*	 statFilterCheck;
	QCheckBox*	 approxFilterCheck;
	QCheckBox*	 icpCheck;

	QPushButton* saveDataButton;
	QPushButton* readDataButton;

	QSlider* slider;

	//#######################################################
	//------------------OpenNI Interface---------------------
	OpenNiInterface* openniInterface;

	//#######################################################
	//---------------Volume Reconstruction-------------------
	VolumeReconstruction* volumeReconstruction;

	//#######################################################
	//----------------------PCL Part-------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	int pair_viewer_index;
	int icp_max;

	std::vector<cv::Mat> colorImagesMatVector;
	PcdPtrVector	     point_cloud_vector;
	std::vector<std::vector<int>> cloudIndexesVector;

	std::vector<std::pair<PcdPtr, PcdPtr>> keypoint_point_cloud_vector;
	std::vector<pcl::Correspondences>	   correspondences_vector;
	
	Matrix4fVector icp_matrix_vector;

	Matrix4fVector sac_translation_matrix_vector;   //After recursion
	Matrix4fVector icp_translation_matrix_vector;   //After recursion
	Matrix4fVector final_translation_matrix_vector; //After recursion with SaC and ICP

	std::vector<cv::Mat> afterThreshNanMatchesImagesVector;


	//#-------------------Filters BEGIN----------------------
	void apply_bilateral_filter(
		PcdPtr src_point_cloud_ptr
		);

	void apply_statistical_outlier_removal_filter(
		PcdPtr in_point_cloud_ptr,
		PcdPtr out_point_cloud_ptr
		);
	void apply_moving_least_squares_filter(
		PcdPtr point_cloud_ptr
		);
	void apply_voxel_grid_reduction(
		PcdPtr in_point_cloud_ptr,
		PcdPtr out_point_cloud_ptr,
		float x_k = 1.0f, float y_k = 1.0f, float z_k = 1.0f
		);
	void filter_point_cloud(
		PcdPtr src_point_cloud_ptr,
		PcdPtr dest_point_cloud_ptr
		);
	void merging_all_point_clouds(
		PcdPtrVector point_cloud_vector,
		PcdPtr in_point_cloud_ptr
		);
	void apply_voxel_grid_reduction_to_all_point_clouds();
	void filter_all_point_clouds();
	void remove_nan_from_all_clouds();	
	void reorganize_all_point_clouds();	
	//#--------------------Filters END-----------------------

	//#----------------Vizualizer BEGIN----------------------
	void add_viewer_debug_text();
	void set_viewer_pose(
		pcl::visualization::PCLVisualizer& viewer,
		const Eigen::Affine3f& viewer_pose
		);
	void visualize_point_clouds(
		PcdPtrVector point_cloud_vector
		);
	void visualize_all_keypoint_clouds();
	void visualize_all_camera_poses();
	void visualize_mesh();

	//#------------------Vizualizer END----------------------

	//#------------------Keypoints BEGIN---------------------
	void calculate_keypoint_clouds(
		PcdPtr cloud_ptr1,
		PcdPtr cloud_ptr2,
		cv::Mat img1,
		cv::Mat img2,
		PcdPtr keypoint_cloud_ptr1,
		PcdPtr keypoint_cloud_ptr2
		);
	void calculate_all_keypoint_pairs();
	//#------------------Keypoints END------------------------

	void perform_sac_registration();

	//---------------------ICP BEGIN--------------------------
	void calculate_icp(
		PcdPtr input_cloud_ptr,
		PcdPtr target_cloud_ptr
	);
	void calculate_icp_for_all_keypoint_pairs();
	void calculate_recursive_icp();
	void apply_icp_vector();
	//----------------------ICP END---------------------------
		
	void pcl_main();

	//#######################################################
	//------------------------IO Partp-----------------------
	void save_data();
	void read_data();
	void prepare_read();

public slots:
	void slot_initialize();

	void slot_take_images();
	void slot_take_op_images();
	void slot_take_one_op_image();

	void slot_draw_scene3d_model();

	void slot_save_data();
	void slot_read_data();

	void slot_change_pair(int);

signals:
	void signal_take_one_image();
	void signal_write_com_port(char);

};

#endif // PCL_OPENNI_QT_KINECT_3D_SCANNER_H
