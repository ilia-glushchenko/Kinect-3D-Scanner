#ifndef OPENNIINTERFACE_H
#define OPENNIINTERFACE_H

#include <QObject>
#include <QSettings>
#include <QFileInfo>
#include <QtSerialPort/QtSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>

#include "OpenNI.h"

#include <chrono>  //pause
#include <thread>  //pause

#include "arucokeypointdetector.h"
#include "pclio.h"
#include "types.h"
#include "tools.h"

class OpenNiInterface : public ScannerBase
{
	Q_OBJECT

public:
	OpenNiInterface(QObject *parent, QSettings* parent_settings);
	~OpenNiInterface();

	void load_calibration_data();
	void initialize_interface();
	void start_stream();
	void start_rotation_stream();
	void start_iterative_rotation_stream();
	void take_one_optimized_image();
	void save_optimized_images();
	void shutdown_interface();

	bool isInit();
	void set_stream_from_record(bool);
	void set_record_stream(bool);
	void set_stream_undistortion(bool);
	void set_stream_bilateral(bool);
	void set_record_to_pcd(bool);

private:
	//###############################################################

	QSerialPort* serial;
	bool device_inited;

	//###############################################################

	bool stream_from_record;
	bool record_stream;
	bool stream_undistortion;
	bool stream_bilateral;
	bool record_to_pcd_data;

	//###############################################################

	cv::Mat calib_matrix;
	cv::Mat dist_coeffs;

	openni::Device device;
	openni::Status rc;
	openni::VideoStream   colorStream;
	openni::VideoStream   depthStream;
	openni::Recorder	  recorder;
	openni::VideoFrameRef frame;

	//###############################################################

	std::vector<cv::Mat> colorImagesMatVector;
	std::vector<cv::Mat> bufferColorImagesMatVector;
	std::vector<cv::Mat> depthImagesMatVector;
	std::vector<cv::Mat> bufferDepthImagesMatVector;

	std::vector<DepthMap> depthMapVector;
	std::vector<DepthMap> bufferDepthMapVector;

	std::vector<std::vector<cv::Vec3f>> worldCoordsVector;
	std::vector<std::vector<cv::Vec3f>> bufferWorldCoordsVector;

	//###############################################################

	void initialize();
	void shutdown();

	void read_frame();
	void take_one_image();
	void take_one_optimized_image(int number);
	void initialize_rotation();
	void shutdown_rotation();
	void rotate(int angle);
	void rotate_and_take_images();
	void rotate_and_take_optimized_images();

	void show_depth_map(DepthMap, QString = "title");

	//###############################################################

	void apply_undistortion_to_depthmap(
		openni::DepthPixel* src_depth_pixels,
		openni::DepthPixel* dest_depth_pixels
		);
	void apply_bilateral_filter(
		openni::DepthPixel* src_depth_pixels,
		openni::DepthPixel* dest_depth_pixels
		);


	//###############################################################

	void depthpixels_to_world_coordinate(
		openni::DepthPixel*     depth_pixels,
		std::vector<cv::Vec3f>* world_coords
	);
	void depthpixels_to_depthmap(
		openni::DepthPixel* depth_pixels,
		DepthMap* depth_map
	);
	void depthpixels_to_cv_mat(
		cv::Mat* image_mat,
		openni::DepthPixel* depth_pixels ,
		int colormap_type = 0,
		DepthMap* depth_map = NULL
	);
	void depth_map_to_point_cloud(
		int image_index,
		Pcd* point_cloud
	);

public slots:
	void slot_set_record_stream(int);
	void slot_set_stream_from_record(int);
	void slot_set_record_to_pcd(int);
	void slot_set_stream_undistortion(int);
	void slot_set_stream_bilateral(int);
	
};

#endif // OPENNIINTERFACE_H
