#ifndef CALIBRATIONINTERFACE_H
#define CALIBRATIONINTERFACE_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

#include <pcl/io/io.h>
#include <pcl/filters/filter.h>

#include "pclio.h"
#include "types.h"

class CalibrationInterface : public ScannerBase
{
	Q_OBJECT

public:
	CalibrationInterface(QObject *parent, QSettings* parent_settings);

	void calibrate();
	void calibrate(PcdPtrVector input_pcd_vector);
	void undistort(Frames& frames);

	void saveCalibrationData();
	void loadCalibrationData();

private:
	PcdPtrVector raw_pcd_data_vector;
	PcdPtrVector udistort_pcd_vector;
	PcdPtrVector calib_plane_vector;
	std::vector<std::vector<int>> matches_vector;

	std::vector<Eigen::Vector3f>  x_vector;
	std::vector<CalibMap> calib_map_vector;

	void prepare();
	void initialize();
	void initialize(PcdPtrVector input_pcd_vector);
	void calibrate_all_pcd();
	void calibrate_one_pcd(int index);
	void calculate_least_squares(int index);
	void calculate_calibration_plane(int index);
	void calculate_calibration_map(int index);

	void undistort_all_pcd();
	void undistort_one_pcd(int index);
};

#endif // CALIBRATIONINTERFACE_H
