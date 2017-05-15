#ifndef CALIBRATIONINTERFACE_H
#define CALIBRATIONINTERFACE_H

#include <QDebug>
#include <QFileInfo>
#include <QObject>
#include <QSettings>
#include <QString>

#include <pcl/filters/filter.h>
#include <pcl/io/io.h>

#include <core/base/types.h>
#include <io/pclio.h>

class CalibrationInterface : public ScannerBase {
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::vector<float> CalibMap;

    CalibrationInterface(QObject* parent, QSettings* parent_settings);

    void calibrate();
    void calibrate(PcdPtrVector& input_pcd_vector);
    void undistort(Frames& frames);

    void saveCalibrationData();
    void loadCalibrationData();

private:
    PcdPtrVector raw_pcd_data_vector;
    PcdPtrVector udistort_pcd_vector;
    PcdPtrVector calib_plane_vector;
    std::vector<std::vector<int> > matches_vector;

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > x_vector;
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
