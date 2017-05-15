#include "io/calibrationinterface.h"

#define WIDTH 640
#define HEIGHT 480

CalibrationInterface::CalibrationInterface(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
{
}

//####################################################################
//--------------------------Calibration-------------------------------

void CalibrationInterface::calibrate()
{
    prepare();
    initialize();
    calibrate_all_pcd();
}

void CalibrationInterface::calibrate(PcdPtrVector& input_pcd_vector)
{
    prepare();
    initialize(input_pcd_vector);
    calibrate_all_pcd();
}

void CalibrationInterface::prepare()
{
    x_vector.clear();
    calib_plane_vector.clear();
    calib_map_vector.clear();
}

void CalibrationInterface::initialize()
{
    for (int i = 0; i < raw_pcd_data_vector.size(); i++) {
        //Remove NaN's
        std::vector<int> matches;
        raw_pcd_data_vector[i]->is_dense = false;
        pcl::removeNaNFromPointCloud(*raw_pcd_data_vector[i], *raw_pcd_data_vector[i], matches);
        matches_vector.push_back(matches);

        //Fill empty x_vector
        x_vector.push_back(Eigen::VectorXf(3));

        //Fill calib plane vector
        calib_plane_vector.push_back(PcdPtr(new Pcd(*raw_pcd_data_vector[i])));

        //Fill calib map vector
        calib_map_vector.push_back(CalibMap(WIDTH * HEIGHT, 0));
    }
}

void CalibrationInterface::initialize(PcdPtrVector input_pcd_vector)
{
    for (int i = 0; i < input_pcd_vector.size(); i++) {
        //Copy PCD data
        raw_pcd_data_vector.push_back(PcdPtr(new Pcd(*input_pcd_vector[i])));

        //Remove NaN's
        std::vector<int> matches;
        pcl::removeNaNFromPointCloud(*raw_pcd_data_vector[i], *raw_pcd_data_vector[i], matches);
        matches_vector.push_back(matches);

        //Fill empty x_vector
        x_vector.push_back(Eigen::VectorXf(3));

        //Fill calib plane vector
        calib_plane_vector.push_back(PcdPtr(new Pcd(*raw_pcd_data_vector[i])));

        //Fill calib map vector
        calib_map_vector.push_back(CalibMap(WIDTH * HEIGHT, 0));
    }
}

void CalibrationInterface::calibrate_all_pcd()
{
    qDebug() << "Calibration...";

    for (uint i = 0; i < raw_pcd_data_vector.size(); ++i) {
        calibrate_one_pcd(i);
    }

    qDebug() << "Done!";
}

void CalibrationInterface::calibrate_one_pcd(int index)
{
    calculate_least_squares(index);
    calculate_calibration_plane(index);
    calculate_calibration_map(index);
}

void CalibrationInterface::calculate_least_squares(int index)
{
    Eigen::VectorXf b_vector(3);
    float v_x1 = 0.0f, v_x2 = 0.0f, v_x3 = 0.0f;

    Eigen::MatrixXf A_matrix(3, 3);
    float x11 = 0.0f, x12 = 0.0f, x13 = 0.0f;
    float x21 = 0.0f, x22 = 0.0f, x23 = 0.0f;
    float x31 = 0.0f, x32 = 0.0f, x33 = 0.0f;

    for (uint i = 0; i < (*raw_pcd_data_vector[index]).size(); ++i) {
        const float x = (*raw_pcd_data_vector[index])[i].x;
        const float y = (*raw_pcd_data_vector[index])[i].y;
        const float z = raw_pcd_data_vector[index].get()->points[i].z;
        x11 += x * x;
        x12 += x * y;
        x13 += x;
        x21 += x * y;
        x22 += y * y;
        x23 += y;
        x31 += x;
        x32 += y;

        v_x1 += x * z;
        v_x2 += y * z;
        v_x3 += z;
    }
    x33 = static_cast<float>(raw_pcd_data_vector[index].get()->points.size());

    A_matrix << x11, x12, x13, x21, x22, x23, x31, x32, x33;
    b_vector << v_x1, v_x2, v_x3;

    x_vector[index] = A_matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_vector);
}

void CalibrationInterface::calculate_calibration_plane(int index)
{
    const float A = x_vector[index][0];
    const float B = x_vector[index][1];
    const float C = x_vector[index][2];

    float D = (*raw_pcd_data_vector[index])[0].z;
    for (uint i = 1; i < (*raw_pcd_data_vector[index]).size(); ++i) {
        D += (*raw_pcd_data_vector[index])[i].z;
        D /= 2;
    }

    for (int i = 0; i < raw_pcd_data_vector[index].get()->points.size(); i++) {
        const float x = (*raw_pcd_data_vector[index])[i].x;
        const float y = (*raw_pcd_data_vector[index])[i].y;
        const float z = (-(A * x) / C) + (-(B * y) / C) + C;

        (*calib_plane_vector[index])[i].x = x;
        (*calib_plane_vector[index])[i].y = y;
        (*calib_plane_vector[index])[i].z = z;
    }
}

void CalibrationInterface::calculate_calibration_map(int index)
{
    for (int i = 0; i < matches_vector[index].size(); i++) {
        const float raw_z = (*raw_pcd_data_vector[index])[i].z;
        const float plane_z = (*calib_plane_vector[index])[i].z;
        const float shift = raw_z - plane_z;
        calib_map_vector[index][matches_vector[index][i]] = shift;
    }
}

//####################################################################
//-------------------------Undistortion-------------------------------

void CalibrationInterface::undistort(Frames& frames)
{
    for (uint i = 0; i < frames.size(); ++i) {
        udistort_pcd_vector.push_back(frames[i].pointCloudPtr);
    }

    undistort_all_pcd();
}

void CalibrationInterface::undistort_all_pcd()
{
    //Reorganize
    for (int i = 0; i < matches_vector.size(); i++) {
        PcdPtr tmp_pdc_ptr(new Pcd);
        tmp_pdc_ptr->width = WIDTH;
        tmp_pdc_ptr->height = HEIGHT;
        tmp_pdc_ptr->resize(HEIGHT * WIDTH);

        for (uint y = 0; y < HEIGHT; ++y) {
            for (uint x = 0; x < WIDTH; ++x) {
                tmp_pdc_ptr->at(x, y).x = 0;
                tmp_pdc_ptr->at(x, y).y = 0;
                tmp_pdc_ptr->at(x, y).z = NAN;
                tmp_pdc_ptr->at(x, y).r = 0;
                tmp_pdc_ptr->at(x, y).g = 0;
                tmp_pdc_ptr->at(x, y).b = 0;
            }
        }

        for (uint j = 0; j < raw_pcd_data_vector[i]->size(); ++j) {
            tmp_pdc_ptr->points[matches_vector[i][j]] = raw_pcd_data_vector[i]->points[j];
        }

        *raw_pcd_data_vector[i] = *tmp_pdc_ptr;
        raw_pcd_data_vector[i]->is_dense = false;
    }

    //Undistort
    qDebug() << "Applying undistortion to point clouds...";
    for (int i = 0; i < udistort_pcd_vector.size(); i++) {
        qDebug() << QString("Undistortion %1 / %2").arg(i + 1).arg(udistort_pcd_vector.size()).toStdString().c_str();
        undistort_one_pcd(i);
    }
    qDebug() << "Done!";
}

void CalibrationInterface::undistort_one_pcd(int index)
{
    if (udistort_pcd_vector.empty() || raw_pcd_data_vector.empty()) {
        return;
    }

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (!std::isnan(udistort_pcd_vector[index]->at(x, y).z)) {
                float z = udistort_pcd_vector[index]->at(x, y).z;

                //Detecting sample radius
                int begin_index = -1;
                int end_index = -1;
                for (int i = 0; i < raw_pcd_data_vector.size(); i++) {
                    if (std::isnan(raw_pcd_data_vector[i]->at(x, y).z)) {
                        continue;
                    }

                    if (raw_pcd_data_vector[i]->at(x, y).z > z) {
                        end_index = i;
                        if (i > 0) {
                            begin_index = i - 1;
                        }
                        break;
                    }
                }

                //Undistort
                if (begin_index == -1 && end_index == -1) {
                    const float shift = calib_map_vector[calib_map_vector.size() - 1][x + y * WIDTH];
                    udistort_pcd_vector[index].get()->at(x, y).z -= shift;
                } else if (begin_index == -1 && end_index != -1) {
                    const float shift = calib_map_vector[0][x + y * WIDTH];
                    udistort_pcd_vector[index].get()->at(x, y).z -= shift;
                } else {
                    const float shift_1 = calib_map_vector[begin_index][x + y * WIDTH];
                    const float shift_2 = calib_map_vector[end_index][x + y * WIDTH];
                    const float z_1 = raw_pcd_data_vector[begin_index].get()->at(x, y).z;
                    const float z_2 = raw_pcd_data_vector[end_index].get()->at(x, y).z;

                    if (z >= z_1 && z < z_2) //If Z fits in the radius
                    {
                        const float n = ((z - z_1) / ((z_2 - z_1) / 100.0f)) / 100.0f;
                        const float k = (shift_2 - shift_1) * n;
                        udistort_pcd_vector[index]->at(x, y).z -= shift_1 + k;
                    } else //If it does not
                    {
                        udistort_pcd_vector[index].get()->at(x, y).z -= shift_1;
                    }
                }
            }
        }
    }
}

//####################################################################
//-------------------------------IO-----------------------------------

void CalibrationInterface::saveCalibrationData()
{
}

void CalibrationInterface::loadCalibrationData()
{
    const bool log = configs.value("CALIBRATION_SETTINGS/NUMBER").toBool();

    if (log) {
        qDebug() << "Load calibration data\n"
                 << "Start reading data...";
    }

    raw_pcd_data_vector.clear();
    int to = configs.value("CALIBRATION_SETTINGS/NUMBER").toInt();
    for (uint i = 0; i < to; ++i) {
        if (log) {
            qDebug() << "Reading" << QFileInfo(settings->fileName()).absolutePath() + "/" + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/" + configs.value("CALIBRATION_SETTINGS/POINT_CLOUD_NAME").toString().arg(i);
        }

        PcdPtr point_cloud_ptr(new Pcd);
        pclio::load_one_point_cloud(
            (QFileInfo(settings->fileName()).absolutePath() + "/"
                + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
                + configs.value("CALIBRATION_SETTINGS/POINT_CLOUD_NAME").toString().arg(i)),
            point_cloud_ptr);
        pclio::scale_one_point_cloud(point_cloud_ptr);

        if (point_cloud_ptr->empty()) {
            continue;
        }

        raw_pcd_data_vector.push_back(point_cloud_ptr);
    }

    if (log) {
        qDebug() << "Done!\n"
                 << "Total:" << raw_pcd_data_vector.size();
    }
}
