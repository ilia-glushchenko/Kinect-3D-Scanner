#include "calibrationinterface.h"

#define WIDTH  640 
#define HEIGHT 480 

CalibrationInterface::CalibrationInterface(QObject *parent)
{
	setParent(parent);
	settings = new QSettings("scaner.ini", QSettings::IniFormat, this);
}

//####################################################################
//--------------------------Calibration-------------------------------

void CalibrationInterface::calibrate()
{
	prepare();
	initialize();
	calibrate_all_pcd();
}

void CalibrationInterface::calibrate(PcdPtrVector input_pcd_vector)
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
	for (int i = 0; i < raw_pcd_data_vector.size(); i++)
	{
		//Remove NaN's
		PcdPtr tmp(new Pcd);
		std::vector<int> matches;
		raw_pcd_data_vector[i].get()->is_dense = false;
		pcl::removeNaNFromPointCloud(*raw_pcd_data_vector[i].get(), *tmp.get(), matches);
		pcl::copyPointCloud(*tmp.get(), *raw_pcd_data_vector[i].get());
		matches_vector.push_back(matches);

		//Fill empty x_vector
		x_vector.push_back(Eigen::VectorXf(3));

		//Fill calib plane vector
		PcdPtr calib_plane(new Pcd);
		pcl::copyPointCloud(*raw_pcd_data_vector[i].get(), *calib_plane.get());
		calib_plane_vector.push_back(calib_plane);

		//Fill calib map vector
		CalibMap calib_map(WIDTH*HEIGHT);
		for (int i = 0; i < calib_map.size(); i++)
			calib_map[i] = 0;
		calib_map_vector.push_back(calib_map);
	}
}

void CalibrationInterface::initialize(PcdPtrVector input_pcd_vector)
{
	for (int i = 0; i < input_pcd_vector.size(); i++)
	{
		//Copy PCD data
		PcdPtr point_cloud_ptr(new Pcd);
		pcl::copyPointCloud(*input_pcd_vector[i].get(), *point_cloud_ptr.get());
		raw_pcd_data_vector.push_back(point_cloud_ptr);

		//Remove NaN's
		std::vector<int> matches;
		pcl::removeNaNFromPointCloud(*point_cloud_ptr.get(), *point_cloud_ptr.get(), matches);
		matches_vector.push_back(matches);

		//Fill empty x_vector
		x_vector.push_back(Eigen::VectorXf(3));

		//Fill calib plane vector
		PcdPtr calib_plane(new Pcd);
		pcl::copyPointCloud(*raw_pcd_data_vector[i].get(), *calib_plane.get());
		calib_plane_vector.push_back(calib_plane);

		//Fill calib map vector
		CalibMap calib_map(WIDTH*HEIGHT);
		for (int i = 0; i < calib_map.size(); i++)
			calib_map[i] = 0;
		calib_map_vector.push_back(calib_map);
	}
}

void CalibrationInterface::calibrate_all_pcd()
{
	qDebug() << "Calibration...";
	for (int i = 0; i < raw_pcd_data_vector.size(); i++)
		calibrate_one_pcd(i);
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

	for (int i = 0; i < raw_pcd_data_vector[index].get()->points.size(); i++)
	{
		float x = raw_pcd_data_vector[index].get()->points[i].x;
		float y = raw_pcd_data_vector[index].get()->points[i].y;
		float z = raw_pcd_data_vector[index].get()->points[i].z;
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

	x_vector[index] =
		A_matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_vector);
}

void CalibrationInterface::calculate_calibration_plane(int index)
{
	float A = x_vector[index][0];
	float B = x_vector[index][1];
	float C = x_vector[index][2];

	float D = raw_pcd_data_vector[index].get()->points[0].z;
	for (int i = 1; i < raw_pcd_data_vector[index].get()->points.size(); i++)
	{
		D += raw_pcd_data_vector[index].get()->points[i].z;
		D /= 2;
	}

	for (int i = 0; i < raw_pcd_data_vector[index].get()->points.size(); i++)
	{
		float x = raw_pcd_data_vector[index].get()->points[i].x;
		float y = raw_pcd_data_vector[index].get()->points[i].y;
		float z = (-(A*x) / C) + (-(B*y) / C) + C;

		calib_plane_vector[index].get()->points[i].x = x;
		calib_plane_vector[index].get()->points[i].y = y;
		calib_plane_vector[index].get()->points[i].z = z;
	}
}

void CalibrationInterface::calculate_calibration_map(int index)
{
	for (int i = 0; i < matches_vector[index].size(); i++)
	{
		float raw_z   = raw_pcd_data_vector[index].get()->points[i].z;
		float plane_z = calib_plane_vector[index].get()->points[i].z;
		float shift   = raw_z - plane_z;
		calib_map_vector[index][matches_vector[index][i]] = shift;
	}
}

//####################################################################
//-------------------------Undistortion-------------------------------

void CalibrationInterface::undistort(Frames& frames)
{
	for (int i = 0; i < frames.size(); i++)
		udistort_pcd_vector.push_back(frames[i].pointCloudPtr);

	undistort_all_pcd();
}

void CalibrationInterface::undistort_all_pcd()
{
	//Reorganize
	for (int i = 0; i < matches_vector.size(); i++)
	{
		PcdPtr tmp_pdc_ptr(new Pcd);
		tmp_pdc_ptr.get()->width = WIDTH;
		tmp_pdc_ptr.get()->height = HEIGHT;
		tmp_pdc_ptr.get()->resize(HEIGHT*WIDTH);

		for (int y = 0; y < HEIGHT; y++)
			for (int x = 0; x < WIDTH; x++)
			{
				tmp_pdc_ptr.get()->at(x, y).x = 0;
				tmp_pdc_ptr.get()->at(x, y).y = 0;
				tmp_pdc_ptr.get()->at(x, y).z = NAN;
				tmp_pdc_ptr.get()->at(x, y).r = 0;
				tmp_pdc_ptr.get()->at(x, y).g = 0;
				tmp_pdc_ptr.get()->at(x, y).b = 0;
			}

		for (int j = 0; j < raw_pcd_data_vector[i].get()->size(); j++)
			tmp_pdc_ptr.get()->points[matches_vector[i][j]] = raw_pcd_data_vector[i].get()->points[j];

		*raw_pcd_data_vector[i].get() = *tmp_pdc_ptr.get();
		raw_pcd_data_vector[i].get()->is_dense = false;
	}

	//Undistort
	qDebug() << "Applying undistortion to point clouds...";
	for (int i = 0; i < udistort_pcd_vector.size(); i++)
	{
		qDebug() << QString("Undistortion %1 / %2").arg(i + 1).arg(udistort_pcd_vector.size()).toStdString().c_str();
		undistort_one_pcd(i); 
	}
	qDebug() << "Done!";
}

void CalibrationInterface::undistort_one_pcd(int index)
{
	if (udistort_pcd_vector.empty())
		return;
	if (raw_pcd_data_vector.empty())
		return;

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
		{		
			if (isnan<float>(udistort_pcd_vector[index].get()->at(x, y).z) == false)
			{
				float z = udistort_pcd_vector[index].get()->at(x, y).z;

				//Detecting sample radius
				int begin_index = -1;
				int end_index   = -1;
				for (int i = 0; i < raw_pcd_data_vector.size(); i++)
				{
					if (isnan<float>(raw_pcd_data_vector[i].get()->at(x, y).z))
						continue;
					if (raw_pcd_data_vector[i].get()->at(x, y).z > z)
					{
						end_index = i;
						if (i > 0)
							begin_index = i - 1;
						break;
					}
				}

				//Undistort
				if (begin_index == -1 && end_index == -1)
				{ 
					float shift =
						calib_map_vector[calib_map_vector.size() - 1][x + y * WIDTH];
					udistort_pcd_vector[index].get()->at(x, y).z -= shift;
				}
				else if (begin_index == -1 && end_index != -1)
				{
					float shift = calib_map_vector[0][x + y * WIDTH];
					udistort_pcd_vector[index].get()->at(x, y).z -= shift;
				}
				else
				{
					float shift_1 = calib_map_vector[begin_index][x + y * WIDTH];
					float shift_2 = calib_map_vector[end_index][x + y * WIDTH];
					float z_1 = raw_pcd_data_vector[begin_index].get()->at(x, y).z;
					float z_2 = raw_pcd_data_vector[end_index].get()->at(x, y).z;
					
					//If Z fits in the radius
					if (z >= z_1 && z < z_2)
					{
						float n = ((z - z_1) / ((z_2 - z_1) / 100.0f)) / 100.0f;
						float k = (shift_2 - shift_1) * n;
						udistort_pcd_vector[index].get()->at(x, y).z -= shift_1 + k;
					}
					//If it does not
					else
					{
						udistort_pcd_vector[index].get()->at(x, y).z -= shift_1;
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
	bool log = settings->value("CALIBRATION/NUMBER").toBool();

	if (log) {
		qDebug() << "Load calibration data";
		qDebug() << "Start reading data...";
	}

	raw_pcd_data_vector.clear();
	int to = settings->value("CALIBRATION/NUMBER").toInt();
	for (int i = 0; i < to; i++)
	{
		PcdPtr point_cloud_ptr(new Pcd);

		if (log)
			qDebug() << "Reading" << settings->value("CALIBRATION/POINT_CLOUD_NAME").toString().arg(i);

		PclIO::load_one_point_cloud(
			settings->value("CALIBRATION/POINT_CLOUD_NAME").toString().arg(i),
			point_cloud_ptr
		);

		if (point_cloud_ptr.get()->points.empty())
			continue;

		raw_pcd_data_vector.push_back(point_cloud_ptr);
	}

	if (log) {
		qDebug() << "Done!";
		qDebug() << "Total:" << raw_pcd_data_vector.size();
	}
}
