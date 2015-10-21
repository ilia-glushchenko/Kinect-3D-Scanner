#include "openniinterface.h"

#define WIDTH  640
#define HEIGHT 480

#define COLORMAP_TYPE_COLORED 0
#define COLORMAP_TYPE_CODED   1

#define INIT_PAUSE_TIME 2500

OpenNiInterface::OpenNiInterface(QObject *parent, QSettings* parent_settings)
	: ScannerBase(parent, parent_settings)
{
	stream_from_record = false;
	record_stream = false;
	stream_undistortion = false;
	stream_bilateral = false;
	record_to_pcd_data = false;

	device_inited = false;
	serial = new QSerialPort(this);

	load_calibration_data();
}

OpenNiInterface::~OpenNiInterface()
{
	shutdown_interface();
}

//###############################################################

void OpenNiInterface::initialize_interface()
{
	if (device_inited == false)
		initialize();
}
void OpenNiInterface::load_calibration_data()
{
	calib_matrix = (CvMat*)cvLoad(
		(settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "\\" +
		 settings->value("OPENNI_SETTINGS/CALIB_MATRIX_NAME").toString()).toStdString().c_str()
	);
	dist_coeffs = (CvMat*)cvLoad(
		(settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "\\" + 
		settings->value("OPENNI_SETTINGS/DIST_COEFF_NAME").toString()).toStdString().c_str()
	);
}
void OpenNiInterface::start_stream()
{
	if (device_inited)
		read_frame();
}
void OpenNiInterface::start_rotation_stream()
{
	if (device_inited)
		rotate_and_take_images();
}
void OpenNiInterface::start_iterative_rotation_stream()
{
	if (device_inited)
		rotate_and_take_optimized_images();
}
void OpenNiInterface::take_one_optimized_image()
{
	if (device_inited)
		take_one_optimized_image(settings->value("OP_REC/NUMBER").toInt());
}
void OpenNiInterface::save_optimized_images()
{
	qDebug() << "Saving data...";
	for (int i = 0; i < worldCoordsVector.size(); i++)
	{
		QString pcd_image_filename_pattern =
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "\\" +
			settings->value("READING_PATTERNS/POINT_CLOUD_IMAGE_NAME").toString();
		cv::imwrite(pcd_image_filename_pattern.arg(i).toStdString(), colorImagesMatVector[i]);

		PcdPtr point_cloud_ptr(new Pcd);
		depth_map_to_point_cloud(i, point_cloud_ptr.get());

		QString pcd_filename_pattern =
			settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "\\" +
			settings->value("READING_PATTERNS/POINT_CLOUD_NAME").toString();
		PclIO::save_one_point_cloud(pcd_filename_pattern.arg(i), point_cloud_ptr);
	}
	qDebug() << "Done!";
}
void OpenNiInterface::shutdown_interface()
{
	if (device_inited)
		shutdown();
}

//###############################################################

bool OpenNiInterface::isInit()
{
	return device_inited;
}
void OpenNiInterface::set_stream_from_record(bool value)
{
	stream_from_record = value;
}
void OpenNiInterface::set_record_stream(bool value)
{
	record_stream = value;
}
void OpenNiInterface::set_stream_undistortion(bool value)
{
	stream_undistortion = value;
}
void OpenNiInterface::set_stream_bilateral(bool value)
{
	stream_bilateral = value;
}
void OpenNiInterface::set_record_to_pcd(bool value)
{
	record_to_pcd_data = value;
}

//###############################################################

void OpenNiInterface::initialize()
{
	using namespace openni;

	QString openni_out_text = settings->value("OPENNI_SETTINGS/OUT_TEXT").toString();

	rc = OpenNI::initialize();
	if (rc != STATUS_OK) {
		qDebug() << QString("%1 %2").arg(openni_out_text).arg("Driver initializatioin failed");
		return;
	}

	if (stream_from_record) {
		QString filePath =
			settings->value("PROJECT_SETTINGS/STREAM_DATA_FOLDER").toString() + "\\" +
			settings->value("OPENNI_SETTINGS/RECORDED_STREAM_FILE_NAME").toString();

		if (tools::fileExists(filePath) == false) {
			rc = STATUS_ERROR;
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't open recording!");
		}
		else
		{
			rc = device.open(filePath.toStdString().c_str());
			PlaybackControl* pbc = device.getPlaybackControl();
			pbc->setRepeatEnabled(settings->value("OPENNI_SETTINGS/REPEAT_RECORDING").toBool());
			pbc->setSpeed(-1);
		}
	}
	else if (rc == STATUS_OK) {
		rc = device.open(ANY_DEVICE);
		if (rc != STATUS_OK) {
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't open device!");
		}
	}

	if (rc == STATUS_OK) {
		rc = device.setDepthColorSyncEnabled(true);
		if (rc != STATUS_OK)
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't sync color and depth frames");

		rc = depthStream.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't create a depth stream");

		rc = colorStream.create(device, SENSOR_COLOR);
		if (rc != STATUS_OK)
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't create a color stream");

		rc = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		if (rc != STATUS_OK)
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't enable registration depth to color");

		rc = depthStream.start();
		if (rc != STATUS_OK)
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't start a depth stream");

		rc = colorStream.start();
		if (rc != STATUS_OK)
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't start a color stream");

		if (record_stream && !stream_from_record) {
			rc = recorder.create("recording.oni");
			if (rc != STATUS_OK)
				qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't create recording!");

			rc = recorder.attach(colorStream);
			if (rc != STATUS_OK)
				qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't attach color stream!");

			rc = recorder.attach(depthStream);
			if (rc != STATUS_OK)
				qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't attach depth stream!");

			rc = recorder.start();
			if (rc != STATUS_OK)
				qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't start recorder!");
		}

		if (rc == STATUS_OK) {
			device_inited = true;
			qDebug() << QString("%1 %2").arg(openni_out_text).arg("Initialized successfully!");
		}
	}
}

void OpenNiInterface::shutdown()
{
	using namespace openni;

	QString openni_out_text = settings->value("OPENNI_SETTINGS/OUT_TEXT").toString();

	if (record_stream)
	{
		recorder.stop();
		recorder.destroy();
	}

	depthStream.stop();
	depthStream.destroy();

	colorStream.stop();
	colorStream.destroy();

	device.close();
	OpenNI::shutdown();

	device_inited = false;
	qDebug() << QString("%1 %2").arg(openni_out_text).arg("Shutdown successful!");
}

void OpenNiInterface::read_frame()
{
	using namespace openni;
	using namespace cv;

	VideoFrameRef depthFrame;
	VideoFrameRef colorFrame;

	Mat depthFrameMat;
	Mat colorFrameMat;

	while (true)
	{
		colorStream.readFrame(&colorFrame);
		const RGB888Pixel* colorBuffer = (const RGB888Pixel*)colorFrame.getData();

		colorFrameMat.create(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3);
		memcpy(
			colorFrameMat.data,
			colorBuffer,
			3 * colorFrame.getHeight()*colorFrame.getWidth()*sizeof(uint8_t)
			);

		cvtColor(colorFrameMat, colorFrameMat, CV_BGR2RGB);

		cv::Mat undist_image;
		if (stream_undistortion)
		{
			cv::undistort(colorFrameMat, undist_image, calib_matrix, dist_coeffs);
			colorFrameMat = undist_image;
		}

		imshow("Color", colorFrameMat);

		//------------------------------------------------------

		if (settings->value("ARUCO_SETTINGS/ENABLE_IN_STREAM").toBool())
		{
			std::vector<aruco::Marker> Markers;
			ArUcoKeypointDetector aruco(this, settings);
			aruco.getMarkersVector(colorFrameMat, &Markers);
		}

		//------------------------------------------------------

		depthStream.readFrame(&depthFrame);
		DepthPixel* src_depth_pixels = (DepthPixel*)depthFrame.getData();
		depthpixels_to_cv_mat(&depthFrameMat, src_depth_pixels, COLORMAP_TYPE_CODED);

		if (stream_undistortion)
		{
			DepthPixel* dest_depth_pixels = new DepthPixel[WIDTH*HEIGHT];
			apply_undistortion_to_depthmap(src_depth_pixels, dest_depth_pixels);
			src_depth_pixels = dest_depth_pixels;
		}

		if (stream_bilateral)
		{
			DepthPixel* dest_depth_pixels = new DepthPixel[WIDTH*HEIGHT];
			apply_bilateral_filter(src_depth_pixels, dest_depth_pixels);
			src_depth_pixels = dest_depth_pixels;
		}

		if (record_to_pcd_data)
		{
			QString pcd_image_filename_pattern =
				settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "\\" +
				settings->value("READING_PATTERNS/POINT_CLOUD_IMAGE_NAME").toString();
			imwrite(pcd_image_filename_pattern.arg(depthFrame.getFrameIndex()).toStdString(), colorFrameMat);

			vector<cv::Vec3f> worldCoords;
			depthpixels_to_world_coordinate(src_depth_pixels, &worldCoords);

			worldCoordsVector.push_back(worldCoords);
			colorImagesMatVector.push_back(colorFrameMat);

			PcdPtr point_cloud_ptr(new Pcd);
			depth_map_to_point_cloud(worldCoordsVector.size() - 1, point_cloud_ptr.get());

			QString pcd_filename_pattern =
				settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "\\" +
				settings->value("READING_PATTERNS/POINT_CLOUD_NAME").toString();			
			PclIO::save_one_point_cloud(pcd_filename_pattern.arg(depthFrame.getFrameIndex()), point_cloud_ptr);

			worldCoordsVector.pop_back();
			colorImagesMatVector.pop_back();
		}

		DepthMap depth_map;
		depthpixels_to_depthmap(src_depth_pixels, &depth_map);
		show_depth_map(depth_map, "stream");

		//--------------------------------------------
		waitKey(30);
		if (device_inited == false)
			break;

	}
}

void OpenNiInterface::take_one_image()
{
	using namespace cv;
	using namespace openni;
	using namespace std;

	VideoFrameRef colorFrame;
	VideoFrameRef depthFrame;

	Mat depthFrameMat;
	Mat colorFrameMat;

	colorStream.readFrame(&colorFrame);
	const RGB888Pixel* colorBuffer = (const RGB888Pixel*)colorFrame.getData();

	colorFrameMat.create(
		colorFrame.getHeight(),
		colorFrame.getWidth(),
		CV_8UC3
	);
	memcpy(
		colorFrameMat.data,
		colorBuffer,
		3 * colorFrame.getHeight()*colorFrame.getWidth()*sizeof(uint8_t)
	);

	cvtColor(colorFrameMat, colorFrameMat, CV_BGR2RGB);

	/*------------------------------------*/

	depthStream.readFrame(&depthFrame);
	DepthPixel* depthpixels = (DepthPixel*)depthFrame.getData();
	DepthMap depthMap;

	if (stream_bilateral) {
		apply_bilateral_filter(depthpixels, depthpixels);
	}

	depthpixels_to_cv_mat(
		&depthFrameMat, depthpixels, COLORMAP_TYPE_CODED, &depthMap
	);

	std::vector<cv::Vec3f> tmpWorldCoords;
	depthpixels_to_world_coordinate(depthpixels, &tmpWorldCoords);
	bufferWorldCoordsVector.push_back(tmpWorldCoords);

	/*------------------------------------*/

	bufferDepthMapVector.push_back(depthMap);

	bufferColorImagesMatVector.push_back(colorFrameMat);
	bufferDepthImagesMatVector.push_back(depthFrameMat);
}

void OpenNiInterface::take_one_optimized_image(int number)
{
	for (int i = 0; i < number; i++)
	{
		qDebug() << QString("%1/%2").arg(i + 1).arg(number);

		take_one_image();

		if (i == number - 1)
		{
			colorImagesMatVector.push_back(bufferColorImagesMatVector[0]);
			depthImagesMatVector.push_back(bufferDepthImagesMatVector[0]);
		}
		if (i == 0)
		{
			worldCoordsVector.push_back(bufferWorldCoordsVector[0]);
			depthMapVector.push_back(bufferDepthMapVector[0]);
		}
		else
		{
			int last_index = worldCoordsVector.size() - 1;

			for (int y = 0; y < HEIGHT; y++)
			{
				for (int x = 0; x < WIDTH; x++)
				{
					int index = x + y * WIDTH;

					double x_w = bufferWorldCoordsVector[0][index][0];
					double y_w = bufferWorldCoordsVector[0][index][1];
					double z_w = bufferWorldCoordsVector[0][index][2];

					if (z_w != 0)
					{
						if (worldCoordsVector[last_index][index][2] != 0)
						{
							worldCoordsVector[last_index][index][2] += z_w;
							worldCoordsVector[last_index][index][1] += y_w;
							worldCoordsVector[last_index][index][0] += x_w;

							worldCoordsVector[last_index][index][2] /= 2.0f;
							worldCoordsVector[last_index][index][1] /= 2.0f;
							worldCoordsVector[last_index][index][0] /= 2.0f;
						}
						else
						{
							worldCoordsVector[last_index][index][2] = z_w;
							worldCoordsVector[last_index][index][1] = y_w;
							worldCoordsVector[last_index][index][0] = x_w;
						}
					}

					double z_d = bufferDepthMapVector[0][index];
					if (z_d != 0)
					{
						if (depthMapVector[last_index][index] != 0)
						{
							depthMapVector[last_index][index] += z_d;
							depthMapVector[last_index][index] /= 2.0f;
						}
						else
						{
							depthMapVector[last_index][index] = z_d;
						}
					}
				}
			}
		}

		bufferWorldCoordsVector.clear();
		bufferDepthMapVector.clear();	
		bufferColorImagesMatVector.clear();
		bufferDepthImagesMatVector.clear();
	}	

	show_depth_map(depthMapVector[depthMapVector.size() - 1], QString("%1").arg(rand()));
}

void OpenNiInterface::initialize_rotation()
{
	serial->setPortName(
		settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()
	);
	serial->setBaudRate(QSerialPort::Baud57600);

	if (!serial->open(QIODevice::WriteOnly)) {
		qDebug() << QObject::tr("Failed to open port %1, error: %2").arg(
			settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()).arg(serial->errorString()
			);
		return;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(INIT_PAUSE_TIME));
}

void OpenNiInterface::shutdown_rotation()
{
	serial->close();
}

void OpenNiInterface::rotate(int angle)
{
	QByteArray writeData = QByteArray::fromStdString(QString("%1\n").arg(angle).toStdString());

	if (writeData.isEmpty()) {
		qDebug() << QObject::tr(
			"Either no data was currently available on the standard input for reading, or an error occurred for port %1, error: %2"
			).arg(settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()).arg(serial->errorString());
		return;
	}

	qint64 bytesWritten = serial->write(writeData);

	if (bytesWritten == -1) {
		qDebug() << QObject::tr("Failed to write the data to port %1, error: %2").arg(
			settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()).arg(serial->errorString()
			);
		return;
	}
	else if (bytesWritten != writeData.size()) {
		qDebug() << QObject::tr("Failed to write all the data to port %1, error: %2").arg(
			settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()).arg(serial->errorString()
			);
		return;
	}
	else if (!serial->waitForBytesWritten(5000)) {
		qDebug() << QObject::tr("Operation timed out or an error occurred for port %1, error: %2").arg(
			settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()).arg(serial->errorString()
			);
		return;
	}

	qDebug() << QObject::tr("Data successfully sent to port %1").arg(
		settings->value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString()
	);
}

void OpenNiInterface::rotate_and_take_images()
{
	initialize_rotation();

	rotate(settings->value("OPENNI_SETTINGS/ROTATION_ANGLE").toInt());
	start_stream();

	rotate(-1);
	shutdown_rotation();	
}

void OpenNiInterface::rotate_and_take_optimized_images()
{
	initialize_rotation();
	for (int i = 0; i < settings->value("OP_REC/TO").toInt(); i += settings->value("OP_REC/STEP").toInt())
	{
		if (i > 0)
			rotate(settings->value("OP_REC/STEP").toInt());
		else
			rotate(0);
		std::this_thread::sleep_for(std::chrono::milliseconds(INIT_PAUSE_TIME));
		qDebug() << i / settings->value("OP_REC/STEP").toInt() 
				 << "/" 
				 << settings->value("OP_REC/TO").toInt() / settings->value("OP_REC/STEP").toInt();
		take_one_optimized_image(settings->value("OP_REC/NUMBER").toInt());
	}
	rotate(-1);
	shutdown_rotation();

	save_optimized_images();
}

void OpenNiInterface::show_depth_map(DepthMap depthMap, QString title)
{
	using namespace std;
	using namespace cv;
	using namespace openni;

	if (depthMap.empty())
	{
		qDebug() << "Error: DepthMap is empty!";
		return;
	}

	Mat depthFrameMat;

	depthFrameMat.create(Size(WIDTH, HEIGHT), CV_8UC3);

	int x, y, i;
	for (y = 0; y < HEIGHT; y++)
		for (x = 0; x < WIDTH; x++)
		{
			i = x + y * WIDTH;

			int val = depthMap[i];
			int r = 0;
			int g = 0;
			int b = 0;

			while (val > 2550)
				val -= 2550 - 255;

			if (val <= 255 * 2)		 //r >
			{
				val -= 255 * 1;
				r = val;
			}
			else if (val <= 255 * 3) // g >
			{
				val -= 255 * 2;
				r = 255;
				g = val;
			}
			else if (val <= 255 * 5) // r <
			{
				val -= 255 * 4;
				r = 255 - val;
				g = 255;
			}
			else if (val <= 255 * 5) // b >
			{
				val -= 255 * 4;
				g = 255;
				b = val;
			}
			else if (val <= 255 * 6) // g <
			{
				val -= 255 * 5;
				b = 255;
				g = 255 - val;
			}
			else if (val <= 255 * 7) // r >
			{
				val -= 255 * 6;
				b = 255;
				r = val;
			}
			else if (val <= 255 * 8) // b <
			{
				val -= 255 * 7;
				r = 255;
				b = 255 - val;
			}

			depthFrameMat.at<Vec3b>(y, x)[0] = b;
			depthFrameMat.at<Vec3b>(y, x)[1] = g;
			depthFrameMat.at<Vec3b>(y, x)[2] = r;

			i++;
		}

	imshow(QString("DepthMap %1").arg(title).toStdString(), depthFrameMat);
}

//###############################################################

void OpenNiInterface::apply_undistortion_to_depthmap(
	openni::DepthPixel* src_depth_pixels,
	openni::DepthPixel* dest_depth_pixels
	)
{
	cv::Mat img, img_res;
	img_res.create(HEIGHT, WIDTH, CV_32FC1);
	img.create(HEIGHT, WIDTH, CV_32FC1);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			img.at<float>(y, x) =
			static_cast<float>(src_depth_pixels[x + y*WIDTH]);

	undistort(img, img_res, calib_matrix, dist_coeffs);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			dest_depth_pixels[x + y*WIDTH] =
			static_cast<uint16_t>(round(img_res.at<float>(y, x)));
}

void OpenNiInterface::apply_bilateral_filter(
	openni::DepthPixel* src_depth_pixels,
	openni::DepthPixel* dest_depth_pixels
	)
{
	int d =
		settings->value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/D"
		).toInt();
	double sigma_color =
		settings->value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_COLOR"
		).toDouble();
	double sigma_space =
		settings->value(
		"OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_SPACE"
		).toDouble();

	cv::Mat img, img_res;
	img_res.create(HEIGHT, WIDTH, CV_32FC1);
	img.create(HEIGHT, WIDTH, CV_32FC1);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			img.at<float>(y, x) =
			static_cast<float>(src_depth_pixels[x + y*WIDTH]);

	bilateralFilter(img, img_res, d, sigma_color, sigma_space);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
			dest_depth_pixels[x + y*WIDTH] =
			static_cast<uint16_t>(round(img_res.at<float>(y, x)));
}

//###############################################################

void OpenNiInterface::depthpixels_to_world_coordinate(
	openni::DepthPixel*     depthpixels,
	std::vector<cv::Vec3f>* worldCoords
	)
{
	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
		{
			int index = x + y * WIDTH;
			float worldX, worldY, worldZ;

			openni::CoordinateConverter::convertDepthToWorld(
				depthStream, x, y, depthpixels[index], &worldX, &worldY, &worldZ);

			cv::Vec3f world_coord;
			world_coord[0] = worldX;
			world_coord[1] = worldY;
			world_coord[2] = worldZ;

			worldCoords->push_back(world_coord);
		}
}

void OpenNiInterface::depthpixels_to_depthmap(
	openni::DepthPixel* depthpixels,
	DepthMap*   depthMap
	)
{
	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
		{
			int i = x + y * WIDTH;
			depthMap->push_back(depthpixels[i]);
		}
}

void OpenNiInterface::depthpixels_to_cv_mat(
	cv::Mat* depthFrameMat,
	openni::DepthPixel* depthpixels,
	int colormap_type,
	DepthMap* depthMap
	)
{
	using namespace std;
	using namespace cv;
	using namespace openni;

	depthFrameMat->create(Size(WIDTH, HEIGHT), CV_8UC3);

	int maximal = 0, minimal = 10000;
	for (int i = 0; i < HEIGHT*WIDTH; i++)
	{
		if (depthpixels[i] > maximal)
			maximal = depthpixels[i];
		if (depthpixels[i] != 0 && depthpixels[i] < minimal)
			minimal = depthpixels[i];
	}


	int i;
	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
		{
			i = x + y * WIDTH;

			int val = depthpixels[i];
			if (depthMap != NULL)
				depthMap->push_back(val);
			i++;

			int r = 0;
			int g = 0;
			int b = 0;

			if (colormap_type == COLORMAP_TYPE_COLORED)
			{
				val = val / (maximal / (255 * 7));

				if (val <= 255 * 2)		 //r >
				{
					val -= 255 * 1;
					r = val;
				}
				else if (val <= 255 * 3) // g >
				{
					val -= 255 * 2;
					r = 255;
					g = val;
				}
				else if (val <= 255 * 5) // r <
				{
					val -= 255 * 4;
					r = 255 - val;
					g = 255;
				}
				else if (val <= 255 * 5) // b >
				{
					val -= 255 * 4;
					g = 255;
					b = val;
				}
				else if (val <= 255 * 6) // g <
				{
					val -= 255 * 5;
					b = 255;
					g = 255 - val;
				}
				else if (val <= 255 * 7) // r >
				{
					val -= 255 * 6;
					b = 255;
					r = val;
				}
				else if (val <= 255 * 8) // b <
				{
					val -= 255 * 7;
					r = 255;
					b = 255 - val;
				}
			}
			else if (colormap_type == COLORMAP_TYPE_CODED)
			{
				int counter = 0;
				int j = val;
				while (j > 256)
				{
					j -= 256;
					counter++;
				}

				b = val - (256 * counter) - 1;
				g = counter - 1;
				r = 0;
			}

			depthFrameMat->at<Vec3b>(y, x)[0] = b;
			depthFrameMat->at<Vec3b>(y, x)[1] = g;
			depthFrameMat->at<Vec3b>(y, x)[2] = r;
		}
}

void OpenNiInterface::depth_map_to_point_cloud(
	int image_index,
	Pcd* point_cloud
	)
{
	point_cloud->width  = WIDTH;
	point_cloud->height = HEIGHT;
	point_cloud->resize(WIDTH * HEIGHT);

	for (int y = 0; y < HEIGHT; y++)
		for (int x = 0; x < WIDTH; x++)
		{
			int index = x + y * WIDTH;

			double z_val = worldCoordsVector[image_index][index][2];
			if (z_val == 0)
				point_cloud->at(x, y).z = NAN;
			else
				point_cloud->at(x, y).z = z_val;

			point_cloud->at(x, y).x = worldCoordsVector[image_index][index][0];
			point_cloud->at(x, y).y = worldCoordsVector[image_index][index][1];

			point_cloud->at(x, y).r = colorImagesMatVector[image_index].at<cv::Vec3b>(y, x)[2];
			point_cloud->at(x, y).g = colorImagesMatVector[image_index].at<cv::Vec3b>(y, x)[1];
			point_cloud->at(x, y).b = colorImagesMatVector[image_index].at<cv::Vec3b>(y, x)[0];
		}
}

//###############################################################

void  OpenNiInterface::slot_set_stream_from_record(int value)
{
	if (value == 0)
		set_stream_from_record(false);
	else
		set_stream_from_record(true);
}
void  OpenNiInterface::slot_set_record_stream(int value)
{
	if (value == 0)
		set_record_stream(false);
	else
		set_record_stream(true);
}
void  OpenNiInterface::slot_set_stream_undistortion(int value)
{
	if (value == 0)
		set_stream_undistortion(false);
	else
		set_stream_undistortion(true);
}
void  OpenNiInterface::slot_set_stream_bilateral(int value)
{
	if (value == 0)
		set_stream_bilateral(false);
	else
		set_stream_bilateral(true);
}
void  OpenNiInterface::slot_set_record_to_pcd(int value)
{
	if (value == 0)
		set_record_to_pcd(false);
	else
		set_record_to_pcd(true);
}
