#include "io/openniinterface.h"

#include <QDir>
#include <QtSerialPort/QSerialPortInfo>

#include <chrono>
#include <thread>

#define WIDTH 640
#define HEIGHT 480

#define INIT_PAUSE_TIME 2500

OpenNiInterface::OpenNiInterface(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
    , max_frames_size(configs.value("OPENNI_SETTINGS/MAX_BUFFER_SIZE").toInt())
    , record_stream(settings->value("STREAM_SETTINGS/ENABLE_STREAM_RECORDING").toBool())
    , stream_from_record(settings->value("STREAM_SETTINGS/ENABLE_REPLAY_RECORD_STREAM").toBool())
    , record_to_pcd_data(settings->value("STREAM_SETTINGS/ENABLE_CONVERT_TO_PCD").toBool())
    , stream_undistortion(settings->value("STREAM_SETTINGS/ENABLE_UNDISTORTION").toBool())
    , stream_bilateral(settings->value("STREAM_SETTINGS/ENABLE_BILATERAL_FILTER").toBool())
    , device_inited(false)
    , serial(new QSerialPort(this))
{
    load_calibration_data();
}

OpenNiInterface::~OpenNiInterface()
{
    shutdown_interface();
}

void OpenNiInterface::clearDataFolder()
{
    const QString pcd_data_folder = QFileInfo(settings->fileName()).absolutePath()
        + "/" + settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString();

    QDir dir(pcd_data_folder);
    while (!dir.exists()) {
        dir.mkdir(".");
    }

    dir.setNameFilters(QStringList() << "*.*");
    dir.setFilter(QDir::Files);
    foreach (QString dirFile, dir.entryList()) {
        dir.remove(dirFile);
    }
}

void OpenNiInterface::load_calibration_data()
{
    const std::string calib_mat_path = (QFileInfo(settings->fileName()).absolutePath() + "/"
                                           + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
                                           + configs.value("OPENNI_SETTINGS/CALIB_MATRIX_NAME").toString())
                                           .toStdString();

    const std::string dist_coeffs_path = (QFileInfo(settings->fileName()).absolutePath() + "/"
                                             + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
                                             + configs.value("OPENNI_SETTINGS/DIST_COEFF_NAME").toString())
                                             .toStdString();

    if (boost::filesystem::exists(calib_mat_path) && boost::filesystem::exists(dist_coeffs_path)) {
        calib_matrix = (CvMat*)cvLoad(calib_mat_path.c_str());
        dist_coeffs = (CvMat*)cvLoad(dist_coeffs_path.c_str());
    } else {
        qDebug() << QString("Cannot load calibration data! \nCalib mat: %1 \nDist coeffs: %2")
                        .arg(calib_mat_path.c_str())
                        .arg(dist_coeffs_path.c_str())
                        .toStdString()
                        .c_str();
    }
}

void OpenNiInterface::initialize_interface()
{
    if (!device_inited) {
        qDebug() << "Initialization...";
        QString openni_out_text = configs.value("OPENNI_SETTINGS/OUT_TEXT").toString();

        openni::Status stat = openni::OpenNI::initialize();
        if (stat != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Driver initializatioin failed");
            return;
        }

        if (stream_from_record) {
            QString filePath = QFileInfo(settings->fileName()).absolutePath() + "/"
                + settings->value("PROJECT_SETTINGS/STREAM_DATA_FOLDER").toString() + "/"
                + configs.value("OPENNI_SETTINGS/RECORDED_STREAM_FILE_NAME").toString();

            if (!boost::filesystem::exists(filePath.toStdString()) || (device.open(filePath.toStdString().c_str()) != openni::STATUS_OK)) {
                qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't open recording!");
                qDebug() << "Path:" << filePath;
                return;
            } else {
                openni::PlaybackControl* pbc = device.getPlaybackControl();
                pbc->setRepeatEnabled(configs.value("OPENNI_SETTINGS/REPEAT_RECORDING").toBool());
                pbc->setSpeed(-1);
            }
        } else if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't open device!");
            return;
        }

        device_inited = true;
        qDebug() << QString("%1 %2").arg(openni_out_text).arg("Initialized successfully!");

        if (device.setDepthColorSyncEnabled(true) != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't sync color and depth frames");
        }

        if (depthStream.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't create a depth stream");
        }
        if (colorStream.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't create a color stream");
        }

        if (depthStream.start() != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't start a depth stream");
        }
        if (colorStream.start() != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't start a color stream");
        }

        if (device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != openni::STATUS_OK) {
            qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't enable registration depth to color");
        }

        if (record_stream && !stream_from_record) {
            const QString filePath = QFileInfo(settings->fileName()).absolutePath() + "/"
                + settings->value("PROJECT_SETTINGS/STREAM_DATA_FOLDER").toString() + "/"
                + configs.value("OPENNI_SETTINGS/RECORDED_STREAM_FILE_NAME").toString();

            if (recorder.create(filePath.toStdString().c_str()) != openni::STATUS_OK) {
                qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't create recording!");
            }

            if (recorder.attach(colorStream) != openni::STATUS_OK) {
                qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't attach color stream!");
            }
            if (recorder.attach(depthStream) != openni::STATUS_OK) {
                qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't attach depth stream!");
            }

            if (recorder.start() != openni::STATUS_OK) {
                qDebug() << QString("%1 %2").arg(openni_out_text).arg("Couldn't start recorder!");
            }
        }

        qDebug() << "Done!";
    }
}

void OpenNiInterface::shutdown_interface()
{
    if (device_inited) {
        qDebug() << "Shutdown...";
        const QString openni_out_text = configs.value("OPENNI_SETTINGS/OUT_TEXT").toString();

        if (record_stream) {
            recorder.stop();
            recorder.destroy();
        }

        depthStream.stop();
        depthStream.destroy();

        colorStream.stop();
        colorStream.destroy();

        device.close();
        openni::OpenNI::shutdown();

        device_inited = false;
        qDebug() << QString("%1 %2").arg(openni_out_text).arg("Shutdown successful!");
    }
}

void OpenNiInterface::start_stream()
{
    if (record_to_pcd_data) {
        qDebug() << "Clearing data folder...";
        clearDataFolder();
    }

    uint frame_index = 0;

    while (device_inited) {
        take_one_frame(++frame_index);
    }
}

void OpenNiInterface::start_rotation_stream()
{
    if (isInit()) {
        initialize_rotation();
        qDebug() << "Rotate:" << configs.value("OPENNI_SETTINGS/ROTATION_ANGLE").toInt();
        rotate(configs.value("OPENNI_SETTINGS/ROTATION_ANGLE").toInt());

        start_stream();

        qDebug() << "Return:" << -configs.value("OPENNI_SETTINGS/ROTATION_ANGLE").toInt();
        rotate(-1 * configs.value("OPENNI_SETTINGS/ROTATION_ANGLE").toInt());
        shutdown_rotation();
    }
}

void OpenNiInterface::take_long_images()
{
    if (device_inited) {
        frames.clear();
        Frames().swap(frames);
        initialize_rotation();

        for (int i = 0; i < configs.value("LONG_IMAGE_SETTINGS/TO").toInt() && device_inited;
             i += configs.value("LONG_IMAGE_SETTINGS/STEP").toInt()) {
            rotate(i > 0 ? configs.value("LONG_IMAGE_SETTINGS/STEP").toInt() : 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(INIT_PAUSE_TIME));

            qDebug() << i / configs.value("LONG_IMAGE_SETTINGS/STEP").toInt()
                     << "/" << configs.value("LONG_IMAGE_SETTINGS/TO").toInt() / configs.value("LONG_IMAGE_SETTINGS/STEP").toInt();

            frames.push_back(take_one_optimized_image(configs.value("LONG_IMAGE_SETTINGS/NUMBER").toInt()));
        }

        rotate(-1);
        shutdown_rotation();
        save_long_image_data();
    }
}

void OpenNiInterface::take_one_long_image()
{
    if (device_inited) {
        frames.push_back(take_one_optimized_image(configs.value("LONG_IMAGE_SETTINGS/NUMBER").toInt()));
    }
}

void OpenNiInterface::save_long_image_data()
{
    qDebug() << "Saving data...";

    clearDataFolder();

    for (uint i = 0; i < frames.size(); ++i) {
        frames[i]->save(i);
    }

    qDebug() << "Done!";
}

bool OpenNiInterface::isInit()
{
    return device_inited;
}

OpenNiInterface::Frame::Ptr OpenNiInterface::take_one_frame(const uint& frame_index)
{
    Frame::Ptr frame(new Frame(colorStream, depthStream, settings, &configs));

    if (stream_bilateral) {
        apply_bilateral_filter(frame->world_coords);
        frame->update_world_coords();
    }

    if (stream_undistortion) {
        apply_undistortion(frame->world_coords);
        frame->update_world_coords();

        cv::Mat undist_mat; //Note: cv::undistort works only with empty out buffer
        cv::undistort(frame->color_frame_mat, undist_mat, calib_matrix, dist_coeffs);
        frame->color_frame_mat = undist_mat;
    }

    if (configs.value("ARUCO_SETTINGS/ENABLE_IN_STREAM").toBool()) {
        std::vector<aruco::Marker> Markers;
        ArUcoKeypointDetector aruco(this, settings);
        aruco.getMarkersVector(frame->color_frame_mat, &Markers);
    }

    cv::imshow("Color", frame->color_frame_mat);
    cv::imshow("Depth", frame->depth_frame_mat);
    cv::waitKey(30);

    if (frame_index > 15 && record_to_pcd_data) {
        qDebug() << "Saving Frame" << frame_index - 15;
        frame->save(frame_index - 15);
    }

    return frame;
}

OpenNiInterface::Frame::Ptr OpenNiInterface::take_one_optimized_image(const uint& number)
{
    Frame::Ptr sum_frame(new Frame(colorStream, depthStream, settings, &configs));
    std::vector<cv::Vec3f> counter_map(WIDTH * HEIGHT, cv::Vec3f(1, 1, 1));

    for (uint i = 0; i < number; ++i) {
        qDebug() << QString("%1/%2").arg(i + 1).arg(number);

        Frame::Ptr tmp_frame(new Frame(colorStream, depthStream, settings, &configs));

        for (uint y = 0; y < HEIGHT; ++y) {
            for (uint x = 0; x < WIDTH; ++x) {
                const uint index = x + y * WIDTH;

                ++counter_map[index][0];
                sum_frame->world_coords[index][0] += tmp_frame->world_coords[index][0];
                ++counter_map[index][1];
                sum_frame->world_coords[index][1] += tmp_frame->world_coords[index][1];

                if (tmp_frame->world_coords[index][2] != 0) {
                    ++counter_map[index][2];
                    sum_frame->world_coords[index][2] += tmp_frame->world_coords[index][2];
                }
            }
        }
    }

    for (uint y = 0; y < HEIGHT; ++y) {
        for (uint x = 0; x < WIDTH; ++x) {
            sum_frame->world_coords[x + y * WIDTH][0] /= counter_map[x + y * WIDTH][0];
            sum_frame->world_coords[x + y * WIDTH][1] /= counter_map[x + y * WIDTH][1];
            sum_frame->world_coords[x + y * WIDTH][2] /= counter_map[x + y * WIDTH][2];
        }
    }

    sum_frame->update_world_coords();
    cv::imshow(QString("DepthMap %1").arg(rand()).toStdString(), sum_frame->depth_frame_mat);

    return sum_frame;
}

void OpenNiInterface::initialize_rotation()
{
    serial->setPortName(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString());
    serial->setBaudRate(QSerialPort::Baud57600);

    if (!serial->open(QIODevice::WriteOnly)) {
        qDebug() << QObject::tr("Failed to open port %1, error: %2")
                        .arg(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString())
                        .arg(serial->errorString());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(INIT_PAUSE_TIME));
}

void OpenNiInterface::shutdown_rotation()
{
    serial->close();
}

void OpenNiInterface::rotate(int angle)
{
    const QByteArray writeData = QByteArray::fromStdString(QString("%1\n").arg(angle).toStdString());

    if (writeData.isEmpty()) {
        qDebug() << QObject::tr(
                        "Either no data was currently available on the standard input for reading, or an error occurred for port %1, error: %2")
                        .arg(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString())
                        .arg(serial->errorString());
        return;
    }

    const qint64 bytesWritten = serial->write(writeData);

    if (bytesWritten == -1) {
        qDebug() << QObject::tr("Failed to write the data to port %1, error: %2")
                        .arg(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString())
                        .arg(serial->errorString());
    } else if (bytesWritten != writeData.size()) {
        qDebug() << QObject::tr("Failed to write all the data to port %1, error: %2")
                        .arg(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString())
                        .arg(serial->errorString());
    } else if (!serial->waitForBytesWritten(5000)) {
        qDebug() << QObject::tr("Operation timed out or an error occurred for port %1, error: %2")
                        .arg(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString())
                        .arg(serial->errorString());
    } else {
        qDebug() << QObject::tr("Data successfully sent to port %1")
                        .arg(configs.value("OPENNI_SETTINGS/SERIAL_PORT_NAME").toString());
    }

    const QByteArray readData = serial->readAll();
    qDebug() << "Read:" << readData.toStdString().c_str();
}

void OpenNiInterface::apply_undistortion(
    std::vector<cv::Vec3f>& world_coords)
{
    cv::Mat img(HEIGHT, WIDTH, CV_32FC1);
    cv::Mat img_res(HEIGHT, WIDTH, CV_32FC1);

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            img.at<float>(y, x) = world_coords[x + y * WIDTH][2];
        }
    }

    undistort(img, img_res, calib_matrix, dist_coeffs);

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            world_coords[x + y * WIDTH][2] = std::round(img_res.at<float>(y, x));
        }
    }
}

void OpenNiInterface::apply_bilateral_filter(
    std::vector<cv::Vec3f>& world_coords)
{
    const int d = configs.value("OPENCV_BILATERAL_FILTER_SETTINGS/D").toInt();
    const double sigma_color = configs.value("OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_COLOR").toDouble();
    const double sigma_space = configs.value("OPENCV_BILATERAL_FILTER_SETTINGS/SIGMA_SPACE").toDouble();

    cv::Mat img(HEIGHT, WIDTH, CV_32FC1);
    cv::Mat img_res(HEIGHT, WIDTH, CV_32FC1);

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            img.at<float>(y, x) = world_coords[x + y * WIDTH][2];
        }
    }

    bilateralFilter(img, img_res, d, sigma_color, sigma_space);

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            world_coords[x + y * WIDTH][2] = std::round(img_res.at<float>(y, x));
        }
    }
}
