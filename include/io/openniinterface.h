#ifndef OPENNIINTERFACE_H
#define OPENNIINTERFACE_H

#include <QDebug>
#include <QFileInfo>
#include <QObject>
#include <QSettings>
#include <QtSerialPort/QtSerialPort>

#include <OpenNI.h>

#include "core/base/types.h"
#include "core/keypoints/arucokeypointdetector.h"
#include "io/pclio.h"
#include "utility/tools.h"

class OpenNiInterface : public ScannerBase {
    Q_OBJECT

public:
    struct Frame {
        typedef boost::shared_ptr<Frame> Ptr;

        QSettings* settings;
        QSettings* configs;

        openni::VideoFrameRef color_frame;
        openni::VideoFrameRef depth_frame;

        cv::Mat color_frame_mat;
        cv::Mat depth_frame_mat;
        std::vector<cv::Vec3f> world_coords;

        Pcd::Ptr point_cloud;

        Frame(openni::VideoStream& colorStream, openni::VideoStream& depthStream, QSettings* settings_, QSettings* configs_)
            : settings(settings_)
            , configs(configs_)
        {
            colorStream.readFrame(&color_frame);
            const openni::RGB888Pixel* color_buffer = (const openni::RGB888Pixel*)color_frame.getData();

            color_frame_mat.create(color_frame.getHeight(), color_frame.getWidth(), CV_8UC3);
            memcpy(color_frame_mat.data, color_buffer, 3 * color_frame.getHeight() * color_frame.getWidth() * sizeof(uint8_t));
            cv::cvtColor(color_frame_mat, color_frame_mat, CV_BGR2RGB);

            depthStream.readFrame(&depth_frame);
            world_coords = depthpixels2world((openni::DepthPixel*)depth_frame.getData(), depthStream);
            depth_frame_mat = world2mat(world_coords);

            point_cloud = make_xyzrgb_pcd(world_coords, color_frame_mat);
        }

        void update_world_coords()
        {
            depth_frame_mat = world2mat(world_coords);
            point_cloud = make_xyzrgb_pcd(world_coords, color_frame_mat);
        }

        void save(const uint& index)
        {
            QString pcd_image_filename_pattern = QFileInfo(settings->fileName()).absolutePath() + "/"
                + settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "/"
                + configs->value("READING_PATTERNS_SETTINGS/POINT_CLOUD_IMAGE_NAME").toString();
            cv::imwrite(pcd_image_filename_pattern.arg(index).toStdString(), color_frame_mat);

            QString pcd_filename_pattern = QFileInfo(settings->fileName()).absolutePath() + "/"
                + settings->value("PROJECT_SETTINGS/PCD_DATA_FOLDER").toString() + "/"
                + configs->value("READING_PATTERNS_SETTINGS/POINT_CLOUD_NAME").toString();
            pclio::save_one_point_cloud(pcd_filename_pattern.arg(index), point_cloud);
        }

        static std::vector<cv::Vec3f> depthpixels2world(
            const openni::DepthPixel* depthpixels,
            const openni::VideoStream& depthStream)
        {
            std::vector<cv::Vec3f> world_coords;

            for (int y = 0; y < HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    cv::Vec3f world_coord;
                    openni::CoordinateConverter::convertDepthToWorld(
                        depthStream, x, y, depthpixels[x + y * WIDTH], &world_coord[0], &world_coord[1], &world_coord[2]);
                    world_coords.push_back(world_coord);
                }
            }

            return world_coords;
        }

        static cv::Mat world2mat(const std::vector<cv::Vec3f>& world_coords)
        {
            cv::Mat depthFrameMat(cv::Size(WIDTH, HEIGHT), CV_8UC3);

            for (int y = 0; y < HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    int val = world_coords[x + y * WIDTH][2];
                    int r = 0, g = 0, b = 0;

                    while (val > 2550) {
                        val -= 2550 - 255;
                    }

                    if (val <= 255 * 2) //r >
                    {
                        val -= 255 * 1;
                        r = val;
                    } else if (val <= 255 * 3) // g >
                    {
                        val -= 255 * 2;
                        r = 255;
                        g = val;
                    } else if (val <= 255 * 5) // r <
                    {
                        val -= 255 * 4;
                        r = 255 - val;
                        g = 255;
                    } else if (val <= 255 * 5) // b >
                    {
                        val -= 255 * 4;
                        g = 255;
                        b = val;
                    } else if (val <= 255 * 6) // g <
                    {
                        val -= 255 * 5;
                        b = 255;
                        g = 255 - val;
                    } else if (val <= 255 * 7) // r >
                    {
                        val -= 255 * 6;
                        b = 255;
                        r = val;
                    } else if (val <= 255 * 8) // b <
                    {
                        val -= 255 * 7;
                        r = 255;
                        b = 255 - val;
                    }

                    depthFrameMat.at<cv::Vec3b>(y, x)[0] = b;
                    depthFrameMat.at<cv::Vec3b>(y, x)[1] = g;
                    depthFrameMat.at<cv::Vec3b>(y, x)[2] = r;
                }
            }

            return depthFrameMat;
        }

        static Pcd::Ptr make_xyzrgb_pcd(
            const std::vector<cv::Vec3f>& world_coords,
            const cv::Mat& color_frame_mat)
        {
            Pcd::Ptr point_cloud(new Pcd);

            point_cloud->width = WIDTH;
            point_cloud->height = HEIGHT;
            point_cloud->resize(WIDTH * HEIGHT);

            for (int y = 0; y < HEIGHT; y++) {
                for (int x = 0; x < WIDTH; x++) {
                    const uint index = x + y * WIDTH;
                    const double z_val = world_coords[index][2];

                    point_cloud->at(x, y).z = z_val == 0 ? NAN : z_val;
                    point_cloud->at(x, y).x = world_coords[index][0];
                    point_cloud->at(x, y).y = world_coords[index][1];

                    point_cloud->at(x, y).r = color_frame_mat.at<cv::Vec3b>(y, x)[2];
                    point_cloud->at(x, y).g = color_frame_mat.at<cv::Vec3b>(y, x)[1];
                    point_cloud->at(x, y).b = color_frame_mat.at<cv::Vec3b>(y, x)[0];
                }
            }

            return point_cloud;
        }
    };
    typedef std::deque<Frame::Ptr> Frames;

    OpenNiInterface(QObject* parent, QSettings* parent_settings);
    ~OpenNiInterface();

    void initialize_interface();
    void shutdown_interface();

    void start_stream();
    void start_rotation_stream();

    void take_long_images();
    void take_one_long_image();
    void save_long_image_data();

    bool isInit();

private:
    bool stream_from_record;
    bool record_stream;
    bool stream_undistortion;
    bool stream_bilateral;
    bool record_to_pcd_data;

    QSerialPort* serial;
    bool device_inited;

    cv::Mat calib_matrix;
    cv::Mat dist_coeffs;

    openni::Device device;
    openni::Status rc;
    openni::VideoStream colorStream;
    openni::VideoStream depthStream;
    openni::Recorder recorder;
    openni::VideoFrameRef frame;

    Frames frames;
    uint max_frames_size;

    void clearDataFolder();

    void load_calibration_data();

    Frame::Ptr take_one_frame(const uint& frame_index);

    Frame::Ptr take_one_optimized_image(const uint& number);

    void initialize_rotation();

    void shutdown_rotation();

    void rotate(int angle);

    void apply_undistortion(std::vector<cv::Vec3f>& world_coords);

    void apply_bilateral_filter(std::vector<cv::Vec3f>& world_coords);
};

#endif // OPENNIINTERFACE_H
