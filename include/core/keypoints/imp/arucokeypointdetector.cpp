#include "core/keypoints/arucokeypointdetector.h"

#define WIDTH 640
#define HEIGHT 480

ArUcoKeypointDetector::ArUcoKeypointDetector(QObject* parent, QSettings* parent_settings)
    : ScannerBase(parent, parent_settings)
{
    CamParam.readFromXMLFile(
        (QFileInfo(settings->fileName()).absolutePath() + "/"
            + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
            + configs.value("ARUCO_SETTINGS/CAMERA_PARAMS_FILE_NAME").toString())
            .toStdString());

    D.fromFile(
        (QFileInfo(settings->fileName()).absolutePath() + "/"
            + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
            + configs.value("ARUCO_SETTINGS/MARKERS_DICT_FILE_NAME").toString())
            .toStdString());
}

ArUcoKeypointDetector::ArUcoKeypointDetector(
    QObject* parent,
    QSettings* parent_settings,
    PcdPtr cloud_ptr1,
    PcdPtr cloud_ptr2,
    cv::Mat img1,
    cv::Mat img2,
    PcdPtr keypoint_cloud_ptr1,
    PcdPtr keypoint_cloud_ptr2)
    : ScannerBase(parent, parent_settings)
{
    point_cloud_ptr1 = cloud_ptr1;
    point_cloud_ptr2 = cloud_ptr2;
    image1 = img1;
    image2 = img2;
    keypoint_point_cloud_ptr1 = keypoint_cloud_ptr1;
    keypoint_point_cloud_ptr2 = keypoint_cloud_ptr2;

    CamParam.readFromXMLFile(
        (QFileInfo(settings->fileName()).absolutePath() + "/"
            + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
            + configs.value("ARUCO_SETTINGS/CAMERA_PARAMS_FILE_NAME").toString())
            .toStdString());

    D.fromFile(
        (QFileInfo(settings->fileName()).absolutePath() + "/"
            + settings->value("PROJECT_SETTINGS/CALIB_DATA_FOLDER").toString() + "/"
            + configs.value("ARUCO_SETTINGS/MARKERS_DICT_FILE_NAME").toString())
            .toStdString());
}

void ArUcoKeypointDetector::detect()
{
    find_keypoints();
}

void ArUcoKeypointDetector::getMarkersVector(
    cv::Mat InImage,
    std::vector<aruco::Marker>* Markers)
{
    recognize_markers(InImage, Markers);
}

void ArUcoKeypointDetector::recognize_markers(
    cv::Mat img,
    std::vector<aruco::Marker>* Markers)
{
    using namespace aruco;

    freopen("nul", "w", stdout);

    cv::Mat InImage(img.size(), img.type());
    img.copyTo(InImage);

    if (configs.value("ARUCO_SETTINGS/GREEN_MARKER").toBool()) {
        cv::Mat InImage_HSV, InImage_GRAY;
        cv::cvtColor(InImage, InImage_GRAY, CV_BGR2GRAY);
        cv::cvtColor(InImage, InImage_HSV, CV_BGR2HSV);

        int b_from = configs.value("ARUCO_SETTINGS/H_B_FROM").toInt();
        int b_to = configs.value("ARUCO_SETTINGS/H_B_TO").toInt();
        int g_from = configs.value("ARUCO_SETTINGS/H_G_FROM").toInt();
        int g_to = configs.value("ARUCO_SETTINGS/H_G_TO").toInt();

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                if (InImage_HSV.at<cv::Vec3b>(y, x)[0] > g_from
                    && InImage_HSV.at<cv::Vec3b>(y, x)[0] < g_to) {
                    InImage_GRAY.at<uchar>(y, x) = 0;
                } else if (InImage_HSV.at<cv::Vec3b>(y, x)[0] > b_from
                    && InImage_HSV.at<cv::Vec3b>(y, x)[0] < b_to) {
                    InImage_GRAY.at<uchar>(y, x) = 255;
                } else {
                    InImage_GRAY.at<uchar>(y, x) = 255;
                }
            }
        }
        cvtColor(InImage_GRAY, InImage, CV_GRAY2BGR);
    }

    CamParam.resize(InImage.size());

    MarkerDetector MDetector;

    HighlyReliableMarkers::loadDictionary(D);
    MDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
    MDetector.setThresholdParams(
        configs.value("ARUCO_SETTINGS/THRESHOLD_FROM").toInt(),
        configs.value("ARUCO_SETTINGS/THRESHOLD_TO").toInt());
    MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
    MDetector.setWarpSize(
        (D[0].n() + configs.value("ARUCO_SETTINGS/WARP_SIZE_COEF_1").toFloat())
        * configs.value("ARUCO_SETTINGS/WARP_SIZE_COEF_2").toFloat());
    MDetector.setMinMaxSize(
        configs.value("ARUCO_SETTINGS/MINIMUM_MARKER_SIZE").toFloat(),
        configs.value("ARUCO_SETTINGS/MAXIMUM_MARKER_SIZE").toFloat());

    float MarkerSize = configs.value("ARUCO_SETTINGS/MARKER_SIZE").toFloat();
    MDetector.detect(InImage, *Markers, CamParam, MarkerSize);

    if (configs.value("ARUCO_SETTINGS/DRAW_EACH_MARKER").toBool()
        || configs.value("ARUCO_SETTINGS/DRAW_STREAM_MARKER").toBool()) {
        for (int i = 0; i < Markers->size(); i++) {
            Markers->at(i).draw(InImage, cv::Scalar(0, 0, 255), 2);
            if (configs.value("ARUCO_SETTINGS/DRAW_CUBES").toBool()) {
                CvDrawingUtils::draw3dCube(InImage, Markers->at(i), CamParam);
            }
        }

        if (configs.value("ARUCO_SETTINGS/DRAW_EACH_MARKER").toBool()) {
            cv::imshow(QString("marker %1").arg(rand()).toStdString(), InImage);
        }
        if (configs.value("ARUCO_SETTINGS/DRAW_STREAM_MARKER").toBool()) {
            cv::imshow("markers", InImage);
        }
        if (configs.value("ARUCO_SETTINGS/DRAW_STREAM_THRESH").toBool()) {
            cv::imshow("thresh", MDetector.getThresholdedImage());
        }
    }

    fclose(stdout);
}

void ArUcoKeypointDetector::find_keypoints()
{
    using namespace cv;
    using namespace std;

    vector<aruco::Marker> marker_vector1;
    vector<aruco::Marker> marker_vector2;
    vector<pair<aruco::Marker, aruco::Marker> > marker_matches;

    recognize_markers(image1, &marker_vector1);
    recognize_markers(image2, &marker_vector2);

    for (int i = 0; i < marker_vector1.size(); i++)
        for (int j = 0; j < marker_vector2.size(); j++) {
            int id1 = marker_vector1[i].id;
            int id2 = marker_vector2[j].id;
            if (id1 == id2) {
                marker_matches.push_back(make_pair(marker_vector1[i], marker_vector2[j]));
                break;
            }
        }

    for (int i = 0; i < marker_matches.size(); i++) {
        Point2f center1 = marker_matches[i].first.getCenter();
        Point2f center2 = marker_matches[i].second.getCenter();

        int x1 = static_cast<int>(round(center1.x));
        int y1 = static_cast<int>(round(center1.y));

        int x2 = static_cast<int>(round(center2.x));
        int y2 = static_cast<int>(round(center2.y));

        if (!std::isnan(point_cloud_ptr1->at(x1, y1).z)
            && !std::isnan(point_cloud_ptr2->at(x2, y2).z)) {
            keypoint_point_cloud_ptr1->width = keypoint_point_cloud_ptr1->points.size() + 1;
            keypoint_point_cloud_ptr1->height = 1;
            keypoint_point_cloud_ptr1->points.push_back(
                point_cloud_ptr1->at(x1, y1));

            keypoint_point_cloud_ptr2->width = keypoint_point_cloud_ptr2->points.size() + 1;
            keypoint_point_cloud_ptr2->height = 1;
            keypoint_point_cloud_ptr2->points.push_back(
                point_cloud_ptr2->at(x2, y2));
        }
    }
    qDebug() << "  ARUCO Keypoints found:" << marker_matches.size();
}
