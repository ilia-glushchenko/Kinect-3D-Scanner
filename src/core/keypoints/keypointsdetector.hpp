#ifndef KEYPOINTS_DETECTOR_H
#define KEYPOINTS_DETECTOR_H

#include "core/base/scannerbase.h"
#include "core/base/scannertypes.h"

template <typename DetectionMethod>
class KeypointsDetector : ScannerBase {
public:
    KeypointsDetector(QObject* parent, QSettings* parent_settings)
        : ScannerBase(parent, parent_settings)
    {
    }

    void setInput(const Frame& input_frame1, const Frame& input_frame2)
    {
        frame1 = input_frame1;
        frame2 = input_frame2;
        keypoints_frame = KeypointsFrame();
    }

    KeypointsFrame detect()
    {
        KeypointsFrame result;

        DetectionMethod detector(
            this, settings,
            frame1.pointCloudPtr, frame2.pointCloudPtr,
            frame1.pointCloudImage, frame2.pointCloudImage,
            result.keypointsPcdPair.first, result.keypointsPcdPair.second);
        detector.detect();

        for (unsigned int i = 0; i < result.keypointsPcdPair.first->size(); ++i) {
            result.keypointsPcdCorrespondences.push_back(pcl::Correspondence(
                i, i, pcl::euclideanDistance(
                          (*result.keypointsPcdPair.first)[i], (*result.keypointsPcdPair.second)[i])));
        }

        return result;
    }

    KeypointsFrame getKeypoints()
    {
        return keypoints_frame;
    }

private:
    Frame frame1;
    Frame frame2;
    KeypointsFrame keypoints_frame;
};

#endif // KEYPOINTS_DETECTOR_H
