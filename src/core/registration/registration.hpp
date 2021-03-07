#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "core/base/scannerbase.h"
#include "core/base/scannertypes.h"

#include "core/keypoints/arucokeypointdetector.h"
#include "core/keypoints/keypointsdetector.hpp"
#include "core/keypoints/keypointsrejection.h"
#include "core/keypoints/surfkeypointdetector.h"

class Registration : public ScannerBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Registration(QObject* parent, QSettings* parent_settings)
        : ScannerBase(parent, parent_settings)
    {
    }

    void setKeypoints(const KeypointsFrames& keypoints_)
    {
        keypoints = keypoints_;
    }

    Matrix4fVector align(Frames& output_transformed_data)
    {
        if (keypoints.empty()) {
            calculate_all_keypoint_pairs();

            calculate_all_keypoint_pairs_rejection();
        }

        calculate_all_keypoint_pairs_registration();

        apply_transformation();

        output_transformed_data = frames;

        return transformations;
    }

    Matrix4fVector getTransformation() const
    {
        return transformations;
    }

    KeypointsFrames getKeypoints() const
    {
        return keypoints;
    }

    KeypointsFrames getTransformedKeypoints() const
    {
        return transformed_keypoints;
    }

    std::vector<float> getFitnessScores() const
    {
        return fitness_scores;
    }

protected:
    Matrix4fVector transformations;
    Eigen::Matrix4f initial_transformation;

    Frames frames;
    KeypointsFrames keypoints;
    KeypointsFrames transformed_keypoints;
    std::vector<float> fitness_scores;

    KeypointsFrame calculate_one_keypoint_pair(const Frame& frame1, const Frame& frame2)
    {
        KeypointsFrame result;

        if (settings->value("PIPELINE_SETTINGS/ARUCO_KEYPOINTS").toBool()) {
            KeypointsDetector<ArUcoKeypointDetector> aruco(this, settings);
            aruco.setInput(frame1, frame2);
            result += aruco.detect();
        }
        if (settings->value("PIPELINE_SETTINGS/SURF_KEYPOINTS").toBool()) {
            KeypointsDetector<SurfKeypointDetector> surf(this, settings);
            surf.setInput(frame1, frame2);
            result += surf.detect();
        }

        return result;
    }

    virtual void calculate_all_keypoint_pairs() = 0;

    void calculate_all_keypoint_pairs_rejection()
    {
        KeypointsRejection rejection(this, settings);
        keypoints = rejection.rejection(keypoints);
    }

    virtual Eigen::Matrix4f calculate_one_keypoint_pair_registration(
        const KeypointsFrame& keypoint_frame, const Eigen::Matrix4f& initial_transformation)
        = 0;

    virtual void calculate_all_keypoint_pairs_registration() = 0;

    void apply_transformation()
    {
        if (frames.size() != transformations.size()) {
            throw std::runtime_error(
                "Registration::aplly_transformation frames.size() != transformations.size()");
        }

        for (unsigned int i = 0; i < frames.size(); ++i) {
            frames[i] = frames[i].transform(transformations[i]);
        }
    }
};

#endif //REGISTRATION_H
