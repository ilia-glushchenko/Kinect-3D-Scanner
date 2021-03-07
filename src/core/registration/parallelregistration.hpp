#ifndef PARALLEL_REGISTRATION_H
#define PARALLEL_REGISTRATION_H

#include "core/registration/registration.hpp"

template <typename RegistrationMethod>
class ParallelRegistration : public Registration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ParallelRegistration(QObject* parent, QSettings* parent_settings)
        : Registration(parent, parent_settings)
    {
    }

    void setInput(
        const Frames& input_frames,
        const unsigned int& input_middle_frame_index,
        const Eigen::Matrix4f& input_initital_transformation)
    {
        if (input_middle_frame_index >= input_frames.size()) {
            throw std::invalid_argument(
                "ParallelRegistration::setInput input_middle_frame_index  >= input_frames.size()");
        }
        if (input_initital_transformation.hasNaN()) {
            throw std::invalid_argument(
                "ParallelRegistration::setInput input_initital_transformation.hasNaN()");
        }

        frames = input_frames;
        middle_frame_index = input_middle_frame_index;
        initial_transformation = input_initital_transformation;
        fitness_scores.clear();
    }

protected:
    unsigned int middle_frame_index;

    void calculate_all_keypoint_pairs()
    {
        for (unsigned int i = 0; i < frames.size(); ++i) {
            if (i != middle_frame_index) {
                keypoints.push_back(calculate_one_keypoint_pair(frames[middle_frame_index], frames[i]));
            }
        }
    }

    Eigen::Matrix4f calculate_one_keypoint_pair_registration(
        const KeypointsFrame& keypoint_frame, const Eigen::Matrix4f& initial_transformation)
    {
        RegistrationMethod registrator(this, settings);
        registrator.setInput(keypoint_frame, initial_transformation);
        const Eigen::Matrix4f result_t = registrator.align();
        fitness_scores.push_back(registrator.getFitnessScore());
        return result_t;
    }

    void calculate_all_keypoint_pairs_registration()
    {
        transformed_keypoints.clear();
        transformations.clear();
        transformations.resize(frames.size(), initial_transformation);

        for (unsigned int i = 0; i < frames.size(); ++i) {
            const unsigned int index = i < middle_frame_index ? i : i - 1;

            if (i != middle_frame_index) {
                transformed_keypoints.push_back(keypoints[index].transformFirst(initial_transformation));
                transformations[i] = calculate_one_keypoint_pair_registration(keypoints[index], initial_transformation);
                transformed_keypoints.back() = transformed_keypoints.back().transformSecond(transformations[i]);
            }
        }
    }
};

#endif // PARALLEL_REGISTRATION_H