#ifndef LINEAR_REGISTRATION_H
#define LINEAR_REGISTRATION_H

#include "core/registration/registration.hpp"

template < typename RegistrationMethod >
class LinearRegistration : public Registration
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LinearRegistration(QObject * parent, QSettings * parent_settings) :
		Registration(parent, parent_settings)
	{
	}

	void setInput(const Frames & input_frames, const Eigen::Matrix4f & input_initital_transformation)
	{
		if (input_initital_transformation.hasNaN())
		{
			throw std::invalid_argument(
				"LinearRegistration::setInput input_initital_transformation.hasNaN()"
			);
		}

		frames = input_frames;
		initial_transformation = input_initital_transformation;
		fitness_scores.clear();
	}

	
protected:
	
	void calculate_all_keypoint_pairs()
	{
		for (unsigned int i = 1; i < frames.size(); ++i)
		{
			keypoints.push_back(calculate_one_keypoint_pair(frames[i - 1], frames[i]));
		}
	}

	Eigen::Matrix4f calculate_one_keypoint_pair_registration(
			const KeypointsFrame & keypoint_frame, const Eigen::Matrix4f & initial_transformation
		)
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
		transformations.push_back(initial_transformation);

		for (unsigned int i = 0; i < keypoints.size(); ++i)
		{
			transformed_keypoints.push_back(keypoints[i].transformFirst(transformations.back()));
			
			transformations.push_back(
				calculate_one_keypoint_pair_registration(keypoints[i], transformations.back()));

			transformed_keypoints.back() = transformed_keypoints.back().transformSecond(transformations.back());
		}
	}

};

#endif // LINEAR_REGISTRATION_H