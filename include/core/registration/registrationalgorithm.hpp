#ifndef REGISTRATION_ALGORITHM_HPP
#define REGISTRATION_ALGORITHM_HPP

#include "core/base/types.h"
#include "core/base/scannerbase.h"
#include "core/reconstruction/volumereconstruction.h"
#include "gui/pcdvizualizer.h"
#include "utility/pcdfilters.h"


class RegistrationAlgorithm : public ScannerBase
{
public:
	struct Loop
	{
		std::vector<uint> inner_indexes;
		Matrix4fVector inner_transformations;
		std::vector<float> inner_t_fitness_scores;
	};


	RegistrationAlgorithm(QObject * parent, QSettings * parent_settings) :
		ScannerBase(parent, parent_settings),
		read_from(settings->value("READING_SETTING/FROM").toInt()),
		read_to(settings->value("READING_SETTING/TO").toInt()),
		read_step(settings->value("READING_SETTING/STEP").toInt())
	{
		if (read_from >= read_to)
		{
			throw std::invalid_argument("RegistrationAlgorithm read_from >= read_to");
		}
		if (read_to - read_from < read_step)
		{
			throw std::invalid_argument("RegistrationAlgorithm read_to - read_from < read_step");
		}
		if (read_step == 0)
		{
			throw std::invalid_argument("RegistrationAlgorithm read_step == 0");
		}
	}

	void setVolumeReconstructor(const VolumeReconstruction::Ptr & inputVolumeReconstruction)
	{
		if (!inputVolumeReconstruction)
		{
			throw std::invalid_argument("MiddleBasedRegistration::setVolumeReconstructor !volumeReconstruction");
		}

		volumeReconstruction = inputVolumeReconstruction;
	}

	void setVisualizer(const PcdVizualizer::Ptr & inputPcdVizualizer)
	{
		if (!inputPcdVizualizer)
		{
			throw std::invalid_argument("MiddleBasedRegistration::setVisualizer !inputPcdVizualizer");
		}

		pcdVizualizer = inputPcdVizualizer;
	}

	void reconstruct()
	{
		prepare_all_loops();
		process_all_loops();

		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			perform_tsdf_meshing();
		}
	}

protected:
	const int read_from;
	const int read_to;
	const int read_step;


	VolumeReconstruction::Ptr volumeReconstruction;
	PcdVizualizer::Ptr pcdVizualizer;

	virtual void prepare_all_loops() = 0;

	virtual void process_all_loops() = 0;

	virtual void perform_tsdf_meshing() = 0;

	template < typename T, typename A >
	void loops_data_vizualization(const std::vector< T, A > & loops)
	{
		Matrix4fVector inner_transformations;
		std::vector<double> inner_t_fitness_scores;

		for (uint i = 0; i < loops.size(); ++i)
		{
			std::copy(loops[i].inner_transformations.begin(), loops[i].inner_transformations.end(),
				std::back_inserter(inner_transformations));
			std::copy(loops[i].inner_t_fitness_scores.begin(), loops[i].inner_t_fitness_scores.end(),
				std::back_inserter(inner_t_fitness_scores));
		}

		pcdVizualizer->visualizeCameraPoses(inner_transformations);
		pcdVizualizer->plotCameraDistances(inner_t_fitness_scores, false, "Fitness scores", "Score");
	}

	
	void vizualization(
			Frames & src_frames,
			const Frames & transformed_frames,
			const KeypointsFrames & transformed_keypoints,
			const Matrix4fVector & transformations
		)
	{
		if (settings->value("CPU_TSDF_SETTINGS/ENABLE_IN_FINAL").toBool())
		{
			PcdFilters::reorganize_all_frames(src_frames);
			PcdPtrVector point_cloud_vector;
			std::transform(src_frames.begin(), src_frames.end(), std::back_inserter(point_cloud_vector),
				[](const Frame & frame){ return frame.pointCloudPtr; });

			volumeReconstruction->addPointCloudVector(point_cloud_vector, transformations);
		}
		else
		{
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_CLOUDS").toBool())
			{
				pcdVizualizer->visualizePointClouds(transformed_frames);
			}
			if (settings->value("FINAL_SETTINGS/DRAW_ALL_KEYPOINT_CLOUDS").toBool())
			{
				pcdVizualizer->visualizeKeypointClouds(transformed_keypoints);
			}
		}
	}
};


#endif //REGISTRATION_ALGORITHM_HPP