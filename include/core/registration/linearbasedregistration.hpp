#ifndef LINEAR_BASED_REGISTRATION_HPP
#define LINEAR_BASED_REGISTRATION_HPP

#include <boost/iterator/counting_iterator.hpp>

#include "core/registration/registrationalgorithm.hpp"
#include "core/registration/sacregistration.h"
#include "core/registration/icpregistration.h"
#include "core/registration/linearregistration.hpp"
#include "io/pcdinputiterator.hpp"


class LinearBasedRegistration : public RegistrationAlgorithm
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct Loop : RegistrationAlgorithm::Loop
	{
		std::pair<Frame, Frame> edge_frames;
		Eigen::Matrix4f first_edge_transformation;

		Loop(const uint & start_index, const uint & loop_size)
		{
			first_edge_transformation = Eigen::Matrix4f::Identity();
			inner_indexes = std::vector<uint>(
				boost::counting_iterator<uint>(start_index),
				boost::counting_iterator<uint>(start_index + loop_size + 1)
			);
			inner_transformations.resize(inner_indexes.size(), Eigen::Matrix4f::Identity());
		}
	};
	typedef std::vector< Loop, Eigen::aligned_allocator< Loop > > Loops;

	LinearBasedRegistration(QObject *parent, QSettings* parent_settings) :
		RegistrationAlgorithm(parent, parent_settings),
		loop_size(settings->value("FINAL_SETTINGS/ITERATIVE_RECONSTRUCTION_STEP").toInt())
	{
	}

private:
	const int loop_size;
	Loops loops;

	void prepare_all_loops()
	{
		for (uint i = read_from + loop_size; i <= read_to; i += loop_size)
		{
			loops.push_back(Loop(i - loop_size, loop_size));
		}

		Frames frames;
		PcdInputIterator it(settings, read_from, read_to, loop_size);
		for (it; it != PcdInputIterator(); ++it)
		{
			frames.push_back(*it);
		}

		if (frames.size() != loops.size() + 1)
		{
			throw std::runtime_error("LinearBasedRegistration::prepare_all_loops frames.size() != loops.size() + 1");
		}

		for (uint i = 1; i < frames.size(); ++i)
		{
			loops[i - 1].edge_frames = std::make_pair(frames[i - 1], frames[i]);
		}
	}

	void process_all_loops()
	{
		for (uint i = 0; i < loops.size(); ++i)
		{
			if (i > 0)
			{
				loops[i].first_edge_transformation = loops[i - 1].inner_transformations.back();
			}
			loops[i] = process_one_loop(loops[i]);
		}

		loops_data_vizualization(loops);
	}

	void perform_tsdf_meshing()
	{
		Matrix4fVector result_t;
		for (uint i = 0; i < loops.size(); ++i)
		{
			std::copy(loops[i].inner_transformations.begin(), loops[i].inner_transformations.end(),
				std::back_inserter(result_t));
		}

		volumeReconstruction->prepareVolume();
		pcdVizualizer->redraw();

		if (settings->value("FINAL_SETTINGS/DRAW_ALL_CAMERA_POSES").toBool()) 
		{
			pcdVizualizer->visualizeCameraPoses(result_t);
		}

		if (settings->value("CPU_TSDF_SETTINGS/DRAW_MESH").toBool())
		{
			pcl::PolygonMesh mesh;
			volumeReconstruction->calculateMesh();
			volumeReconstruction->getPoligonMesh(mesh);
			pcdVizualizer->visualizeMesh(mesh);
		}
	}

	Loop process_one_loop(const Loop & loop)
	{
		Loop result_loop(loop);

		Frames inner_frames(1, result_loop.edge_frames.first);
		Frames transformed_inner_frames;
		PcdInputIterator it(settings, result_loop.inner_indexes.front(), result_loop.inner_indexes.back(), read_step);
		for (it; it != PcdInputIterator(); ++it)
		{
			inner_frames.push_back(*it);
		}
		inner_frames.push_back(result_loop.edge_frames.second);

		PcdFilters filters(this, settings);
		filters.setInput(inner_frames);
		filters.filter(inner_frames);

		LinearRegistration<SaCRegistration> linear_sac(this, settings);
		linear_sac.setInput(inner_frames, result_loop.first_edge_transformation);
		const Matrix4fVector sac_t = linear_sac.align(transformed_inner_frames);

		LinearRegistration<ICPRegistration> linear_icp(this, settings);
		linear_icp.setInput(transformed_inner_frames, Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
		linear_icp.setKeypoints(linear_sac.getTransformedKeypoints());
		const Matrix4fVector icp_t = linear_icp.align(transformed_inner_frames);

		Matrix4fVector result_t;
		for (unsigned int i = 0; i < icp_t.size(); ++i)
		{
			result_t.push_back(icp_t[i] * sac_t[i]);
		}
		vizualization(inner_frames, transformed_inner_frames, linear_icp.getTransformedKeypoints(), result_t);

		result_loop.inner_transformations = result_t;
		result_loop.inner_t_fitness_scores = linear_icp.getFitnessScores();

		return result_loop;
	}

};

#endif //LINEAR_BASED_REGISTRATION_HPP