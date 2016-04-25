#ifndef MIDDLE_BASED_REGISTRATION_HPP
#define MIDDLE_BASED_REGISTRATION_HPP

#include <boost/iterator/counting_iterator.hpp>

#include "core/registration/registrationalgorithm.hpp"
#include "core/registration/edgebalancer.hpp"
#include "core/registration/errormetric.hpp"
#include "core/registration/icpregistration.h"
#include "core/registration/sacregistration.h"
#include "core/registration/parallelregistration.hpp"
#include "io/pcdinputiterator.hpp"


class MiddleBasedRegistration : public RegistrationAlgorithm
{
public:
	typedef PcdInputIterator Iter;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct Loop : RegistrationAlgorithm::Loop
	{
		uint middle_index;
		Frame middle_frame;
		Eigen::Matrix4f middle_transformation;

		Loop(const uint & from_, const uint & to_, const uint & middle_): 
			middle_index(middle_),
			middle_transformation(Eigen::Matrix4f::Identity())
		{
			if (to_ - from_ < 2)
			{
				throw std::invalid_argument("Loop packet_size < 2");
			}

			inner_indexes = std::vector<uint>(
				boost::counting_iterator<uint>(from_), boost::counting_iterator<uint>(to_)
			);

			inner_transformations.resize(inner_indexes.size(), Eigen::Matrix4f::Identity());
		}
	
	};
	typedef std::vector<Loop, Eigen::aligned_allocator<Loop>> Loops;


	MiddleBasedRegistration(QObject *parent, QSettings* parent_settings) :
		RegistrationAlgorithm(parent, parent_settings),
		loop_size(settings->value("FINAL_SETTINGS/MIDDLE_BASED_RECONSTRUCTION_STEP").toInt())
	{
	}

private:
	const int loop_size;
	Loops loops;
	
	void prepare_all_loops()
	{
		const uint edges_from = read_from + loop_size / 2;
		const uint edges_to = read_to - loop_size / 2;

		Frames middle_frames;
		Frames transformed_middle_frames;

		if (!settings->value("FINAL_SETTINGS/MIDDLE_BASED_RECONSTRUCTION_EDGE_BALANCING").toBool())
		{
			for (uint i = read_from + loop_size; i <= read_to; i += loop_size)
			{
				loops.push_back(Loop(i - loop_size, i, i - loop_size / 2));
			}

			PcdInputIterator it(settings, loops.front().middle_index, loops.back().middle_index, loop_size);
			for (it; it != PcdInputIterator(); ++it)
			{
				middle_frames.push_back(*it);
			}
		}
		else
		{
			EdgeBalancer<CameraDistanceMetric, PcdInputIterator> eb(
				Iter(settings, edges_from, edges_to, read_step), Iter(), loop_size, settings);
			std::vector<uint> edge_indices = eb.balance();

			Iter it(settings, edges_from, edges_to, read_step);
			for (uint edge_index = 0; it != Iter(); ++it, ++edge_index)
			{
				if (std::find(edge_indices.begin(), edge_indices.end(), edge_index) != edge_indices.end())
				{
					middle_frames.push_back(*it);
				}
			}

			std::for_each(edge_indices.begin(), edge_indices.end(), [&](uint & i) { i += edges_from; });

			for (uint i = 0; i < edge_indices.size(); ++i)
			{
				const uint lower_loop_border = (i == 0) ? read_from
					: edge_indices[i - 1] + (edge_indices[i] - edge_indices[i - 1]) / 2;
				const uint upper_loop_border = (i + 1 == edge_indices.size()) ? read_to
					: (edge_indices[i] + (edge_indices[i + 1] - edge_indices[i]) / 2);
				
				//Note: incorrect in case of read_step > 1
				const uint middle_index = std::abs(int(lower_loop_border) - int(edge_indices[i]));

				loops.push_back(Loop(lower_loop_border, upper_loop_border, middle_index));
			}
		}

		if (loops.size() != middle_frames.size())
		{
			throw std::runtime_error("MiddleBasedRegistration::prepare_all_loops loops.size() != middle_frames.size()");
		}

		PcdFilters filters(this, settings);
		filters.setInput(middle_frames);
		filters.filter(middle_frames);
		
		LinearRegistration<SaCRegistration> linear_sac(this, settings);
		linear_sac.setInput(middle_frames, Eigen::Matrix4f::Identity());
		Matrix4fVector sac_t = linear_sac.align(transformed_middle_frames);

		LinearRegistration<ICPRegistration> linear_icp(this, settings);
		linear_icp.setInput(transformed_middle_frames, Eigen::Matrix4f::Identity());
		linear_icp.setKeypoints(linear_sac.getTransformedKeypoints());
		Matrix4fVector icp_t = linear_icp.align(transformed_middle_frames);

		for (uint i = 0; i < icp_t.size(); ++i)
		{
			loops[i].middle_frame = middle_frames[i];
			loops[i].middle_transformation = icp_t[i] * sac_t[i];
		}
	}

	void process_all_loops()
	{
		for (uint i = 0; i < loops.size(); ++i)
		{
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

		Frames transformed_inner_frames;
		Frames inner_frames;
		PcdInputIterator it(settings, result_loop.inner_indexes.front(), result_loop.inner_indexes.back(), read_step);
		for (it; it != PcdInputIterator(); ++it)
		{
			inner_frames.push_back(*it);
		}

		PcdFilters filters(this, settings);
		filters.setInput(inner_frames);
		filters.filter(inner_frames);

		ParallelRegistration<SaCRegistration> parallel_sac(this, settings);
		parallel_sac.setInput(inner_frames, loop.middle_index, result_loop.middle_transformation);
		const Matrix4fVector sac_t = parallel_sac.align(transformed_inner_frames);

		ParallelRegistration<ICPRegistration> parallel_icp(this, settings);
		parallel_icp.setInput(transformed_inner_frames, loop.middle_index, Eigen::Matrix4f::Identity());
		parallel_icp.setKeypoints(parallel_sac.getTransformedKeypoints());
		const Matrix4fVector icp_t = parallel_icp.align(transformed_inner_frames);

		Matrix4fVector result_t;
		for (uint i = 0; i < icp_t.size(); ++i)
		{
			result_t.push_back(icp_t[i] * sac_t[i]);
		}

		inner_frames.push_back(result_loop.middle_frame);
		transformed_inner_frames.push_back(result_loop.middle_frame.transform(result_loop.middle_transformation));
		vizualization(inner_frames, transformed_inner_frames, parallel_icp.getTransformedKeypoints(), result_t);
	
		result_loop.inner_transformations = result_t;
		result_loop.inner_t_fitness_scores = parallel_icp.getFitnessScores();

		return result_loop;
	}

};

#endif //MIDDLE_BASED_REGISTRATION_HPP