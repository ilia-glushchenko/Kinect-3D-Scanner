#ifndef CORRECTION_HPP
#define CORRECTION_HPP

#include "core/base/types.h"
#include "core/base/scannerbase.h"


template < typename CorrectionMethod >
class Correction : public ScannerBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Correction(QObject * parent, QSettings * parent_settings) :
		ScannerBase(parent, parent_settings)
	{
	}

	void setInput(
			const Frames & inner_frames_,
			const KeypointsFrames & inner_keypoints_frames_,
			const Matrix4fVector & inner_frames_transformations_,
			const KeypointsFrame & edge_keypoints_
		)
	{
		inner_frames = inner_frames_;
		inner_keypoints_frames = inner_keypoints_frames_;
		src_inner_keypoints_frames = inner_keypoints_frames_;
		inner_frames_transformations = inner_frames_transformations_;
		edge_keypoints = edge_keypoints_;
	}

	Matrix4fVector correct(Frames & corrected_frames)
	{
		merge_keypoints_frames();
		calculate_correction();
		perform_correction();

		return result_t;
	}

	Frames getFrames() const
	{
		return inner_frames;
	}

	Matrix4fVector getTransformations() const
	{
		return result_t;
	}

	KeypointsFrames getKeypoints() const
	{
		return src_inner_keypoints_frames;
	}

	KeypointsFrames getTransformedKeypoints() const
	{
		return inner_keypoints_frames;
	}

private:
	Frames inner_frames;
	KeypointsFrames inner_keypoints_frames;
	KeypointsFrames src_inner_keypoints_frames;
	Matrix4fVector inner_frames_transformations;
	KeypointsFrame edge_keypoints;

	CorrespondencesVector merged_correspondences;
	PcdPtrVector merged_keypoints;
	Matrix4fVector result_t;


	void merge_keypoints_frames()
	{
		for (size_t j = 0; j <= inner_keypoints_frames.size(); j++)
		{
			PcdPtr new_keypoint_cloud(new Pcd);

			if (j == 0)
			{
				pcl::transformPointCloud(
					*edge_keypoints.keypointsPcdPair.first,
					*edge_keypoints.keypointsPcdPair.first,
					inner_frames_transformations.front()
					);

				for (uint i = 0; i < edge_keypoints.keypointsPcdPair.first->size(); ++i)
				{
					new_keypoint_cloud->push_back((*edge_keypoints.keypointsPcdPair.first)[i]);
				}

				pcl::Correspondences correspondences;
				for (uint i = 0; i < inner_keypoints_frames[0].keypointsPcdPair.first->size(); i++)
				{
					new_keypoint_cloud->push_back((*inner_keypoints_frames[0].keypointsPcdPair.first)[i]);

					pcl::Correspondence correspondence;
					correspondence = inner_keypoints_frames[0].keypointsPcdCorrespondences[i];
					correspondence.index_query = new_keypoint_cloud->size() - 1;
					correspondences.push_back(correspondence);
				}
				merged_correspondences.push_back(correspondences);
			}
			else if (j == inner_keypoints_frames.size())
			{
				for (uint i = 0; i < inner_keypoints_frames[j - 1].keypointsPcdPair.second->size(); ++i)
				{
					new_keypoint_cloud->push_back((*inner_keypoints_frames[j - 1].keypointsPcdPair.second)[i]);
				}

				pcl::transformPointCloud(
					*edge_keypoints.keypointsPcdPair.second,
					*edge_keypoints.keypointsPcdPair.second,
					inner_frames_transformations.back()
					);

				pcl::Correspondences correspondences;
				for (uint i = 0; i < edge_keypoints.keypointsPcdPair.second->size(); ++i)
				{
					new_keypoint_cloud->push_back((*edge_keypoints.keypointsPcdPair.second)[i]);

					const uint index_query = edge_keypoints.keypointsPcdCorrespondences[i].index_query;
					const PointType a(new_keypoint_cloud->back());
					const PointType b((*merged_keypoints[0])[index_query]);

					correspondences.push_back(pcl::Correspondence(
						new_keypoint_cloud->size() - 1, index_query, pcl::euclideanDistance(a, b)));
				}
				merged_correspondences.push_back(correspondences);
			}
			else
			{
				for (size_t i = 0; i < inner_keypoints_frames[j - 1].keypointsPcdPair.second->size(); i++) {
					new_keypoint_cloud->push_back(inner_keypoints_frames[j - 1].keypointsPcdPair.second->at(i));
				}

				pcl::Correspondences correspondences;
				for (size_t i = 0; i < inner_keypoints_frames[j].keypointsPcdPair.first.get()->points.size(); i++)
				{
					new_keypoint_cloud->push_back((*inner_keypoints_frames[j].keypointsPcdPair.first)[i]);

					pcl::Correspondence correspondence;
					correspondence = inner_keypoints_frames[j].keypointsPcdCorrespondences[i];
					correspondence.index_query = new_keypoint_cloud->size() - 1;
					correspondences.push_back(correspondence);
				}
				merged_correspondences.push_back(correspondences);
			}

			merged_keypoints.push_back(new_keypoint_cloud);
		}
	}


	void calculate_correction()
	{
		CorrectionMethod correction(this, settings);
		correction.setInput(merged_keypoints, merged_correspondences);
		result_t = correction.correct();
	}


	void perform_correction()
	{
		for (uint i = 0; i < result_t.size(); ++i)
		{
			inner_frames[i].transform(result_t[i]);

			if (i == 0)
			{
				inner_keypoints_frames[i].transformFirst(result_t[i]);
			}
			if (i + 1 == result_t.size())
			{
				inner_keypoints_frames[i - 1].transformSecond(result_t[i]);
			}
			if (i > 0 && i < inner_keypoints_frames.size())
			{
				inner_keypoints_frames[i - 1].transformSecond(result_t[i]);
				inner_keypoints_frames[i].transformFirst(result_t[i]);
			}
		}

	}

};

#endif //CORRECTION_HPP