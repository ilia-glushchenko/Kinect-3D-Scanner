#include "core/keypoints/keypointsrejection.h"

KeypointsRejection::KeypointsRejection(QObject *parent, QSettings* parent_settings)
	: ScannerBase(parent, parent_settings)
{
	inlier_threshold	= settings->value("SAC_SETTINGS/INLIER_THRESHOLD").toDouble();
	max_iter			= settings->value("SAC_SETTINGS/MAX_ITERATIONS").toInt();
	min_idsac_threshold = settings->value("SAC_SETTINGS/IDSAC_MINIMUM").toInt();
}

KeypointsFrames KeypointsRejection::rejection(
	KeypointsFrames & in_keypointsFrames
	)
{
	KeypointsFrames out_keypointsFrames;
	perform_rejection(in_keypointsFrames, out_keypointsFrames);
	return out_keypointsFrames;
}

KeypointsFrame KeypointsRejection::rejection(
		KeypointsFrame & in_keypointsFrame
	)
{
	KeypointsFrame out_keypointsFrame;
	perform_rejection(in_keypointsFrame, out_keypointsFrame);
	return out_keypointsFrame;
}

//----------------------------------------------------------------

/** \brief Performs SaC and IDSaC rejections for multiple clouds, puts filtered clouds into output. */
void KeypointsRejection::perform_rejection(
	KeypointsFrames& in_keypointsFrames,
	KeypointsFrames& out_keypointsFrames
	)
{
	for (size_t i = 0; i < in_keypointsFrames.size(); i++)
	{
		KeypointsFrame keypointsFrame;
		perform_rejection(in_keypointsFrames[i], keypointsFrame);
		out_keypointsFrames.push_back(keypointsFrame);
	}
}

/** \brief Performs SaC and IDSaC rejections for one cloud, puts filtered cloud into output. */
void KeypointsRejection::perform_rejection(
		KeypointsFrame & in_keypointsFrame,
		KeypointsFrame & out_keypointsFrame
	)
{
	KeypointsFrame buffer_keypointsFrame;
	copyKeypointsFrame(in_keypointsFrame, buffer_keypointsFrame);

	PcdPtr & input_point_cloud_ptr  = buffer_keypointsFrame.keypointsPcdPair.second;
	PcdPtr & target_point_cloud_ptr = buffer_keypointsFrame.keypointsPcdPair.first;
	pcl::Correspondences & correspondences = buffer_keypointsFrame.keypointsPcdCorrespondences;

	
	//###########################################################
	//Calculate SaC rejection and get Inliers
	//###########################################################
	pcl::Correspondences inliers;
	calculate_sac(
		input_point_cloud_ptr, target_point_cloud_ptr, correspondences,
		inlier_threshold, max_iter,
		inliers, Eigen::Matrix4f()
	);

	
	//###########################################################
	//Insert camera points and their correspondeces
	//###########################################################
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	if (settings->value("SAC_SETTINGS/ADD_CAMERAS").toBool())
	{
		add_camera_pose_points(
			input_point_cloud_ptr, target_point_cloud_ptr, inliers,
			transformation_matrix, transformation_matrix
		);
	}


	//###########################################################
	//Updating keypoint clouds and inliers
	//###########################################################
	PcdPtr inliers_input_point_cloud_ptr(new Pcd);
	PcdPtr inliers_target_point_cloud_ptr(new Pcd);
	pcl::Correspondences inliers_correspondences;

	update_clouds(
		input_point_cloud_ptr,         target_point_cloud_ptr,	       inliers,
		inliers_input_point_cloud_ptr, inliers_target_point_cloud_ptr, inliers_correspondences
	);

	pcl::copyPointCloud(*inliers_input_point_cloud_ptr, *input_point_cloud_ptr);
	pcl::copyPointCloud(*inliers_target_point_cloud_ptr, *target_point_cloud_ptr);
	inliers = inliers_correspondences;

	qDebug() << QString("  SuC   rejection: %1 / %2").arg(correspondences.size()).arg(inliers.size()).toStdString().c_str();
	correspondences = inliers;


	//###########################################################
	//Copy untransformed frame
	//###########################################################
	KeypointsFrame result_KeypointsFrame;
	copyKeypointsFrame(buffer_keypointsFrame, result_KeypointsFrame);


	//###########################################################
	//Calculate and apply SaC transform according to camera position
	//###########################################################	
	Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
	calculate_sac(
		input_point_cloud_ptr, target_point_cloud_ptr, correspondences,
		DISABLED_INLIER_THRESHOLD, max_iter,
		correspondences, best_transformation_matrix
	);
	pcl::transformPointCloud(
		*input_point_cloud_ptr.get(),
		*input_point_cloud_ptr.get(),
		best_transformation_matrix
	);


	//###########################################################
	//Iterative decrementive Sample Consensus rejection
	//###########################################################
	std::vector<int> exclude_indexes;
	if (settings->value("SAC_SETTINGS/IDSAC_ENABLE").toBool())
	{
		iterative_decrementive_sample_consensus_rejection(buffer_keypointsFrame, exclude_indexes);
	}
		
	
	//###########################################################
	//Updating Untransformed Frame and copy it into Out Frame
	//###########################################################
	PcdPtr result_input_point_cloud_ptr  = result_KeypointsFrame.keypointsPcdPair.second;
	PcdPtr result_target_point_cloud_ptr = result_KeypointsFrame.keypointsPcdPair.first;
	pcl::Correspondences& result_inliers = result_KeypointsFrame.keypointsPcdCorrespondences;

	for (int i = 0; i < exclude_indexes.size(); i++)
	{
		PcdPtr _result_input_point_cloud_ptr(new Pcd);
		PcdPtr _result_target_point_cloud_ptr(new Pcd);
		pcl::Correspondences _result_inliers;

		for (int j = 0; j < result_inliers.size(); j++) {
			if (j != exclude_indexes[i]) {
				_result_inliers.push_back(result_inliers[j]);
			}
		}
			
		pcl::Correspondences buffer_correspondeces;
		update_clouds(
			 result_input_point_cloud_ptr,  result_target_point_cloud_ptr, _result_inliers,
			_result_input_point_cloud_ptr, _result_target_point_cloud_ptr, buffer_correspondeces
		);

		pcl::copyPointCloud(*_result_input_point_cloud_ptr, *result_input_point_cloud_ptr);
		pcl::copyPointCloud(*_result_target_point_cloud_ptr, *result_target_point_cloud_ptr);
		result_inliers = buffer_correspondeces;
	}

	copyKeypointsFrame(result_KeypointsFrame, out_keypointsFrame);
}

/** \brief Calculates IDSaC rejection and reterns exclude indexes vector. */
void KeypointsRejection::iterative_decrementive_sample_consensus_rejection(
	KeypointsFrame & keypointsFrame,
	std::vector<int> & exclude_indexes
	)
{
	PcdPtr result_input_point_cloud_ptr(new Pcd(*keypointsFrame.keypointsPcdPair.second));
	PcdPtr result_target_point_cloud_ptr(new Pcd(*keypointsFrame.keypointsPcdPair.first));
	pcl::Correspondences result_inliers(keypointsFrame.keypointsPcdCorrespondences);
	

	while (result_input_point_cloud_ptr.get()->points.size() > min_idsac_threshold)
	{
		//Calculating camera distances
		const PointType & a = result_input_point_cloud_ptr->back();
		const PointType & b = result_target_point_cloud_ptr->back();
		float current_distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));

		int exclude_index = -1;

		//Iterative excluding of all points except the last one which is a camera point
		for (size_t i = 0; i < result_inliers.size() - 1; i++)
		{
			PcdPtr _result_input_point_cloud_ptr(new Pcd);
			PcdPtr _result_target_point_cloud_ptr(new Pcd);
			pcl::Correspondences _result_inliers;

			//Filling new clouds excluding i point from clouds
			for (size_t j = 0; j < result_inliers.size(); j++) 
			{
				if (j != i) 
				{
					_result_inliers.push_back(result_inliers[j]);
				}
			}

			pcl::Correspondences buffer_correspondeces;
			update_clouds(
				result_input_point_cloud_ptr, result_target_point_cloud_ptr, _result_inliers,
				_result_input_point_cloud_ptr, _result_target_point_cloud_ptr, buffer_correspondeces
			);
			_result_inliers = buffer_correspondeces;

			//Trying sac
			Eigen::Matrix4f best_transformation_matrix(Eigen::Matrix4f::Identity());
			calculate_sac(
				_result_input_point_cloud_ptr, _result_target_point_cloud_ptr, _result_inliers,
				DISABLED_INLIER_THRESHOLD, max_iter,
				_result_inliers, best_transformation_matrix
			);

			pcl::transformPointCloud(
				*_result_input_point_cloud_ptr, *_result_input_point_cloud_ptr, best_transformation_matrix
			);

			//Comparing camera distances knowing - camera is the last point of the cloud
			const PointType & a = _result_input_point_cloud_ptr->back();
			const PointType & b = _result_target_point_cloud_ptr->back();
			const float new_distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));

			if (new_distance < current_distance) 
			{
				current_distance = new_distance;
				exclude_index = i;
			}
		}

		//If there is a better solution
		if (exclude_index >= 0)
		{
			PcdPtr _result_input_point_cloud_ptr(new Pcd);
			PcdPtr _result_target_point_cloud_ptr(new Pcd);
			pcl::Correspondences _result_inliers;

			for (size_t i = 0; i < result_inliers.size(); i++) {
				if (i != exclude_index) {
					_result_inliers.push_back(result_inliers[i]);
				}
			}

			pcl::Correspondences buffer_correspondeces;
			update_clouds(
				result_input_point_cloud_ptr, result_target_point_cloud_ptr, _result_inliers,
				_result_input_point_cloud_ptr, _result_target_point_cloud_ptr, buffer_correspondeces
				);

			pcl::copyPointCloud(*_result_input_point_cloud_ptr.get(), *result_input_point_cloud_ptr.get());
			pcl::copyPointCloud(*_result_target_point_cloud_ptr.get(), *result_target_point_cloud_ptr.get());
			result_inliers = buffer_correspondeces;

			exclude_indexes.push_back(exclude_index);
		}
		else
		{
			break;
		}
	}

	pcl::copyPointCloud(*result_input_point_cloud_ptr.get(), *keypointsFrame.keypointsPcdPair.second.get());
	pcl::copyPointCloud(*result_target_point_cloud_ptr.get(), *keypointsFrame.keypointsPcdPair.first.get());
	qDebug() << "  IDSaC rejection:" << keypointsFrame.keypointsPcdCorrespondences.size() << "/" << result_inliers.size();
	keypointsFrame.keypointsPcdCorrespondences = result_inliers;
}

/** \brief Calculates SaC best transformation and rejection than copyes into out_transformation_matrix and out_inliers_correspondences. */
void KeypointsRejection::calculate_sac(
		const PcdPtr & input_point_cloud_ptr, 
		const PcdPtr & target_point_cloud_ptr,
		const pcl::Correspondences & correspondences, 
		const double & inlier_threshold, const int & max_iter,
		pcl::Correspondences & out_inliers, Eigen::Matrix4f & out_transformation_matrix
	)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> sac;
	sac.setInputSource(input_point_cloud_ptr);
	sac.setInputTarget(target_point_cloud_ptr);
	sac.setInlierThreshold(inlier_threshold);
	sac.setMaximumIterations(max_iter);
	sac.setInputCorrespondences(pcl::CorrespondencesPtr(new pcl::Correspondences(correspondences)));

	pcl::Correspondences new_inliers;
	sac.getCorrespondences(new_inliers);
	
	out_inliers = new_inliers;
	out_transformation_matrix = sac.getBestTransformation();
}

/** \brief Updates in_point_clouds accroding to in_corrspondeces and copyes it into out clouds and correspondeces. */
void KeypointsRejection::update_clouds(
		const PcdPtr & in_input_point_cloud_ptr1,
		const PcdPtr & in_traget_point_cloud_ptr2,
		const pcl::Correspondences & in_correspondences,
		PcdPtr & out_input_point_cloud_ptr1,
		PcdPtr & out_traget_point_cloud_ptr2,
		pcl::Correspondences & out_correspondences
	)
{
	for (size_t i = 0; i < in_correspondences.size(); i++)
	{
		const PointType a = (*in_input_point_cloud_ptr1)[in_correspondences[i].index_query];
		const PointType b = (*in_traget_point_cloud_ptr2)[in_correspondences[i].index_match];

		out_input_point_cloud_ptr1->push_back(a);
		out_traget_point_cloud_ptr2->push_back(b);

		pcl::Correspondence correspondence;
		correspondence.index_query = out_traget_point_cloud_ptr2->size() - 1;
		correspondence.index_match = out_input_point_cloud_ptr1->size() - 1;
		correspondence.distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
		out_correspondences.push_back(correspondence);
	}

	out_input_point_cloud_ptr1->is_dense = false;
	out_traget_point_cloud_ptr2->is_dense = false;
}

/** \brief Insertrs camera points according to translation matrixes. */
void KeypointsRejection::add_camera_pose_points(
		PcdPtr & input_point_cloud_ptr,
		PcdPtr & target_point_cloud_ptr,
		pcl::Correspondences & correspondences,
		const Eigen::Matrix4f & input_transformation_matrix,
		const Eigen::Matrix4f & target_transformation_matrix
	)
{
	PointType a;
	a.x = input_transformation_matrix(0, 3);
	a.y = input_transformation_matrix(1, 3);
	a.z = input_transformation_matrix(2, 3);
	input_point_cloud_ptr->push_back(a);

	PointType b;
	b.x = target_transformation_matrix(0, 3);
	b.y = target_transformation_matrix(1, 3);
	b.z = target_transformation_matrix(2, 3);
	target_point_cloud_ptr->push_back(b);

	pcl::Correspondence correspondence;
	correspondence.index_query = input_point_cloud_ptr->size() - 1;
	correspondence.index_match = target_point_cloud_ptr->size() - 1;
	correspondence.distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
	correspondences.push_back(correspondence);
}

/** \brief Copy keypoint frames. */
void KeypointsRejection::copyKeypointsFrame(const KeypointsFrame & in_frame, KeypointsFrame & out_frame)
{
	out_frame.keypointsPcdCorrespondences = in_frame.keypointsPcdCorrespondences;
	
	out_frame.keypointsPcdPair = std::make_pair(
		PcdPtr(new Pcd(*in_frame.keypointsPcdPair.first)), PcdPtr(new Pcd(*in_frame.keypointsPcdPair.second))
	);

	out_frame.keypointsNormalPcdPair = std::make_pair(
		NormalPcdPtr(new NormalPcd(*in_frame.keypointsNormalPcdPair.first)), 
		NormalPcdPtr(new NormalPcd(*in_frame.keypointsNormalPcdPair.second))
	);
}
