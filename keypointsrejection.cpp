#include "keypointsrejection.h"

KeypointsRejection::KeypointsRejection(QObject *parent)
	: QObject(parent)
{
	settings = new QSettings("scaner.ini", QSettings::IniFormat, this);

	inlier_threshold	= settings->value("SAC_SETTINGS/INLIER_THRESHOLD").toDouble();
	max_iter			= settings->value("SAC_SETTINGS/MAX_ITERATIONS").toInt();
	min_idsac_threshold = settings->value("SAC_SETTINGS/IDSAC_MINIMUM").toInt();
}

void KeypointsRejection::rejection(
	KeypointsFrames& in_keypointsFrames,
	KeypointsFrames& out_keypointsFrames
	)
{
	perform_rejection(in_keypointsFrames, out_keypointsFrames);
}

void KeypointsRejection::rejection(
	KeypointsFrame& in_keypointsFrame,
	KeypointsFrame& out_keypointsFrame
	)
{
	perform_rejection(in_keypointsFrame, out_keypointsFrame);
}

//----------------------------------------------------------------

/** \brief Performs SaC and IDSaC rejections for multiple clouds, puts filtered clouds into output. */
void KeypointsRejection::perform_rejection(
	KeypointsFrames& in_keypointsFrames,
	KeypointsFrames& out_keypointsFrames
	)
{
	for (int i = 0; i < in_keypointsFrames.size(); i++)
	{
		KeypointsFrame keypointsFrame;
		perform_rejection(in_keypointsFrames[i], keypointsFrame);
		out_keypointsFrames.push_back(keypointsFrame);
	}
}

/** \brief Performs SaC and IDSaC rejections for one cloud, puts filtered cloud into output. */
void KeypointsRejection::perform_rejection(
	KeypointsFrame& in_keypointsFrame,
	KeypointsFrame& out_keypointsFrame
	)
{
	KeypointsFrame buffer_keypointsFrame;
	copyKeypointsFrame(in_keypointsFrame, buffer_keypointsFrame);

	PcdPtr input_point_cloud_ptr  = buffer_keypointsFrame.keypointsPcdPair.second;
	PcdPtr target_point_cloud_ptr = buffer_keypointsFrame.keypointsPcdPair.first;
	pcl::Correspondences& correspondences = buffer_keypointsFrame.keypointsPcdCorrespondences;

	
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

	pcl::copyPointCloud(*inliers_input_point_cloud_ptr.get(), *input_point_cloud_ptr.get());
	pcl::copyPointCloud(*inliers_target_point_cloud_ptr.get(), *target_point_cloud_ptr.get());
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
		iterative_decrementive_sample_consensus_rejection(buffer_keypointsFrame, exclude_indexes);


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

		for (int j = 0; j < result_inliers.size(); j++)
			if (j != exclude_indexes[i])
				_result_inliers.push_back(result_inliers[j]);

		pcl::Correspondences buffer_correspondeces;
		update_clouds(
			 result_input_point_cloud_ptr,  result_target_point_cloud_ptr, _result_inliers,
			_result_input_point_cloud_ptr, _result_target_point_cloud_ptr, buffer_correspondeces
		);

		pcl::copyPointCloud(*_result_input_point_cloud_ptr.get(), *result_input_point_cloud_ptr.get());
		pcl::copyPointCloud(*_result_target_point_cloud_ptr.get(), *result_target_point_cloud_ptr.get());
		result_inliers = buffer_correspondeces;
	}

	copyKeypointsFrame(result_KeypointsFrame, out_keypointsFrame);
}

/** \brief Calculates IDSaC rejection and reterns exclude indexes vector. */
void KeypointsRejection::iterative_decrementive_sample_consensus_rejection(
	KeypointsFrame& keypointsFrame,
	std::vector<int>& exclude_indexes
	)
{
	PcdPtr result_input_point_cloud_ptr(new Pcd);
	PcdPtr result_target_point_cloud_ptr(new Pcd);
	pcl::Correspondences result_inliers;

	pcl::copyPointCloud(*keypointsFrame.keypointsPcdPair.second.get(), *result_input_point_cloud_ptr.get());
	pcl::copyPointCloud(*keypointsFrame.keypointsPcdPair.first.get(), *result_target_point_cloud_ptr.get());
	result_inliers = keypointsFrame.keypointsPcdCorrespondences;

	while (result_input_point_cloud_ptr.get()->points.size() > min_idsac_threshold)
	{
		//Calculating camera distances
		PointType a, b;
		a = result_input_point_cloud_ptr.get()->points[result_input_point_cloud_ptr.get()->points.size() - 1];
		b = result_target_point_cloud_ptr.get()->points[result_target_point_cloud_ptr.get()->points.size() - 1];
		float current_distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));

		int exclude_index = -1;

		//Iterative excluding of all points except the last one which is a camera point
		for (int i = 0; i < result_inliers.size() - 1; i++)
		{
			PcdPtr _result_input_point_cloud_ptr(new Pcd);
			PcdPtr _result_target_point_cloud_ptr(new Pcd);
			pcl::Correspondences _result_inliers;

			//Filling new clouds excluding i point from clouds
			for (int j = 0; j < result_inliers.size(); j++) {
				if (j != i) {
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
			Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
			calculate_sac(
				_result_input_point_cloud_ptr, _result_target_point_cloud_ptr, _result_inliers,
				DISABLED_INLIER_THRESHOLD, max_iter,
				_result_inliers, best_transformation_matrix
				);

			pcl::transformPointCloud(
				*_result_input_point_cloud_ptr.get(),
				*_result_input_point_cloud_ptr.get(),
				best_transformation_matrix
				);

			//Comparing camera distances knowing - camera is the last point of the cloud
			PointType a = _result_input_point_cloud_ptr.get()->points[_result_input_point_cloud_ptr.get()->points.size() - 1];
			PointType b = _result_target_point_cloud_ptr.get()->points[_result_target_point_cloud_ptr.get()->points.size() - 1];
			float new_distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));

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

			for (int i = 0; i < result_inliers.size(); i++) {
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
	PcdPtr input_point_cloud_ptr, PcdPtr target_point_cloud_ptr, pcl::Correspondences& ñorrespondences,
	double inlier_threshold, int max_iter,
	pcl::Correspondences& inliers, Eigen::Matrix4f& out_transformation_matrix
	)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> sac;
	sac.setInputCloud(input_point_cloud_ptr);
	sac.setTargetCloud(target_point_cloud_ptr);
	sac.setInlierThreshold(inlier_threshold);
	sac.setMaxIterations(max_iter);

	boost::shared_ptr<pcl::Correspondences> correspondences_ptr(new pcl::Correspondences);
	for (int i = 0; i < ñorrespondences.size(); i++)
		correspondences_ptr.get()->push_back(ñorrespondences[i]);
	sac.setInputCorrespondences(correspondences_ptr);

	pcl::Correspondences new_inliers;
	sac.getCorrespondences(new_inliers);
	inliers = new_inliers;

	out_transformation_matrix = sac.getBestTransformation();
}

/** \brief Updates in_point_clouds accroding to in_corrspondeces and copyes it into out clouds and correspondeces. */
void KeypointsRejection::update_clouds(
	PcdPtr in_input_point_cloud_ptr1,
	PcdPtr in_traget_point_cloud_ptr2,
	pcl::Correspondences in_correspondences,
	PcdPtr out_input_point_cloud_ptr1,
	PcdPtr out_traget_point_cloud_ptr2,
	pcl::Correspondences& out_correspondences
	)
{
	for (int i = 0; i < in_correspondences.size(); i++)
	{
		PointType a = in_input_point_cloud_ptr1.get()->points[in_correspondences[i].index_query];
		PointType b = in_traget_point_cloud_ptr2.get()->points[in_correspondences[i].index_match];

		out_input_point_cloud_ptr1.get()->points.push_back(a);
		out_traget_point_cloud_ptr2.get()->points.push_back(b);

		pcl::Correspondence correspondence;
		correspondence.index_query = out_traget_point_cloud_ptr2.get()->points.size() - 1;
		correspondence.index_match = out_input_point_cloud_ptr1.get()->points.size() - 1;
		correspondence.distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));
		out_correspondences.push_back(correspondence);
	}

	out_input_point_cloud_ptr1.get()->width = out_input_point_cloud_ptr1.get()->points.size();
	out_input_point_cloud_ptr1.get()->height = 1;
	out_input_point_cloud_ptr1.get()->resize(out_input_point_cloud_ptr1.get()->width);
	out_input_point_cloud_ptr1.get()->is_dense = false;

	out_traget_point_cloud_ptr2.get()->width = out_traget_point_cloud_ptr2.get()->points.size();
	out_traget_point_cloud_ptr2.get()->height = 1;
	out_traget_point_cloud_ptr2.get()->resize(out_traget_point_cloud_ptr2.get()->width);
	out_traget_point_cloud_ptr2.get()->is_dense = false;
}

/** \brief Insertrs camera points according to translation matrixes. */
void KeypointsRejection::add_camera_pose_points(
	PcdPtr input_point_cloud_ptr,
	PcdPtr target_point_cloud_ptr,
	pcl::Correspondences& correspondences,
	Eigen::Matrix4f& input_transformation_matrix,
	Eigen::Matrix4f& target_transformation_matrix
	)
{
	PointType a;
	a.x = input_transformation_matrix(0, 3);
	a.y = input_transformation_matrix(1, 3);
	a.z = input_transformation_matrix(2, 3);
	a.r = 0; a.g = 0; a.b = 0;
	input_point_cloud_ptr.get()->points.push_back(a);
	input_point_cloud_ptr.get()->width = input_point_cloud_ptr.get()->points.size();
	input_point_cloud_ptr.get()->height = 1;
	input_point_cloud_ptr.get()->resize(input_point_cloud_ptr.get()->width);

	PointType b;
	b.x = target_transformation_matrix(0, 3);
	b.y = target_transformation_matrix(1, 3);
	b.z = target_transformation_matrix(2, 3);
	b.r = 0; b.g = 0; b.b = 0;
	target_point_cloud_ptr.get()->points.push_back(b);
	target_point_cloud_ptr.get()->width = target_point_cloud_ptr.get()->points.size();
	target_point_cloud_ptr.get()->height = 1;
	target_point_cloud_ptr.get()->resize(target_point_cloud_ptr.get()->width);

	pcl::Correspondence correspondence;
	correspondence.index_query = input_point_cloud_ptr.get()->points.size() - 1;
	correspondence.index_match = target_point_cloud_ptr.get()->points.size() - 1;
	correspondence.distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));
	correspondences.push_back(correspondence);
}

/** \brief Copy keypoint frames. */
void KeypointsRejection::copyKeypointsFrame(KeypointsFrame& in_frame, KeypointsFrame& out_frame)
{
	PcdPtr new_first_pcd(new Pcd);
	PcdPtr new_second_pcd(new Pcd);

	out_frame.keypointsPcdCorrespondences = in_frame.keypointsPcdCorrespondences;

	pcl::copyPointCloud(*in_frame.keypointsPcdPair.first.get(), *new_first_pcd.get());
	pcl::copyPointCloud(*in_frame.keypointsPcdPair.second.get(), *new_second_pcd.get());

	out_frame.keypointsPcdPair = std::make_pair(new_first_pcd, new_second_pcd);

	NormalPcdPtr normal_first_pcd(new NormalPcd);
	NormalPcdPtr normal_second_pcd(new NormalPcd);
	if (!in_frame.keypointsNormalPcdPair.first.get()->empty() &&
		!in_frame.keypointsNormalPcdPair.second.get()->empty())
	{
		pcl::copyPointCloud(*in_frame.keypointsNormalPcdPair.first.get(), *normal_first_pcd.get());
		pcl::copyPointCloud(*in_frame.keypointsNormalPcdPair.second.get(), *normal_second_pcd.get());
	}

	out_frame.keypointsNormalPcdPair = std::make_pair(normal_first_pcd, normal_second_pcd);
}