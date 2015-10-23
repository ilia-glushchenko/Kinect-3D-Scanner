#include "core/registration/sacregistration.h"

SaCRegistration::SaCRegistration(
	QObject *parent, QSettings* parent_settings
	) : ScannerBase(parent, parent_settings)
{
	inlier_threshold = settings->value("SAC_SETTINGS/INLIER_THRESHOLD").toDouble();
	max_iter		 = settings->value("SAC_SETTINGS/MAX_ITERATIONS").toInt();
}

void SaCRegistration::calculateSaCTransformation(
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix,
	int middle_index
	)
{
	if (keypointsFrames.empty())
		return;

	prepare();

	if (middle_index != -1)
		calculate_middle_based_all_keypoint_pair_sac(keypointsFrames, intial_transformation_matrix, middle_index);
	else
		calculate_all_keypoint_pair_sac(keypointsFrames, intial_transformation_matrix);
}

void SaCRegistration::applyTransfomation(
	Frames& frames
	)
{
	if (frames.empty())
		return;

	apply_sac_vector(frames);
}

void SaCRegistration::getTransformation(
	Matrix4fVector& sac_translation_matrix_vector
	)
{
	if (_sac_translation_matrix_vector.empty())
		return;

	for (int i = 0; i < _sac_translation_matrix_vector.size(); i++)
		sac_translation_matrix_vector.push_back(_sac_translation_matrix_vector[i]);
}

//----------------------------------------------------

void SaCRegistration::prepare()
{
	_sac_translation_matrix_vector.clear();
}

void SaCRegistration::calculate_sac(
	KeypointsFrame& keypointsFrame,
	double inlier_threshold, int max_iter,
	pcl::Correspondences& inliers, Eigen::Matrix4f& out_transformation_matrix
	)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> sac;
	sac.setInputSource(keypointsFrame.keypointsPcdPair.second);
	sac.setInputTarget(keypointsFrame.keypointsPcdPair.first);
	sac.setInlierThreshold(inlier_threshold);
	sac.setMaximumIterations(max_iter);

	boost::shared_ptr<pcl::Correspondences> correspondences_ptr(new pcl::Correspondences);
	for (int i = 0; i < keypointsFrame.keypointsPcdCorrespondences.size(); i++)
		correspondences_ptr.get()->push_back(keypointsFrame.keypointsPcdCorrespondences[i]);
	sac.setInputCorrespondences(correspondences_ptr);

	pcl::Correspondences new_inliers;
	sac.getCorrespondences(new_inliers);
	inliers = new_inliers;

	out_transformation_matrix = sac.getBestTransformation();
}

void SaCRegistration::calculate_one_keypoint_pair_sac(
	KeypointsFrame& keypointsFrame,
	Eigen::Matrix4f& transformation_matrix
	)
{
	//Calculate SaC transform according to camera position
	Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
	calculate_sac(
		keypointsFrame,
		DISABLED_INLIER_THRESHOLD, max_iter,
		keypointsFrame.keypointsPcdCorrespondences,
		best_transformation_matrix
	);

	//Transform Keypoint clouds
	pcl::transformPointCloud(
		*keypointsFrame.keypointsPcdPair.first,
		*keypointsFrame.keypointsPcdPair.first,
		transformation_matrix
	);
	transformation_matrix = transformation_matrix * best_transformation_matrix;
	pcl::transformPointCloud(
		*keypointsFrame.keypointsPcdPair.second,
		*keypointsFrame.keypointsPcdPair.second,
		transformation_matrix
	);

	if (settings->value("SAC_SETTINGS/ENABLE_LOG").toBool())
		std::cout << "\n" << transformation_matrix << "\n";
}

void SaCRegistration::calculate_all_keypoint_pair_sac(
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix
	)
{
	qDebug() << "Calculating SaC registrations...";

	Eigen::Matrix4f transformation_matrix = intial_transformation_matrix;
	_sac_translation_matrix_vector.push_back(transformation_matrix);

	for (size_t i = 0; i < keypointsFrames.size(); i++)
	{
		qDebug() << QString("SaC registration: %1 / %2").arg(i + 1).arg(keypointsFrames.size()).toStdString().c_str();
		calculate_one_keypoint_pair_sac(
			keypointsFrames[i],
			transformation_matrix
		);

		if (settings->value("SAC_SETTINGS/ENABLE_LOG").toBool())
			std::cout << "\n" << transformation_matrix << "\n";

		_sac_translation_matrix_vector.push_back(transformation_matrix);
	}
	qDebug() << "Done!";
}

void SaCRegistration::calculate_middle_based_all_keypoint_pair_sac(
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix,
	int middle_index
	)
{
	Eigen::Matrix4f transformation_matrix = intial_transformation_matrix;
	for (size_t i = 0; i < keypointsFrames.size(); i++)
	{
		qDebug() << QString("SaC registration: %1 / %2").arg(i + 1).arg(keypointsFrames.size()).toStdString().c_str();
		calculate_one_keypoint_pair_sac(
			keypointsFrames[i],
			transformation_matrix
		);

		if (settings->value("SAC_SETTINGS/ENABLE_LOG").toBool())
			std::cout << std::endl << transformation_matrix << std::endl;

		_sac_translation_matrix_vector.push_back(transformation_matrix);
		transformation_matrix = intial_transformation_matrix;
	}
}

void SaCRegistration::apply_sac_vector(
	Frames& frames
	)
{
	if (_sac_translation_matrix_vector.size() != frames.size())
		return;

	if (_sac_translation_matrix_vector.empty() || frames.empty())
		return;

	for (int i = 0; i < frames.size(); i++)
	{
		pcl::transformPointCloud(
			*frames[i].pointCloudPtr.get(),
			*frames[i].pointCloudPtr.get(),
			_sac_translation_matrix_vector[i]
		);
	}
}

/** \brief Copy keypoint frames. */
void SaCRegistration::copyKeypointsFrame(
	KeypointsFrame& in_frame, 
	KeypointsFrame& out_frame
	)
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
