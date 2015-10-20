#include "icpregistration.h"

ICPRegistration::ICPRegistration(
	QObject *parent
	)
{
	setParent(parent);
	settings = new QSettings("scaner.ini", QSettings::IniFormat, this);
}

void ICPRegistration::calculateICPTransformation(
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix,
	int middle_index
	)
{
	if (keypointsFrames.empty())
		return;

	prepare();
	if (middle_index != -1)
		calculate_middle_based_all_keypoint_pair_icp(keypointsFrames, intial_transformation_matrix, middle_index);	
	else
		calculate_all_keypoint_pair_icp(keypointsFrames, intial_transformation_matrix);
}

void ICPRegistration::applyTransfomation(
	Frames& frames
	)
{
	if (frames.empty())
		return;

	apply_icp_vector(frames);
}

void ICPRegistration::getTransformation(
	Matrix4fVector& icp_translation_matrix_vector
	)
{
	if (_icp_translation_matrix_vector.empty())
		return;

	for (int i = 0; i < _icp_translation_matrix_vector.size(); i++)
		icp_translation_matrix_vector.push_back(_icp_translation_matrix_vector[i]);
}

//----------------------------------------------------

void ICPRegistration::prepare()
{
	_icp_translation_matrix_vector.clear();
}

void ICPRegistration::calculate_one_keypoint_pair_icp(
	KeypointsFrame& keypointsFrame,
	Eigen::Matrix4f& transformation_matrix
	)
{
	PcdPtr input_point_cloud_ptr  = keypointsFrame.keypointsPcdPair.second;
	PcdPtr target_point_cloud_ptr = keypointsFrame.keypointsPcdPair.first;
	pcl::Correspondences correspondeces = keypointsFrame.keypointsPcdCorrespondences;

	if (settings->value("ICP_SETTINGS/POINT_TO_PLANE").toBool())
	{
		/*
		pcl::registration::TransformationEstimationPointToPlaneLLS<PointType, PointType> icp;

		Eigen::Matrix4f final_transformation_maxtrix;
		icp.estimateRigidTransformation(
			*input_point_cloud_ptr.get(),
			*target_point_cloud_ptr.get(),
			correspondeces,
			final_transformation_maxtrix
		);

		//Transformation
		pcl::transformPointCloud(
			*target_point_cloud_ptr.get(),
			*target_point_cloud_ptr.get(),
			transformation_matrix
		);

		transformation_matrix = final_transformation_maxtrix * transformation_matrix;

		pcl::transformPointCloud(
			*input_point_cloud_ptr.get(),
			*input_point_cloud_ptr.get(),
			transformation_matrix
		);

		if (settings->value("ICP_SETTINGS/ENABLE_LOG").toBool())
			qDebug() << "Point To Plane ICP converged.";
		*/
	}
	else
	{
		int maxIter
			= settings->value("ICP_SETTINGS/MAX_ITERATIONS").toInt();
		double trans_epsilon
			= settings->value("ICP_SETTINGS/TRANSFORMATION_EPSILON").toDouble();
		double euclid_epsilon
			= settings->value("ICP_SETTINGS/EUCLIDEAN_EPSILON").toDouble();

		pcl::IterativeClosestPointNonLinear<PointType, PointType> icp;
		icp.setInputCloud(input_point_cloud_ptr);
		icp.setInputTarget(target_point_cloud_ptr);
		icp.setMaximumIterations(maxIter);
		icp.setTransformationEpsilon(trans_epsilon);
		icp.setEuclideanFitnessEpsilon(euclid_epsilon);
		PcdPtr output_cloud_ptr(new Pcd);
		icp.align(*output_cloud_ptr.get());

		if (icp.hasConverged())
		{
			//ICP aligment
			pcl::transformPointCloud(
				*target_point_cloud_ptr.get(),
				*target_point_cloud_ptr.get(),
				transformation_matrix
				);

			Eigen::Matrix4f final_transformation_maxtrix = icp.getFinalTransformation();
			transformation_matrix = final_transformation_maxtrix * transformation_matrix;

			pcl::transformPointCloud(
				*input_point_cloud_ptr.get(),
				*input_point_cloud_ptr.get(),
				transformation_matrix
				);

			if (settings->value("ICP_SETTINGS/ENABLE_LOG").toBool())
			{
				qDebug() << "ICP converged.";
				qDebug() << "The score is " << icp.getFitnessScore();
			}
		}
		else
		{
			if (settings->value("ICP_SETTINGS/ENABLE_LOG").toBool())
				qDebug() << "ICP did not converge.";
		}
	}

}

void ICPRegistration::calculate_all_keypoint_pair_icp(
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix
	)
{
	qDebug() << "Calculating ICP registrations...";

	Eigen::Matrix4f transformation_matrix = intial_transformation_matrix;
	_icp_translation_matrix_vector.push_back(transformation_matrix);

	for (int i = 0; i < keypointsFrames.size(); i++)
	{
		qDebug() << QString("ICP registrations %1 / %2").arg(i + 1).arg(keypointsFrames.size()).toStdString().c_str();
		calculate_one_keypoint_pair_icp(
			keypointsFrames[i],
			transformation_matrix
		);

		if (settings->value("ICP_SETTINGS/ENABLE_LOG").toBool())
			std::cout << std::endl << transformation_matrix << std::endl;

		_icp_translation_matrix_vector.push_back(transformation_matrix);
	}
	qDebug() << "Done!";
}

void ICPRegistration::calculate_middle_based_all_keypoint_pair_icp(
	KeypointsFrames& keypointsFrames,
	Eigen::Matrix4f& intial_transformation_matrix,
	int middle_index
	)
{
	Eigen::Matrix4f transformation_matrix = intial_transformation_matrix;
	for (int i = 0; i < keypointsFrames.size(); i++)
	{
		qDebug() << QString("ICP registrations %1 / %2").arg(i + 1).arg(keypointsFrames.size()).toStdString().c_str();
		calculate_one_keypoint_pair_icp(
			keypointsFrames[i],
			transformation_matrix
		);

		if (settings->value("ICP_SETTINGS/ENABLE_LOG").toBool())
			std::cout << std::endl << transformation_matrix << std::endl;

		_icp_translation_matrix_vector.push_back(transformation_matrix);
		transformation_matrix = intial_transformation_matrix;
	}
}

void ICPRegistration::apply_icp_vector(
	Frames& frames
	)
{
	if (_icp_translation_matrix_vector.size() != frames.size())
		return;

	if (_icp_translation_matrix_vector.empty() || frames.empty())
		return;

	for (int i = 0; i < frames.size(); i++)
	{
		pcl::transformPointCloud(
			*frames[i].pointCloudPtr.get(),
			*frames[i].pointCloudPtr.get(),
			_icp_translation_matrix_vector[i]
		);
	}
}
