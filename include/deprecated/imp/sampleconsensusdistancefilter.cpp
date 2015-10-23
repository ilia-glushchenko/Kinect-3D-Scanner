#include "utility/sampleconsensusdistancefilter.h"

SampleConsensusDistanceFilter::SampleConsensusDistanceFilter(
	QObject *parent
	)
{
	setParent(parent);

	settings = new QSettings("scaner.ini", QSettings::IniFormat, this);

	inlier_threshold =
		settings->value("SAC_SETTINGS/INLIER_THRESHOLD").toFloat();
	max_iter =
		settings->value("SAC_SETTINGS/MAX_ITERATIONS").toInt();
	distance_threshold =
		settings->value("SAC_DISTANCE_FILTER/DISTANCE_THRESHOLD").toFloat();
	triplets_size =
		settings->value("SAC_DISTANCE_FILTER/TRIPLETS_SIZE").toInt();
	allow_zero_distances =
		settings->value("SAC_DISTANCE_FILTER/ALLOW_ZERO_DISTANCE").toBool();
}

void SampleConsensusDistanceFilter::calculate(
	PcdPtr src_input_point_cloud_ptr,
	PcdPtr src_target_point_cloud_ptr,
	pcl::Correspondences src_correspondences_vector,
	PcdPtr dst_input_point_cloud_ptr,
	PcdPtr dst_target_point_cloud_ptr,
	pcl::Correspondences* dst_correspondences_vector
	)
{
	_src_input_point_cloud_ptr  = src_input_point_cloud_ptr;
	_src_target_point_cloud_ptr = src_target_point_cloud_ptr;
	_src_correspondences_vector = src_correspondences_vector;
	_dst_input_point_cloud_ptr  = dst_input_point_cloud_ptr;
	_dst_target_point_cloud_ptr = dst_target_point_cloud_ptr;
	_dst_correspondences_vector = dst_correspondences_vector;

	perform_sac();
}

void SampleConsensusDistanceFilter::perform_sac()
{
	sort_correspondences();
	//calculate_all_triplets();
	//perform_final_correspondences_rejection();
	iterative_rejection();
	fill_dst_point_clouds();
}

void SampleConsensusDistanceFilter::sort_correspondences()
{
	_correspondences.correspondences = &_src_correspondences_vector;

	//Bubble sorting correspondences
	for (int i = 0; i < _src_correspondences_vector.size(); i++)
	{
		float max_z_dist_input
			= _src_input_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_query].z;
		float max_z_dist_target
			= _src_target_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_match].z;
		float averege_z_dist = (max_z_dist_input + max_z_dist_target) / 2.0f;

		int min_index = i;

		for (int j = i; j < _src_correspondences_vector.size(); j++)
		{
			float current_max_z_dist_input
				= _src_input_point_cloud_ptr.get()->points[_src_correspondences_vector[j].index_query].z;
			float current_max_z_dist_target
				= _src_target_point_cloud_ptr.get()->points[_src_correspondences_vector[j].index_match].z;
			float current_averege_z_dist = (current_max_z_dist_input + current_max_z_dist_target) / 2.0f;

			if (current_averege_z_dist < averege_z_dist)
			{
				averege_z_dist = current_averege_z_dist;
				min_index = j;
			}
		}

		if (min_index != i)
		{
			pcl::Correspondence tmp = _src_correspondences_vector[i];
			_src_correspondences_vector[i] = _src_correspondences_vector[min_index];
			_src_correspondences_vector[min_index] = tmp;
		}

		_correspondences.averege_z_distance.push_back(averege_z_dist);
	}

	//Remove zero correspondonces
	if (allow_zero_distances == false)
		remove_zero_distances_correspondences();

	if (settings->value("SAC_DISTANCE_FILTER/DINAMIC_TRIPLET").toBool())
		triplets_size = _correspondences.correspondences->size();

	//Cut the tail
	while (_correspondences.correspondences->size() % triplets_size != 0)
	{
		_correspondences.correspondences->pop_back();
		_correspondences.averege_z_distance.pop_back();
	}
}

void SampleConsensusDistanceFilter::remove_zero_distances_correspondences()
{
	pcl::Correspondences new_src_correspondences_vector;
	std::vector<float> new_averege_z_distance;

	for (int i = 0; i < _correspondences.correspondences->size(); i++)
		if (_correspondences.correspondences->at(i).distance > 0)
		{
			new_src_correspondences_vector.push_back(_correspondences.correspondences->at(i));
			new_averege_z_distance.push_back(_correspondences.averege_z_distance[i]);
		}
			
	*_correspondences.correspondences = new_src_correspondences_vector;
	_correspondences.averege_z_distance = new_averege_z_distance;
}

void SampleConsensusDistanceFilter::calculate_all_triplets()
{
	//Calculate SaC for each triplet
	for (int i = 0; i + triplets_size <= _src_correspondences_vector.size(); i += triplets_size)
	{
		pcl::Correspondences triplet_correspondences_vector;
		for (int j = i; j < i + triplets_size; j++)
			triplet_correspondences_vector.push_back(_src_correspondences_vector[j]);

		PcdPtr input_point_cloud_ptr(new Pcd);
		PcdPtr target_point_cloud_ptr(new Pcd);

		calculate_one_triplet_transformation(
			triplet_correspondences_vector,
			input_point_cloud_ptr,
			target_point_cloud_ptr			
		);

		estimate_one_triplet_rejections(
			input_point_cloud_ptr,
			target_point_cloud_ptr
		);
	}
}

void SampleConsensusDistanceFilter::calculate_one_triplet_transformation(
	pcl::Correspondences correspondences_vector,
	PcdPtr input_point_cloud_ptr,
	PcdPtr target_point_cloud_ptr
	)
{
	pcl::copyPointCloud(*_src_input_point_cloud_ptr.get(), *input_point_cloud_ptr.get());
	pcl::copyPointCloud(*_src_target_point_cloud_ptr.get(), *target_point_cloud_ptr.get());

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	pcl::registration::TransformationEstimationSVD<PointType, PointType> svd;
	svd.estimateRigidTransformation(		*input_point_cloud_ptr.get(),		*target_point_cloud_ptr.get(),		correspondences_vector,		transformation	);
	pcl::transformPointCloud(
		*input_point_cloud_ptr.get(),
		*input_point_cloud_ptr.get(),
		transformation
	);
}

void SampleConsensusDistanceFilter::estimate_one_triplet_rejections(
	PcdPtr input_point_cloud_ptr,
	PcdPtr target_point_cloud_ptr
	)
{
	//Filling 2d correspondences vector with bool vars according to threshold passing
	std::vector<bool> estimated_correspondences;

	for (int i = 0; i < _src_correspondences_vector.size(); i++)
	{
		PointType input_point 
			= input_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_query];
		PointType target_point 
			= target_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_match];

		float _distance = sqrtf(
			powf(input_point.x - target_point.x, 2) + 
			powf(input_point.y - target_point.y, 2) +
			powf(input_point.z - target_point.z, 2)
		);

		if (_distance < distance_threshold)
			estimated_correspondences.push_back(true);
		else
			estimated_correspondences.push_back(false);
	}

	estimated_correspondences_vector.push_back(estimated_correspondences);
}

void SampleConsensusDistanceFilter::perform_final_correspondences_rejection()
{
	for (int i = 0; i < _src_correspondences_vector.size(); i++)
	{
		bool passed_threshold = true;
		for (int j = 0; j < estimated_correspondences_vector.size(); j++)
			if (estimated_correspondences_vector[j][i] == false)
			{
				passed_threshold = false;
				break;
			}

		if (passed_threshold)
			_dst_correspondences_vector->push_back(_src_correspondences_vector[i]);			
	}
}

void SampleConsensusDistanceFilter::fill_dst_point_clouds()
{
	pcl::Correspondences new_correspondences;

	for (int i = 0; i < _dst_correspondences_vector->size(); i++)
	{
		_dst_input_point_cloud_ptr.get()->points.push_back(
			_src_input_point_cloud_ptr.get()->points[_dst_correspondences_vector->at(i).index_query]
		);
		_dst_target_point_cloud_ptr.get()->points.push_back(
			_src_target_point_cloud_ptr.get()->points[_dst_correspondences_vector->at(i).index_match]
		);

		pcl::Correspondence correspondence;
		correspondence.index_query = i;
		correspondence.index_match = i;

		PointType input_point =
			_src_input_point_cloud_ptr.get()->points[_dst_correspondences_vector->at(i).index_query];
		PointType target_point =
			_src_target_point_cloud_ptr.get()->points[_dst_correspondences_vector->at(i).index_match];

		float _distance = sqrtf(
			powf(input_point.x - target_point.x, 2) +
			powf(input_point.y - target_point.y, 2) +
			powf(input_point.z - target_point.z, 2)
		);

		correspondence.distance = _distance;

		new_correspondences.push_back(correspondence);
	}

	*_dst_correspondences_vector = new_correspondences;
	
	_dst_input_point_cloud_ptr.get()->width = _dst_input_point_cloud_ptr.get()->points.size();
	_dst_input_point_cloud_ptr.get()->height = 1;
	_dst_input_point_cloud_ptr.get()->resize(_dst_input_point_cloud_ptr.get()->width);
	
	_dst_target_point_cloud_ptr.get()->width = _dst_target_point_cloud_ptr.get()->points.size();
	_dst_target_point_cloud_ptr.get()->height = 1;
	_dst_target_point_cloud_ptr.get()->resize(_dst_target_point_cloud_ptr.get()->width);

	qDebug() << "SaC Distance Rejection:" << _dst_input_point_cloud_ptr.get()->points.size()  << "/" << _src_input_point_cloud_ptr.get()->points.size();
}

void SampleConsensusDistanceFilter::iterative_rejection()
{
	int bad_points_counter;
	do {
		bad_points_counter = 0;

		PcdPtr input_point_cloud_ptr(new Pcd);
		PcdPtr target_point_cloud_ptr(new Pcd);

		pcl::Correspondences new_cor;

		for (int i = 0; i < _src_correspondences_vector.size(); i++)
		{
			input_point_cloud_ptr.get()->points.push_back(
				_src_input_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_query]
			);
			target_point_cloud_ptr.get()->points.push_back(
				_src_target_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_match]
			);

			PointType a = _src_input_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_query];
			PointType b = _src_target_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_match];

			pcl::Correspondence cor;
			cor.index_match = i;
			cor.index_query = i;
			cor.distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));

			new_cor.push_back(cor);
		}

		_src_correspondences_vector = new_cor;

		input_point_cloud_ptr.get()->width = input_point_cloud_ptr.get()->points.size();
		input_point_cloud_ptr.get()->height = 1;
		input_point_cloud_ptr.get()->resize(input_point_cloud_ptr.get()->width);

		target_point_cloud_ptr.get()->width = target_point_cloud_ptr.get()->points.size();
		target_point_cloud_ptr.get()->height = 1;
		target_point_cloud_ptr.get()->resize(target_point_cloud_ptr.get()->width);

		//------------------------------

		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
		pcl::registration::TransformationEstimationSVD<PointType, PointType> svd;
		svd.estimateRigidTransformation(			*input_point_cloud_ptr.get(),			*target_point_cloud_ptr.get(),			_src_correspondences_vector,			transformation		);
		pcl::transformPointCloud(
			*input_point_cloud_ptr.get(),
			*input_point_cloud_ptr.get(),
			transformation
		);

		//-------------------------------

		pcl::Correspondences new_correspondences_vector;

		for (int i = 0; i < _src_correspondences_vector.size(); i++)
		{
			PointType a = input_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_query];
			PointType b = target_point_cloud_ptr.get()->points[_src_correspondences_vector[i].index_match];

			float _distance = sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2) + powf(a.z - b.z, 2));

			if (_distance < distance_threshold)
				new_correspondences_vector.push_back(_src_correspondences_vector[i]);
			else
				bad_points_counter++;
		}

		_src_correspondences_vector = new_correspondences_vector;

	} while (bad_points_counter != 0);
}