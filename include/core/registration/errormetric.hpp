#ifndef ERRORMETRIC_HPP
#define ERRORMETRIC_HPP

#include <pcl/common/distances.h>
#include <pcl/common/point_tests.h>


struct DeltaZMetric
{
	typedef Eigen::Matrix4f Matrix;

	static double calculate(const Matrix & input_mat, const Matrix & target_mat)
	{
		const double & target_z = target_mat(2, 3);
		const double & input_z = input_mat(2, 3);
		const double result = target_z - input_z;
		return result;
	}
};


struct CameraDistanceMetric
{
	typedef Eigen::Matrix4f Matrix;

	static double calculate(const Matrix & input_mat, const Matrix & target_mat)
	{
		return pcl::euclideanDistance(
			pcl::PointXYZ(target_mat(0, 3), target_mat(1, 3), target_mat(2, 3)),
			pcl::PointXYZ(input_mat(0, 3), input_mat(1, 3), input_mat(2, 3))
		);

	}
};



#endif //ERRORMETRIC_HPP