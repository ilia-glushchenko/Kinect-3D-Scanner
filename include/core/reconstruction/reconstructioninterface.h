#ifndef RECONSTRUCTIONINTERFACE_H
#define RECONSTRUCTIONINTERFACE_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>
#include <chrono>  //pause
#include <thread>  //pause

#include "core/reconstruction/volumereconstruction.h"
#include "gui/imagesviewerwidget.h"
#include "gui/pcdvizualizer.h"
#include "core/base/types.h"

class ReconstructionInterface : public ScannerBase
{
	Q_OBJECT

public:
	ReconstructionInterface(QObject *parent, QSettings* parent_settings);

	void set_use_reconstruction(bool);
	void set_use_undistortion(bool);
	void set_use_bilateral_filter(bool);
	void set_use_statistical_outlier_removal_filter(bool);
	void set_use_moving_least_squares_filter(bool);	

private:
	bool use_reconstruction;
	bool use_undistortion;
	bool use_bilateral_filter;
	bool use_statistical_outlier_removal_filter;
	bool use_moving_least_squares_filter;

	VolumeReconstruction::Ptr volumeReconstruction;
	PcdVizualizer::Ptr		  pcdVizualizer;

	void perform_tsdf_integration(
		Frames & frames,
		Matrix4fVector & final_translation_matrix_vector
	);

	void perform_tsdf_meshing(
		Matrix4fVector& final_translation_matrix_vector
	);

	void perform_reconstruction();

	void perform_iterative_reconstruction();

	void perform_partition_recursive_reconstruction();

	void perform_lum_reconstruction();

public slots:
	void slot_perform_reconstruction();
	void slot_change_pair(int);

	void slot_set_use_reconstruction  (int);
	void slot_set_use_undistortion	  (int);
	void slot_set_use_bilateral_filter(int);
	void slot_set_use_statistical_outlier_removal_filter(int);
	void slot_set_use_moving_least_squares_filter		(int);
};

#endif // RECONSTRUCTIONINTERFACE_H
