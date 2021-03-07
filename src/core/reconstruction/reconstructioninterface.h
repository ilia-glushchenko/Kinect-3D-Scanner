#ifndef RECONSTRUCTIONINTERFACE_H
#define RECONSTRUCTIONINTERFACE_H

#include <QDebug>
#include <QObject>
#include <QSettings>
#include <QString>
#include <chrono> //pause
#include <thread> //pause

#include "core/base/scannertypes.h"
#include "core/reconstruction/volumereconstruction.h"
#include "gui/imagesviewerwidget.h"
#include "gui/pcdvizualizer.h"

class ReconstructionInterface : public ScannerBase {
    Q_OBJECT

public:
    ReconstructionInterface(QObject* parent, QSettings* parent_settings);

    void reloadSettings();

private:
    VolumeReconstruction::Ptr volumeReconstruction;
    PcdVizualizer::Ptr pcdVizualizer;

    void perform_tsdf_integration(
        Frames& frames,
        Matrix4fVector& final_translation_matrix_vector);

    void perform_tsdf_meshing(
        Matrix4fVector& final_translation_matrix_vector);

    void perform_reconstruction();

    void perform_iterative_reconstruction();

    void perform_partition_recursive_reconstruction();

    void perform_lum_reconstruction();

public slots:
    void slot_perform_reconstruction();
};

#endif // RECONSTRUCTIONINTERFACE_H
