#ifndef IMAGESVIEWERWIDGET_H
#define IMAGESVIEWERWIDGET_H

#include <QDebug>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMouseEvent>
#include <QPixmap>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include "opencv2/legacy/legacy.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/stitcher.hpp"

class ImagesViewerWidget : public QWidget {
    Q_OBJECT

public:
    ImagesViewerWidget(std::vector<cv::Mat> images_vector, QString window_title);

protected:
    void mousePressEvent(QMouseEvent* e);

private:
    int width;
    int height;

    int image_index;

    std::vector<QImage> imagesVector;

    QVBoxLayout* vBox;
    QImage mainImage;
    QLabel* mainLabel;
    QSlider* slider;

    QString title;

    QImage Mat2QImage(cv::Mat const& src);

private slots:
    void slot_change_image(int);
};

#endif // IMAGESVIEWERWIDGET_H
