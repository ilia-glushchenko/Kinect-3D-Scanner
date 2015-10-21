#include "imagesviewerwidget.h"

ImagesViewerWidget::ImagesViewerWidget(std::vector<cv::Mat> images_vector, QString window_title)
{
	setParent(0);

	if (images_vector.empty() == false)
	{
		title = window_title;

		for (size_t i = 0; i < images_vector.size(); i++)
			imagesVector.push_back(Mat2QImage(images_vector[i]));

		image_index = 0;

		width  = imagesVector[0].width();
		height = imagesVector[0].height();
		resize(width, height);

		mainImage = imagesVector[0];

		mainLabel = new QLabel("");
		mainLabel->setPixmap(QPixmap::fromImage(mainImage));
		mainLabel->resize(width, height);
		
		slider = new QSlider(Qt::Horizontal);
		slider->setRange(0, imagesVector.size()-1);
		slider->setTickInterval(10);
		slider->setSingleStep(1);
		connect(slider, SIGNAL(sliderMoved(int)),
			this, SLOT(slot_change_image(int)));

		vBox = new QVBoxLayout;
		vBox->addWidget(mainLabel);
		vBox->addWidget(slider);
		
		setLayout(vBox);

		setWindowTitle(QString("%1 Image #%2").arg(title).arg(image_index));
	}

	if (imagesVector.empty() == false)
		show();
	else
		close();
}

void ImagesViewerWidget::mousePressEvent(QMouseEvent *e)
{
	if (e->buttons() == Qt::RightButton)
	{
		if (image_index + 1 < imagesVector.size())
		{
			image_index++;

			mainImage = imagesVector[image_index];
			mainLabel->setPixmap(QPixmap::fromImage(mainImage));
			mainLabel->resize(width, height);
			mainLabel->repaint();

			setWindowTitle(QString("%1 Image #%2").arg(title).arg(image_index));
		}

	}
	if (e->buttons() == Qt::LeftButton)
	{
		if (image_index - 1 >= 0)
		{
			image_index--;

			mainImage = imagesVector[image_index];
			mainLabel->setPixmap(QPixmap::fromImage(mainImage));
			mainLabel->resize(width, height);
			mainLabel->repaint();

			setWindowTitle(QString("%1 Image #%2").arg(title).arg(image_index));
		}
	}
}

QImage ImagesViewerWidget::Mat2QImage(cv::Mat const& src)
{
	cv::Mat temp; 
	cvtColor(src, temp, CV_BGR2RGB); 
	QImage dest((uchar*)temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
	QImage dest2(dest);
	dest2.detach();
	return dest2;
}

void ImagesViewerWidget::slot_change_image(int index)
{
	image_index = index;

	mainImage = imagesVector[image_index];
	mainLabel->setPixmap(QPixmap::fromImage(mainImage));
	mainLabel->resize(width, height);
	mainLabel->repaint();

	setWindowTitle(QString("%1 Image #%2").arg(title).arg(image_index));
}