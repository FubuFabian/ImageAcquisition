#ifndef IMAGEACQUISITIONWIDGET_H
#define IMAGEACQUISITIONWIDGET_H

#include <QWidget>

#include <igstkView2D.h>
#include <igstkQTWidget.h>
#include <igstkTracker.h>
#include <igstkConfigure.h>

#include "QVTKImageWidget.h"

#include <QPalette>

class ImageAcquisition;

namespace Ui {
class ImageAcquisitionWidget;
}

class ImageAcquisitionWidget : public QWidget
{
    Q_OBJECT
    
public:

	typedef ::itk::Vector<double, 3>    VectorType;
	typedef ::itk::Versor<double>       VersorType; 

    explicit ImageAcquisitionWidget(QWidget *parent = 0);
    ~ImageAcquisitionWidget();
 
	typedef igstk::Tracker TrackerType;
    typedef TrackerType::Pointer TrackerPointer;
    
	igstk::QTWidget * qtUltrasoundDisplay; ///<A qt widget
	QVTKImageWidget * qtImageDisplay;

    igstk::View2D::Pointer View; ///<The IGSTK View

	/** \brief Show the widget nd start displaying the scene*/
    void Show();

	/** \brief Returns the quit flag*/
	bool HasQuitted();

	/** \brief Set the tracker for the instruments
	* \param[in] Polaris tracker*/
    void SetTracker( TrackerType * tracker);

	/** \brief Set the 3DScene*/
    void setImageAcquisition(ImageAcquisition *);

	/** \brief Set the instruments coords in the widget*/
	void setCoords(std::vector<double>);

	void displayImages();

	void imageTaken();

private:
    Ui::ImageAcquisitionWidget *ui;

	bool quit; ///<Quit Flag
	TrackerPointer m_Tracker; ///<Polaris Tracker
	ImageAcquisition* imageAcquisition; ///<Scene3D object with the IGSTK objects

	QPalette redPal;
	QPalette greenPal;

	std::vector< vtkSmartPointer<vtkImageData> > imageStack;
	std::vector<VersorType> rotationStack;
	std::vector<VectorType> translationStack;

	int numberOfImages;

	bool multipleImagesFlag;
	bool recordFlag;

	void imagesAcquired();

private slots:

    void singleImage();
    void multipleImages();
	void recordImages(bool);
	void configTracker();
	void startTracking();
	void Quit();
	void saveImages();
	void displaySelectedImage(int);
	void clearImages();

};

#endif // IMAGEACQUISITIONWIDGET_H
