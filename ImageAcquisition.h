

#include <iostream>

#include "igstkPolarisTracker.h"
#include "igstkPolarisTrackerTool.h"

#include "igstkTransformObserver.h"

#include "ImageAcquisitionWidget.h"
#include "igstkEpiphanVideoImager.h"
#include "igstkEpiphanVideoImagerTool.h"
#include "igstkVideoFrameSpatialObject.h"

#include <igstkVideoFrameRepresentation.h>

#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageFlip.h>

#include <itkImage.h>
#include <itkImageIOBase.h>

using namespace std;

class ImageAcquisition
{

	typedef itk::Image<unsigned char,2> ImageType;

    typedef igstk::PolarisTrackerTool         TrackerToolType;
    typedef TrackerToolType::TransformType    TransformType;
    typedef igstk::TransformObserver          ObserverType;
	
	typedef igstk::VideoFrameSpatialObject<unsigned char, 1> 
										VideoFrameObjectType;
	typedef igstk::VideoFrameRepresentation<VideoFrameObjectType>
										VideoFrameRepresentationType;

	typedef ::itk::Vector<double, 3>    VectorType;
	typedef ::itk::Versor<double>       VersorType;

public:

	static ImageAcquisition *New()
	{
		return new ImageAcquisition;
	}
 
	/** \brief Initialize the 2D scene, creates all the scene objects*/
    void init2DScene();

	/** \brief Configure the Polaris tracker
	* \param[in] ROM files*/
	void configTracker(std::string, std::string, std::string, std::string, QString, QString, QString, int);
	
	/** \brief Start Tracking*/
	void startTracking();

	void startImaging();

	void clearAcquiredImages();

	void setSingleImageFlagTrue();

	void setMultipleImagesFlag(bool);

	void imagesAcquired();

	std::vector< vtkSmartPointer<vtkImageData> > getImages();
	std::vector<VersorType> getRotations();
	std::vector<VectorType> getTranslations();

private:

    bool startImagingFlag;
	bool configTrackerFlag;
	bool startTrackingFlag;
	bool multipleImagesFlag;
	bool singleImageFlag;

	int numberOfImages;

	unsigned int frameSize[3];
	unsigned int imageSize;

	TransformType identityTransform; ///<Transformation for the tracked objects

	igstk::PolarisTracker::Pointer tracker; ///<Tracker object
	TrackerToolType::Pointer referenceTool; ///<Tool for the reference axes
	TrackerToolType::Pointer ultrasoundProbeTool; ///<Tool to track the us probe
	TransformType probeCalibrationTransform;

	ObserverType::Pointer coordSystemAObserverReferenceTool; ///<Oberver for the reference tool events
	ObserverType::Pointer coordSystemAObserverUltrasoundProbe; ///<Oberver for the ultrasound probe events

	ImageAcquisitionWidget * imageAcquisitionWidget;

    VideoFrameObjectType::Pointer m_VideoFrame;
	VideoFrameRepresentationType::Pointer m_VideoFrameRepresentation;

	igstk::EpiphanVideoImager::Pointer videoImager;
    igstk::VideoImagerTool::Pointer videoImagerTool;

	void stopImaging();

	void addImageToStack(unsigned char *);

	void addImagesToStack(std::vector<unsigned char *>);
	/**
    * \brief convert an itk image to a vtk image
    * [out] a vtk image
    */
    vtkSmartPointer<vtkImageData> convertToVTKImage(ImageType::Pointer itkImage);

	std::vector< vtkSmartPointer<vtkImageData> > imageStack;
	std::vector<unsigned char *> imageDataPtrStack;
	std::vector<VectorType> translationStack;
	std::vector<VersorType> rotationStack;

	vtkSmartPointer<vtkImageFlip> flipYFilter;

};



