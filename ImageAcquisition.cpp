#include "ImageAcquisition.h"
#include "PolarisTracker.h"
#include "itkImageToVTKImageFilter.h"

#include <vtkBMPWriter.h>

#include <QtTest/QTest>
#include <QErrorMessage>
#include <QString>
#include <QFile>
#include <QTextStream>

#include <igstkImageSpatialObject.h>

#include <itkImportImageFilter.h>

void ImageAcquisition::configTracker(std::string referenceToolFilename, std::string ultrasoundProbeFilename, 
							std::string needleFilename, std::string pointerFilename, QString probeCalibrationFilename,
							 QString needleCalibrationFilename,  QString pointerCalibrationFilename, int port)
{
	PolarisTracker* polarisTracker = PolarisTracker::New();
	polarisTracker->setLoggerOn(false);
	std::cout<<"-Initializing SerialCommunication"<<std::endl;
	polarisTracker->initializeSerialCommunication(port);
	std::cout<<"-Initializing Tracker"<<std::endl;
	polarisTracker->initializeTracker();
	std::cout<<"-Initializing Reference Tracker Tool"<<std::endl;
	polarisTracker->initializeTrackerTool(referenceToolFilename);
	std::cout<<"-Initializing Ulatrasound Probe Tracker Tool"<<std::endl;
	polarisTracker->initializeTrackerTool(ultrasoundProbeFilename);

	std::cout<<"-Attaching all tools"<<std::endl;
	polarisTracker->attachAllTools();
	std::cout<<"-Creating observers for all tools"<<std::endl;
	polarisTracker->createToolsObervers();

	tracker = polarisTracker->getTracker();

	referenceTool = polarisTracker->getTools().at(0);
	ultrasoundProbeTool = polarisTracker->getTools().at(1);
		
	coordSystemAObserverReferenceTool = polarisTracker->getObservers().at(0);
	coordSystemAObserverReferenceTool->ObserveTransformEventsFrom(referenceTool);
	coordSystemAObserverUltrasoundProbe = polarisTracker->getObservers().at(1);
	coordSystemAObserverUltrasoundProbe->ObserveTransformEventsFrom(ultrasoundProbeTool);

	std::cout<<std::endl<<"-Reference Tracker Tool ID: "<<referenceTool->GetTrackerToolIdentifier()<<std::endl;
	std::cout<<"-Ultrasound Tracker Probe Tool ID: "<<ultrasoundProbeTool->GetTrackerToolIdentifier()<<std::endl;
	
	imageAcquisitionWidget->SetTracker(tracker);

	configTrackerFlag = true;
}

void ImageAcquisition::startTracking()
{
	if(configTrackerFlag)
	{

		int numberOfTakenImages = 0;

		while( !imageAcquisitionWidget->HasQuitted() )
		{
			QTest::qWait(0.001);
			igstk::PulseGenerator::CheckTimeouts();

			TransformType             usTransform;                                                 
			VectorType                usTranslation;
			VersorType				  usRotation;
                        
			coordSystemAObserverReferenceTool->Clear();
			referenceTool->RequestGetTransformToParent();
			coordSystemAObserverUltrasoundProbe->Clear();
			ultrasoundProbeTool->RequestComputeTransformTo(referenceTool);

			if(singleImageFlag){

				if (coordSystemAObserverUltrasoundProbe->GotTransform())
				{
					unsigned char * imageDataPtr = m_VideoFrame->GetImagePtr();

					usTransform = coordSystemAObserverUltrasoundProbe->GetTransform();
                    usTransform = TransformType::TransformCompose(usTransform,probeCalibrationTransform);
					usTranslation = usTransform.GetTranslation();
					usRotation = usTransform.GetRotation();

					this->addImageToStack(imageDataPtr);
					translationStack.push_back(usTranslation);
					rotationStack.push_back(usRotation);

					imageAcquisitionWidget->displayImages();

					singleImageFlag = false;
				}
			}else if(multipleImagesFlag){

				if (coordSystemAObserverUltrasoundProbe->GotTransform())
				{
					unsigned char * imageDataPtr = new unsigned char[imageSize];
					memcpy(imageDataPtr,m_VideoFrame->GetImagePtr(),imageSize);
					imageDataPtrStack.push_back(imageDataPtr);

					usTransform = coordSystemAObserverUltrasoundProbe->GetTransform();
                    usTransform = TransformType::TransformCompose(usTransform,probeCalibrationTransform);
					usTranslation = usTransform.GetTranslation();
					usRotation = usTransform.GetRotation();

					translationStack.push_back(usTranslation);
					rotationStack.push_back(usRotation);

					this->imageAcquisitionWidget->imageTaken();
				}
		    }
                       
		}

		tracker->RequestClose();
		this->stopImaging();

		delete imageAcquisitionWidget;
	}
	else{
		QErrorMessage errorMessage;
        errorMessage.showMessage(
            "Tracker is not configure, </ br> please configure tracker first");
        errorMessage.exec();
	}
}

void ImageAcquisition::init2DScene()
{
	igstk::RealTimeClock::Initialize();
    imageAcquisitionWidget = new ImageAcquisitionWidget();

	imageAcquisitionWidget->setImageAcquisition(this);

	this->startImaging();
    
    float Cx = frameSize[0]/ 2;
    float Cy = frameSize[1]/ 2;
    
	imageAcquisitionWidget->View->RequestResetCamera();
	imageAcquisitionWidget->View->SetCameraPosition( Cx, Cy , -1050 );
	imageAcquisitionWidget->View->SetCameraViewUp( 0, -1, 0 );
	imageAcquisitionWidget->View->SetCameraFocalPoint( Cx, Cy, 0.0 );
	imageAcquisitionWidget->View->SetCameraParallelProjection( false );
	imageAcquisitionWidget->View->SetRendererBackgroundColor(0,0,0);

	igstk::Transform identityTransform;
    identityTransform.SetToIdentity( igstk::TimeStamp::GetLongestPossibleTime() );

	m_VideoFrame->RequestSetTransformAndParent(identityTransform,imageAcquisitionWidget->View);
    imageAcquisitionWidget->View->RequestDetachFromParent();

	m_VideoFrameRepresentation = VideoFrameRepresentationType::New();
    m_VideoFrameRepresentation->RequestSetVideoFrameSpatialObject(m_VideoFrame);

	imageAcquisitionWidget->View->RequestAddObject(m_VideoFrameRepresentation);
	imageAcquisitionWidget->Show();

	configTrackerFlag = false;
	singleImageFlag = false;
	multipleImagesFlag = false;
}

void ImageAcquisition::startImaging()
{
	videoImager = igstk::EpiphanVideoImager::New();
    videoImager->RequestSetFrequency(60);
    videoImager->RequestOpen();
	
	frameSize[0] = videoImager->FrameSize[0];
	frameSize[1] = videoImager->FrameSize[1];
	frameSize[2] = 1;

	imageSize = frameSize[0]*frameSize[1]*frameSize[2];
	
	m_VideoFrame = VideoFrameObjectType::New();
	m_VideoFrame->SetWidth(frameSize[0]);
    m_VideoFrame->SetHeight(frameSize[1]);
    m_VideoFrame->SetPixelSizeX(1);
    m_VideoFrame->SetPixelSizeY(1);
    m_VideoFrame->SetNumberOfScalarComponents(frameSize[2]);
    m_VideoFrame->Initialize();

    igstk::EpiphanVideoImagerTool::Pointer videoImagerEpiphanTool = igstk::EpiphanVideoImagerTool::New();

    videoImagerEpiphanTool->SetFrameDimensions(frameSize);
    videoImagerEpiphanTool->SetPixelDepth(8);
    videoImagerEpiphanTool->RequestSetVideoImagerToolName("Frame grabber");
    videoImagerEpiphanTool->RequestConfigure();

    videoImagerTool = videoImagerEpiphanTool;

    videoImagerTool->RequestAttachToVideoImager(videoImager);
    m_VideoFrame->SetVideoImagerTool(videoImagerTool);

    videoImager->RequestStartImaging();
}

void ImageAcquisition::stopImaging()
{
	videoImager->RequestStopImaging();
    videoImager->RequestClose();
}

void ImageAcquisition::setMultipleImagesFlag(bool flag)
{
	this->multipleImagesFlag = flag;
}

void ImageAcquisition::setSingleImageFlagTrue()
{
	this->singleImageFlag = true;
}

std::vector< vtkSmartPointer<vtkImageData> > ImageAcquisition::getImages()
{
	return this->imageStack;
}

typedef ::itk::Versor<double>       VersorType;
std::vector<VersorType> ImageAcquisition::getRotations()
{
	return this->rotationStack;
}

typedef ::itk::Vector<double, 3>    VectorType;
std::vector<VectorType> ImageAcquisition::getTranslations()
{
	return this->translationStack;
}

void ImageAcquisition::addImageToStack(unsigned char * imageDataPtr)
{	
	ImageType::SizeType imageSize;
	imageSize[0] = frameSize[0];
	imageSize[1] = frameSize[1];

	ImageType::IndexType imageStart;
	imageStart.Fill(0);

	ImageType::RegionType imageRegion;
	imageRegion.SetSize(imageSize);
	imageRegion.SetIndex(imageStart);

	itk::ImportImageFilter<unsigned char,2>::Pointer importFilter = itk::ImportImageFilter<unsigned char,2>::New();
	importFilter->SetRegion(imageRegion);
	importFilter->SetImportPointer(reinterpret_cast<ImageType::PixelType*>(imageDataPtr),
																		frameSize[0]*frameSize[1], false);
	importFilter->Update();

	ImageType::Pointer itkImage = importFilter->GetOutput();

	vtkSmartPointer<vtkImageData> imageData = convertToVTKImage(itkImage);

	flipYFilter = vtkSmartPointer<vtkImageFlip>::New();	
	flipYFilter->SetFilteredAxis(1); 
	flipYFilter->SetInput(imageData);
	flipYFilter->Update();

	imageStack.push_back(flipYFilter->GetOutput());
}

void ImageAcquisition::addImagesToStack(std::vector<unsigned char *> imageDataPtrStack)
{

	std::cout<<imageDataPtrStack.size();
	for(int i = 0; i<imageDataPtrStack.size(); i++)
	{
		ImageType::SizeType imageSize;
		imageSize[0] = frameSize[0];
		imageSize[1] = frameSize[1];

		ImageType::IndexType imageStart;
		imageStart.Fill(0);

		ImageType::RegionType imageRegion;
		imageRegion.SetSize(imageSize);
		imageRegion.SetIndex(imageStart);

		itk::ImportImageFilter<unsigned char,2>::Pointer importFilter = itk::ImportImageFilter<unsigned char,2>::New();
		importFilter->SetRegion(imageRegion);
		importFilter->SetImportPointer(reinterpret_cast<ImageType::PixelType*>(imageDataPtrStack.at(i)),
																		frameSize[0]*frameSize[1], false);
		importFilter->Update();

		ImageType::Pointer itkImage = ImageType::New();
	    itkImage = importFilter->GetOutput();

		vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
		imageData = convertToVTKImage(itkImage);

		flipYFilter = vtkSmartPointer<vtkImageFlip>::New();	
		flipYFilter->SetFilteredAxis(1); 
		flipYFilter->SetInput(imageData);
		flipYFilter->Update();

		imageStack.push_back(flipYFilter->GetOutput());
	}
}

void ImageAcquisition::clearAcquiredImages()
{
	imageStack.clear();
	rotationStack.clear();
	translationStack.clear();
}

typedef itk::Image<unsigned char *,2> Imagetype;
vtkSmartPointer<vtkImageData> ImageAcquisition::convertToVTKImage(ImageType::Pointer itkImage)
{   
    typedef itk::ImageToVTKImageFilter<ImageType> VTKConverterType;
    VTKConverterType::Pointer vtkConverter = VTKConverterType::New();
    vtkConverter->SetInput(itkImage);
    vtkConverter->Update();
    
    vtkSmartPointer<vtkImageData> tempImage = vtkSmartPointer<vtkImageData>::New();
    tempImage->DeepCopy(vtkConverter->GetOutput());
    
    return tempImage;
}

void ImageAcquisition::imagesAcquired()
{
	this->addImagesToStack(imageDataPtrStack);
	imageAcquisitionWidget->displayImages();
	imageDataPtrStack.clear();
}