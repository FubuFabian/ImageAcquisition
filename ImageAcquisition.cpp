#include "ImageAcquisition.h"
#include "PolarisTracker.h"

#include <vtkBMPWriter.h>

#include <QtTest/QTest>
#include <QErrorMessage>
#include <QString>
#include <QFile>
#include <QTextStream>

#include <igstkImageSpatialObject.h>



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

	std::cout<<std::endl;
	std::cout<<"Loading Probe Calibration Data"<<std::endl;

	std::vector<double> probeCalibrationData;
	probeCalibrationData.reserve(8);

	if (!probeCalibrationFilename.isEmpty())
    {
        QFile file(probeCalibrationFilename);
        if (!file.open(QIODevice::ReadOnly))
           return;

		QTextStream stream(&file);
        QString line;

		for(int i=0;i<8;i++)
        {
			line = stream.readLine();       
			probeCalibrationData.push_back(line.toDouble());
        }
		 file.close(); 	
	}else
	{
		 std::cout<<"No Probe Calibration Data Loaded"<<std::endl;
	}

	igstk::Transform::ErrorType errorValue;
	errorValue=10;

	double validityTimeInMilliseconds;
	validityTimeInMilliseconds = igstk::TimeStamp::GetLongestPossibleTime();

	TransformType::VectorType probeTranslation;
	TransformType::VersorType probeRotation;

	probeTranslation[0] = probeCalibrationData[0];
	probeTranslation[1] = probeCalibrationData[1];
	probeTranslation[2] = probeCalibrationData[2];
    probeRotation.Set(probeCalibrationData[3], probeCalibrationData[4], probeCalibrationData[5], 0.0);
	probeCalibrationTransform.SetTranslationAndRotation(probeTranslation, probeRotation, errorValue, validityTimeInMilliseconds);

	
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
					usTransform = coordSystemAObserverUltrasoundProbe->GetTransform();
                    usTransform = TransformType::TransformCompose(usTransform,probeCalibrationTransform);
					usTranslation = usTransform.GetTranslation();
					usRotation = usTransform.GetRotation();


					m_VideoFrame->RequestGetVTKImage();

					vtkImageData * vtkImage = vtkImageData::New();
					vtkImage->DeepCopy(m_VideoFrame->GetImageData());

					this->addImageToStack(vtkImage);
					translationStack.push_back(usTranslation);
					rotationStack.push_back(usRotation);

					imageAcquisitionWidget->displayImages();

					singleImageFlag = false;
				}
			}else if(multipleImagesFlag){
				if (coordSystemAObserverUltrasoundProbe->GotTransform())
				{
					usTransform = coordSystemAObserverUltrasoundProbe->GetTransform();
                    usTransform = TransformType::TransformCompose(usTransform,probeCalibrationTransform);
					usTranslation = usTransform.GetTranslation();
					usRotation = usTransform.GetRotation();

					m_VideoFrame->RequestGetVTKImage();

					vtkImageData * vtkImage = vtkImageData::New();
					vtkImage->DeepCopy(m_VideoFrame->GetImageData());
					imageTempStack.push_back(vtkImage);

					translationStack.push_back(usTranslation);
					rotationStack.push_back(usRotation);

					numberOfTakenImages++;

					if(numberOfTakenImages == numberOfImages){
						this->addImagesToStack(imageTempStack);
						imageAcquisitionWidget->acquireMultipleImages();
						imageAcquisitionWidget->displayImages();
						multipleImagesFlag = false;
						numberOfTakenImages = 0;
						imageTempStack.clear();
					}

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

void ImageAcquisition::setMultipleImagesFlagTrue(int numberOfImages)
{
	this->multipleImagesFlag = true;
	this->numberOfImages = numberOfImages;
}

void ImageAcquisition::setSingleImageFlagTrue()
{
	this->singleImageFlag = true;
}

void ImageAcquisition::setNumberOfImages(int numberOfImages)
{
	this->numberOfImages = numberOfImages;
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

void ImageAcquisition::addImageToStack(vtkImageData * vtkImage)
{
	vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
	imageData = vtkImage;

	flipYFilter = vtkSmartPointer<vtkImageFlip>::New();	
	flipYFilter->SetFilteredAxis(1); 
	flipYFilter->SetInput(imageData);
	flipYFilter->Update();

	imageStack.push_back(flipYFilter->GetOutput());
}

void ImageAcquisition::addImagesToStack(std::vector<vtkImageData *> vtkImageStack)
{
	for(int i = 0; i<vtkImageStack.size(); i++)
	{
		vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
		imageData = vtkImageStack.at(i);
		
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