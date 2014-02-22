#include "ImageAcquisitionWidget.h"
#include "ui_ImageAcquisitionWidget.h"
#include "ImageAcquisition.h"
#include "PolarisConfigurationWidget.h"

#include <QString>
#include <QMainWindow>
#include <QtGui>
#include <QtTest/QTest>
#include <QVBoxLayout>
#include <QLayout>

#include <vtkImageData.h>
#include <vtkBMPWriter.h>
#include <vtkSmartPointer.h>

ImageAcquisitionWidget::ImageAcquisitionWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageAcquisitionWidget)
{
    ui->setupUi(this);

	this->View = igstk::View2D::New();
	this->qtUltrasoundDisplay = new igstk::QTWidget();
	this->qtUltrasoundDisplay->RequestSetView(this->View);
	QGridLayout * ultrasoundLayout = new QGridLayout;
	ultrasoundLayout->setContentsMargins(0,0,0,0);
	ultrasoundLayout->addWidget(qtUltrasoundDisplay,0,0);
    ui->ultrasoundDisplay->setLayout(ultrasoundLayout);

	this->qtImageDisplay = new QVTKImageWidget();
	QVBoxLayout *imagesLayout = new QVBoxLayout;
    imagesLayout->setContentsMargins(0, 0, 0, 0);
    imagesLayout->setSpacing(0);
    imagesLayout->QLayout::addWidget(qtImageDisplay);
    ui->imagesDisplay->setLayout(imagesLayout);

	ui->startTrackingBtn->setEnabled(false);
	ui->singleImageBtn->setEnabled(false);
	ui->multipleImageBtn->setEnabled(false);
	ui->saveImagesBtn->setEnabled(false);
	ui->clearBtn->setEnabled(false);

	this->quit = false;

    ui->coordsX->setText("");
    ui->coordsY->setText("");
    ui->coordsZ->setText("");
	ui->rotX->setText("");
    ui->rotY->setText("");
    ui->rotZ->setText("");
	ui->rotW->setText("");


    ui->numberImages->setText("100");

	redPal = QPalette(ui->trackingLabel->palette());
	greenPal = QPalette(ui->trackingLabel->palette());
	redPal.setColor(ui->trackingLabel->foregroundRole(), Qt::red);
	greenPal.setColor(ui->trackingLabel->foregroundRole(), Qt::green);

	ui->trackingLabel->setPalette(redPal);

	ui->imageSlider->hide();
	ui->imageSlider->setTickInterval(1);
}

ImageAcquisitionWidget::~ImageAcquisitionWidget()
{
    delete ui;
}

void ImageAcquisitionWidget::Show()
{
    this->quit = false;
    this->show();
    this->View->RequestStart();
}

void ImageAcquisitionWidget::Quit()
{
    this->View->RequestStop();

	this->m_Tracker->RequestStopTracking();
    this->quit = true;
    this->hide();
}

void ImageAcquisitionWidget::startTracking()
{
	ui->singleImageBtn->setEnabled(true);
	ui->multipleImageBtn->setEnabled(true);
    this->m_Tracker->RequestStartTracking();
	imageAcquisition->startTracking();
}

void ImageAcquisitionWidget::configTracker()
{
	PolarisConfigurationWidget *polarisConfiguration = new PolarisConfigurationWidget();
	polarisConfiguration->show();

	while(!polarisConfiguration->hasQuitted())
	{
		QTest::qWait(0.001);
	}

	std::vector<QString> files = polarisConfiguration->getFiles();

    std::string referenceToolFilename = std::string(files[0].toAscii().data());
    std::string ultrasoundProbeFilename = std::string(files[1].toAscii().data());
    std::string needleFilename = std::string(files[2].toAscii().data());
	std::string pointerFilename = std::string(files[3].toAscii().data());

	int port = polarisConfiguration->getPort();

	imageAcquisition->configTracker(referenceToolFilename, ultrasoundProbeFilename, needleFilename, 
							pointerFilename, files[4], files[5], files[6], port);

	ui->startTrackingBtn->setEnabled(true);
}

void ImageAcquisitionWidget::setCoords(std::vector<double> coords)
{
	QString str0 = QString::number(coords[0]);
	ui->coordsX->setText(str0);

	QString str1 = QString::number(coords[1]);
	ui->coordsY->setText(str1);

	QString str2 = QString::number(coords[2]);
	ui->coordsZ->setText(str2);

}

void ImageAcquisitionWidget::SetTracker( TrackerType * tracker)
{
	this->m_Tracker = tracker;
}


bool ImageAcquisitionWidget::HasQuitted()
{
    return this->quit;
}

void ImageAcquisitionWidget::setImageAcquisition(ImageAcquisition* imageAcquisition)
{
    this->imageAcquisition = imageAcquisition;
}

void ImageAcquisitionWidget::singleImage()
{
	ui->saveImagesBtn->setEnabled(true);
	ui->clearBtn->setEnabled(true);
	this->imageAcquisition->setSingleImageFlagTrue();
}

void ImageAcquisitionWidget::multipleImages()
{
	ui->trackingLabel->setPalette(greenPal);
	ui->saveImagesBtn->setEnabled(true);
	ui->clearBtn->setEnabled(true);
	this->imageAcquisition->setMultipleImagesFlagTrue(ui->numberImages->text().toInt());
}

void ImageAcquisitionWidget::saveImages()
{
	typedef ::itk::Vector<double, 3>    VectorType;
	typedef ::itk::Versor<double>       VersorType;
	
	/*imageStack = imageAcquisition->getImages();
	rotationStack = imageAcquisition->getRotations();
	translationStack = imageAcquisition->getTranslations();*/

	std::cout<<"Saving Acquired "<<imageStack.size()<<" Images"<<std::endl;
		
	vtkSmartPointer<vtkBMPWriter> writer = vtkSmartPointer<vtkBMPWriter>::New();
	QString saveDirectory = QFileDialog::getExistingDirectory(
                this, "Choose Directory to Save Images", QDir::currentPath(), 
				QFileDialog::ShowDirsOnly );
	QString filename;
	std::string str;
	const char * saveFile;

	QString saveRotationsFile = QFileDialog::getSaveFileName(
                this, tr("Choose Directory to Save Rotations"), QDir::currentPath(),tr("Txt (*.txt)"));
    QFile fileRotations(saveRotationsFile);
    if(!fileRotations.open(QIODevice::WriteOnly | QIODevice::Text)){
        std::cout<<"Could not open File to save the rotations"<<std::endl;
        return;
    }
	QTextStream outRotations(&fileRotations);
	QString saveTranslationsFile = QFileDialog::getSaveFileName(
                this, tr("Choose Directory to Tanslations Rotations"), QDir::currentPath(),tr("Txt (*.txt)"));

    QFile fileTranslations(saveTranslationsFile);

    if(!fileTranslations.open(QIODevice::WriteOnly | QIODevice::Text)){
        std::cout<<"Could not open File to save the rotations"<<std::endl;
        return;
    }

	QTextStream outTranslations(&fileTranslations);

	for(int i=0; i < imageStack.size(); i++){			
		
		char imageNumber[4];
		sprintf(imageNumber,"%d",i);
		filename = saveDirectory;
		if(i<10)
			filename.append("/IMG000");
		else if(i<100)
			filename.append("/IMG00");
		else if(i<1000)
			filename.append("/IMG0");
		else
			filename.append("/IMG");

	 	filename.append(imageNumber);
		filename.append(".bmp");
		str = std::string(filename.toAscii().data());
		saveFile = str.c_str();
		writer->SetFileName(saveFile);
		writer->SetInput(imageStack.at(i));
		writer->Write();

		outRotations<<rotationStack.at(i).GetW()<<" ";
		outRotations<<rotationStack.at(i).GetX()<<" ";
		outRotations<<rotationStack.at(i).GetY()<<" ";
		outRotations<<rotationStack.at(i).GetZ()<<"\n";

		outTranslations<<translationStack.at(i)[0]<<" ";
		outTranslations<<translationStack.at(i)[1]<<" ";
		outTranslations<<translationStack.at(i)[2]<<"\n";
	}	
	
	fileRotations.close();
	fileTranslations.close();


}

void ImageAcquisitionWidget::acquireMultipleImages()
{
		ui->trackingLabel->setPalette(redPal);		
}


void ImageAcquisitionWidget::displayImages()
{
	imageStack = imageAcquisition->getImages();
	rotationStack = imageAcquisition->getRotations();
	translationStack = imageAcquisition->getTranslations();

	ui->imageSlider->show();
    ui->imageSlider->setRange(0, imageStack.size() - 1);

	this->qtImageDisplay->setAndDisplayMultipleImages(imageStack);

	QString str0 = QString::number(translationStack.at(0)[0]);
	ui->coordsX->setText(str0);
	QString str1 = QString::number(translationStack.at(0)[1]);
	ui->coordsY->setText(str1);
	QString str2 = QString::number(translationStack.at(0)[2]);
	ui->coordsZ->setText(str2);
	QString str3 = QString::number(rotationStack.at(0).GetX());
	ui->rotX->setText(str3);
	QString str4 = QString::number(rotationStack.at(0).GetY());
	ui->rotY->setText(str4);
	QString str5 = QString::number(rotationStack.at(0).GetZ());
	ui->rotZ->setText(str5);
	QString str6 = QString::number(rotationStack.at(0).GetW());
	ui->rotW->setText(str6);

}

void ImageAcquisitionWidget::displaySelectedImage(int idx)
{
    this->qtImageDisplay->displaySelectedImage(idx);

	QString str0 = QString::number(translationStack.at(idx)[0]);
	ui->coordsX->setText(str0);
	QString str1 = QString::number(translationStack.at(idx)[1]);
	ui->coordsY->setText(str1);
	QString str2 = QString::number(translationStack.at(idx)[2]);
	ui->coordsZ->setText(str2);
	QString str3 = QString::number(rotationStack.at(idx).GetX());
	ui->rotX->setText(str3);
	QString str4 = QString::number(rotationStack.at(idx).GetY());
	ui->rotY->setText(str4);
	QString str5 = QString::number(rotationStack.at(idx).GetZ());
	ui->rotZ->setText(str5);
	QString str6 = QString::number(rotationStack.at(idx).GetW());
	ui->rotW->setText(str6);
}

void ImageAcquisitionWidget::clearImages()
{
	ui->saveImagesBtn->setEnabled(false);
	ui->clearBtn->setEnabled(false);
	imageAcquisition->clearAcquiredImages();

	vtkSmartPointer<vtkImageData> blankImage = vtkSmartPointer<vtkImageData>::New();
	this->qtImageDisplay->setAndDisplayImage(blankImage);
	ui->imageSlider->hide();

	ui->coordsX->setText("");
    ui->coordsY->setText("");
    ui->coordsZ->setText("");
	ui->rotX->setText("");
    ui->rotY->setText("");
    ui->rotZ->setText("");
	ui->rotW->setText("");

}


