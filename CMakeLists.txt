CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(ImageAcquisition)


OPTION (USE_EPIPHAN "Provide support for the Epiphan" OFF)

IF(COMMAND CMAKE_POLICY)
	   CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)

FIND_PACKAGE(ITK)
IF(ITK_FOUND)
	INCLUDE(${ITK_USE_FILE})
ELSE(ITK_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build US Probe Calibration without ITK.  Please set ITK_DIR.")
ENDIF(ITK_FOUND)

FIND_PACKAGE (VTK)
IF (VTK_FOUND)
   INCLUDE(${USE_VTK_FILE})
ELSE(VTK_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build US Probe Calibration without VTK.  Please set ITK_DIR.")   
ENDIF(VTK_FOUND)

FIND_PACKAGE(LSQRRecipes)
IF(LSQRRecipes_FOUND)
    INCLUDE(${LSQRRecipes_USE_FILE})
ELSE(LSQRRecipes_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build US Probe Calibration without LSQRRecipes.")   
ENDIF(LSQRRecipes_FOUND)

FIND_PACKAGE(IGSTK)
IF(IGSTK_FOUND)
	INCLUDE(${IGSTK_USE_FILE})
ELSE(IGSTK_FOUND)
	MESSAGE(FATAL_ERROR "Cannot build US Probe Calibration without IGSTK.  Please set IGSTK_DIR.")
ENDIF(IGSTK_FOUND)

FIND_PACKAGE(FLTK)
 IF (FLTK_FOUND)
  INCLUDE_DIRECTORIES (${FLTK_INCLUDE_DIR})
ELSE (FLTK_FOUND)
  MESSAGE( FATAL_ERROR "This application requires FLTK. One of these components is missing. Please verify configuration")
ENDIF (FLTK_FOUND)

# Find QT 
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE( ${QT_USE_FILE} )

IF (USE_EPIPHAN)

  SET( EPIPHAN_SDK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Epiphan )
  SET( EPIPHAN_INCLUDE_DIR ${EPIPHAN_SDK_BASE_DIR} CACHE PATH "Path to the Epiphan SDK include directory." FORCE )
  SET( EPIPHAN_WIN32_BINARY_DIR ${EPIPHAN_SDK_BASE_DIR}/Win32 CACHE PATH "Path to the Epiphan 32-bit dll folder" FORCE )
  SET( EPIPHAN_WIN32_LIB_DIR ${EPIPHAN_SDK_BASE_DIR}/Win32 CACHE PATH "Path to the Epiphan 32-bit lib folder" FORCE )
  SET( EPIPHAN_WIN64_BINARY_DIR ${EPIPHAN_SDK_BASE_DIR}/x64 CACHE PATH "Path to the Epiphan 64-bit dll folder" FORCE )
  SET( EPIPHAN_WIN64_LIB_DIR ${EPIPHAN_SDK_BASE_DIR}/x64 CACHE PATH "Path to the Epiphan 64-bit lib folder" FORCE )
  SET( EPIPHAN_SAVE_DIR ${EPIPHAN_SDK_BASE_DIR}/save CACHE PATH "Path to the Save Images Libraries" FORCE )


  IF( ${CMAKE_GENERATOR} MATCHES Win64 )
    SET (EPIPHAN_BINARY_DIR ${EPIPHAN_WIN64_BINARY_DIR})
    SET (EPIPHAN_LIB_DIR ${EPIPHAN_WIN64_LIB_DIR})
  ELSE()
    SET (EPIPHAN_BINARY_DIR ${EPIPHAN_WIN32_BINARY_DIR})
    SET (EPIPHAN_LIB_DIR ${EPIPHAN_WIN32_LIB_DIR})
  ENDIF()

ENDIF (USE_EPIPHAN)

# support for out-of-source build
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${IGSTK_SOURCE_DIR}/Examples/TransformReaders
  ${IGSTK_SOURCE_DIR}/Examples/TransformWriters
  ${EPIPHAN_INCLUDE_DIR}
)

LINK_DIRECTORIES(${EPIPHAN_LIB_DIR})
 
# Set your files and resources here
SET(AppSrcs main.cpp mainwindow.cpp QVTKImageWidget.cpp QVTKImageWidgetCommand.cpp 
    ProbeCalibrationWidget.cpp Calibration.cpp VolumeReconstructionWidget.cpp 
    VolumeReconstruction.cpp CropImagesWidget.cpp Scene3D.cpp Scene3DWidget.cpp
    PolarisTracker.cpp igstkNeedleObject.cpp igstkNeedleObjectRepresentation.cpp
    igstkUSProbeObject.cpp igstkUSProbeObjectRepresentation.cpp
    igstkPolarisPointerObject.cpp igstkPolarisPointerObjectRepresentation.cpp
    igstkTargetObject.cpp igstkTargetObjectRepresentation.cpp
    CheckCalibrationErrorWidget.cpp vtkTracerInteractorStyle.cpp
    EstimateSphereFromPoints.cpp SphereFunction.cpp igstkUSImageObject.cpp 
    igstkImageSpatialObjectVolumeRepresentation.txx QVTKVolumeSliceWidget.cpp
    VTKThreeViews.cpp ChangeVolumePropertiesWidget.cpp
    PivotCalibration.cpp PolarisConfigurationWidget.cpp SegmentationTrainingWidget.cpp
    SegmentationIntensityAndTextureImages.cpp ImageSegmentation.cpp
    ImageSegmentationWidget.cpp VolumeSegmentation.cpp
    VolumeSegmentationWidget.cpp igstkEpiphanVideoImager.cxx igstkEpiphanVideoImagerTool.cxx
    igstkVideoImager.cxx igstkVideoImagerTool.cxx igstkFrame.cxx ImageAcquisitionWidget.cpp
    ImageAcquisition.cpp igstkVideoFrameSpatialObject.txx)
    
SET(AppHeaders mainwindow.h QVTKImageWidget.h QVTKImageWidgetCommand.h 
    ProbeCalibrationWidget.h Calibration.h VolumeReconstructionWidget.h
    VolumeReconstruction.h CropImagesWidget.h Scene3D.h Scene3DWidget.h
    PolarisTracker.h igstkNeedleObject.h igstkNeedleObjectRepresentation.h
    igstkUSProbeObject.h igstkUSProbeObjectRepresentation.h
    igstkPolarisPointerObject.h igstkPolarisPointerObjectRepresentation.h
    igstkTargetObject.h igstkTargetObjectRepresentation.h
    CheckCalibrationErrorWidget.h vtkTracerInteractorStyle.h
    EstimateSphereFromPoints.h SphereFunction.h igstkUSImageObject.h 
    igstkImageSpatialObjectVolumeRepresentation.h QVTKVolumeSliceWidget.h
    VTKThreeViews.h ChangeVolumePropertiesWidget.h
    PivotCalibration.h PolarisConfigurationWidget.h SegmentationTrainingWidget.h
    itkScalarToTextureHUImageFilter.h itkScalarImageToRunLengthFeaturesFilter.h 
    itkScalarImageToRunLengthMatrixFilter.h itkHistogramToRunLengthFeaturesFilter.h 
    itkHistogramEqualizationFilter.h SegmentationIntensityAndTextureImages.h
    itkImageToVTKImageFilter.h itkVTKImageToImageFilter.h ImageSegmentation.h
    ImageSegmentationWidget.h VolumeSegmentation.h VolumeSegmentationWidget.h
    igstkNeedleObjectRepresentation.h igstkEpiphanVideoImager.h igstkEpiphanVideoImagerTool.h
    igstkVideoImager.h igstkVideoImagerTool.h igstkFrame.h ImageAcquisitionWidget.h
    ImageAcquisition.h igstkVideoFrameSpatialObject.h)
    
SET(AppUI mainwindow.ui ProbeCalibrationWidget.ui VolumeReconstructionWidget.ui 
    CropImagesWidget.ui Scene3DWidget.ui CheckCalibrationErrorWidget.ui
    VTKThreeViews.ui ChangeVolumePropertiesWidget.ui PolarisConfigurationWidget.ui
    SegmentationTrainingWidget.ui ImageSegmentationWidget.ui VolumeSegmentationWidget.ui
    ImageAcquisitionWidget.ui)

SET(AppUIFLTK PivotCalibrationUI.fl)

FLTK_WRAP_UI(ImageAcquisition ${AppUIFLTK}) 
 
# for generate qt aditional files 
QT4_WRAP_UI(UISrcs ${AppUI})
QT4_WRAP_CPP(MOCSrcs ${AppHeaders} )
 
 
# Packaging source code in same files 
SOURCE_GROUP("Resources" FILES
  ${AppUI}
  ${AppUIFLTK}
)
 
SOURCE_GROUP("Generated" FILES
  ${UISrcs}
  ${MOCSrcs}
  ${AppUIFLTK}
)
 
ADD_EXECUTABLE(ImageAcquisition ${AppSrcs} ${AppHeaders} ${UISrcs} ${MOCSrcs} ${AppUIFLTK})

TARGET_LINK_LIBRARIES(ImageAcquisition QVTK ${IGSTK_LIBRARIES} ${VTK_LIBRARIES} ${ITK_LIBRARIES} 
				${FLTK_LIBRARIES} TransformReaders TransformWriters 
				LSQRRecipes frmgrab v2u_lib jpeg)

