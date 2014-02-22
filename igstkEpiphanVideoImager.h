/*=========================================================================

  Program:   Image Guided Surgery Software Toolkit
  Module:    $RCSfile: igstkEpiphanVideoImager.h,v $
  Language:  C++
  Date:      $Date: 2009-06-17 21:58:17 $
  Version:   $Revision: 1.3 $

  Copyright (c) ISC  Insight Software Consortium.  All rights reserved.
  See IGSTKCopyright.txt or http://www.igstk.org/copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __igstkEpiphanVideoImager_h
#define __igstkEpiphanVideoImager_h

#ifdef _MSC_VER
#pragma warning ( disable : 4018 )
//Warning about: identifier was truncated to '255' characters in the debug
//information (MVC6.0 Debug)
#pragma warning( disable : 4284 )
#endif

#include "igstkVideoImager.h"
#include "igstkEpiphanVideoImagerTool.h"

#include <sys/types.h>
#include <stdio.h>
#include <map>
#include <signal.h>

#include "frmgrab.h"

class vtkImageData;

namespace igstk {

/** \class EpiphanVideoImager
 * \brief This derivation of the VideoImager class provides communication
 * to the Epiphan frame grabber
 *
 * This class controlls the communication with the video device.
 * The communication with the frame grabber is established with the unicap
 * library over firewire
 *
 * \ingroup VideoImager
 */

class EpiphanVideoImager : public VideoImager
{
public:
  /** Macro with standard traits declarations. */
  igstkStandardClassTraitsMacro( EpiphanVideoImager, VideoImager )

public:

	
  enum VideoFormatType
  {
    VIDEO_FORMAT_UNKNOWN,
    VIDEO_FORMAT_RGB24,
    VIDEO_FORMAT_Y8
  };


  /** Get the number of tools that have been detected. */
  igstkGetMacro( NumberOfTools, unsigned int );

  /*!
    Set the clip rectangle size to apply to the image in pixel coordinates.
    If the ClipRectangleSize is (0,0) then the values are ignored and the whole frame is captured.
    Width of the ClipRectangle typically have to be a multiple of 4.
  */
  void setClipRectangleSize(std::vector<int>);
  /*!
    Get the clip rectangle size to apply to the image in pixel coordinates.
    If the ClipRectangleSize is (0,0) then the values are ignored and the whole frame is captured.
  */
  std::vector<int> getClipRectangleSize();

  /*!
    Set the clip rectangle origin to apply to the image in pixel coordinates.
    If the ClipRectangleSize is (0,0) then the whole frame is captured.
  */
  void setClipRectangleOrigin(std::vector<int>);
  /*!
    Get the clip rectangle origin to apply to the image in pixel coordinates.
    If the ClipRectangleSize is (0,0) then the whole frame is captured.
  */
  std::vector<int> getClipRectangleOrigin();

  igstkGetMacro(AcquisitionRate,double);

  igstkSetMacro(AcquisitionRate,double);

  igstkGetMacro(FrameNumber,unsigned long);

  igstkSetMacro(GrabberLocation,char*);

   /*! Set the Epiphan device video format (e.g. "VIDEO_FORMAT_Y8" ) */
  igstkSetMacro(VideoFormat,VideoFormatType);
  /*! Get the Epiphan device video format (e.g. "VIDEO_FORMAT_Y8" ) */
  igstkGetMacro(VideoFormat,VideoFormatType);



protected:

  EpiphanVideoImager(void);

  virtual ~EpiphanVideoImager(void);

  /** Typedef for internal boolean return type. */
  typedef VideoImager::ResultType   ResultType;

  /** Open communication with the imaging device. */
  virtual ResultType InternalOpen( void );

  /** Close communication with the imaging device. */
  virtual ResultType InternalClose( void );

  /** Put the imaging device into imaging mode. */
  virtual ResultType InternalStartImaging( void );

  /** Take the imaging device out of imaging mode. */
  virtual ResultType InternalStopImaging( void );

  /** Update the status and the transforms for all VideoImagerTools. */
  virtual ResultType InternalUpdateStatus( void );

  /** Update the status and the frames.
      This function is called by a separate thread. */
  virtual ResultType InternalThreadedUpdateStatus( void );

  /** Reset the imaging device to put it back to its original state. */
  virtual ResultType InternalReset( void );

  /** Verify imager tool information */
  virtual ResultType VerifyVideoImagerToolInformation( 
                                                  const VideoImagerToolType * );

  /** The "ValidateSpecifiedFrequency" method checks if the specified
   * frequency is valid for the imaging device that is being used. */
  virtual ResultType ValidateSpecifiedFrequency( double frequencyInHz );

  /** Print object information */
  virtual void PrintSelf( std::ostream& os, itk::Indent indent ) const;

  /** Remove imager tool entry from internal containers */
  virtual ResultType RemoveVideoImagerToolFromInternalDataContainers( const
                                     VideoImagerToolType * imagerTool );

  /** Add imager tool entry to internal containers */
  virtual ResultType AddVideoImagerToolToInternalDataContainers( const
                                     VideoImagerToolType * imagerTool );

private:

  EpiphanVideoImager(const Self&);   //purposely not implemented
  void operator=(const Self&);   //purposely not implemented

  /** Initialize camera */
  bool Initialize();

  /** A mutex for multithreaded access to the buffer arrays */
  itk::MutexLock::Pointer  m_BufferLock;

  /** Total number of tools detected. */
  unsigned int   m_NumberOfTools;

  /** A buffer to hold frames */
  typedef std::map< std::string, igstk::Frame* >
                                VideoImagerToolFrameContainerType;

  typedef igstk::Frame   FrameType;
  VideoImagerToolFrameContainerType           m_ToolFrameBuffer;

  /** Error map container */
  typedef std::map< unsigned int, std::string>  ErrorCodeContainerType;
  static ErrorCodeContainerType   m_ErrorCodeContainer;

  /** boolean to indicate if error code list is created */
  static bool m_ErrorCodeListCreated;

  /** Container holding status of the tools */
  std::map< std::string, int >  m_ToolStatusContainer;

  /** Members and functions for communication with Epiphan library */
  public:

  /*! Video format (e.g. Y8) */
  VideoFormatType m_VideoFormat;

  /*! String to specify the framegrabber to connect to (auto-detection is attempted if unspecified) */
  const char* m_GrabberLocation;

  /*! Epiphan Pointer to the grabber */
  FrmGrabber* FrameGrabber;

  /*! Frame size of the captured image */
  int FrameSize[2];

  bool Recording;

  double MAXIMUM_FREQUENCY;

  double m_AcquisitionRate;

  unsigned long m_FrameNumber;

  static itk::MutexLock::Pointer  m_FrameBufferLock;
  static unsigned char  *pixels[1];// = NULL;

  /*! Crop rectangle origin for the grabber (in pixels) */
  std::vector<int>  m_ClipRectangleOrigin;

  /*! Crop rectangle size for the grabber (in pixels). If it is (0,0) then the whole frame will be captured. */
  std::vector<int>  m_ClipRectangleSize;

};

}  // namespace igstk

#endif //__igstk_EpiphanVideoImager_h_
