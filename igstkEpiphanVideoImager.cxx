/*=========================================================================

  Program:   Image Guided Surgery Software Toolkit
  Module:    $RCSfile: igstkEpiphanVideoImager.cxx,v $
  Language:  C++
  Date:      $Date: 2009-06-18 09:29:14 $
  Version:   $Revision: 1.7 $

  Copyright (c) ISC  Insight Software Consortium.  All rights reserved.
  See IGSTKCopyright.txt or http://www.igstk.org/copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#if defined(_MSC_VER)
// Warning about: identifier was truncated to '255' characters in the
// debug information (MVC6.0 Debug)
#pragma warning( disable : 4786 )
#endif


#include "igstkEpiphanVideoImager.h"
#include "igstkEvents.h"
#include "vtkImageData.h"
#include <itksys/SystemTools.hxx>

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkRGBPixel.h"

#include <string.h>
#include <stdarg.h>

#include "v2u_lib.h"
#include "v2u_version.h"
#include "avi_writer.h"

#define MAX_DEVICES 64
#define MAX_FORMATS 64

#ifdef _WIN32
#define strcasecmp(s1,s2) _stricmp(s1,s2)
#endif /* _WIN32 */

#ifndef V2U_COUNT
#  define V2U_COUNT(array) (sizeof(array)/sizeof((array)[0]))
#endif /* V2U_COUNT */

namespace igstk
{

//Initialize static variables
unsigned char* EpiphanVideoImager::pixels[1] = {NULL};
itk::MutexLock::Pointer EpiphanVideoImager::m_FrameBufferLock
                                                        = itk::MutexLock::New();

/** Constructor: Initializes all internal variables. */
EpiphanVideoImager::EpiphanVideoImager(void):m_StateMachine(this)
{

  this->m_ClipRectangleOrigin.reserve(2);
  this->m_ClipRectangleSize.reserve(2);

  this->m_VideoFormat = VIDEO_FORMAT_RGB24;
  this->m_ClipRectangleOrigin.assign(0,0);
  this->m_ClipRectangleOrigin.assign(1,0);
  this->m_ClipRectangleSize.assign(0,0);
  this->m_ClipRectangleSize.assign(1,0);
  this->m_GrabberLocation = NULL;
  this->m_NumberOfTools = 0;

  this->SetThreadingEnabled( true );

  // Lock for the data buffer that is used to transfer the
  // frames from thread that is communicating
  // with the imager to the main thread.
  m_BufferLock = itk::MutexLock::New();
}

/** Destructor */
EpiphanVideoImager::~EpiphanVideoImager(void)
{

}

EpiphanVideoImager::ResultType
                                   EpiphanVideoImager::InternalOpen( void )
{
  igstkLogMacro( DEBUG,
                   "igstk::EpiphanVideoImager::InternalOpen called ...\n");

  if( ! this->Initialize() )
    {
    igstkLogMacro( CRITICAL, "Error initializing");
    return FAILURE;
    }

  return SUCCESS;
}

/** Initialize socket */
bool EpiphanVideoImager::Initialize( void )
{
  igstkLogMacro( DEBUG,
                     "igstk::EpiphanVideoImager::Initialize called ...\n");

   // Initialize frmgrab library
  FrmGrabNet_Init();

  if ( this->m_GrabberLocation != NULL )
  {
    if ( (this->FrameGrabber = FrmGrab_Open( this->m_GrabberLocation )) == NULL )
    {
      if ( (this->FrameGrabber = FrmGrabLocal_Open()) == NULL )
      {
		  std::cout<<"Epiphan Device Not found"<<std::endl;;
        return FAILURE;
      }
      const char UNKNOWN_DEVICE[]="UNKNOWN";
      const char* connectedTo=FrmGrab_GetLocation((FrmGrabber*)this->FrameGrabber);
      if (connectedTo==NULL)
      {
        connectedTo=UNKNOWN_DEVICE;
      }

	  std::cout<<"Epiphan Device with the requested location not found. Connected to another device instead."<<std::endl;
    }
  }
  else
  {
	  std::cout<<"Serial Number not specified. Looking for any available device."<<std::endl;
    if ( (this->FrameGrabber = FrmGrabLocal_Open()) == NULL )
    {
      std::cout<<"Epiphan Device Not found"<<std::endl;
      return FAILURE;
    }
  }

  std::cout<<"Connection success."<<std::endl;

  V2U_VideoMode vm;
  if (!FrmGrab_DetectVideoMode((FrmGrabber*)this->FrameGrabber,&vm))
  {
	  std::cout<<"No signal detected"<<std::endl;
    return FAILURE;
  }

  this->MAXIMUM_FREQUENCY = 60;
  FrmGrab_SetMaxFps((FrmGrabber*)this->FrameGrabber,this->MAXIMUM_FREQUENCY);

  //this->MAXIMUM_FREQUENCY = vm.vfreq/1000;
  //if (this->ValidateSpecifiedFrequency(this->GetAcquisitionRate()))
  //{
	 // FrmGrab_SetMaxFps((FrmGrabber*)this->FrameGrabber,this->GetAcquisitionRate());
  //}else
  //{
  //	  FrmGrab_SetMaxFps((FrmGrabber*)this->FrameGrabber,this->MAXIMUM_FREQUENCY);
  //}

  if (vm.width==0 || vm.height==0)
  {
	  std::cout<<"No valid signal detected. Invalid frame size is received from the framegrabber"<<std::endl;
    return FAILURE;
  }

  this->FrameSize[0] = vm.width;
  this->FrameSize[1] = vm.height;

  if( (this->m_ClipRectangleSize[0] > 0) && (this->m_ClipRectangleSize[1] > 0) )
  {
    if (this->m_ClipRectangleSize[0]%4!=0)
    {
		std::cout<<"ClipRectangleSize[0] is not a multiple of 4. Acquired image may be skewed."<<std::endl;
    }
    this->FrameSize[0] = this->m_ClipRectangleSize[0];
    this->FrameSize[1] = this->m_ClipRectangleSize[1];
  }

  std::cout<<"Frame Size: "<<this->FrameSize[0]<<"x"<<this->FrameSize[1]<<std::endl;
  
  std::cout<<"Request Open Success."<<std::endl;


  return SUCCESS;

}

/** Verify imager tool information. */
EpiphanVideoImager::ResultType
EpiphanVideoImager
::VerifyVideoImagerToolInformation( const VideoImagerToolType * imagerTool )
{
  igstkLogMacro( DEBUG, "igstk::EpiphanVideoImager"
                          << "::VerifyVideoImagerToolInformation called ...\n");

  return SUCCESS;
}

/** Detach camera. */
EpiphanVideoImager::ResultType
                                  EpiphanVideoImager::InternalClose( void )
{
  igstkLogMacro( DEBUG,
                  "igstk::EpiphanVideoImager::InternalClose called ...\n");

  /*
    Close the device
    This invalidates the handle
  */

  if( this->Recording )
  {
	  if( this->InternalStopImaging() != SUCCESS )
    {
		std::cout<<"Unable to stop recording."<<std::endl;
    }
  }

  if (this->FrameGrabber != NULL) {
    FrmGrab_Close((FrmGrabber*)this->FrameGrabber);
  }
  this->FrameGrabber = NULL;

  return SUCCESS;
}

/** Put the imaging device into imaging mode. */
EpiphanVideoImager::ResultType
                           EpiphanVideoImager::InternalStartImaging( void )
{
  igstkLogMacro( DEBUG,
    "igstk::EpiphanVideoImager::InternalStartImaging called ...\n");

  if( !FrmGrab_Start((FrmGrabber*)this->FrameGrabber) )
  {
	  std::cout<<"Unable to start frame grabber."<<std::endl;
    return FAILURE;
  }
	
  Recording = true;

  std::cout<<"Start Imaging Success."<<std::endl;

  return SUCCESS;
}

/** Take the imaging device out of imaging mode. */
EpiphanVideoImager::ResultType
                            EpiphanVideoImager::InternalStopImaging( void )
{
  igstkLogMacro( DEBUG,
    "igstk::EpiphanVideoImager::InternalStopImaging called ...\n");
  /*
    Stop the device
  */
  
  FrmGrab_Stop((FrmGrabber*)this->FrameGrabber);
  Recording = false;

  return SUCCESS;
}

/** Reset the imaging device to put it back to its original state. */
EpiphanVideoImager::ResultType
                                  EpiphanVideoImager::InternalReset( void )
{
  igstkLogMacro( DEBUG,
                  "igstk::EpiphanVideoImager::InternalReset called ...\n");
  return SUCCESS;
}

/** Update the status and the transforms for all VideoImagerTools. */
EpiphanVideoImager::ResultType
                                 EpiphanVideoImager::InternalUpdateStatus()
{
  igstkLogMacro( DEBUG,
    "igstk::EpiphanVideoImager::InternalUpdateStatus called ...\n");

  // This method and the InternalThreadedUpdateStatus are both called
  // continuously in the Imaging state.  This method is called from
  // the main thread, while InternalThreadedUpdateStatus is called
  // from the thread that actually communicates with the device.
  // A shared memory buffer is used to transfer information between
  // the threads, and it must be locked when either thread is
  // accessing it.
  m_BufferLock->Lock();

  typedef VideoImagerToolFrameContainerType::const_iterator  InputConstIterator;

  InputConstIterator inputItr = this->m_ToolFrameBuffer.begin();
  InputConstIterator inputEnd = this->m_ToolFrameBuffer.end();

  VideoImagerToolsContainerType imagerToolContainer =
  this->GetVideoImagerToolContainer();

  unsigned int toolId = 0;

  while( inputItr != inputEnd )
    {
    // only report tools that have useful data
    if (! this->m_ToolStatusContainer[inputItr->first])
    {
      igstkLogMacro( DEBUG, "igstk::EpiphanVideoImager"
                         << "::InternalUpdateStatus: " <<
                            "tool " << inputItr->first << " is not in view\n");
      // report to the imager tool that the imager is not available
      this->ReportImagingToolNotAvailable(
        imagerToolContainer[inputItr->first]);

      ++inputItr;
      continue;
    }

    // report to the imager tool that the tool is sending frames
    this->ReportImagingToolStreaming(imagerToolContainer[inputItr->first]);

    this->SetVideoImagerToolFrame(
          imagerToolContainer[inputItr->first], (inputItr->second) );

    this->SetVideoImagerToolUpdate(
      imagerToolContainer[inputItr->first], true );

    ++inputItr;
    ++toolId;
    }

  m_BufferLock->Unlock();

  return SUCCESS;
}

/** Update the shared memory buffer and the tool's internal frame. This function
 *  is called by the thread that communicates with the imager while
 *  the imager is in the Imaging state. */
EpiphanVideoImager::ResultType
EpiphanVideoImager::InternalThreadedUpdateStatus( void )
{
  igstkLogMacro( DEBUG,
   "igstk::EpiphanVideoImager::InternalThreadedUpdateStatus called ...\n");

   if (!this->Recording)
  {
    // drop the frame, we are not recording data now
	std::cout<<"Not Recording."<<std::endl;
    return SUCCESS;
  }
  
  // Lock the buffer that this method shares with InternalUpdateStatus
  m_BufferLock->Lock();

  //reset the status of all the imager tools
  typedef VideoImagerToolFrameContainerType::const_iterator  InputConstIterator;
  InputConstIterator inputItr = this->m_ToolFrameBuffer.begin();
  InputConstIterator inputEnd = this->m_ToolFrameBuffer.end();

  while( inputItr != inputEnd )
  {
    this->m_ToolStatusContainer[inputItr->first] = 0;
    ++inputItr;
  }

  try
  {
    igstkLogMacro( DEBUG, "InternalThreadedUpdateStatus Receive passed" );
    
	V2U_GrabFrame2 * frameBuffer = NULL;

    V2U_UINT32 videoFormat=V2U_GRABFRAME_FORMAT_Y8;

    V2URect *cropRect=NULL;
      cropRect=new V2URect;
      cropRect->x = 0;
      cropRect->y = 0;
	  cropRect->width = this->FrameSize[0];
	  cropRect->height = this->FrameSize[1];

    frameBuffer = FrmGrab_Frame((FrmGrabber*)this->FrameGrabber, videoFormat, cropRect);

	// Check if an imager tool was added with this device name
    typedef VideoImagerToolFrameContainerType::iterator InputIterator;

    //TODO toolname hard coded
    InputIterator deviceItr = this->m_ToolFrameBuffer.find( "Frame grabber" );

    if( deviceItr != this->m_ToolFrameBuffer.end() )
    {      // create the frame
      VideoImagerToolsContainerType imagerToolContainer =
                                           this->GetVideoImagerToolContainer();
      
	  FrameType* frame = new FrameType();
      frame = this->GetVideoImagerToolFrame(
                                        imagerToolContainer[deviceItr->first]);

      unsigned int frameDims[3];
      imagerToolContainer[deviceItr->first]->GetFrameDimensions(frameDims);
 
      EpiphanVideoImager::m_FrameBufferLock->Lock();
	  
	  if( frameBuffer != NULL)
      {
	    int numberOfBytesToSkip;
	    numberOfBytesToSkip = 0;

	    unsigned char* byteImageDataPtr=(unsigned char*)frameBuffer->pixbuf;
	
        memcpy(frame->GetImagePtr(),
			frameBuffer->pixbuf,
                frameDims[0]*frameDims[1]*frameDims[2]);

		/*FILE * out = fopen("C:/Users/Fabian/Desktop/hola.jpeg", "wb");
        if (out) {
          V2U_SAVE_PROC save_image = v2u_write_jpeg;

		  save_image(out, frameBuffer->crop.width, frameBuffer->crop.height,
                                frameBuffer->palette, frameBuffer->pixbuf);
		}*/
      }

	  FrmGrab_Release((FrmGrabber*)this->FrameGrabber, frameBuffer);

      EpiphanVideoImager::m_FrameBufferLock->Unlock();

     //update frame validity time
     frame->SetTimeToExpiration(this->GetValidityTime());

     this->m_ToolFrameBuffer[ deviceItr->first ] = frame;
     this->m_ToolStatusContainer[ deviceItr->first ] = 1;
     }
     m_BufferLock->Unlock();

     return SUCCESS;
  }
  catch(...)
  {
    igstkLogMacro( CRITICAL, "Unknown error catched" );
	std::cout<<"Catch ";
    m_BufferLock->Unlock();
    return FAILURE;
  }

}

EpiphanVideoImager::ResultType
EpiphanVideoImager::
AddVideoImagerToolToInternalDataContainers( const VideoImagerToolType * imagerTool )
{
  igstkLogMacro( DEBUG,
    "igstk::EpiphanVideoImager::RemoveVideoImagerToolFromInternalDataContainers "
                 "called ...\n");

  if ( imagerTool == NULL )
  {
    return FAILURE;
  }

  const std::string imagerToolIdentifier =
                  imagerTool->GetVideoImagerToolIdentifier();

  igstk::Frame* frame = new igstk::Frame;;

  this->m_ToolFrameBuffer[ imagerToolIdentifier ] = frame;
  this->m_ToolStatusContainer[ imagerToolIdentifier ] = 0;

  return SUCCESS;
}


EpiphanVideoImager::ResultType
EpiphanVideoImager::
RemoveVideoImagerToolFromInternalDataContainers
( const VideoImagerToolType * imagerTool )
{
  igstkLogMacro( DEBUG,
    "igstk::EpiphanVideoImager::RemoveVideoImagerToolFromInternalDataContainers "
                 "called ...\n");

  const std::string imagerToolIdentifier =
                      imagerTool->GetVideoImagerToolIdentifier();

  // remove the tool from the frame buffer and status container
  this->m_ToolStatusContainer.erase( imagerToolIdentifier );
  this->m_ToolFrameBuffer.erase( imagerToolIdentifier );

  return SUCCESS;
}

/**The "ValidateSpecifiedFrequency" method checks if the specified
  * frequency is valid for the imaging device that is being used. */
EpiphanVideoImager::ResultType
EpiphanVideoImager::ValidateSpecifiedFrequency( double frequencyInHz )
{

  if ( this->GetAcquisitionRate()<0.0 || this->GetAcquisitionRate()>this->MAXIMUM_FREQUENCY )
  {
    return FAILURE;
  }

  return SUCCESS;
}

/** Print Self function */
void EpiphanVideoImager::PrintSelf( std::ostream& os, itk::Indent indent ) const
{
  Superclass::PrintSelf(os, indent);

  os << indent << " output Imaging Source Converter parameters " << std::endl;
}

void EpiphanVideoImager::setClipRectangleSize(std::vector<int> clipRectangleSize)
{
	this->m_ClipRectangleSize = clipRectangleSize;
}

std::vector<int> EpiphanVideoImager::getClipRectangleSize()
{
	return this->m_ClipRectangleSize;
}

void EpiphanVideoImager::setClipRectangleOrigin(std::vector<int> clipRectangleOrigin)
{
	this->m_ClipRectangleOrigin = clipRectangleOrigin;
}

std::vector<int> EpiphanVideoImager::getClipRectangleOrigin()
{
	return this->m_ClipRectangleOrigin;
}

} // end of namespace igstk
