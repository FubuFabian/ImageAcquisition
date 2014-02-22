/* 
 * File:   itkScalarToTextureRLImageFilter.h
 * Author: Fabian
 *
 * Created on July 29, 2013, 4:06 PM
 */

#ifndef ITKSCALARTOTEXTUREHUIMAGEFILTER_H
#define	ITKSCALARTOTEXTUREHUIMAGEFILTER_H

#include "itkImageToImageFilter.h"
#include "itkScalarImageToRunLengthFeaturesFilter.h"
#include <itkImageRegionConstIterator.h>
#include <itkDenseFrequencyContainer2.h>

namespace itk {

/** \class ScalarToTextureHUImageFilter. This class generates a texture image, 
 * by processing the texture value of each pixel using 
 * run length matrices parameters
 */
template<class TInputImage, class TMaskImage = Image<unsigned char, 
  ::itk::GetImageDimension<TInputImage>::ImageDimension>, 
  class TOutputImage = TInputImage>
class ITK_EXPORT ScalarToTextureHUImageFilter :
    public ImageToImageFilter<TInputImage, TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef ScalarToTextureHUImageFilter                      Self;
  typedef ImageToImageFilter<TInputImage, TOutputImage>   Superclass;
  typedef SmartPointer<Self>                              Pointer;
  typedef SmartPointer<const Self>                        ConstPointer;

  /** Standard New method. */
  itkNewMacro( Self );

  /** ImageDimension constants */
  itkStaticConstMacro( ImageDimension, unsigned int,
                       TInputImage::ImageDimension );

  /** Some convenient typedefs. */
  typedef float                                   RealType;
  typedef TInputImage                             InputImageType;
  typedef TMaskImage                              MaskImageType;
  typedef TOutputImage                            OutputImageType;
  typedef itk::Statistics::DenseFrequencyContainer2 HistogramType;
  typedef itk::Statistics::ScalarImageToRunLengthFeaturesFilter
        <InputImageType, HistogramType> RunLengthFilterType;
  typedef typename InputImageType::OffsetType OffsetType;
  
  typedef typename RunLengthFilterType::OffsetVector OffsetVectorType;

  /** Runtime information support. */
  itkTypeMacro( ScalarToTextureHUImageFilter,
                ImageToImageFilter );

  void SetMaskImage( const MaskImageType *mask )
    {
    this->SetNthInput( 1, const_cast<MaskImageType *>( mask ) ); 
    }
  const MaskImageType* GetMaskImage() const
    {
    return static_cast<MaskImageType*>( const_cast<DataObject *>
      ( this->ProcessObject::GetInput( 1 ) ) ); 
    }  
  void SetInput1( const TInputImage *input )
    {
    this->SetInput( input ); 
    }  
  void SetInput2( const TMaskImage *mask )
    {
    this->SetMaskImage( mask );
    }  
  
  typedef ImageRegionConstIterator<InputImageType>
  ImageRegionIteratorType;
  typedef typename ImageRegionIteratorType::SizeType RegionSizeType;
  
  itkSetMacro( RegionSize, RegionSizeType );
  itkGetConstMacro( RegionSize, RegionSizeType );

protected:
  ScalarToTextureHUImageFilter();
  ~ScalarToTextureHUImageFilter() {};
  void PrintSelf( std::ostream& os, Indent indent ) const;

  void GenerateData();
  

private:
  ScalarToTextureHUImageFilter( const Self& ); //purposely not implemented
  void operator=( const Self& ); //purposely not implemented

  RegionSizeType                       m_RegionSize;
  typename MaskImageType::Pointer      m_MaskImage;
  
  

}; // end of class

} // end namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkScalarToTextureHUImageFilter.txx"
#endif

#endif	/* ITKSCALARTOTEXTUREHUIMAGEFILTER_H */

