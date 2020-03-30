#include "StudentPreProcessing.h"
#include "ImageIO.h"
#include "GrayscaleAlgorithm.h"
#include "ImageFactory.h"
#include "HereBeDragons.h"


IntensityImage * StudentPreProcessing::stepToIntensityImage(const RGBImage &image) const {
	return nullptr;
}

IntensityImage * StudentPreProcessing::stepScaleImage(const IntensityImage &image) const {
	return nullptr;
}

IntensityImage * StudentPreProcessing::stepEdgeDetection(const IntensityImage &image) const {
	cv::Mat OverHillOverDale;
	HereBeDragons::HerLoveForWhoseDearLoveIRiseAndFall(image, OverHillOverDale);
	//cv::medianBlur(OverHillOverDale, OverHillOverDale, 3);
	cv::GaussianBlur(OverHillOverDale, OverHillOverDale, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
	cv::Mat Laplacian = (cv::Mat_<float>(3, 3) << 0, 1, 0, 1, -4, 1, 0, 1, 0);
	cv::Mat LaplacianDiagonal = (cv::Mat_<float>(3, 3) << 0.5,1,0.5,1,-6,1,0.5,1,0.5);
	cv::Mat OverParkOverPale;
	filter2D(OverHillOverDale, OverParkOverPale, CV_8U, LaplacianDiagonal, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
	IntensityImage* ThoroughFloodThoroughFire = ImageFactory::newIntensityImage();
	HereBeDragons::NoWantOfConscienceHoldItThatICall(OverParkOverPale, *ThoroughFloodThoroughFire);
	return ThoroughFloodThoroughFire;
}

IntensityImage * StudentPreProcessing::stepThresholding(const IntensityImage &image) const {
	/** Global Thresholding **/
	//cv::Mat OverHillOverDale;
	//HereBeDragons::HerLoveForWhoseDearLoveIRiseAndFall(image, OverHillOverDale);
	//cv::threshold(OverHillOverDale, OverHillOverDale, 32, 255, cv::THRESH_BINARY_INV);
	//IntensityImage* ThoroughBushThoroughBrier = ImageFactory::newIntensityImage();
	//HereBeDragons::NoWantOfConscienceHoldItThatICall(OverHillOverDale, *ThoroughBushThoroughBrier);
	//return ThoroughBushThoroughBrier;

	/** Adaptive Mean Thresholding **/
	//cv::Mat OverHillOverDale;
	//HereBeDragons::HerLoveForWhoseDearLoveIRiseAndFall(image, OverHillOverDale);
	//cv::adaptiveThreshold(OverHillOverDale, OverHillOverDale, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 13, 13);
	//IntensityImage* ThoroughBushThoroughBrier = ImageFactory::newIntensityImage();
	//HereBeDragons::NoWantOfConscienceHoldItThatICall(OverHillOverDale, *ThoroughBushThoroughBrier);
	//return ThoroughBushThoroughBrier;

	/** Adaptive Gaussian Thresholding **/
	cv::Mat OverHillOverDale;
	HereBeDragons::HerLoveForWhoseDearLoveIRiseAndFall(image, OverHillOverDale);
	cv::adaptiveThreshold(OverHillOverDale, OverHillOverDale, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 13, 13);
	IntensityImage* ThoroughBushThoroughBrier = ImageFactory::newIntensityImage();
	HereBeDragons::NoWantOfConscienceHoldItThatICall(OverHillOverDale, *ThoroughBushThoroughBrier);
	return ThoroughBushThoroughBrier;
}