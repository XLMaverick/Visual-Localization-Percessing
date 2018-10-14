#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

namespace feat
{
	const double EPS = 2.2204e-16;
	struct SBlob
	{
		Point position;
		double value;
		double sigma;
	};
	static bool compareBlob(const SBlob& lhs, const SBlob& rhs);
	Mat getHOGKernel(Size& ksize, double sigma);
	void extBlobFeat(Mat& imgSrc, vector<SBlob>& blobs);


	// edge detection
	enum sobelDirection{SOBEL_HORZ, SOBEL_VERT, SOBEL_BOTH};
	double getSobelEdge(const Mat& imgSrc, Mat& imgDst, double thresh = -1, int direction = SOBEL_BOTH);
	static double getCannyThresh(const Mat& inputArray, double percentage);
	void getCannyEdge(const Mat& imgSrc, Mat& imgDst, double lowThresh = -1, double highThresh = -1, double sigma = 1);


	void detectHarrisCorners(const Mat& imgSrc, Mat& imgDst, double alpha);
	void drawCornerOnImage(Mat& image, const Mat&binary);
	void detectHarrisLaplace(const Mat& imgSrc, Mat& imgDst);
}

void feat::detectHarrisCorners(const Mat& imgSrc, Mat& imgDst, double alpha)
{
	Mat gray;
	if (imgSrc.channels() == 3)
	{
		cvtColor(imgSrc, gray, CV_BGR2GRAY);
	}
	else
	{
		gray = imgSrc.clone();
	}
	gray.convertTo(gray, CV_64F);

	Mat xKernel = (Mat_<double>(1,3) << -1, 0, 1);
	Mat yKernel = xKernel.t();

	Mat Ix,Iy;
	filter2D(gray, Ix, CV_64F, xKernel);
	filter2D(gray, Iy, CV_64F, yKernel);

	Mat Ix2,Iy2,Ixy;
	Ix2 = Ix.mul(Ix);
	Iy2 = Iy.mul(Iy);
	Ixy = Ix.mul(Iy);

	Mat gaussKernel = getGaussianKernel(7, 1);
	filter2D(Ix2, Ix2, CV_64F, gaussKernel);
	filter2D(Iy2, Iy2, CV_64F, gaussKernel);
	filter2D(Ixy, Ixy, CV_64F, gaussKernel);
	

	Mat cornerStrength(gray.size(), gray.type());
	for (int i = 0; i < gray.rows; i++)
	{
		for (int j = 0; j < gray.cols; j++)
		{
			double det_m = Ix2.at<double>(i,j) * Iy2.at<double>(i,j) - Ixy.at<double>(i,j) * Ixy.at<double>(i,j);
			double trace_m = Ix2.at<double>(i,j) + Iy2.at<double>(i,j);
			cornerStrength.at<double>(i,j) = det_m - alpha * trace_m *trace_m;
		}
	}
	// threshold
	double maxStrength;
	minMaxLoc(cornerStrength, NULL, &maxStrength, NULL, NULL);
	Mat dilated;
	Mat localMax;
	dilate(cornerStrength, dilated, Mat());
	compare(cornerStrength, dilated, localMax, CMP_EQ);
	

	Mat cornerMap;
	double qualityLevel = 0.01;
	double thresh = qualityLevel * maxStrength;
	cornerMap = cornerStrength > thresh;
	bitwise_and(cornerMap, localMax, cornerMap);
	
	imgDst = cornerMap.clone();
	
}

void feat::drawCornerOnImage(Mat& image, const Mat& binary)
{
    Mat_<uchar>::const_iterator it = binary.begin<uchar>();
    Mat_<uchar>::const_iterator itd = binary.end<uchar>();
    for (int i = 0; it != itd; it++, i++)
    {
        if (*it)
            circle(image, Point(i%image.cols, i / image.cols), 3, Scalar(0, 255, 0), 1);
    }
}

int main(int argc, char** argv )
{
	Mat image;
	Mat cornerMap;
	image = imread("../test1.bmp",IMREAD_GRAYSCALE );
	if ( !image.data )
	{
		printf("No image data \n");
		printf("Please check the path and name of image_origianl \n");
		return -1;
	}

	feat::detectHarrisCorners(image, cornerMap,0.001);
	feat::drawCornerOnImage(image, cornerMap);

	namedWindow("Display Image", WINDOW_AUTOSIZE );
	imshow("Display Image", cornerMap);
	waitKey(0);
    
	return 0;
}
