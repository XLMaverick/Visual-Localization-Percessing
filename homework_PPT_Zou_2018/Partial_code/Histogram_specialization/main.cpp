#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;

cv::Mat histogram_specilalization(cv::Mat srcImage, cv::Mat dstImage)
{
	Mat output = srcImage.clone();

	int grayArray[256];
	float srcpdf[256] = {0.0f};
	float dstpdf[256] = {0.0f};
	int gray_tag = 0;
	float gray_sum = 0.0f;

	memset(grayArray,0,sizeof(grayArray));
	for(size_t nrow = 0; nrow < srcImage.rows; nrow++)
	{
		for(size_t ncol = 0;ncol < srcImage.cols; ncol++)
		{
			gray_tag = srcImage.at<uchar>(nrow,ncol);
			grayArray[gray_tag]++;		
		}
	}
	gray_sum = 0.0f;
	for(int i = 0;i < 256;i++)
	{
		gray_sum = gray_sum+grayArray[i];
		srcpdf[i] = gray_sum / (srcImage.rows*srcImage.cols);
		// std::cout<<srcpdf[i]<<std::endl;
	}

	memset(grayArray,0,sizeof(grayArray));
	for(size_t nrow = 0;nrow < dstImage.rows;nrow++)
	{
		for(size_t ncol = 0; ncol<dstImage.cols; ncol++)
		{
			gray_tag = dstImage.at<uchar>(nrow, ncol);
			grayArray[gray_tag]++;
		}
	}
	gray_sum = 0.0f;
	for(int i=0;i < 256;i++)
	{
		gray_sum = gray_sum + grayArray[i];
		dstpdf[i] = gray_sum / (dstImage.rows * dstImage.cols); 
		//std::cout<<dstpdf[i]<<std::endl;
	}

	int histogram_history[256] = {0};
	int minTag = 0;
	for(int i = 0;i<256;i++)
	{
		float minMap = 10.0f;
		for(int j = 0;j<256;j++)
		{
			if(minMap > abs(srcpdf[i]-dstpdf[j]))
			{
				minMap = abs(srcpdf[i]-dstpdf[j]);
				minTag = j;
			}
		}
		histogram_history[i] = minTag;
	}

	for(size_t nrow = 0;nrow<output.rows;nrow++)
	{
		for(size_t ncol = 0; ncol < output.cols; ncol++)
		{
			int temp = output.at<uchar>(nrow,ncol);
			output.at<uchar>(nrow,ncol) = histogram_history[temp];
		}
	}

	return output;
} 

cv::Mat histogram_equalization(cv::Mat srcImage)
{
	
	Mat output = srcImage.clone();

	int grayArray[256];
	float srcpdf[256] = {0.0f};

	int gray_tag = 0;
	float gray_sum = 0.0f;

	memset(grayArray,0,sizeof(grayArray));
	for(size_t nrow = 0; nrow < srcImage.rows; nrow++)
	{
		for(size_t ncol = 0;ncol < srcImage.cols; ncol++)
		{
			gray_tag = srcImage.at<uchar>(nrow,ncol);
			grayArray[gray_tag]++;		
		}
	}
	gray_sum = 0.0f;
	for(int i = 0;i < 256;i++)
	{
		gray_sum = gray_sum+grayArray[i];
		srcpdf[i] = gray_sum / (srcImage.rows*srcImage.cols);
		// std::cout<<srcpdf[i]<<std::endl;
	}

	int histogram_history[256] = {0};
	for(int i = 0;i<256;i++)
	{
		histogram_history[i] = (int)(255*srcpdf[i]+0.5);
	}

	for(size_t nrow = 0;nrow<output.rows;nrow++)
	{
		for(size_t ncol = 0; ncol < output.cols; ncol++)
		{
			int temp = output.at<uchar>(nrow,ncol);
			output.at<uchar>(nrow,ncol) = histogram_history[temp];
		}
	}

	return output;

} 

void histogram_show(cv::Mat srcImage)
{

    int channels = 0;

    MatND dstHist;

    int histSize[] = { 256 };       
   
   
    float midRanges[] = { 0, 256 };
    const float *ranges[] = { midRanges };

    calcHist(&srcImage, 1, &channels, Mat(), dstHist, 1, histSize, ranges, true, false);

    Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);
    double g_dHistMaxValue;
    minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(dstHist.at<float>(i) * 256 * 0.9 / g_dHistMaxValue);

        line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 255, 255));
    }

    imshow("【图片直方图】", drawImage);
}


int main(int argc, char** argv )
{
	Mat image_origianl, image_target, image_adjust;
	image_origianl = imread("../test2.bmp",IMREAD_GRAYSCALE );
	if ( !image_origianl.data )
	{
		printf("No image data \n");
		printf("Please check the path and name of image_origianl \n");
		return -1;
	}
	image_target = imread("../test3.bmp",IMREAD_GRAYSCALE );
	if ( !image_target.data )
	{
		printf("No image data \n");
		printf("Please check the path and name of image_target \n");
		return -1;
	}

	histogram_show(image_origianl);
	image_adjust = histogram_equalization(image_origianl);
	histogram_show(image_adjust);

	namedWindow("Display Image", WINDOW_AUTOSIZE );
	imshow("Display Image", image_adjust);
	waitKey(0);
    
	return 0;
}
