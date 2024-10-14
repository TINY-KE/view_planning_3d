#include<iostream>
#include<cv.h>
#include<highgui.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include<stdio.h>
#include<opencv2/opencv.hpp>

 using namespace cv;
 using namespace std;

 int main(int argc, char* argv[]){

    Mat src = imread("/home/zhjd/20181008153451124.jpg");

    resize(src, src, Size(), 0.25, 0.25, 1);
    imshow("src", src);

    Mat bin;
    cvtColor(src, bin, CV_BGR2GRAY);
    threshold(bin, bin, 80, 255, CV_THRESH_BINARY);
    imshow("bin", bin);

    Mat Dist, Labels;
    distanceTransform(bin, Dist, CV_DIST_L1, 3);
    normalize(Dist, Dist, 0, 1, NORM_MINMAX);
    imshow("dist1", Dist);


    distanceTransform(bin, Dist, CV_DIST_L2, 3);
    normalize(Dist, Dist, 0, 1, NORM_MINMAX);
    imshow("dist2", Dist);

    //distanceTransform(bin, Dist, Labels, CV_DIST_L1, 3, DIST_LABEL_CCOMP);
    //normalize(Dist, Dist, 0, 1, NORM_MINMAX);
    //imshow("dist2", Dist);
    //imshow("labels2", Labels);
     //
    //distanceTransform(bin, Dist, Labels, CV_DIST_L1, 3, DIST_LABEL_PIXEL);
    //normalize(Dist, Dist, 0, 1, NORM_MINMAX);
    ////normalize(Labels, Labels, 0, 255, NORM_MINMAX);
    //imshow("dist3", Dist);
    //imshow("labels3", Labels);

    waitKey();
    return 0;

 }



//
//using namespace std;
////打印8位无符号数据
//void PrintImage8U(IplImage* image)
//{
//
//	 for(int j=0;j<image->height;j++)
//	 {
//		unsigned char * ptr = (unsigned char * )(image->imageData+image->widthStep*j);  //行首指针
//		 for(int i=0;i<image->width;i++)
//		 {
//			 unsigned char v = ptr[i];
//			 cout<<(int)v<<"\t";
//		 }
//		 cout<<endl;
//	 }
//
//}
////打印32位浮点数数据
//void PrintImage32F(IplImage* image)
//{
//
//	 for(int j=0;j<image->height;j++)
//	 {
//		 float * ptr = (float * )(image->imageData+image->widthStep*j);  //行首指针
//		 for(int i=0;i<image->width;i++)
//		 {
//			 float v = ptr[i];
//			 cout<<(float)v<<"\t";
//		 }
//		 cout<<endl;
//	 }
//
//}
//
////测试距离函数
//void TestDist()
//{
//	//创建矩阵
//	IplImage* src = cvCreateImage(cvSize(9,9),IPL_DEPTH_8U,1);
//	IplImage* dst = cvCreateImage(cvSize(9,9),IPL_DEPTH_32F,1);
//	unsigned char * ptr = (unsigned char * )(src->imageData);  //数据指针
//	*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0; *ptr++ = 0;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0; *ptr++ = 0;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255; *ptr++ = 0;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255; *ptr++ = 0;*ptr++ = 0;ptr +=3;
//    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 0;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0; ptr +=3;
//    *ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;
//
//	//计算距离  CV_DIST_L1计算出来的距离是精确的
//	//cvDistTransform( src, dst, CV_DIST_L1, 3, NULL, NULL );
//	cvDistTransform( src, dst, cv::DIST_L2, 3, NULL, NULL );
//
//	cout<<"变换前"<<endl;
//	PrintImage8U(src);
//	cout<<"变换后"<<endl;
//	PrintImage32F(dst);
//	cvReleaseImage(&src); //释放数据
//	cvReleaseImage(&dst); //释放数据
//}
//
//
//
//int main()
//{
//	TestDist();
//	return 0;
//}