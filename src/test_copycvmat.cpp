#include<iostream>
#include<cv.h>
#include<highgui.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <Python.h>
#include <boost/python.hpp>

namespace bp = boost::python;

int main() {
    // 初始化Python解释器
    Py_Initialize();

    // 导入SciPy模块
    bp::object scipy = bp::import("scipy");

    // 调用SciPy模块中的函数
    bp::object result = scipy.attr("function_name")(arguments);

    // 从Python对象中获取结果
    // ...

    // 关闭Python解释器
    Py_Finalize();

    return 0;
}



//#include<stdio.h>
//#include<opencv2/opencv.hpp>
//
// using namespace cv;
// using namespace std;
///*************************************************
////  Method:    convertTo3Channels
////  Description: 将单通道图像转为三通道图像
////  Returns:   cv::Mat
////  Parameter: binImg 单通道图像对象
//*************************************************/
//Mat convertTo3Channels(const Mat& binImg)
//{
//    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
//    vector<Mat> channels;
//    for (int i=0;i<3;i++)
//    {
//        channels.push_back(binImg);
//    }
//    merge(channels,three_channel);
//    return three_channel;
//}
//
//Mat convertTo3Channels2(const Mat& binImg)
//{
//    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
//    for (int i=0;i<3;i++)
//    {
//        three_channel.push_back(binImg);
//        //channels.push_back(binImg);
//    }
//    return three_channel;
//}
//
// int main(int argc, char* argv[]){
//
//    cv::Mat srcMat(3, 3, CV_8UC1);
//    srcMat.at<uchar>(0, 0) = 1;
//    srcMat.at<uchar>(0, 1) = 2;
//    srcMat.at<uchar>(0, 2) = 3;
//    srcMat.at<uchar>(1, 0) = 4;
//    srcMat.at<uchar>(1, 1) = 5;
//    srcMat.at<uchar>(1, 2) = 6;
//    srcMat.at<uchar>(2, 0) = 7;
//    srcMat.at<uchar>(2, 1) = 8;
//    srcMat.at<uchar>(2, 2) = 9;
//    std::cout<<"1: "<<srcMat<<std::endl;
//
//    cv::Mat second_Mat = convertTo3Channels2(srcMat);
//    std::cout<<"2: "<<second_Mat<<std::endl;
//
//    return 0;
//
// }
//
//
//
////
////using namespace std;
//////打印8位无符号数据
////void PrintImage8U(IplImage* image)
////{
////
////	 for(int j=0;j<image->height;j++)
////	 {
////		unsigned char * ptr = (unsigned char * )(image->imageData+image->widthStep*j);  //行首指针
////		 for(int i=0;i<image->width;i++)
////		 {
////			 unsigned char v = ptr[i];
////			 cout<<(int)v<<"\t";
////		 }
////		 cout<<endl;
////	 }
////
////}
//////打印32位浮点数数据
////void PrintImage32F(IplImage* image)
////{
////
////	 for(int j=0;j<image->height;j++)
////	 {
////		 float * ptr = (float * )(image->imageData+image->widthStep*j);  //行首指针
////		 for(int i=0;i<image->width;i++)
////		 {
////			 float v = ptr[i];
////			 cout<<(float)v<<"\t";
////		 }
////		 cout<<endl;
////	 }
////
////}
////
//////测试距离函数
////void TestDist()
////{
////	//创建矩阵
////	IplImage* src = cvCreateImage(cvSize(9,9),IPL_DEPTH_8U,1);
////	IplImage* dst = cvCreateImage(cvSize(9,9),IPL_DEPTH_32F,1);
////	unsigned char * ptr = (unsigned char * )(src->imageData);  //数据指针
////	*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0; *ptr++ = 0;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0; *ptr++ = 0;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255; *ptr++ = 0;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255;*ptr++ = 255; *ptr++ = 0;*ptr++ = 0;ptr +=3;
////    *ptr++ = 0;*ptr++ = 0;*ptr++ = 255;*ptr++ = 0;*ptr++ = 255;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0; ptr +=3;
////    *ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;*ptr++ = 0;
////
////	//计算距离  CV_DIST_L1计算出来的距离是精确的
////	//cvDistTransform( src, dst, CV_DIST_L1, 3, NULL, NULL );
////	cvDistTransform( src, dst, cv::DIST_L2, 3, NULL, NULL );
////
////	cout<<"变换前"<<endl;
////	PrintImage8U(src);
////	cout<<"变换后"<<endl;
////	PrintImage32F(dst);
////	cvReleaseImage(&src); //释放数据
////	cvReleaseImage(&dst); //释放数据
////}
////
////
////
////int main()
////{
////	TestDist();
////	return 0;
////}