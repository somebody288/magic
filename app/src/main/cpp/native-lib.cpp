#include <jni.h>
#include <string>
#include <iostream>
#include <altivec.h>
#include "utils/utils.hpp"

void binaryImg(Mat &src, Mat &binaryImage);
void clipImg(Mat &src,Mat &dist);
void parseLine(Mat &src);
void rotateImg(Mat &src,Mat &dst,double angle);

void analysis(std::vector<int> tmpCols);

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_MainActivity_coverImg2Gray(
        JNIEnv* env,
        jobject /* this */,
        jobject bitmap) {


    cv::Mat imageSource,clipImage,binaryImage,rotateImage,grayImage;
    BitmapToMat(env,bitmap,imageSource, JNI_FALSE);

    //裁剪
    clipImg(imageSource,clipImage);
    //二值化
    binaryImg(clipImage,binaryImage);
    //解析
    rotateImg(binaryImage,rotateImage ,-1.8);
//    cvtColor(rotateImage, grayImage, COLOR_BGR2GRAY);
    parseLine(rotateImage);
    MatToBitmap(env,rotateImage,bitmap, JNI_FALSE);
}

/**
 * 裁剪
 * @param grayImage
 * @return
 */
void clipImg(Mat &src,Mat &dist) {
    dist = src;
//    Mat imgGray,sobelX,sobelXAbs,blurredSobelXAbs,threshSobelX,sobelY,
//        sobelYAbs,blurredSobelYAbs,threshSobelY,sobelXY,thresh,blurred,closed;
//    cvtColor(src, imgGray, COLOR_BGR2GRAY);
//    Sobel(imgGray,sobelX,CV_32F,1,0,-1);
//    convertScaleAbs(sobelX,sobelXAbs);
//    cv::blur(sobelXAbs,blurredSobelXAbs,cv::Size(3,3));
//    threshold(blurredSobelXAbs,threshSobelX,0,255,THRESH_BINARY+THRESH_OTSU);
//
//    Sobel(imgGray,sobelY,CV_32F,0,1,-1);
//    convertScaleAbs(sobelY,sobelYAbs);
//    cv::blur(sobelYAbs,blurredSobelYAbs,cv::Size(3,3));
//    threshold(blurredSobelYAbs,threshSobelY,0,255,THRESH_BINARY+THRESH_OTSU);
//    bitwise_and(threshSobelX,threshSobelY,sobelXY);
//    thresh = sobelXY;
//    blur(sobelXY,blurred,Size(7,7));
//    threshold(blurred,thresh,0,255,THRESH_BINARY+THRESH_OTSU);
//    Mat kernel = getStructuringElement(MORPH_RECT,Size(51,51));
//    morphologyEx(thresh,closed,MORPH_CLOSE,kernel);
//    kernel = Mat::ones(Size(15,15),CV_8U);
//    erode(closed,closed,kernel,Point(-1,-1),5);
//    dilate(closed,closed,kernel,Point(-1,-1),5);
//    std::vector<std::vector<Point>> contours;
//    findContours(closed,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
//    if (contours.size() == 0){
//        return;
//    }
//    std::max(contours,contourArea);

}

void binaryImg(Mat &src,Mat &binaryImage) {
    Mat hsv;
    std::vector<Mat> channels;
    cvtColor(src,hsv,COLOR_BGR2HSV);
    split(hsv,channels);
    threshold(channels.at(2),binaryImage,100,255,THRESH_BINARY);
}

void rotateImg(Mat &image,Mat &dst, double angle) {
    /*对旋转的进行改进，由于图形是一个矩形，旋转后的新图像的形状是一个原图像的外接矩形因此需要重新计算出旋转后的图形的宽和高*/
    int width = image.cols;
    int height = image.rows;
    double radian = angle * CV_PI / 180.;
    // 角度转换为弧度
    double width_rotate = fabs(width * cos(radian)) + fabs(height * sin(radian));
    double height_rotate = fabs(width * sin(radian)) + fabs(height * cos(radian));//旋转中心 原图像中心点
    cv::Point2f center((float) width / 2.0, (float) height / 2.0);//旋转矩阵
    Mat m1 = cv::getRotationMatrix2D(center, angle,1.0);
    // m1为2行3列通道数为1的矩阵
    // 变换矩阵的中心点相当于平移一样 原图像的中心点与新图像的中心点的相对位置
    m1.at<double>(0, 2) += (width_rotate - width) / 2.;
    m1.at<double>(1, 2) += (height_rotate - height) / 2.;
    cv::warpAffine(image, dst, m1, cv::Size(width_rotate, height_rotate), cv::INTER_LINEAR,0,Scalar(255));
    resize(dst,dst,Size(width,height));
}

void parseLine(Mat &src) {
    int rows = src.rows;
    int cols = src.cols;

    for (int row = 0; row < rows; row++) {
        std::vector<int> tmpCols;
        for (int col = 0; col < cols; col++) {
            int tmpPv = src.at<uchar>(row, col);
            tmpCols.push_back(tmpPv);
        }
        analysis(tmpCols);
    }
}

void analysis(std::vector<int> tmpCols){
    int count=1;
    int upVal = -1;
    std::string ma;
    for (int i = 0; i < tmpCols.size(); i++) {
        int tmpPx = tmpCols[i];
        if (upVal == -1){
            upVal = tmpPx;
            continue;
        } else {
            if (upVal == tmpPx) {
                upVal = tmpPx;
                count ++;
                if (count == 50) {
                    if (tmpPx == 255) {
                        ma = ma +"0";
                    }
                    if (tmpPx == 0){
                        ma = ma +"1";
                    }
                    count = 1;
                }
            } else {
                upVal = -1;
                count = 1;
            }
        }
    }
    LOGI("---");
}
