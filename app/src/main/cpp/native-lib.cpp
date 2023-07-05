#include <jni.h>
#include <string>
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
        LOGI("-------------------------");
    }
}

void analysis(std::vector<int> tmpCols){
    // 统一每一个像素点占了多少
    int count=1,val;
    for (int i = 0; i < tmpCols.size(); ++i) {
        int tmpPx = tmpCols[i];
        if (val == tmpPx) {
            count++;
        } else {
            LOGI("px-> %d,%d",val,count);
            val = tmpPx;
            count = 1;
        }
    }

}
