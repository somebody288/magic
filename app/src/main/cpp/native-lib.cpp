#include <jni.h>
#include <string>
#include "utils/utils.hpp"


void binaryImg(Mat &src, Mat &binaryImage);
void clipImg(Mat &grayImage,Mat &dist);
void parseLine(Mat &src);
void rotateImg(Mat &src,Mat &dst,double angle);

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
    cvtColor(imageSource, grayImage, COLOR_BGR2GRAY );
    //裁剪
    clipImg(grayImage,clipImage);
    //二值化
    binaryImg(clipImage,binaryImage);
    //解析
    rotateImg(binaryImage,rotateImage ,-1.8);
    parseLine(rotateImage);
    MatToBitmap(env,rotateImage,bitmap, JNI_FALSE);
}

/**
 * 裁剪
 * @param grayImage
 * @return
 */
void clipImg(Mat &grayImage,Mat &dist) {
    dist = grayImage;
}

void binaryImg(Mat &src,Mat &binaryImage) {
//    Mat hsv,splitImg;
//    cvtColor(src,hsv,COLOR_BGR2HSV);
//    split(hsv,splitImg);
    threshold(src,binaryImage,100,255,THRESH_BINARY);

//    int rows = src.rows;
//    int cols = src.cols;
//    int dims = src.channels();
//    int tempIndex = 5;
//    for (int row = 0; row < rows; row++) {
//        for (int col = 0; col < cols; col++) {
//            if (dims == 1) {
//                int sum = 0;
//                for (int i = 0; i < tempIndex; i++) {
//                    if (col < cols) {
//                        int tmpPv = src.at<uchar>(row, col + i);
//                        sum = sum + tmpPv;
//                    } else {
//                        break;
//                    }
//                }
//                double avg = sum / tempIndex;
//                int value ;
//                if (avg < 127) { // 变成黑色
//                    value = 0;
//                } else {
//                    value = 255;
//                }
//                src.at<uchar>(row, col) = value;
////                LOGI("单通道 %d",  src.at<uchar>(row, col));
//            }
////            if (dims == 3) {
////                Vec3b bgr = src.at<Vec3b>(row, col);
////                LOGI("单通道 b->%s,r->%s,g->%s",bgr[0],bgr[1],bgr[2]);
////            }
//        }
//    }
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
    int dims = src.channels();
    int tempIndex = 24;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; ) {
//            if (dims == 1) {
//                int sum = 0;
//                for (int i = 0; i < tempIndex; i++) {
//                    if (col < cols) {
//                        int tmpPv = src.at<uchar>(row, col + i);
//                        sum = sum + tmpPv;
//                    } else {
//                        break;
//                    }
//                }
//                if (sum == 0) {
//                    LOGI(" 标识->1");
//                } else {
//                    LOGI(" 标识->0");
//                }
//                col += tempIndex;
//            }
        }
        LOGI("-------------------------");
    }
}
