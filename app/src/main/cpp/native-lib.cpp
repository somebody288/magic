#include <jni.h>
#include <string>
#include "utils/utils.hpp"


void binaryImage(Mat &src);
void clip(Mat &grayImage,Mat &dist);
void parseLine(Mat &src);

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
    //源图像
    Mat src,grayImage,distClipMat;
    //将Bitmap转换为Mat
    BitmapToMat(env,bitmap,src, JNI_FALSE);

    cvtColor(src, grayImage, COLOR_BGR2GRAY );
    clip(grayImage,distClipMat);
    binaryImage(distClipMat);
    parseLine(distClipMat);
    //将Mat转换为Bitmap
    MatToBitmap(env,distClipMat,bitmap, JNI_FALSE);

    //释放Mat
    src.release();
    grayImage.release();
    distClipMat.release();

}

/**
 * 裁剪
 * @param grayImage
 * @return
 */
void clip(Mat &grayImage,Mat &dist) {
    dist = grayImage;
}

void binaryImage(Mat &src) {
    int rows = src.rows;
    int cols = src.cols;
    int dims = src.channels();
    int tempIndex = 5;
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (dims == 1) {
                int sum = 0;
                for (int i = 0; i < tempIndex; i++) {
                    if (col < cols) {
                        int tmpPv = src.at<uchar>(row, col + i);
                        sum = sum + tmpPv;
                    } else {
                        break;
                    }
                }
                double avg = sum / tempIndex;
                int value ;
                if (avg < 127) { // 变成黑色
                    value = 0;
                } else {
                    value = 255;
                }
                src.at<uchar>(row, col) = value;
//                LOGI("单通道 %d",  src.at<uchar>(row, col));
            }
//            if (dims == 3) {
//                Vec3b bgr = src.at<Vec3b>(row, col);
//                LOGI("单通道 b->%s,r->%s,g->%s",bgr[0],bgr[1],bgr[2]);
//            }
        }
    }
}

void parseLine(Mat &src) {
//    Mat dist ;
//    Canny(src,dist, 40, 255, 255)

    int rows = src.rows;
    int cols = src.cols;
    int dims = src.channels();
    int tempIndex = 24;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; ) {
            if (dims == 1) {
                int sum = 0;
                for (int i = 0; i < tempIndex; i++) {
                    if (col < cols) {
                        int tmpPv = src.at<uchar>(row, col + i);
                        sum = sum + tmpPv;
                    } else {
                        break;
                    }
                }
                if (sum == 0) {
                    LOGI(" 标识->1");
                } else {
                    LOGI(" 标识->0");
                }
                col += tempIndex;
            }
        }
        LOGI("-------------------------");
    }
}
