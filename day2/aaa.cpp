#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int main()
{
    // 相机内参矩阵
    Mat camera_matrix = (Mat_<double>(3, 3) << 5.711696077477573e+03, 0, 1485.80898385214,
                                                      0, 5.711569629386420e+03, 1015.10525161836,
                                                      0, 0, 1);

    // 相机到平面的距离（单位：米）
    double distance = 0.212;

    // 读取图像
    Mat src = imread("4.bmp");
    resize(src, src, Size(), 0.3, 0.3);

    Mat src1 = src.clone();
    if (src.empty()) {
        cerr << "Error: Could not load image." << endl;
        return -1;
    }

    // 高斯模糊
    GaussianBlur(src, src, Size(19, 19), 1.5);

    // 转换为灰度图像
    cvtColor(src, src, COLOR_BGR2GRAY);

    // 二值化
    threshold(src, src, 128, 255, THRESH_BINARY_INV);

    // 形态学操作
    Mat element = getStructuringElement(MORPH_RECT, Size(9, 9));
    erode(src, src, element);
    bitwise_not(src, src);
    erode(src, src, element);
    bitwise_not(src, src);

    // 显示预处理后的图像
    imshow("Preprocessed Image", src);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 定义角落区域的大小（像素）
    int corner_size = 50;

    // 绘制轮廓
    Mat contourImage = src1.clone(); // 使用原始图像的副本
    for (int i = 0; i < contours.size(); i++) {
        // 计算轮廓的中心点
        Moments moments = cv::moments(contours[i]);
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;

        // 检查中心点是否位于角落区域
        bool is_corner = (cx < corner_size || cx > src.cols - corner_size) && (cy < corner_size || cy > src.rows - corner_size);
        if (is_corner) {
            continue; // 跳过角落区域的轮廓
        }

        // 绘制轮廓
        drawContours(contourImage, contours, i, Scalar(0, 255, 0), 2); // 用绿色绘制轮廓

        // 寻找最小外接圆
        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);

        // 计算像素到厘米的转换系数
        double focal_length = camera_matrix.at<double>(0, 0); // 焦距（像素）
        double pixel_to_cm = (distance * 100) / focal_length; // 转换系数（像素到厘米）

        // 将直径从像素转换为厘米
        double diameter_cm = 2 * radius * pixel_to_cm;

        // 获取轮廓的右下角坐标，用于放置文本
        int x = boundingRect(contours[i]).x + boundingRect(contours[i]).width;
        int y = boundingRect(contours[i]).y + boundingRect(contours[i]).height;

        // 在轮廓右下角标出直径
        putText(contourImage, to_string(diameter_cm).substr(0, 5) + " cm", Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    }

    imshow("Contours with Diameter in cm (Excluding Corners)", contourImage);
    waitKey(0);
    return 0;
}








