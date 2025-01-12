#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    Mat targetImage = imread("2_2.jpeg", IMREAD_GRAYSCALE);
    vector<Mat> templates;
    for (int i = 0; i < 10; ++i) {
        string filename = to_string(i) + ".JPG";
        Mat templateImage = imread(filename, IMREAD_GRAYSCALE);
        templates.push_back(templateImage);
    }

    // 计算目标图像的Hu矩
    Moments targetMoments = moments(targetImage, true);
    double targetHuMoments[7];
    HuMoments(targetMoments, targetHuMoments);

    // 计算模板图像的Hu矩并进行匹配
    double minDiff = DBL_MAX;
    int detectedNumber = -1;
    for (int i = 0; i < templates.size(); ++i) {
        Moments templateMoments = moments(templates[i], true);
        double templateHuMoments[7];
        HuMoments(templateMoments, templateHuMoments);

        // 计算Hu矩的差异
        double diff = 0;
        for (int j = 0; j < 7; ++j) {
            diff += abs(targetHuMoments[j] - templateHuMoments[j]);
        }

        // 更新最小差异和检测到的数字
        if (diff < minDiff) {
            minDiff = diff;
            detectedNumber = i;
        }
    }

    // 设置一个阈值来判断匹配是否成功
    double threshold = 0.6;
    if (minDiff < threshold) {
        cout << "Detected number: " << detectedNumber << endl;
    } else {
        cout << "No number detected." << endl;
    }

    return 0;
}
