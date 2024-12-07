#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

Mat logT(const Mat& img) {
    Mat logImg = Mat::zeros(img.size(), img.type());
    double c = 255.0 / log(1 + 255);

    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            uchar pixVal = img.at<uchar>(y, x);
            logImg.at<uchar>(y, x) = static_cast<uchar>(c * log(1 + pixVal));
        }
    }
    return logImg;
}

Mat gammaC(const Mat& img, double gamma) {
    Mat gammaImg = Mat::zeros(img.size(), img.type());
    double c = 255.0 / pow(255, gamma);

    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            uchar pixVal = img.at<uchar>(y, x);
            gammaImg.at<uchar>(y, x) = static_cast<uchar>(c * pow(pixVal, gamma));
        }
    }
    return gammaImg;
}

int main() {
    Mat img = imread("fengjing.jpg", IMREAD_GRAYSCALE);
    resize(img, img, Size(), 0.2, 0.2);
    if (img.empty()) {
        cerr << "Could not open or find the image!" << endl;
        return -1;
    }

    Mat logImg = logT(img);

    double gamma = 0.5;
    Mat gammaImg = gammaC(img, gamma);

    imshow("原来", img);
    imshow("对数", logImg);
    imshow("伽马", gammaImg);
    waitKey(0);

    return 0;
}


