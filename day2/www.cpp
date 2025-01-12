#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    Mat targetImage = imread("2.jpeg", IMREAD_GRAYSCALE);
    vector<Mat> templates;

    for (int i = 0; i < 10; ++i) {
        string filename = to_string(i) + ".JPG";
        Mat templateImage = imread(filename, IMREAD_GRAYSCALE);
        templates.push_back(templateImage);
    }

    Mat result;
    double maxVal = 0;
    int detectedNumber = -1;
    int matchMethod = TM_SQDIFF;
    double threshold = 0.8;

    for (int i = 0; i < templates.size(); ++i) {
        matchTemplate(targetImage, templates[i], result, matchMethod);
        double minVal, tempMaxVal;
        Point minLoc, maxLoc;
        minMaxLoc(result, &minVal, &tempMaxVal, &minLoc, &maxLoc);

        if (tempMaxVal > maxVal) {
            maxVal = tempMaxVal;
            detectedNumber = i;
        }
    }

    if (detectedNumber != -1) {
        cout << "Detected number: " << detectedNumber << endl;
    } else {
        cout << "No number detected." << endl;
    }

    return 0;
}



