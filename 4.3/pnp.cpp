#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    ifstream cameraMatrixFile("camera_matrix.txt");
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cameraMatrixFile >> cameraMatrix.at<double>(i, j);
        }
    }

    ifstream distCoeffsFile("distortion_matrix.txt");
    Mat distCoeffs = Mat::zeros(1, 5, CV_64F);
    for (int i = 0; i < 5; i++) {
        distCoeffsFile >> distCoeffs.at<double>(0, i);
    }

    ifstream objectPointsFile("object_points.txt");
    vector<Point3f> objectPoints;
    double x, y, z;
    while (objectPointsFile >> x >> y >> z) {
        objectPoints.push_back(Point3f(x, y, z));
    }

    ifstream imagePointsFile("image_points.txt");
    vector<Point2f> imagePoints;
    double u, v;
    while (imagePointsFile >> u >> v) {
        imagePoints.push_back(Point2f(u, v));
    }

    Mat rvec, tvec;
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    Mat translation = tvec;
    Mat rotation;
    Rodrigues(rvec, rotation);

    double distance = norm(translation) / 100.0;

    cout << "平移向量：" << endl << translation << endl;
    cout << "旋转矩阵：" << endl << rotation << endl;
    cout << "机器人距离摄像头的距离：" << distance << " 米" << endl;

    return 0;
}
