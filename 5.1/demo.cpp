#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;

void readCameraMatrix(const string &filename, Mat &cameraMatrix)
{
    ifstream file(filename);
    if (!file)
    {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            file >> cameraMatrix.at<double>(i, j);
        }
    }

    file.close();
}

void readDistortionMatrix(const string &filename, Mat &distCoeffs)
{
    ifstream file(filename);
    if (!file)
    {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }

    for (int i = 0; i < 5; i++)
    {
        file >> distCoeffs.at<double>(0, i);
    }

    file.close();
}

int main()
{
    Mat img = imread("bubing.png");
    if (img.empty())
    {
        cout << "无法打开或找到图像!" << endl;
        return -1;
    }

    int knSize = 9;
    double sigma = 9.0;
    GaussianBlur(img, img, cv::Size(knSize, knSize), sigma);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));
    morphologyEx(img, img, MORPH_CLOSE, kernel);

    Mat imgHSV, mask;
    int hmin = 0, smin = 0, vmin = 53;
    int hmax = 49, smax = 79, vmax = 255;
    Scalar lower_red(0, 0, 53);
    Scalar upper_red(49, 79, 255);

    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, lower_red, upper_red, mask);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);
        rectangle(img, rect, Scalar(0, 255, 0), 2);
        putText(img, "enermy", rect.tl(), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
    }

    imshow("Mask", mask);
    imshow("Image with Rectangles", img);

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64F);
    readCameraMatrix("camera_matrix.txt", cameraMatrix);
    readDistortionMatrix("distortion_matrix.txt", distCoeffs);

    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(0, 0, 0));
    objectPoints.push_back(Point3f(8, 0, 0));
    objectPoints.push_back(Point3f(8, 4, 0));
    objectPoints.push_back(Point3f(0, 4, 0));

    vector<Point2f> imagePoints;
    if (!contours.empty())
    {
        Rect rect = boundingRect(contours[0]);
        imagePoints.push_back(rect.tl());
        imagePoints.push_back(Point2f(rect.x + rect.width, rect.y));
        imagePoints.push_back(Point2f(rect.x + rect.width, rect.y + rect.height));
        imagePoints.push_back(Point2f(rect.x, rect.y + rect.height));
    }

    Mat rvec, tvec;
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    double distance = norm(tvec);

    cout << "相机和灯条之间的距离: " << distance << " 厘米" << endl;

    waitKey(0);
    return 0;
}






    
