#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

int main()
{
    ifstream fin("calibdata.txt");
    if (!fin)
    {
        cerr << "无法打开文件 calibdata.txt" << endl;
        return -1;
    }

    ofstream fout("calibration_result.txt");
    if (!fout)
    {
        cerr << "无法打开文件 calibration_result.txt" << endl;
        return -1;
    }

    int image_count = 0;
    Size image_size;
    Size board_size = Size(7, 7);
    vector<Point2f> image_points_buf;
    vector<vector<Point2f>> image_points_seq;
    string filename;

    cout << "开始提取角点………………" << endl;
    while (getline(fin, filename))
    {
        Mat imageInput = imread(filename);
        if (imageInput.empty())
        {
            cerr << "无法打开图像文件: " << filename << endl;
            continue;
        }

        image_count++;
        if (image_size.width == 0 && image_size.height == 0)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
        }

        bool found = findChessboardCorners(imageInput, board_size, image_points_buf);
        if (!found)
        {
            cout << "can not find chessboard corners in " << filename << "!\n";
            continue;
        }
        else
        {
            Mat view_gray;
            cvtColor(imageInput, view_gray, COLOR_BGR2GRAY);
            cornerSubPix(view_gray, image_points_buf, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            image_points_seq.push_back(image_points_buf);
            drawChessboardCorners(view_gray, board_size, image_points_buf, true);
            resize(view_gray,view_gray,Size(),0.5,0.5);
            imshow("Camera Calibration", view_gray);
            waitKey(1000);
        }
    }

    Size square_size = Size(10, 10);
    vector<vector<Point3f>> object_points;
    vector<int> point_counts;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
    vector<Mat> tvecsMat, rvecsMat;

    for (int t = 0; t < image_count; t++)
    {
        vector<Point3f> tempPointSet;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
        point_counts.push_back(board_size.width * board_size.height);
    }

    double rms = calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    cout << "标定完成!RMS误差:" << rms << endl;
    fout << "标定完成!RMS误差:" << rms << endl;

    cout << "开始评价标定结果………………\n";
    fout << "开始评价标定结果………………\n";
    double total_err = 0.0;
    double err = 0.0;
    vector<Point2f> image_points2;
    cout << "\t每幅图像的标定误差:\n";
    fout << "\t每幅图像的标定误差:\n";
    for (int i = 0; i < image_count; i++)
    {
        vector<Point3f> tempPointSet = object_points[i];
        projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
        vector<Point2f> tempImagePoint = image_points_seq[i];
        Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
        Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
        for (int j = 0; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err /= point_counts[i];
        cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
        fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
    }
    cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
    fout << "总体平均误差：" << total_err / image_count << "像素" << endl;
    cout << "评价完成！" << endl;
    fout << "评价完成！" << endl;

    cout << "相机内参数矩阵：" << endl;
    fout << "相机内参数矩阵：" << endl;
    cout << cameraMatrix << endl << endl;
    fout << cameraMatrix << endl << endl;
    cout << "畸变系数：" << endl;
    fout << "畸变系数：" << endl;
    cout << distCoeffs << endl << endl;
    fout << distCoeffs << endl << endl;

    for (int i = 0; i < image_count; i++)
    {
        cout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        cout << rvecsMat[i] << endl;
        fout << rvecsMat[i] << endl;
        Mat rotation_matrix;
        Rodrigues(rvecsMat[i], rotation_matrix);
        cout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        cout << rotation_matrix << endl;
        fout << rotation_matrix << endl;
        cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        cout << tvecsMat[i] << endl << endl;
        fout << tvecsMat[i] << endl << endl;
    }

    return 0;
}
