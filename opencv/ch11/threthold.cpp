#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/ha/Academy_ROS2/opencv/data/";

int main()
{
    //Mat img = imread(folder + "neutrophils.png", IMREAD_GRAYSCALE);
    Mat img = imread(folder + "sudoku.jpg", IMREAD_GRAYSCALE);
    Mat dst;
    //threshold(img, dst, 180, 255, THRESH_BINARY);
    //threshold(img, dst, 0, 255, THRESH_OTSU);
    adaptiveThreshold(img, dst, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 33, 5);

    imshow("img", img);
    imshow("dst", dst);
    waitKey(0);
    return 0;
}