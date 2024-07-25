#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
void on_level_chage(int pos, void*data);

String folder = "/home/ha/Academy_ROS2/opencv/data/";

int main()
{
    Scalar white = Scalar(255, 255, 255);
    Scalar yellow = Scalar(0, 255, 255);
    Scalar blue = Scalar(255, 0, 0);
    Scalar green = Scalar(0, 255, 0);
    Scalar red = Scalar(0, 0, 255);
    Mat img = imread(folder + "lenna.bmp");
    int a = 0;

    namedWindow("img");
    createTrackbar("level","img",&a, 255, on_level_chage, &img);
    imshow("img", img);
    waitKey(0);
    destroyAllWindows();
    return 0;
}

void on_level_chage(int pos, void*data)
{
    Mat img = *(Mat *)data;
    Scalar gray = Scalar (pos-125, pos-125, pos-125);
    imshow("img", img + gray);
}