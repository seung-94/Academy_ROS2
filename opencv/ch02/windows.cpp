#include <iostream>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

string folder = "/home/ha/Academy_ROS2/opencv/data/";

int main ()
{
     Mat img;
    img = imread(folder + "lena.bmp");
    namedWindow("lena");
    imshow("lena", img);
    int key = waitKey(0);
    cout<< key << endl;
    Size size;
    size.height = 100;
    size.width = 100;
    destroyWindow("lena");
    resizeWindow("lena", size);
    imshow("lena", img);
    waitKey(0);
    return 0;
}