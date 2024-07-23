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
    
    while (waitKey(0) != -1)
    {
        //int key = waitKey(0);
        cout<< "lena" << endl;
        moveWindow("lena", 50, 0);
    }
    return 0;
}