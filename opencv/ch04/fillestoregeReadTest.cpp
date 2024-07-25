#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/ha/Academy_ROS2/opencv/data/";

int main()
{
    FileStorage fs;
    String name;
    Mat img;
    int age;
    Point pt1;
    int v;

    fs.open(folder+"myData.json", FileStorage::READ);
    if(!fs.isOpened()){

        cerr<<"file open failed"<<endl;
        return 1;
    }
    fs["name"]>> name;
    fs["age"] >> age;
    fs["point"] >> pt1;
    fs["vector"] >> v;
    fs["mat1"] >> img;

    fs.release();

    cout<<name<<endl;
    cout<<age<<endl;
    cout<<pt1<<endl;
    cout<<v<<endl;

    imshow("img", img);
    waitKey(0);

    return 0;
}