#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/ha/Academy_ROS2/opencv/data/";
void onMouse(int event, int x, int y, int flags, void *data);

Mat src;
bool flag = false
Point2f srcPts[4], dstPts[4];

int main()
{
    mydata.img = imread(folder + "card.bmp");

    namedWindow("src");
    setMouseCallback("src", onMouse);

    imshow("src", src);
    waitKey(0);
    destroyAllWindows();
    return 0;
}

void onMouse(int event, int x, int y, int flags, void *data)
{
    static int cnt = 0;
    switch (event)
    {
    case EVENT_LBUTTONDOWN:
        if (cnt < 4){
            srcPts[cnt++] Point2f(x,y);
        }
        cout << "mouse left button down" << x << y << endl;
        ptr->ptOld = Point(x, y);
        ptr->flag = true;
        break;
    }
}







int main()
{
    Mat src = imread(folder + "tekapo.bmp");
    Point2f srcPts[4], dstPts[4];
    srcPts[0] = Point2f(0, 0);
    srcPts[1] = Point2f(src.cols - 1, 0);
    srcPts[2] = Point2f(src.cols - 1, src.rows - 1);
    srcPts[3] = Point2f(0, src.rows - 1);
    dstPts[0] = Point2f(150, 50);
    dstPts[1] = Point2f(src.cols - 1 - 50, 150);
    dstPts[2] = Point2f(src.cols - 1 - 250, src.rows - 1 + 50);
    dstPts[3] = Point2f(50, src.rows - 1 + 150);

    Mat M = getPerspectiveTransform(srcPts, dstPts);
    cout << M << endl;
    Mat dst;

    Size sz1 = src.size();
    warpPerspective(src, dst, M, Size());

    imshow("src", src);
    imshow("dst", dst);
    waitKey(0);
    return 0;
}