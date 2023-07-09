/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      An GridMap Dilate Test
 * 
*********************************************************************/

#include<iostream>
#include<memory>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include<GridMap.h>

#define IMG_HIGHT 800
#define IMG_WIDTH 1200

using namespace std;
using namespace cv;

const cv::Vec3b IMG_COLOR = {0,0,0};
cv::Mat img2show(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));
cv::Mat img4data(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));

shared_ptr<Maps::GridMap> ptr1;
shared_ptr<Maps::GridMap> ptr2;
shared_ptr<Maps::GridMap> ptr3;


void ClearImage()
{
    for(int rowidx = 0; rowidx<IMG_HIGHT; rowidx++)
    {
        for(int pixidx = 0; pixidx<IMG_WIDTH; pixidx++)
        {
            img2show.at<Vec3b>(rowidx,pixidx) = {255,255,255};
        }
    }
}
static void __attribute__((unused)) dilate(Vec3b Color,int n,Mat & img)
{
    Mat img_backup = img.clone();
    int & rows = img_backup.rows;
    int & cols = img_backup.cols;
    for(int row = n;row<rows-n;row++){
        for(int col = n;col<cols-n;col++){
            bool flag = false;
            for(int i = -n;i<=n;i++){
                for(int j=-n;j<=n;j++){
                    if(img_backup.at<Vec3b>(row+i,col+j) == Color){
                        img.at<Vec3b>(row,col) = Color;
                        flag = true;
                        break;
                    }
                }
                if(flag) break;
            }
        }
    }
}

static void __attribute__((unused)) dilate(Vec3b Color,int n,Mat &img , Mat &img2)
{
    int & rows = img.rows;
    int & cols = img.cols;
    for(int row = n;row<rows-n;row++){
        for(int col = n;col<cols-n;col++){
            bool flag = false;
            for(int i = -n;i<=n;i++){
                for(int j=-n;j<=n;j++){
                    if(img.at<Vec3b>(row+i,col+j) == Color){
                        img2.at<Vec3b>(row,col) = Color;
                        flag = true;
                        break;
                    }
                }
                if(flag) break;
            }
        }
    }
}

void ShowGridMap(shared_ptr<Maps::GridMap> ptr){
    ClearImage();
    // cout<<ptr->getSizeX()<<' '<<ptr->getSizeY()<<endl;
    for(int x=0, maxx=ptr->getSizeX() ;x<maxx;x++)
        for(int y=0,maxy=ptr->getSizeY();y<maxy;y++){
            if((*ptr).HasObstacle(x,y)) img2show.at<Vec3b>(y,x) = IMG_COLOR;
        }
}

double islbuttondown = false;
void EventMouseClick(int event, int x, int y, int flags, void* ustc)
{
    if(event == CV_EVENT_MOUSEMOVE && flags == CV_EVENT_FLAG_LBUTTON){
        img2show.at<Vec3b>(y,x) = IMG_COLOR;
        ptr1->SetObstacleByIdx((unsigned int)x,y);
        ptr2 = ptr1;
    }
    if(event == CV_EVENT_RBUTTONDOWN){
        ClearImage();
        ptr3 = ptr2;
        ptr3->Dilate(ptr2);
        ShowGridMap(ptr3);
    }
    // if (event == CV_EVENT_RBUTTONDOWN)
    // {

    // }

    imshow("GridMap test",img2show);

}


int main()
{
    ptr1 = make_shared<Maps::GridMap>(IMG_WIDTH,IMG_HIGHT,1);
    imshow("GridMap test",img2show);
    cvSetMouseCallback("GridMap test", EventMouseClick);
    while(1)
    {
        if(cv::waitKey(1) == 27) return 0;
    }
}

