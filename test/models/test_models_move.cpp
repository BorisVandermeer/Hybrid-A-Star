/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      An HAS Move Test
 * 
*********************************************************************/

#include<iostream>
#include<memory>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include<VehicleModel.h>

#define IMG_HIGHT 800
#define IMG_WIDTH 1200

using namespace std;
using namespace cv;

const cv::Vec3b IMG_COLOR = {0,0,0};
cv::Mat img2show(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));

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
static void dilate(Vec3b Color,int n,Mat & img)
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

double islbuttondown = false;
void EventMouseClick(int event, int x, int y, int flags, void* ustc)
{
    if(event == CV_EVENT_MOUSEMOVE && flags == CV_EVENT_FLAG_LBUTTON){

    }
    if(event == CV_EVENT_RBUTTONDOWN){

    }
    // if (event == CV_EVENT_RBUTTONDOWN)
    // {

    // }

    imshow("GridMap test",img2show);

}

int main()
{
    Models::VehicleMoveModel Model(80,40,15,50);
    double stepsize = 1.0;
    
    for(int R = -400; R<=400; R+=20){
        Points::Pos2D pos(IMG_WIDTH/2,IMG_HIGHT/2,-M_PI/2);
        img2show.at<Vec3b>(pos.y,pos.x) = IMG_COLOR;
        for(double dist = stepsize;dist<M_PI*fabs(R);dist+=stepsize){
            pos = Model.MoveByRadius(pos,R,stepsize);
            if(pos.x>0&&pos.y>0&&pos.x<img2show.cols&&pos.y<img2show.rows)
                img2show.at<Vec3b>(pos.y,pos.x) = IMG_COLOR;
        }
    }

    imshow("Vehicle Move test",img2show);
    while(1)
    {
        if(cv::waitKey(1) == 27) break;
    }
    
    for(double R = -M_PI/3; R<=M_PI/3; R+=M_PI/30){
        if(fabs(R)<0.00001) R+=M_PI/30;
        Points::Pos2D pos(IMG_WIDTH/2,IMG_HIGHT/2,M_PI/2);
        img2show.at<Vec3b>(pos.y,pos.x) = IMG_COLOR;
        for(double dist = stepsize;dist<fabs(2*M_PI*50/tan(R));dist+=stepsize){
            pos = Model.MoveBySteerting(pos,R,stepsize);
            if(pos.x>0&&pos.y>0&&pos.x<img2show.cols&&pos.y<img2show.rows)
                img2show.at<Vec3b>(pos.y,pos.x) = IMG_COLOR;
        }
    }

    imshow("Vehicle Move test",img2show);

    while(1)
    {
        if(cv::waitKey(1) == 27) return 0;
    }
}

