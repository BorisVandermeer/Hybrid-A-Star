/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      An OpenCV RSCurve Test
 * 
*********************************************************************/

#include<iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include<Curves/ReedsShepp.h>
#include<Points/RefPointsGenerator.h>

#define IMG_HIGHT 900
#define IMG_WIDTH 1200

using namespace std;
using namespace cv;

const cv::Vec3b IMG_COLOR = {0,0,0};
cv::Mat img(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));

int clickcounter = 0;
double Radius = 40;
Curves::RSCurveStateSpace::State From;
Curves::RSCurveStateSpace::State To;
Curves::RSCurveStateSpace RSPSS;

PointsTools::PointsGenHandller PGH;

void __attribute__((unused)) ClearImage()
{
    for(int rowidx = 0; rowidx<IMG_HIGHT; rowidx++)
    {
        for(int pixidx = 0; pixidx<IMG_WIDTH; pixidx++)
        {
            img.at<Vec3b>(rowidx,pixidx) = {255,255,255};
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

void EventMouseClick(int event, int x, int y, int flags, void* ustc)
{
    Curves::RSCurve s;
    Points::Pos2Ds poses;
    // cout<<x<<' '<<y<<endl;
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        // cout<<x<<' '<<y<<endl;
        switch(clickcounter){
            case 0:
                From.x = x;
                From.y = y;
                clickcounter++;
                ClearImage();
                img.at<Vec3b>(From.y,From.x) = IMG_COLOR;
                dilate(IMG_COLOR,4,img);
                break;
            case 1:
                From.phi = atan2(y-From.y,x-From.x);
                ClearImage();
                img.at<Vec3b>(From.y,From.x) = IMG_COLOR;
                img.at<Vec3b>(From.y+30*sin(From.phi),From.x+30*cos(From.phi)) = IMG_COLOR;
                dilate(IMG_COLOR,4,img);
                clickcounter++;
                break;
            case 2:
                To.x = x;
                To.y = y;
                ClearImage();
                img.at<Vec3b>(From.y,From.x) = IMG_COLOR;
                img.at<Vec3b>(From.y+30*sin(From.phi),From.x+30*cos(From.phi)) = IMG_COLOR;
                img.at<Vec3b>(To.y,To.x) = IMG_COLOR;
                dilate(IMG_COLOR,4,img);
                clickcounter++;
                break;
            case 3:
                To.phi = atan2(y-To.y,x-To.x);
                ClearImage();
                img.at<Vec3b>(From.y,From.x) = IMG_COLOR;
                img.at<Vec3b>(From.y+30*sin(From.phi),From.x+30*cos(From.phi)) = IMG_COLOR;
                img.at<Vec3b>(To.y,To.x) = IMG_COLOR;
                img.at<Vec3b>(To.y+30*sin(To.phi),To.x+30*cos(To.phi)) = IMG_COLOR;
                dilate(IMG_COLOR,4,img);
                cout<<From.x<<' '<<From.y<<' '<<From.phi<<endl;
                cout<<To.x<<' '<<To.y<<' '<<To.phi<<endl;
                s = RSPSS.RSCurveCalc(From,To);
                cout<<"RSType : "<<s.type_[0]<<s.type_[1]<<s.type_[2]<<s.type_[3]<<s.type_[4]<<endl;
                PGH.GetPoses(poses,s,1,From);
                for(int i = 0, maxi=poses.size();i<maxi;i++){
                    if(poses.y[i]>0&&poses.x[i]>0&&poses.x[i]<IMG_WIDTH&&poses.y[i]<IMG_HIGHT)
                        img.at<Vec3b>(poses.y[i],poses.x[i]) = IMG_COLOR;
                }
                dilate(IMG_COLOR,2,img);
                clickcounter++;
                break;
            default:
                break;
        }
        imshow("RSCurve test",img);
    }
    if (event == CV_EVENT_RBUTTONDOWN)
    {
        cout<<"CV_EVENT_RBUTTONDOWN"<<endl;
        clickcounter = 0;
        ClearImage();
        imshow("RSCurve test",img);
    }
}

int main()
{
    cout<<"Set Turning Radius(Pix) : ";
    cin>>Radius;
    cout<<endl;
    // Radius = 50;
    RSPSS.setRadius(Radius);
    imshow("RSCurve test",img);
    cvSetMouseCallback("RSCurve test", EventMouseClick);
    while(1)
    {
        if(cv::waitKey(1) == 27) return 0;
    }
}