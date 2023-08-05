/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      An OpenCV Spline Test
 * 
*********************************************************************/

#include<iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <Interplot/SplineCurve.h>

#define IMG_HIGHT 900
#define IMG_WIDTH 1200

const cv::Vec3b IMG_COLOR = {0,0,0};
const cv::Vec3b IMG_COLOR2 = {255,0,0};

using namespace std;
using namespace cv;

cv::Mat img(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));;
enum mode{
    launch = 0, picking = 1, showing = 2, picking2 = 3
};
mode curmode;
Points::PosPoint2Ds pts;
vector<double> &x_s = pts.x;
vector<double> &y_s = pts.y;

Interplot::SplineCurve sp;

void ClearImage()
{
    for(int rowidx = 0; rowidx<IMG_HIGHT; rowidx++)
    {
        for(int pixidx = 0; pixidx<IMG_WIDTH; pixidx++)
        {
            img.at<Vec3b>(rowidx,pixidx) = {255,255,255};
        }
    }
}

static void dilate(Vec3b Color,int n,Mat & img)
{
    Mat img_backup = img.clone();
    int & rows = img_backup.rows;
    int & cols = img_backup.cols;
    for(int row = n;row<rows-n;row++)
    {
        for(int col = n;col<cols-n;col++)
        {
            bool flag = false;
            for(int i = -n;i<=n;i++)
            {
                for(int j=-n;j<=n;j++)
                {
                    // if(   img_backup.at<Vec3b>(row+i,col+j)[0]==Color[0]
                    //    && img_backup.at<Vec3b>(row+i,col+j)[1]==Color[1]
                    //    && img_backup.at<Vec3b>(row+i,col+j)[2]==Color[2])
                    if(img_backup.at<Vec3b>(row+i,col+j) == Color)
                    {
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

int MBUTTON_CNT = 0;
Points::Pos2D target;
void EventMouseClick(int event, int x, int y, int flags, void* ustc)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        cout<<"EVENT_LBUTTONDOWN"<<endl;
        if(curmode != picking)
        {
            x_s.clear();y_s.clear();
            curmode = picking;
        }
        
        ClearImage();
        x_s.push_back(x);y_s.push_back(y);
        for(int i=0, maxi = x_s.size();i<maxi;i++)
        {
            img.at<Vec3b>(y_s[i],x_s[i]) = IMG_COLOR;
        }
        dilate(IMG_COLOR,4, img);
        imshow("Spline test",img);
    }
    if (event == EVENT_RBUTTONDOWN)
    {
        cout<<"EVENT_RBUTTONDOWN"<<endl;
        curmode = showing;
        if(x_s.size()>2){
            sp.setPoints(pts);
        }

        for(int s = 0;s<sp.max_s+1;s++){
            auto ans = sp(static_cast<double>(s));
            if(ans.x>0&&ans.x<IMG_WIDTH&&ans.y>0&&ans.y<IMG_HIGHT){
                img.at<Vec3b>(ans.y,ans.x) = IMG_COLOR;
            }
        }

        dilate(IMG_COLOR,4, img);
        imshow("Spline test",img);
    }
    if (event == EVENT_MBUTTONDOWN)
    {
        if(curmode == showing){
            target.x = x,target.y=y;
            curmode = picking2;
        }
        else if(curmode == picking2){
            target.phi = atan2(y-target.y,x-target.x);
            auto s = sp.getDirectionalProjection(target,sp.max_s,0);
            Points::PosPoint2D ans = sp(s);
            if(ans.x>0&&ans.x<IMG_WIDTH&&ans.y>0&&ans.y<IMG_HIGHT){
                img.at<Vec3b>(ans.y,ans.x) = IMG_COLOR2;
                dilate(IMG_COLOR2,4, img);
                imshow("Spline test",img);
            }
            curmode = showing;
        }
        
    }
}

int main()
{
    curmode = launch;
    imshow("Spline test",img);
    setMouseCallback("Spline test", EventMouseClick);
    while(1)
    {
        if(cv::waitKey(1) == 27) return 0;
    }
}