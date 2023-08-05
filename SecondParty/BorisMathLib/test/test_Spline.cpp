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
#include <opencv2/highgui/highgui_c.h>

#include <Interplot/Spline.h>

#define IMG_HIGHT 900
#define IMG_WIDTH 1200

const cv::Vec3b IMG_COLOR = {0,0,0};

using namespace std;
using namespace cv;

cv::Mat img(IMG_HIGHT,IMG_WIDTH,CV_8UC3,Scalar(255,255,255));;
enum mode{
    launch = 0, picking = 1, showing = 2
};
mode curmode;
vector<double> x_s;
vector<double> y_s;

Interplot::Spline sp;



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

void EventMouseClick(int event, int x, int y, int flags, void* ustc)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        cout<<"CV_EVENT_LBUTTONDOWN"<<endl;
        if(curmode != picking)
        {
            x_s.clear();y_s.clear();
            curmode = picking;
        }
        
        ClearImage();
        if(x_s.empty()||x>x_s.back()) {
        x_s.push_back(x);y_s.push_back(y);}
        for(int i=0,maxi=x_s.size();i<maxi;i++)
        {
            img.at<Vec3b>(y_s[i],x_s[i]) = IMG_COLOR;
        }
        dilate(IMG_COLOR,4, img);
        imshow("Spline test",img);
    }
    if (event == CV_EVENT_RBUTTONDOWN)
    {
        cout<<"CV_EVENT_RBUTTONDOWN"<<endl;
        curmode = showing;
        if(x_s.size()>2){
            Interplot::Spline::RefPoints points;
            points.x = x_s;
            points.y = y_s;

            sp.set_points(points);
        }
        
        for(int i=0;i<IMG_WIDTH;i++)
        {
            if(sp(i)>0&&sp(i)<IMG_HIGHT){
                int tmp = sp(i);
                img.at<Vec3b>(tmp,i) = IMG_COLOR;
            }
        }

        dilate(IMG_COLOR,4, img);
        imshow("Spline test",img);
    }

}

int main()
{
    
    curmode = launch;
    imshow("Spline test",img);
    cvSetMouseCallback("Spline test", EventMouseClick);
    while(1)
    {
        if(cv::waitKey(1) == 27) return 0;
    }
}