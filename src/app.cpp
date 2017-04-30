/*
 * =====================================================================================
 *
 *       Filename:  app.cpp
 *
 *    Description:  implementing vector idea
 *
 *        Version:  1.0
 *        Created:  04/29/2017 14:44:48
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ada Kaminkure (AdaCode), ada@adacode.io
 *        Company:  ADACODE.IO
 *
 * =====================================================================================
 */


#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void renderGabor(Mat &input, Mat &output, double p_thata)
{
  // initilization for gabor filter
  const int ksize = 9;
  const double sigma = 2;
  const double lambd = 10;
  const Size kernelSize(ksize, ksize);
  const double gamma = 1;
  // calculation gabor filter
  const Mat gaborKernel = getGaborKernel(kernelSize, sigma, p_thata, lambd, gamma);
  filter2D(input, output, input.depth(), gaborKernel);

  // Draw axis
  const int cols = output.cols;
  const int rows = output.rows;
  const Point center(cols/2, rows/2);
  const Point upperLeft(0, 0);
  const Point upperRight(cols, 0);
  const Point lowerLeft(0, rows);
  const Point lowerRight(cols, rows);
  const Point halfLeft(0, rows/2);
  const Point halfRight(cols, rows/2);
  const Point halfUpper(cols/2, 0);
  const Point halfLower(cols/2, rows);

  line(output, center, halfRight, Scalar(0,255,0), 1);
  line(output, center, upperRight, Scalar(0,255,0), 1);
  line(output, center, halfUpper, Scalar(0,255,0), 1);
  line(output, center, upperLeft, Scalar(0,255,0), 1);
  line(output, center, halfLeft, Scalar(0,255,0), 1);
  line(output, center, lowerLeft, Scalar(0,255,0), 1);
  line(output, center, halfLower, Scalar(0,255,0), 1);
  line(output, center, lowerRight, Scalar(0,255,0), 1);
}

Point searchMaxBright(Mat &p_src, vector<Point> p_store)
{
    Mat graySrc = p_src.clone();
    Mat disSrc = p_src.clone();
    cvtColor(p_src, graySrc, CV_BGR2GRAY);
    int sum = 0;
    int maxValue = -1;
    int cellRange = 1;
    Point maxPoint;

    for(int i = 0; i < p_store.size(); i+=1)
    {
      Point target = p_store[i];
      Vec3b px1 = disSrc.at<Vec3b>(target.y, target.x-cellRange);
      Vec3b px2 = disSrc.at<Vec3b>(target.y-cellRange, target.x);
      Vec3b px3 = disSrc.at<Vec3b>(target.y+cellRange, target.x);
      Vec3b px4 = disSrc.at<Vec3b>(target.y, target.x+cellRange);

      sum = px1[1] + px2[1] + px3[1] + px4[1];

      if( sum >= maxValue ){
        maxValue = sum;
        maxPoint = target;
      }
      // circle(disSrc, target, 1, Scalar(255,100,255), -1);
      // imshow("Debug", disSrc);
      // // waitKey(1);
    }
    return maxPoint;
}

int getLinePointY(Point p1, Point p2, int p_x)
{
  int divider = p2.x - p1.x;
  if(divider == 0)
    return p_x;

  double m = (p2.y - p1.y)/(double)(p2.x - p1.x);
  return m*(p_x - p1.x) + p1.y;
}

vector<Point> getLinePointSet(Mat &src, string thata)
{
  int x_start;
  int x_end;
  int cols = src.cols;
  int rows = src.rows;

  const Point center(cols/2, rows/2);
  const Point upperLeft(0, 0);
  const Point upperRight(cols, 0);
  const Point lowerLeft(0, rows);
  const Point lowerRight(cols-10, rows-10);
  const Point halfLeft(0, rows/2);
  const Point halfRight(cols, rows/2);
  const Point halfUpper(cols/2, 0);
  const Point halfLower(cols/2, rows);

  Point p1, p2;
  vector<Point> store;

  if( thata == "0")
  {
    x_start = center.x;
    x_end = cols;
    p1 = center;
    p2 = halfRight;
  }
  else if( thata == "45")
  {
    x_start = center.x;
    x_end = cols;
    p1 = center;
    p2 = upperRight;
  }
  else if( thata == "90")
  {
    x_start = center.y;
    x_end = halfUpper.y;
    p1 = halfUpper;
    p2 = center;
  }
  else if( thata == "135")
  {
    x_start = center.x;
    x_end = upperLeft.x;
    p1 = upperLeft;
    p2 = center;
  }
  else if( thata == "180")
  {
    x_start = center.x;
    x_end = halfLeft.x;
    p1 = halfLeft;
    p2 = center;
  }
  else if( thata == "225")
  {
    x_start = center.x;
    x_end = lowerLeft.x;
    p1 = lowerLeft;
    p2 = center;
  }
  else if( thata == "270")
  {
    x_start = center.y;
    x_end = halfLower.y;
    p1 = center;
    p2 = halfLower;
  }
  else if( thata == "315")
  {
    x_start = center.x;
    x_end = lowerRight.x;
    p1 = center;
    p2 = lowerRight;
  }
  else{
    cout << "Thata out range";
    return store;
  }

int iThata = stoi(thata);
if( iThata < 90 || iThata >= 270)
{
// Right Loop
  for(int x = x_start; x <= x_end; x++)
  {
    Point action;
    if( iThata == 270)
    {
      action.x = cols/2;
      action.y = x;
    }
    else
    {
      action.x = x;
      action.y = getLinePointY(p1, p2, x);
    }
    store.push_back(action);
  }
}
else
{
// Left Loop
  for(int x = x_start; x >= x_end; x--)
  {
    Point action;
    if( iThata == 90 )
    {
      action.x = cols/2;
      action.y = x;
    }
    else
    {
      action.x = x;
      action.y = getLinePointY(p1, p2, x);
    }
    store.push_back(action);
  }

}
  return store;
}

vector< vector<Point> > generatePointsStore(Mat &src)
{
  vector< vector<Point> > store;
  vector<Point> action;

  for(int i=0; i<=315; i+=45)
  {
      cout << i << endl;
      action = getLinePointSet(src, to_string(i));
      store.push_back(action);
  }

  return store;
}

bool isContain(Point p_point, vector<Point> p_store)
{
    for(int i=0; i< p_store.size(); i++)
    {
      bool isEqX = ( p_point.x == p_store[i].x );
      bool isEqY = ( p_point.y == p_store[i].y );
      if( isEqX && isEqY){
        return true;
      }
    }
  return false;
}

int main()
{
  Mat src = imread("./data/brain1.jpg");
  Mat dst = imread("./data/brain2.jpg");
  if(src.data == NULL || dst.data == NULL)
  {
    cout << "ERROR::ADA Image not found" << endl;
    return -1;
  }

  // Create Points Store for each lines
  // 0 = '0', 1 = '45', 2 = '90' ... 7 = '315'
  const vector< vector<Point> > Store = generatePointsStore(src);
  Mat gaborResult;
  int offsetX = 0;
  int offsetY = 0;
  int count = 1;

  for(int thata = 0; thata < 180; thata += 30)
  {
    if( count%4 == 0)
    {
      offsetX = 0;
      offsetY += 300;
    }
    renderGabor(src, gaborResult, thata);

    for(int i=0; i<Store.size(); i++)
    {
      Point markPoint = searchMaxBright(gaborResult, Store[i]);
      circle(gaborResult, markPoint, 5, Scalar(0,0,255), -1);
    }

    const string windowNamed = "Gabor Result thata = " + to_string(thata);
    namedWindow(windowNamed, 0);
    moveWindow(windowNamed, offsetX, offsetY);
    imshow(windowNamed, gaborResult);
    offsetX += src.cols/1.5;
    count++;
  }

  waitKey(0);
  return 0;
}
