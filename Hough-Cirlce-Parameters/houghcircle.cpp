#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>

using namespace cv;
using namespace std;

/** @function main */
int main(int argc, char** argv)
{
  Mat src, src_gray;
  VideoCapture cap;
  string title = "HoughCircle Tunner";
  ofstream of;
  cap.open(1);
  if ( !cap.isOpened())
      return -1;
  cap >> src;
  namedWindow( title, CV_WINDOW_AUTOSIZE );
  /// Read the image
  int edgeDet_index = 200;
  int centerDet_index = 100;
  int min_radius = 0;
  int max_radius = 0;
  createTrackbar("Canny Detector", title, &edgeDet_index, 256, 0);
  createTrackbar("Center Detector", title, &centerDet_index, 200, 0);
  createTrackbar("Max Radius", title, &max_radius, 100, 0);
  createTrackbar("min Radius", title, &min_radius, 50, 0);

   while (!src.empty()) {
       cap >> src;
  /// Convert it to gray
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

  vector<Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8,
          edgeDet_index,
          centerDet_index,
          min_radius,
          max_radius);

  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      cout << "point : " << center << "\n";
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }

  /// Show your results
  namedWindow( title, CV_WINDOW_AUTOSIZE );
  imshow( title, src );

    char c = waitKey(10);
    if ( c == 27 )
        break;
    if ( c == 's') {
        of.open("houghParameters.txt");
        of << edgeDet_index << " "
            << centerDet_index << " "
            << min_radius << " "
            << max_radius << "\n";
        of.close();
    }
   }
  return 0;
}
