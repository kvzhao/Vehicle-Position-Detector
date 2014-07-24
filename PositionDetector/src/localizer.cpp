#include "localizer.h"
#include <iostream>
using namespace std;

localizer::localizer()
{
    camNum = 1;
    isResultVisible = true;
    namedWindow( nameMainFrame, 0 );
//    namedWindow( nameHistFrame, 0 );
//
/*
    setMouseCallback( "CamShift Demo", onMouse, 0 );
    createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
    createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
    createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );
*/
}

void localizer::init() {

    cap.open(camNum);
    if( !cap.isOpened() )
    {
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: \n";
        //parser.printMessage();
    }
    if (isCamshiftSet) {
            trackObject = -1;
    }
}

void localizer::setCamshiftParameters(int Vmin, int Vmax, int Smin) {
    // Rude
    if (Vmin) vmin = Vmin;
    if (Vmax) vmax = Vmax;
    if (Smin) smin = Smin;
}

void localizer::setROI_Rect(int x, int y, int w, int h) {
    roi_x = x;
    roi_y = y;
    roi_w = w;
    roi_h = h;

    selection.x = roi_x;
    selection.y = roi_y;
    selection.width = roi_w;
    selection.height = roi_h;

    isCamshiftSet = true;
}

void localizer::setHoughCircleParameters(int p1,int p2) {
}

void localizer::showResult() {

    if (isResultVisible) {
        imshow(nameMainFrame, image);
        //imshow(nameHistFrame, histimg);
    }
}

bool localizer::getFrame() {
        if( !paused )
        {
            cap >> frame;
            if( frame.empty() )
                return false;
        }

        frame.copyTo(image);

        if( !paused )
        {
            cvtColor(image, hsv, COLOR_BGR2HSV);

        }
        else if( trackObject < 0 )
            paused = false;

        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        return true;
}

void localizer::detectVehicle() {

            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;

                inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                        Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);

                if( trackObject < 0 )
                {
                    Mat roi(hue, selection), maskroi(mask, selection);
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, NORM_MINMAX);

                    trackWindow = selection;
                    trackObject = 1;

                    histimg = Scalar::all(0);
                    int binW = histimg.cols / hsize;
                    Mat buf(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf, buf, COLOR_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                        rectangle( histimg, Point(i*binW,histimg.rows),
                                   Point((i+1)*binW,histimg.rows - val),
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }
                }

                calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                backproj &= mask;
                RotatedRect trackBox = CamShift(backproj, trackWindow,
                                    TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
                if( trackWindow.area() <= 1 )
                {
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       trackWindow.x + r, trackWindow.y + r) &
                                  Rect(0, 0, cols, rows);
                }

                if( backprojMode )
                    cvtColor( backproj, image, COLOR_GRAY2BGR );
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA);
            }
}
