#ifndef __LOCALIZER_H__
#define __LOCALIZER_H__

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using std::string;

class localizer {
    public:
        localizer();
        void init();
        bool getFrame();

        void detectVehicle();
        void detectTarget();

        void showResult();

        void setCamshiftParameters(int,int,int);
        void setROI_Rect(int, int, int, int);
        void setHoughCircleParameters(int,int);

    private:
        VideoCapture cap;

        Mat frame;
        Mat image, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
        Rect trackWindow;
    	Point origin;
	    Rect selection;

        int camNum;
        int hsize = 16;
        float hranges[2] = {0,180};
        const float* phranges = hranges;

        bool paused = false;
	    bool backprojMode = false;
    	bool selectObject = false;
    	int trackObject = 0;
    	bool showHist = true;

        bool isResultVisible = false;
        bool isCamshiftSet = false;

        // Camshift Parameters
    	int vmin = 10, vmax = 256, smin = 30;
        int roi_x, roi_y, roi_w, roi_h;
        // Hough Circle Parameters

        string nameMainFrame = "Main Frame";
        string nameHistFrame = "Histograme";
};

#endif
