#ifndef __LOCALIZER_H__
#define __LOCALIZER_H__

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using std::string;

bool targetSortCriteria(const Vec3f &a, const Vec3f &b);
bool compare(Vec4f c1, Vec4f c2);
void sort_Label(vector<Vec4f> &list, vector<Vec4f> &vec);
void Label(vector<Vec4f> &list, vector<Vec4f> &vec,float error);


class localizer {
    public:
        localizer();
        void init();
        bool getFrame();

        Point3d detectVehicle();
        vector<Vec3f> detectTargets();

        void showResult();
        void showDataInfo();

        /* Vmin, Vmax, Smin */
        void setCamshiftParameters(int,int,int);
        /* x, y, w, h */
        void setROI_Rect(int, int, int, int);
        /* edge, center, rmin, rmax */
        void setHoughCircleParameters(int,int,int,int);

    private:
        VideoCapture cap;

        Mat frame;
        Mat image, gray_img;
        Mat hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
        Rect trackWindow;
    	Point origin;
	    Rect selection;

        int camNum;
        int hsize = 16;
        float hranges[2] = {0,180};
        const float* phranges = hranges;

    	int trackObject = 0;
        int label_num = 0;
        bool paused = false;
	    bool backprojMode = false;
    	bool selectObject = false;
    	bool showHist = true;

        bool isResultVisible = false;
        bool isCamshiftSet = false;
        bool findTargets = false; // which means initialization
        bool findVehicle = false;

        // Environment States
        Point3d vechile_pose;
        /* x, y, r, vote, label */
        vector<Vec6f> target_list;
        /* x, y, r, votes */
        vector<Vec4f> candidate_list;

        void voting(vector<Vec3f> &);

        void sort_Label(vector<Vec4f> &data);

        // extend data from vec3f to candidate list vec4f
        void putDataInCandidates(vector<Vec3f> &data);

        void updateCandidates(vector<Vec3f> &data);

        void voteCandidates(vector<Vec3f> &data, int tol);
        void candidatesRuling();
        void candidatesPromotion();
        void electedRemoval(vector<Vec3f> &);

        void labelConsistency(vector<Vec4f> &data, double error);

        // Camshift Parameters
    	int vmin = 10, vmax = 256, smin = 30;
        int roi_x, roi_y, roi_w, roi_h;
        // Hough Circle Parameters
        int edgeDet = 200, centerDet = 100;
        int min_radius = 0, max_radius = 0;

        string nameMainFrame = "Main Frame";
        string nameHistFrame = "Histograme";
};

#endif
