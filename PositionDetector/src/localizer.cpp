#include "localizer.h"

#include <iostream>
#include <algorithm>

using namespace std;

#define XCOORD 0
#define YCOORD 1
#define RADIUS 2
#define VOTE   3
#define LABEL  4
#define VISIBLE 5

localizer::localizer()
{
    camNum = 1;
    isResultVisible = true;
    namedWindow( nameMainFrame, 0 );
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

void localizer::setHoughCircleParameters(int edge,int center, int rmin, int rmax) {
    // Set
    if (edgeDet != edge) edgeDet = edge;
    if (centerDet != center) centerDet = center;
    if (min_radius != rmin) min_radius = rmin;
    if (max_radius != rmax) max_radius = rmax;
}

void localizer::showResult() {

    if (isResultVisible) {

        if (findTargets) {

            // Draw candidate_list
            for (size_t i = 0; i < candidate_list.size(); ++i) {
                Point center(cvRound(candidate_list[i][XCOORD]), cvRound(candidate_list[i][YCOORD]));
                int radius = cvRound(candidate_list[i][RADIUS]);
                circle( image, center, 3, Scalar(0,255,255), -1, 8, 0 );
                circle( image, center, radius, Scalar(0,255,255), 3, 8, 0 );
            }

            // Draw target_list
            for (size_t i = 0; i < target_list.size(); ++i) {
                Point center(cvRound(target_list[i][XCOORD]), cvRound(target_list[i][YCOORD]));
                int radius = cvRound(target_list[i][RADIUS]);
                if (target_list[i][VISIBLE]) {
                    circle( image, center, 3, Scalar(0,225,0), -1, 8, 0 );
                    circle( image, center, radius, Scalar(0,255,0), 3, 8, 0 );
                } else {
                    circle( image, center, 3, Scalar(255,225,0), -1, 8, 0 );
                    circle( image, center, radius, Scalar(255,255,0), 3, 8, 0 );
                }
            }
        }

        imshow(nameMainFrame, image);
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
            cvtColor(image, gray_img, CV_BGR2GRAY);
            GaussianBlur(gray_img, gray_img, Size(9,9), 2, 2);
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

Point3d localizer::detectVehicle() {

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

                Point3d pose(trackBox.center.x, trackBox.center.y, trackBox.angle);

                if( backprojMode )
                    cvtColor( backproj, image, COLOR_GRAY2BGR );
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA);

                return pose;
            }
}

bool targetSortCriteria(const Vec3f &a, const Vec3f &b) {
    return (a[0] + a[1]) > (b[0] + b[1]);
}

// Enlarge the vector from vec3f to vec4f
void localizer::putDataInCandidates(vector<Vec3f> &data) {
      // Preprocessing raw data
      for (size_t i = 0; i < data.size(); ++i) {
          Point center(cvRound(data[i][XCOORD]), cvRound(data[i][YCOORD]));
          Vec4f p(cvRound(data[i][XCOORD]),
                  cvRound(data[i][YCOORD]),
                  cvRound(data[i][RADIUS]),
                  0 /* tickets */ );
          // Initialize the candidates vector
          candidate_list.push_back(p);
          findTargets = true;
      }
      cout << "init circle is " << data.size() << " long, and candidate list is "
          << candidate_list.size() << " long\n";
}

// vec is the unlabeled data - candidates
void localizer::voteCandidates(vector<Vec3f> &data, int tol) {

    vector<Vec3f>::iterator v3iter;
    vector<Vec4f>::iterator v4iter;

    for(v3iter = data.begin(); v3iter < data.end(); v3iter++) {
        bool gotit = false;
        // naive searching algorithm
        for(v4iter = candidate_list.begin(); v4iter < candidate_list.end(); v4iter++) {
            // if data found in candidate list within tolerance window
            if(fabs((*v3iter)[XCOORD] - (*v4iter)[XCOORD]) <= tol
             && fabs((*v3iter)[YCOORD]- (*v4iter)[YCOORD]) <= tol) {
                gotit = true;
                (*v4iter)[VOTE] += 3;
                cout << "Gotit\n";
            }
        }

        if(!gotit) {
            // if data not found, it is a new comer, then add it to the list
            cout << "miss!! ";
            Vec4f p(cvRound((*v3iter)[XCOORD]),
                    cvRound((*v3iter)[YCOORD]),
                    cvRound((*v3iter)[RADIUS]),
                    2 /* tickets */ );
            candidate_list.push_back(p);
            cout << "add to list, size " << candidate_list.size() << "\n";
        }
    }
}

void localizer::candidatesRuling() {
    // for each iteration, all candidates lose their influence
    vector<Vec4f>::iterator it;
    for (it = candidate_list.begin(); it < candidate_list.end(); it++) {
        cout << "num of iterator " << *it << " contains " << (*it)[VOTE] << " votes\n";
        (*it)[VOTE]--;
    }
}

void localizer::candidatesPromotion() {

    //const static int vote_thres = 30;
    static float label = 0;
    vector<Vec4f>::iterator it;
    for (it = candidate_list.begin(); it < candidate_list.end(); it++) {
        // promote candidate to target list
        if ((*it)[VOTE] >= 30 ) {
            cout << (*it) << " possibly promoted!! \n";

            int tol = 10;
            bool alreadyFound;
            for (size_t i = 0; i < target_list.size(); i++) {
                if(fabs((*it)[XCOORD] - target_list[i][XCOORD]) <= tol
                 && fabs((*it)[YCOORD]- target_list[i][YCOORD]) <= tol) {
                    alreadyFound = true;
                    cout << "already in target" << endl;
                    (*it)[VOTE] = 32;
                    break;
                } else {
                    cout << "it is a new promoted candidate" << endl;
                    cout << "target length is " << target_list.size() << endl;
                    alreadyFound = false;
                }
            }

            if (!alreadyFound) {
                Vec6f p(cvRound((*it)[XCOORD]),
                        cvRound((*it)[YCOORD]),
                        cvRound((*it)[RADIUS]),
                        cvRound((*it)[VOTE]),
                        label,
                        1);
                label++;
                target_list.push_back(p);
                cout << "target length is " << target_list.size() << " label "
                    << label_num << endl;
            } else {
                cout << "do nothing\n";
            }
        }

        // TODO remove components from target list

        // remove from the candidates
        if ((*it)[VOTE] <= 0 ) {
            candidate_list.erase(it);
        }
    }
}

void localizer::electedRemoval(vector<Vec3f> &cur) {

    vector<Vec3f>::iterator it;
    int tol = 20;
    for (size_t i = 0; i < target_list.size(); i++) {
        for (it = cur.begin(); it < cur.end(); it++) {
            if(fabs((*it)[XCOORD] - target_list[i][XCOORD]) <= tol
             && fabs((*it)[YCOORD]- target_list[i][YCOORD]) <= tol) {
                // which means found
            } else {
                // target is not found in data
                target_list[i][VOTE] -= 5;
                cout << target_list[i] << "seems not shown\n";
            }
        }

        if( target_list[i][VOTE] <= 0) {
            target_list[i][VOTE] =0;
            target_list[i][VISIBLE] = 0;
            cout << "target " << target_list[i][LABEL] << " is removed";
        }
    }
}

void localizer::showDataInfo() {
    cout << "Candidate contains " << candidate_list.size() << ", target " << target_list.size() << endl;
}

vector<Vec6f> localizer::detectTargets() {
  vector<Vec3f> circles;
  /// Apply the Hough Transform to find the circles
  HoughCircles( gray_img, circles, CV_HOUGH_GRADIENT, 1, gray_img.rows/8,
          edgeDet,
          centerDet,
          min_radius,
          max_radius);

  std::sort(circles.begin(), circles.end(), targetSortCriteria);

  // init the candidates
  if (candidate_list.size() == 0 && circles.size() != 0 ) {
      // If no target list, initialize it
      cout << "candidates intialize\n";
      // Preprocessing raw data
      putDataInCandidates(circles);
      findTargets = true;
      // Sort and label the data

    } else if (circles.size() != 0) {
      // find and vote circles[j] in candidates, if cant find then add
        voteCandidates(circles, 10);
        candidatesRuling();
        candidatesPromotion();
        electedRemoval(circles);
    }

  return target_list;
}

/*
 * Search circles in candidates -> push to target_list
 */
struct FindInData: public std::binary_function<Vec4f, Vec3f, bool> {
    bool operator() (const Vec4f &a, const Vec3f &b) const {
        return a[0] == b[0] && a[1] == b[1];
    }
};

