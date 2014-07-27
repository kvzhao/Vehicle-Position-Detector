#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

#include "localizer.h"

using namespace std;

const int MAX_CHARS_PER_LINE = 512;
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";


int main(int argc, char* argv[]) {

  localizer posFinder;

  // create a file-reading object
  ifstream fin, fin2;
  fin.open("camshiftParameters.txt"); // open a file
  fin2.open("houghParameters.txt");

  if (!fin.good()) {
	cout << "Find find Camshift paremters, init with naive param.\n";
    posFinder.setCamshiftParameters(0, 0, 0);
    //return 1; // exit if file not found
  } else {
      // TODO: read from file
      cout << "read the text file\n";
    int Vmin, Vmax, Smin;
    int x, y, w, h;
    // read each line of the file
    string line;
    vector<string> vparams;
    // read each line
    while (getline(fin, line)) {
        istringstream iss(line);
        string word;
        while (iss >> word)
            vparams.push_back (word);
    }
    fin.close();
    Vmin = atoi( vparams[0].c_str() );
    Vmax = atoi( vparams[1].c_str() );
    Smin = atoi( vparams[2].c_str() );
    x    = atoi( vparams[3].c_str() );
    y    = atoi( vparams[4].c_str() );
    w    = atoi( vparams[5].c_str() );
    h    = atoi( vparams[6].c_str() );
    cout << "Vmin: " << Vmin << "\nVmax: " << Vmax << "\nSmin: " << Smin << "\n";
    cout << "x: " << x <<" y: " << y << " w: " << w << " h: " << h << endl;

    posFinder.setCamshiftParameters(Vmin, Vmax, Smin);
    posFinder.setROI_Rect(x, y, w, h);
  }

  if(!fin2.good()) {
	cout << "Find find Hough Circle paremters, init with naive param.\n";
    posFinder.setHoughCircleParameters(200, 100, 0, 0);
  } else {
    int edge, cen, min, max;
    // read each line of the file
    string line;
    vector<string> vparams;
    // read each line
    while (getline(fin2, line)) {
        istringstream iss(line);
        string word;
        while (iss >> word)
            vparams.push_back (word);
    }
    fin2.close();
    edge = atoi( vparams[0].c_str() );
    cen  = atoi( vparams[1].c_str() );
    min  = atoi( vparams[2].c_str() );
    max  = atoi( vparams[3].c_str() );
    cout << "edge: " << edge << " cen: " << cen << " min: " << min << " max: " << max << "\n";
    posFinder.setHoughCircleParameters(edge, cen, min ,max);
  }

    /*
     * Camshift parameters
     * Hough circle parameters
     */
    posFinder.init();

    while(posFinder.getFrame()) {

        Point3d v_pose = posFinder.detectVehicle();
        // cout << "Vehicle: " << v_pose << "\n";
        vector<Vec3f> town_pos = posFinder.detectTargets();

        posFinder.showResult();
        posFinder.showDataInfo();

        char c = waitKey(15);
        if( c == 27 )
            break;
    }

    return 0;
}
