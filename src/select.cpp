/* detect_eval.cpp
 * 
 * Library for tracker performance testing and evaluation based on an improved IoU method
 * described in http://www.scitepress.org/DigitalLibrary/Link.aspx?doi=10.5220/0006714805810587
 *
 * Author: Tzu'An Lee
 * email: zian.li@mines-ales.org
 */


#include "opencv2/core.hpp"
#include "opencv2/video.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <list> 
#include <iterator>
#include <fstream>
#include <ctime>
#include <unistd.h>


using namespace std;
using namespace cv;

static vector<Rect2d> cutRect(const String videoname) {

    vector<Rect2d> bounds;

    VideoCapture video;
    video.open(videoname);
    
    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    
    else {
        
        const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        Mat frame;
        //resizeWindow("Boundary Selection",)
        
        for (int i = 0; i < n_frames; ++i) {
            bool ok = video.read(frame);
            
            if (!ok) {
                cerr << "(cutRect) Problem occured in reading video frame\n";
                break;
            }
            else {
                namedWindow("Boundary Selection", WINDOW_NORMAL);
                Rect2d bbox = selectROI("Boundary Selection", frame);
                bounds.push_back(bbox);
                cout << i << " frames selected" << " out of " << n_frames << endl;
            }
        }
    }
    
    return bounds;
}

vector<Rect2d> read_box(String fname) {
/*
 * Read the saved file (txt) into a list of Rect2d
 */ 
 
    ifstream fp;
    fp.open(fname);
    
    vector<String> saved_box;
    
    String line;
    while (getline(fp,line)) {
        saved_box.push_back(line);
    }    
    fp.close();

    vector<Rect2d> read;
    int w, h, x, y;
    for (vector<String>::iterator it = saved_box.begin();
         it != saved_box.end(); it++) {
         
        sscanf((*it).c_str(), "[%d x %d from (%d, %d)]", &w, &h, &x, &y);
        read.push_back(Rect2d(x,y,w,h));
    }
    
    return read;
}

void save_box(vector<Rect2d> bounds, String fname) {
/*
 * Save selected boundaries to file (txt)
 */ 
    ofstream fp;
    fp.open(fname);
    
    for ( vector<Rect2d>::iterator it = bounds.begin(); 
          it != bounds.end(); it++) {
         
        fp << *it << "\n";
    }
    
    fp.close();
    
    cout << "Manually captured boundaries." << endl 
         << "Saved as: " << fname << endl;

}


int main(int argc, char ** argv) {

    string fname = argv[1];
    string ofname = fname;
    ofname.append(".txt");
    vector<Rect2d> boundaries = cutRect(argv[1]); 
    
    save_box(boundaries, ofname);
    
    for (vector<Rect2d>::iterator it = boundaries.begin();
                             it != boundaries.end(); it++) {
        cout << *it << "\n";
    }
    
    return 0;


}

