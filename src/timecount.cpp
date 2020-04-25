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


static Rect2d selectInit(const String videoname) {

    VideoCapture video;
    video.open(videoname);
    Rect2d bbox;
    
    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    
    else {
        
        Mat frame;
        
        bool ok = video.read(frame);
            
        if (!ok) {
            cerr << "(cutRect) Problem occured in reading video frame\n";
            
        }
        else {
            namedWindow("Boundary Selection", WINDOW_NORMAL);
            bbox = selectROI("Boundary Selection", frame);
            waitKey(1);
            destroyAllWindows();
        }
    }
    
    return bbox;
}



static Ptr<Tracker> createTrackerType(const string trackername) {
    // create tracker according to the trackername specified
 
    Ptr<Tracker> tracker;
    if (trackername == "MIL") {
        tracker = TrackerMIL::create();
    }
    if (trackername == "Boosting") {
        tracker = TrackerBoosting::create();
    }
    if (trackername == "KCF") {
        tracker = TrackerKCF::create();
    }
    if (trackername == "TLD") {
        tracker = TrackerTLD::create();
    }
    if (trackername == "MOSSE") {
        tracker = TrackerMOSSE::create();
    }
    if (trackername == "CSRT") {
        tracker = TrackerCSRT::create();
    }
    if (trackername == "MF") {
        tracker = TrackerMedianFlow::create();
    }
    if (trackername == "HOG"){
        //tracker = TrackerFeature::create("HOG");
    }
    if (trackername == "HAAR"){
        //tracker = TrackerFeature::create("HAAR");
    }
    
    return tracker;
}


void runTracking(const String videoname, Ptr<Tracker> tracker, Rect2d initbbox) {

    // run the calculation according to the number of evaluation selected
    VideoCapture video;
    video.open(videoname);
    Mat frame;
    
    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    
    else {
        const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        Mat frame;
        // initbbox is the initial bbox for the object, it is the first elem in vector
        Rect2d trackingbox = initbbox;
        
        for (int i = 0; i < n_frames; ++i) {
            bool readok = video.read(frame);
            if (!readok) {
                cerr << "(calculateIoU) Problem occured in reading video frames\n";
                break;
            }
            else {
                if (i == 0) { 
                    tracker->init(frame, initbbox);
                }
                else {
                    bool trackok = tracker->update(frame, trackingbox);
                }
            }
        }
    }
}


int main(int argc, char ** argv) {
    
    string vidname = argv[1];
    //String trackername = argv[2];
    
    Rect2d initbbox = selectInit(vidname);
    
    vector<string> trackers;
    
    trackers.push_back("MIL");
    trackers.push_back("Boosting");
    trackers.push_back("MOSSE");
    trackers.push_back("CSRT");
    trackers.push_back("KCF");
    trackers.push_back("TLD");
    trackers.push_back("MF");
    
    for (vector<string>::iterator it = trackers.begin(); it != trackers.end(); it++) {
        Ptr<Tracker> tracker = createTrackerType(*it);
            
        clock_t c_start = clock();

        for (int i = 0; i < 10; i++) {
            runTracking(vidname, tracker, initbbox);
        }
        clock_t c_end = clock();
        
        double time_elapsed_ms = 1000 * (c_end - c_start) / CLOCKS_PER_SEC;
        
        cout << *it << " tracker, CPU time used: " << time_elapsed_ms / 10 << "ms" << endl;   
    }
    
    return 0;
}


