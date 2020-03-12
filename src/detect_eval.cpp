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
#include "opencv2/tracking/tracker.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <list> 
#include <iterator>
#include <fstream>

using namespace std;
using namespace cv;


static Ptr<Tracker> createTrackerType(const String trackername) {
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
    if (trackername == "MOOSE") {
        //tracker = TrackerMOOSE::create();
    }
    if (trackername == "CRST") {
        //tracker = TrackerCRST::create();
    }
    
    return tracker;
}

/* New version: cut Rect for every frame */
/*

static vector<Rect2d> cutRect(const String videoname, const unsigned int n_eval) {

    vector<Rect2d> bounds;

    VideoCapture video;
    video.open(videoname);
    
    if ( !video.isOpened() ) {
        cout << "Could not open video." << endl;
        exit(1);
    }
    
    else {
        const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        Mat frame;
        int intervals = n_frames / n_eval;
        
        for (int i=0; i < n_eval; ++i) {
            for (int j=0; j < intervals; ++j) {
                // pass unselected video frames 
                bool ok = video.read(frame);
                if (!ok) break;
                
                // select the first frame
                if (i==0 && j==0) {
                    Rect2d bbox = selectROI("Boundary Selection", frame);
                    bounds.push_back(bbox);
                }
                
            }
            // for every period (n_eval periods in total), select a boundary
            bool ok = video.read(frame);
            if (!ok) break;
            Rect2d bbox = selectROI("Boundary Selection", frame);
            bounds.push_back(bbox);
        }
    }
    
    return bounds;
}
*/

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
        
        for (int i = 0; i < n_frames; ++i) {
            bool ok = video.read(frame);
            if (!ok) {
                cerr << "(cutRect) Problem occured in reading video frame\n";
                break;
            }
            else {
                Rect2d bbox = selectROI("Boundary Selection", frame);
                bounds.push_back(bbox);
            }
        }
    }
    
    return bounds;
}

/* New version: for every frame */
/*
vector<double> calculateIoU(const String videoname, const unsigned int n_eval, 
                            Ptr<Tracker> tracker, vector<Rect2d> bounds, bool unbiased) {
                            
    double IoU_eval(Rect2d bbox_a, Rect2d bbox_d);
    double unbiased_IoU_eval(Rect2d bbox_a, Rect2d bbox_d, double A_bg);
    
    // run the calculation according to the number of evaluation selected
    VideoCapture video;
    video.open(videoname);
    Mat frame;
    vector<double> results;
    
    if ( !video.isOpened() ) {
        cout << "Could not open video." << endl;
        exit(1);
    }
    
    else {
        //int intervals = n_frames / n_eval;
        const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        int intervals = n_frames / n_eval;
        Mat frame;
        // initbbox is the initial bbox for the object, it is the first elem in vector
        Rect2d initbbox = bounds.at(0);
        Rect2d trackingbox = initbbox;
        int area;
        
        for (int i=0; i < n_eval; ++i) {
        
            for (int j=0; j < intervals; ++j) {
                // track videos, without IoU calculation
                bool readok = video.read(frame);
                if (!readok) break;
                
                // on the first frame, initiate the tracker
                if (i==0 && j==0) {
                    tracker->init(frame, initbbox);
                    
                    // also read the area of the video frame
                    area = frame.rows * frame.cols;
                }
                
                bool trackok = tracker->update(frame, trackingbox);

            }
            // for every period (n_eval periods in total), evaluate the IoU
            bool trackok = tracker->update(frame, trackingbox);
            // get the annotation box
            Rect2d annotbox = bounds.at(i+1);
            
            if (trackok) {
                double acc;
                if (unbiased) {
                    acc = unbiased_IoU_eval(annotbox, trackingbox, (double) area);
                }
                else {
                    acc = IoU_eval(annotbox, trackingbox);
                }
                
                results.push_back(acc);
            }
            // of tracking failed, just append -1.0 to vector, we'll know it's a failure 
            else { 
                results.push_back(-1.0);
            } 

        }        

    }
    
    return results;
}

*/

vector<double> calculateIoU(const String videoname, Ptr<Tracker> tracker,
                                     vector<Rect2d> bounds, bool unbiased) {
                            
    double IoU_eval(Rect2d bbox_a, Rect2d bbox_d);
    double unbiased_IoU_eval(Rect2d bbox_a, Rect2d bbox_d, double A_bg);
    
    // run the calculation according to the number of evaluation selected
    VideoCapture video;
    video.open(videoname);
    Mat frame;
    vector<double> results;
    
    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    
    else {
        const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        Mat frame;
        // initbbox is the initial bbox for the object, it is the first elem in vector
        Rect2d initbbox = bounds.at(0);
        Rect2d trackingbox = initbbox;
        int area;
        
        for (int i = 0; i < n_frames; ++i) {
            bool readok = video.read(frame);
            if (!readok) {
                cerr << "(calculateIoU) Problem occured in reading video frames\n";
                break;
            }
            else {
                Rect2d annotbox = bounds.at(i);
                if (i == 0) { 
                    tracker->init(frame, initbbox);
                    area = frame.rows * frame.cols;
                }
                else {
                    bool trackok = tracker->update(frame, trackingbox);
                    if (trackok) {
                        double acc;
                        if (unbiased) {
                            acc = unbiased_IoU_eval(annotbox, trackingbox, (double) area);
                            results.push_back(acc);
                        }
                        else {
                            acc = IoU_eval(annotbox, trackingbox);
                            results.push_back(acc);
                        }
                    }
                    // of tracking failed, just append -1.0 to vector, we'll know it's a failure 
                    else { 
                        results.push_back(-1.0);
                    }
                }
            }
        }
    }
    
    return results;
}



double IoU_eval(Rect2d bbox_a, Rect2d bbox_d) {
/* calculate IoU accuracy of label bbox and prediction box */

    // bbox_a: annotation bbox, bbox_d: detection result bbox
    Rect2d bbox_da = bbox_a & bbox_d;
    double A_da = (double) bbox_da.area();
    double A_d1a = (double) bbox_a.area() - A_da;
    double A_da1 = (double) bbox_d.area() - A_da;
    
    //cout << A_da << " " << A_d1a << " " << A_da1 << endl; //dmsg
    double acc = A_da / (A_da + A_d1a + A_da1);
    return acc;
}

double unbiased_IoU_eval(Rect2d bbox_a, Rect2d bbox_d, double A_bg) {
/* calculate unbiased IoU accuracy of label bbox and prediction bbox 
 * According to the paper: Countering bias in tracking evaluations
 * by G. Hager et al, https://www.scitepress.org/Papers/2018/67148/67148.pdf
 */
    // bbox_a: annotation bbox, bbox_d: detection result bbox 
    Rect2d bbox_da = bbox_a & bbox_d;
    
    // A_d1a1 = A_bg - A_union(bbox_a, bbox_d): background area - area of union of two boxes
    double A_da = (double) bbox_da.area();
    double A_d1a = (double) bbox_a.area() - A_da;
    double A_da1 = (double) bbox_d.area() - A_da;
    double A_union_da = A_da + A_d1a + A_da1;
    double A_d1a1 = A_bg - A_union_da;
    
    // cout << A_da << " " << A_d1a << " " << A_da1 << " " << A_d1a1 << endl; //dmsg
    
    // cast to double
    double w0 = pow ((A_da + A_da1 + A_d1a),2)  / 
                ( pow((A_da + A_da1 + A_d1a),2) + pow((A_d1a1 + A_da1 + A_d1a),2) ) ;
                
    double wbg = 1 - w0;
    
    double acc =   w0  *  A_da / (A_da + A_d1a + A_da1)  
                 + wbg * A_d1a1 / (A_d1a1 + A_d1a + A_da1) ;
    
    return acc;
    
}

vector<Rect2d> read_box(vector<String> saved_box) {
/*
 * Read the saved file (txt) into a list of Rect2d
 */    
    vector<Rect2d> read;
    int w, h, x, y;
    for (vector<String>::iterator it = saved_box.begin();
         it != saved_box.end(); it++) {
         
        sscanf((*it).c_str(), "[%d x %d from (%d, %d)]", &w, &h, &x, &y);
        read.push_back(Rect2d(x,y,w,h));
    }
    
    return read;
}

/**

int main() {
    //cout << "Wrong arguments, for usage, $ ./detectionEvaluator -h" << endl;
    
    vector<Rect2d> boundaries = cutRect("video-1584004494.mp4"); 
    
    Ptr<Tracker> tracker = createTrackerType("MIL");

    vector<double> res = calculateIoU("video-1584004494.mp4", tracker, boundaries, true);
    
    for (vector<double>::iterator it = res.begin(); it != res.end(); it++) {
        cout << *it << "\n";
    }
    
    ifstream fp;
    fp.open("save.txt");
    String line;
    vector<String> lines;
    
    while (getline(fp,line)) {
        lines.push_back(line);
    }
    
    vector<Rect2d> bbox = read_box(lines);
    
    for (vector<Rect2d>::iterator it = bbox.begin(); it != bbox.end(); it++) {
        cout << *it << "\n";
    }
    
    return 0;


}

**/
