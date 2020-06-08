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
                    // of tracking failed, just append 0.0 to vector, we'll know it's a failure 
                    else { 
                        results.push_back(0.0);
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
    
    if ( bbox_da.area() == 0 ) {
        return 0.0;
    }
    
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
    
    cout << bbox_d.area() << endl;
    
    // if they don't intersect at all => precision = 0 
    if ( bbox_da.area() == 0 ) {
        return 0.0;
    }
    
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

void drawrect(String vidname, String outfname, vector<Rect2d> bounds, const Scalar & colour) {

    VideoCapture video;
    video.open(vidname);
    
    VideoWriter vout(outfname, VideoWriter::fourcc('M','J','P','G'), 20, 
                Size( video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT) )); 
    
    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    else {
    
        const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        Mat frame;
        
        namedWindow("Tracking");
        Rect2d bbox = bounds.at(0);

        //usleep(1000);
        
        for (int i = 0; i < n_frames; ++i) {
            
            Rect2d bbox = bounds.at(i);
            bool readok = video.read(frame);
            
            if (!readok) {
                cerr << "(drawrect) Problem occured in reading video frames\n";
                break;
            }
            else {
                //rectangle(frame, Point2f(bbox.x, bbox.y), 
                //          Point2f(bbox.x + bbox.width, bbox.y + bbox.height), colour);
                rectangle(frame, bbox, colour, 2, 8, 0);
                //this_thread::sleep_for(std::chrono::milliseconds(100));
                
                vout.write(frame);
            }
        }
    }
    
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

void calculateIoU_genvid(const string videoname, const string outfname,  
                            string trackertype, vector<Rect2d> bounds, bool verbose) {
                            
    Ptr<Tracker> tracker = createTrackerType(trackertype);
    double unbiased_IoU_eval(Rect2d bbox_a, Rect2d bbox_d, double A_bg);
    
    // run the calculation according to the number of evaluation selected
    VideoCapture video;
    video.open(videoname);
    
    VideoWriter vout(outfname, VideoWriter::fourcc('M','J','P','G'), 20, 
                Size( video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT) ));
                
    //vector<double> results;
    
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
        
        if (verbose) {
            cout << "VIDEO_FILE: " 
                 << videoname << endl 
                 << "TRACKER: " << trackertype << endl 
                 << "----------------" << endl ;
        }
        
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
                    rectangle(frame, initbbox, Scalar(255,0,0), 2, 8, 0);
                    vout << frame;
                }
                else {
                    bool trackok = tracker->update(frame, trackingbox);
                    if (trackok) {
                        double acc;
                        acc = unbiased_IoU_eval(annotbox, trackingbox, (double) area);
                        //rectangle(frame, initbbox, Scalar(0,255,255), 2, 8, 0);
                        
                        double acciou = IoU_eval(annotbox, trackingbox); 
                        
                        string msg = trackertype;
                        msg.append(" No.");
                        msg.append(to_string(i));
                        msg.append(" frame");
                        
                        string msg2 = "IoU: ";
                        msg2.append(to_string(acciou));
                        
                        string msg3 = "unbiased IoU: ";
                        msg3.append(to_string(acc));
                        
                        //cout << msg << endl;
                        
                        rectangle(frame, annotbox, Scalar(255,0,0), 2, 8, 0);
                        rectangle(frame, trackingbox, Scalar(0,255,255), 2, 8, 0);
                        
                        putText(frame, msg, Point2f(10,25), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,230,255),2);
                        putText(frame, msg2, Point2f(10,60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,230,255),2);
                        putText(frame, msg3, Point2f(10,95), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,230,255),2);
                        
                        if (verbose) {
                            cout << msg << endl 
                                 << msg2 << endl 
                                 << msg3 << endl
                                 << "----------------" << endl;
                        }
                        //results.push_back(acc);
                        vout << frame;
                    }
                    // of tracking failed, just append 0.0 to vector, we'll know it's a failure 
                    else { 
                        //results.push_back(0.0);
                        string msg = trackertype;
                        msg.append(" No.");
                        msg.append(to_string(i));
                        msg.append(" frame");
                        string msg2 = "Tracking failed!";
                        putText(frame, msg, Point2f(10,25), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,230,255),2);
                        putText(frame, msg2, Point2f(10,60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255),2);
                        
                        cout << "-1" << endl;
                        
                        if (verbose) {
                            cout << msg << endl 
                                 << msg2 << endl
                                 << "----------------" 
                                 << endl;
                        }
                        
                        vout << frame;
                    }
                }
            }
            
        }
    }
    
    //return results;
}

static Mat bc_adjust(const Mat& orig_img, double alpha, double beta) {
    
    Mat new_img = Mat::zeros( orig_img.size(), orig_img.type() );
    for( int y = 0; y < orig_img.rows; y++ ) {
        for( int x = 0; x < orig_img.cols; x++ ) {
            for( int c = 0; c < orig_img.channels(); c++ ) {
                new_img.at<Vec3b>(y,x)[c] =
                  saturate_cast<uchar>( alpha*orig_img.at<Vec3b>(y,x)[c] + beta );
            }
        }
    }
    return new_img;
}

static Mat gamma_corr(const Mat& orig_img, double gamma) {
    
    Mat new_img = Mat::zeros( orig_img.size(), orig_img.type() );
    for( int y = 0; y < orig_img.rows; y++ ) {
        for( int x = 0; x < orig_img.cols; x++ ) {
            for( int c = 0; c < orig_img.channels(); c++ ) {
                new_img.at<Vec3b>(y,x)[c] =
                  saturate_cast<uchar>( pow( 
                      (double) orig_img.at<Vec3b>(y,x)[c]/255, 
                      gamma ) * 255 );
            }
        }
    }
    return new_img;
}


void vid_gamma_corr(const string vidname, double gamma) {

    VideoCapture video;
    video.open(vidname);
    string outfname = vidname;
    outfname.append(".avi");
    
    VideoWriter vout(outfname, VideoWriter::fourcc('M','J','P','G'), 20,
                     Size( video.get(CAP_PROP_FRAME_WIDTH),

                     video.get(CAP_PROP_FRAME_HEIGHT) ));    
    Mat frame;
    Mat outframe;
    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    else {
        const int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        for (int i = 0; i < n_frames; i++) {
            bool readok = video.read(frame);
            if (!readok) {
                cerr << "Problem occured during frame reading." << endl;
            }
            else {
                outframe = gamma_corr(frame, gamma);
                vout << outframe;
            }
        }
    }
}

int main(int argc, char ** argv) {
    
    String vidname = argv[1];
    String textname = argv[2];
    String trackername = argv[3];
    
    //string newvidname = vidname;
    //newvidname.append("2.avi");
    
    //vector<Rect2d> bounds =  cutRect(vidname);
    //save_box(bounds, "result.txt");
    
    vector<Rect2d> bounds = read_box(textname);
    //drawrect( vidname, "output.avi", bounds, Scalar(0,255,255) );
    calculateIoU_genvid(vidname, "output.avi", trackername, bounds, false);
    
    return 0;
}

/*
int main(int argc, char ** argv) {
    String vidname = argv[1];
    vid_gamma_corr(vidname, 0.7);
    
    return 0;
}

*/


