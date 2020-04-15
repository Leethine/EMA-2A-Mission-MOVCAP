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
#include "opencv2/highgui.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <list>
#include <iterator>
#include <fstream>
#include <unistd.h>


using namespace std;
using namespace cv;

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


void vid_bw_corr(const string vidname, double alpha, double beta) {

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
                outframe = bc_adjust(frame, alpha, beta);
                vout << outframe;
            }
        }
    }
}


int main(int argc, char ** argv) {
    string vidname = argv[1];
    if (argc == 3) {
        vid_gamma_corr( vidname, stod(argv[2]) );
    }
    
    else if (argc == 4) {
        vid_bw_corr( vidname, stod(argv[2]), stod(argv[3]) );
    }
    return 0;
}

