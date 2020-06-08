/*
 * The following code is modified from 
 * https://github.com/joaofaro/KCFcpp/blob/master/src/runtracker.cpp
 * The Licence of the original code is attached below

Copyright (c) 2015, Joao Faro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of KCFcpp nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking/tracker.hpp>

#include <dirent.h>
#include <unistd.h>
#include "../include/kalman_filter.h"

#include <chrono>

using namespace std;
using namespace cv;
using namespace chrono;

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

void webcam_run(const string vidname, const string trackername) {
    Ptr<Tracker> tracker = createTrackerType(trackername);
    
	cv::VideoCapture video;
	video.open(vidname);
	
	cv::namedWindow("Tracking");

	cv::Mat frame;
	cv::Rect2d box;
	//bool is_first = true;

    string outfname = vidname;
    outfname.append("_output.avi");
    VideoWriter vout(outfname, VideoWriter::fourcc('M','J','P','G'), 20, 
                Size( video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT) ));
	
	Kalman kalman;
	//microseconds T;

    if ( !video.isOpened() ) {
        cerr << "Could not open video." << endl;
        exit(1);
    }
    
    //const unsigned int n_frames = video.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
    video.read(frame);
    Rect2d initbox = cv::selectROI("Tracking", frame);
    tracker->init(frame, initbox);
    if(waitKey(0) == 27) destroyWindow("Tracking");
    
    printf("Initiated\n");
    vout << frame;
	while (video.read(frame)) {

		auto T = duration_cast<microseconds>(system_clock::now().time_since_epoch());

		tracker->update(frame, box);
		cv::rectangle(frame, box, cv::Scalar(0, 0, 255), 3);

		auto T_new = duration_cast<microseconds>(system_clock::now().time_since_epoch());
		auto kalman_box = kalman.predict(float((T_new - T).count()) / 1'000'000, box);
        //cout << kalman_box;
		cv::rectangle(frame, kalman_box, cv::Scalar(0, 255, 0), 3);
			
		printf("after update {%d, %d, %d, %d}  ---  {%d, %d, %d, %d}\n",
				box.x, box.y, box.width, box.height,
			    kalman_box.x, kalman_box.y, kalman_box.width, kalman_box.height
				);
		//cv::imshow("Tracking",frame);
		vout << frame;
	}

	//video.release();
    cv::destroyAllWindows();
}


int main(int argc, char* argv[]){
//int main(void){
    
	webcam_run(argv[1], argv[2]);

}
