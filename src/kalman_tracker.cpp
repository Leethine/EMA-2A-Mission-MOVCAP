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
