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


static Ptr<Tracker> createTrackerType(const String trackername);

static vector<Rect2d> cutRect(const String videoname);

vector<double> calculateIoU(const String videoname, Ptr<Tracker> tracker,
                                     vector<Rect2d> bounds, bool unbiased);

double IoU_eval(Rect2d bbox_a, Rect2d bbox_d);

double unbiased_IoU_eval(Rect2d bbox_a, Rect2d bbox_d, double A_bg);

//vector<Rect2d> read_box(vector<String> saved_box);
vector<Rect2d> read_box(String fname);

void save_box(vector<Rect2d> bounds, String fname);



