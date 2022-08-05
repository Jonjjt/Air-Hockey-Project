#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
#include <numeric>
#include <algorithm>
#include <thread>

#include "tcamimage.h"
#include "tcamprop.h"
#include <cuda_runtime.h>
#include <cuda.h>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp" // highgui - an interface to video and image capturing.
#include <opencv2/calib3d.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>



class ImageProcessing
{
private:
    /* data */
    // Variables
    
    // [GENERAL]
    cv::Mat originalImg, processedImg, croppedRef, denoisedImg;

    // [UNDISTORT]
    cv::Size imageSize;
    cv::Mat cameraMatrix, distCoeffs, map1, map2;

    // [SEGMENTATION]

    /* [NOTE : WORKED]
    const int H_min = 0;
    const int S_min = 60;
    const int V_min = 90;
    const int H_max = 10;
    const int S_max = 200;
    const int V_max = 200;
    */
    const int H_min = 0;
    const int S_min = 50;
    const int V_min = 90;
    const int H_max = 10;
    const int S_max = 220;
    const int V_max = 220;
    const int morph_size = 2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*morph_size+1, 2*morph_size+1), cv::Point(morph_size,morph_size) );
    cv::Mat thresholdImg, binaryImg, maskImg;
    std::vector<std::vector<cv::Point> > contours;
    cv::Point2f center, center_check;
    float objRadius;
    cv::Moments M;

    // [KALMAN FILTER]
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int imgType = CV_32F;
    cv::Mat state, meas;
    cv::KalmanFilter kf;

    // [TIME]
    double ticks = 0;
    double dT = 0;

    // [KALMAN CHECK]
    bool FOUND = false;
    int notFoundCount = 0;

public:
    ImageProcessing(int frameWidth, int frameHeight, cv::Rect frameROI, int type);
    ~ImageProcessing();

    // Initialise variables - Done when new ImageProcessing object is made (in constructor)
    void init(); 

    // Convert to opencv img
    void camToOpenCV(cv::Mat frame);

    // [Optional] undistort image
    void undistortImg(int cameraWidth, int cameraHeight); // [COMPLETE]

    // [Optional] crop image
    cv::Mat cropImg();

    // Perform segmentation

        // Denoise image (preprocessing)
        void denoise();

        // HSV color segmentation
        void segment();

    // Kalman
    void initKalman(); // Done when new ImageProcessing object is made (in constructor)

        // Predict next state
        cv::Mat predictKalman();

        // find predicted target position

        // [DONE IN SIMULATION] 
        // predict trajectory / target position after intersecting with table walls

    // show output and return processed image to be saved to video
    cv::Mat showFeed();
};


