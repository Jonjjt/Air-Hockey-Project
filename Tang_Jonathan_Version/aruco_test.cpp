////////////////////////////////////////////////////////////////////////
/* air_hockey_camera_code

It uses the the examples/cpp/common/tcamcamera.* files as wrapper around the
GStreamer code and property handling. Adapt the CMakeList.txt accordingly.

As sample image processing an OpenCV cv::Mat is generated and passed on to processing class
*/

// [ TIS SDK HEADERS ]
#include "tcamimage.h"
#include "tcamprop.h"

// [ GENERAL HEADERS ]
#include <iostream>
#include <string>
#include <chrono>
#include <algorithm>
#include <thread>
#include <vector>
#include <numeric>
#include <fstream>


// [ TBB HEADERS ]
#include <tbb/concurrent_queue.h>
#include <tbb/pipeline.h>

// [ OPENCV HEADERS ]
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp" 
#include <opencv2/calib3d.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

// [ CUDA HEADERS ]
#include <cuda_runtime.h>
#include <cuda.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudawarping.hpp>


//////////[ MAIN FUNCTION ]//////////////////////////////////////////////////////////////////

//----[ INITIALISE GLOBAL VARIABLES ]-----------------------------------------------

const int iterations = 10; // Number of trials
int frameMax = 10000;
const std::string results_filename = "../results/results_cpp.csv";

// -------[ INITIALISE DATA STRUCTS ]------------------------------------------

// [ TIME ]
int64 work_begin;

struct TimeData
{
  // [ TIME ARRAYS ]
  std::vector<double> capT, getT, cvT, upT, unT, dnT, crT, reT, inT, 
  smT, hsvT, thT, moT, mo1T, mo2T, seT, viT, kal_num;

  double _capT, _getT, _cvT, _upT, _unT, _dnT, _crT, _reT, _inT, 
  _smT, _hsvT, _thT, _moT, _mo1T, _mo2T, _seT, _viT, kal_total, kal_true;

  double t_total, frame_num;
};

/////////////////////////////////////////////////////////////////////////////////////////////

//////////[ PARALLEL PROGRAM TEST ]/////////////////////////////////////////////////

// -------[ INITIALISE DATA STRUCTS ]------------------------------------------

// [ PIPELINE FLAG ]
volatile bool done = false;  // volatile is enough here. We don't need a mutex for simple flag

// [ GENERAL ]
struct ProcessingChainData
{
  /* [ NOTE ]
      Structure to travel along with the pipeline.
      Contains all the local data
  */
  // [ GENERAL ]
  cv::Mat img = cv::Mat(1080, 1440, CV_8UC4);
  cv::Mat bgrImg = cv::Mat(1080, 1440, CV_8UC3);
  //cv::Mat unImg = cv::Mat(1080, 1440, CV_8UC3);

  cv::Mat cropImg, smallImg, unImg, notImg, smoothImg, biImg,
      hsvImg, threshImg, morph1Img, morph2Img, resultImg;

  // [ GPU CUDA ]
  cv::cuda::GpuMat g_upImg = cv::cuda::GpuMat(1080, 1440, CV_8UC3); 
  cv::cuda::GpuMat g_unImg = cv::cuda::GpuMat(1080, 1440, CV_8UC3); 

  // [ TIME KEEP ]
  double cap_T, get_T, cv_T, up_T, un_T, dn_T, cr_T, re_T, in_T, sm_T, hsv_T, th_T, mo1_T, mo2_T, se_T, vi_T, _kal; 

  // [ SEGMENT ]
  std::vector<std::vector<cv::Point>> contours;
  cv::Point2f center; 
  float objRadius;
};


//----[ INITIALISE FUNCTIONS ]-----------------------------------------------------

/* [imageProcessingTBB function]
    Function to run the pipeline.
    Requires the cv::VideoCapture to feed it frames 
    and a queue to send the output of the pipeline to.
*/
void imageProcessingTBB(TcamImage &cam,
                      tbb::concurrent_bounded_queue<ProcessingChainData *> &guiQueue, 
                      cv::Mat map1, cv::Mat map2);


//***[ PARALLEL PROGRAM TEST: MAIN FUNCTION ]***************************************
/* [The main() function]
    - creates the pipeline to work in another thread
    - thransfers a cv::VideoCapture and a bounded queue as input and output
    - The pipeline reads the video frames by itself
    - the for-loop is iterating over the queue to get processed frames to show to the user
      - maintains responses to keyboard commansds

    [!]
    - Limit of queue capacity is 2 mssgs (to control RAM usage)
    - Termination handles the flush of the queue.
      - Called before calling join() on the pipeline thread
       [!] - want parallel pipeline to finish with join() to avoid any random crashes
    
    [?] A better loop would keep the last ProcessingChainData as long as a new one is not 
    present and will handle UI input and output regardless of the existence of a new input 
    from the queue. 
*/

double parallelProgram(TimeData &time)
{
  /* [ PARALLEL PROGRAM TEST ]
      To be used for performance comparison with sequential Test
      Only uses OpenCV (no CUDA)

      Pipeline:
      - Capture image
      - Convert to BGR (3)
      - undistort / crop img
      - smooth (Gauss)
      - Invert image and thresh
      - segment (contours)
      - Kalman filter (tracking)
      - Visualise image
  */

  // ----[ INITIALISE VARRIABLES ]------------------------------------------
  
  std::cout << "---[ STARTING PARALLEL TEST INITIALISATION ]---" << std::endl;

  done = false;

  // [ TIME ]
  int frame_counter = 0;
  //int frameMax = 1000;
  //int frameMax = 23600;
  double fps;
  double secs;
  double ticks = 0;
  double dT = 0;

  double t_start, t_total;

  // [ CAMERA ]
  TcamImage cam("25020018"); // Serail number of camera

  int frameRate_n = 2500000; // Frame rate numerator
  int frameRate_d = 10593;   // Frame rate denumerator
  cam.set_capture_format("RGBx", gsttcam::FrameSize{1440, 1080}, gsttcam::FrameRate{frameRate_n, frameRate_d});

  // [ IMAGE ]
  int frameHeight = cam.getHeight();
  int frameWidth = cam.getWidth();
  int pixls = cam.getBytesPerPixel();
  int frameRate = (frameRate_n / frameRate_d);
  int sampleRate = 2 * frameRate;

  // [ VIDEO ]
  cv::Size frameSize(685, 410);
  double videoFrameRate = 150;
  cv::VideoWriter trackVideo("trackVideo_150.avi", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), videoFrameRate, frameSize, true);
  //cv::VideoWriter threshVideo("threshVideo_150.avi", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), videoFrameRate, frameSize, true);
  std::vector<cv::Mat> threshVideoFeed, trackVideoFeed;
  trackVideoFeed.reserve(frameMax);
  threshVideoFeed.reserve(frameMax);

  // [ UNDISTORT ]
  int imgWidth = 1440;
  int imgHeight = 1080;
  cv::Size imgSize = cv::Size(imgWidth, imgHeight);
  cv::Mat cameraMatrix, distCoeffs, map1, map2;
  // Read calibration file
  cv::FileStorage fs("default.xml", cv::FileStorage::READ);
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  fs.release();
  // Get optimal camera matrix
  cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize, 1, imgSize, 0);
  // Create undistort maps for full image
  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                              newCameraMatrix, imgSize,
                              CV_32FC1, map1, map2);

  //cv::cuda::GpuMat map_x;
	//cv::cuda::GpuMat map_y;
  //map_x = cv::cuda::GpuMat(map1);
  //map_y = cv::cuda::GpuMat(map2);

  

  //std::cout << "type map: " << map1.type() << std::endl;

  tbb::concurrent_bounded_queue<ProcessingChainData *> guiQueue;
  guiQueue.set_capacity(15); // TBB NOTE: flow control so the pipeline won't fill too much RAM
  auto pipelineRunner = std::thread(imageProcessingTBB, std::ref(cam),
                                    std::ref(guiQueue), map1, map2);

  std::cout << "Pipeline Initialised ..." << std::endl;
  // TBB NOTE: GUI is executed in main thread
  ProcessingChainData *pData = 0;

  // -----[ RUN TEST ]--------------------------------------------------------------------------
  std::cout << "Test has started ..." << std::endl;

  // [ START TIME ]
  cam.start();
  t_start = (double)cv::getTickCount();
  //start = cv::getTickCount();
  //work_begin = cv::getTickCount();

  for (; !done;)
  {
    //std::cout << "Entered for loop ..." << std::endl;
    if (guiQueue.try_pop(pData))
    {
      char c = (char)cv::waitKey(1);
      if (c == 27 || c == 'q' || c == 'Q')
      {
        done = true;
      }

      // [ VISUALISE IMAGE ]
      double t1 = (double)cv::getTickCount();

      cv::imshow("Original Image", pData->img);
      //cv::imshow("undistort Image", pData->unImg);
      //cv::imshow("Crop Image", pData->cropImg);
      cv::imshow("Small  Image", pData->smallImg);
      //cv::imshow("Invert Image", pData->notImg);
      cv::imshow("Thresh Image", pData->threshImg);
      //cv::imshow("Morph - Open", pData->morph1Img);
      //cv::imshow("Morph - Close", pData->morph2Img);

      double t2 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
      pData->vi_T = t2;

      //threshVideoFeed.push_back(pData->threshImg);
      //trackVideoFeed.push_back(pData->smallImg);
      //std::cout << pData->smallImg.size().width << pData->smallImg.size().height << std::endl;

      // Save time
      time.getT.push_back(pData->get_T);
      time.cvT.push_back(pData->cv_T);
      time.upT.push_back(pData->up_T);
      time.unT.push_back(pData->un_T);
      time.dnT.push_back(pData->dn_T);
      //time.crT.push_back(pData->cr_T);
      time.reT.push_back(pData->re_T);
      time.inT.push_back(pData->in_T);
      time.hsvT.push_back(pData->hsv_T);
      time.thT.push_back(pData->th_T);
      time.mo1T.push_back(pData->mo1_T);
      //time.mo2T.push_back(pData->mo2_T);
      time.seT.push_back(pData->se_T);
      time.viT.push_back(pData->vi_T);
      time.kal_num.push_back(pData->_kal);

      frame_counter++;

      if (frame_counter > (frameMax-1))
      {
        t_total = double((cv::getTickCount() - t_start) / cv::getTickFrequency());
        done = true;
      }
      else if (done)
      {
        t_total = double((cv::getTickCount() - t_start) / cv::getTickFrequency());
      }
      /*
        if (frame_counter == 4)
        {
          cv::waitKey(0);
        }
        */

      delete pData;
      pData = 0;
    }
  }

  std::cout << "Parallel Processing Test Completed ..." << std::endl;

  // -----[ DATA CLEANUP ]--------------------------------------------------------------------------

  std::cout << "Starting Test Cleanup ..." << std::endl;

  do
  {
    delete pData;
  } while (guiQueue.try_pop(pData));
  pipelineRunner.join(); // TBB NOTE: wait for the pipeline to finish

  cam.stop();
  cv::destroyAllWindows();



  return fps;
}

//***[ PARALLEL PROGRAM TEST: TBB IMAGE PROCESSING FUNCTION ]********************

void imageProcessingTBB(TcamImage &cam,
                        tbb::concurrent_bounded_queue<ProcessingChainData *> &guiQueue, 
                        cv::Mat map1, cv::Mat map2)
{
  // [ COLORS ]
  cv::Scalar greenCol = cv::Scalar(0, 255, 0);
  cv::Scalar redCol = cv::Scalar(0, 0, 255);
  cv::Scalar blueCol = cv::Scalar(255, 0, 0);

  // [ IMAGE ]
  int frameHeight = cam.getHeight();
  int frameWidth = cam.getWidth();
  int pixls = cam.getBytesPerPixel();
  int frameRate_n = 2500000; // Frame rate numerator
  int frameRate_d = 10593;   // Frame rate denumerator
  int frameRate = (frameRate_n / frameRate_d);
  int sampleRate = 2*230;//4.24; 

  // [ CROP ]
  int offset_x = 35;
  int offset_y = 200;

  int imgWidth = 1440;
  int imgHeight = 1080;

  cv::Rect roi;
  roi.x = offset_x;
  roi.y = offset_y;
  roi.width = imgWidth - (offset_x*2);
  roi.height = imgHeight - (offset_y*2);
     
  // [ RESIZE ]
  //cv::Mat frame = cv::Mat(1080, 1440, CV_8UC4);
  const double scale = 2;
  const double fx = 1 / scale;


  cv::Mat frame = cv::Mat(1080, 1440, CV_8UC4);


  // [ THRESHOLDING ]
  // Create binary image using HSV color thresholding on inverted image
  // -> Want to binarise the color cyan in inverted image instead of red
  // Cyan is 90
// [ THRESHOLDING ]
  // Create binary image using HSV color thresholding on inverted image
  // -> Want to binarise the color cyan in inverted image instead of red
  // Cyan is 90

  // GREEN

  const cv::Scalar minThresh = cv::Scalar(120, 25, 20);
  const cv::Scalar maxThresh = cv::Scalar(200, 150,220);

  // const cv::Scalar minThresh = cv::Scalar(90, 90, 0);
  // const cv::Scalar maxThresh = cv::Scalar(120, 255, 255);

  //WHITE

  // const cv::Scalar minThresh = cv::Scalar(0, 0, 0);
  // const cv::Scalar maxThresh = cv::Scalar(30, 30, 255);



  // const cv::Scalar minThresh = cv::Scalar(85, 30, 150);
  // const cv::Scalar maxThresh = cv::Scalar(100, 70, 255);


  // const cv::Scalar minThresh = cv::Scalar(120, 25, 20);
  // const cv::Scalar maxThresh = cv::Scalar(200, 150,220);

  // [ MORPH ]
  const int morph_size = 3;
  const cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                              cv::Point(morph_size, morph_size));

  // [ KALMAN FILTER ]
  int stateSize = 6;
  int measSize = 4;
  int contrSize = 0;
  unsigned int imgType = CV_32F;
  cv::Mat state, meas;
  cv::KalmanFilter kf;
  bool FOUND = false;
  int notFoundCount = 0;    

  double ticks = 0;
  double dT = 0;                                        
  
  // Initialise Kalman filter (done once)
  std::cout << "Initialising Kalman ..." << std::endl;
  // [ INITIALISE KALMAN FILTER ]
  kf = cv::KalmanFilter(stateSize, measSize, contrSize, imgType);
  // Initialise state matrix
  state = cv::Mat(stateSize, 1, imgType); // [x,y,v_x,v_y,w,h]
  /* [NOTE]
      x,y          centroid position of the object (i.e. puck)
      v_x,v_y      velocity of the object's centroid position (pixels/s)
      w,h          size of the bounding box (i.e radius of puck)
    */

  // Initialise measurement matrix
  meas = cv::Mat(measSize, 1, imgType); // [z_x,z_y,z_w,z_h]
  /* [NOTE]
    z_x,z_y      measured centroid position of the object (i.e. puck)
    z_w,z_h      measured size of the bounding box (i.e radius of puck)
  */

  // Initialise Transition State Matrix A
  // [Note: set dT at each processing step!]
  cv::setIdentity(kf.transitionMatrix);

  // Initialise Measure Matrix H
  kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, imgType);
  kf.measurementMatrix.at<float>(0) = 1.0f;
  kf.measurementMatrix.at<float>(7) = 1.0f;
  kf.measurementMatrix.at<float>(16) = 1.0f;
  kf.measurementMatrix.at<float>(23) = 1.0f;

  // Initialize Process Noise Covariance Matrix Q
  kf.processNoiseCov.at<float>(0) = 1e-2;
  kf.processNoiseCov.at<float>(7) = 1e-2;
  kf.processNoiseCov.at<float>(14) = 5.0f;
  kf.processNoiseCov.at<float>(21) = 5.0f;
  kf.processNoiseCov.at<float>(28) = 1e-2;
  kf.processNoiseCov.at<float>(35) = 1e-2;

  // Initialize Measure Noise Covariance Matrix R
  cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
  std::cout << "Kalman Initialised ..." << std::endl;


  // ------[ PARALLEL PIPELINE ] ----------------------------------------


  tbb::parallel_pipeline(10, // TBB NOTE: (recomendation) NumberOfFilters
    // 1st filter [ CAPTURE IMAGE ]
    tbb::make_filter<void, ProcessingChainData*>(tbb::filter::serial_in_order,
    [&](tbb::flow_control& fc)->ProcessingChainData*
    {
      
      double t1 = (double)cv::getTickCount();
      // TBB NOTE: this filter feeds the input into the pipeline
      
        auto pData = new ProcessingChainData;
        // On succes do something with the image data. Here we create
        // a cv::Mat and save the image
        if (cam.snapImage(sampleRate))
        {
          t1 = (double)cv::getTickCount();

          memcpy(frame.data, cam.getImageData(), cam.getImageDataSize());

          if (done || frame.empty())
          {
            // 'done' is the custom exit flag
            // being set and check in and out of the pipeline
            done = true;

            double t2 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
            pData->get_T = t2;

            // tell the TBB to stop the timeline
            fc.stop();
            return 0;
          }
          else
          {
            pData->img = frame.clone();

            double t2 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
            pData->get_T = t2;
          }
        }

        return pData;// On succes do something with the image data. Here we create
      }
      )&
      // 2nd filter [ CONVERT IMAGE TO BGR ]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        //cv::cvtColor(pData->img, pData->bgrImg, cv::COLOR_BGRA2BGR, 3);
        cv::cvtColor(pData->img, pData->bgrImg, cv::COLOR_RGBA2BGR, 3);
        //cv::cvtColor(pData->img, pData->bgrImg, cv::COLOR_RGBA2BGR, 3);

        pData->cv_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 
        
        return pData;
      }
      )&
      // 3rd filter [ UNDISTORT]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        cv::remap(pData->bgrImg, pData->unImg, map1, map2, cv::INTER_LINEAR);

        pData->un_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 
        
        return pData;
      }
      )&
      // 6th filter [ CROP AND RESIZE IMAGE ] 
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        pData->cropImg = pData->unImg(roi);
        cv::resize(pData->cropImg, pData->smallImg, cv::Size(), fx, fx, cv::INTER_LINEAR);

        pData->re_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 

        return pData;
      }
      )&
      // 7th filter [ INVERT IMAGE ]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        cv::bitwise_not(pData->smallImg, pData->notImg);

        pData->in_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 

        return pData;
      }
      )&
      // 8th filter [ CONVERT TO HSV SPACE ]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        // Convert image frame to HSV color space
        cv::cvtColor(pData->notImg, pData->hsvImg, cv::COLOR_BGR2HSV);

        pData->hsv_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 

        return pData;
      }
      )&
      // 9th filter [ THRESH HSV IMAGE ]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        cv::inRange(pData->hsvImg, minThresh, maxThresh, pData->threshImg);

        pData->th_T  = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 

        return pData;
      }
      )&
      // 10th filter [ MORPH IMAGE - OPEN ]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        cv::morphologyEx(pData->threshImg, pData->morph1Img, cv::MORPH_OPEN, element);

        pData->mo1_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency();

        return pData;
      }
      )&
      // 11th filter [ SEGMENT PUCK AND UPDATE KALMAN ]
      tbb::make_filter<ProcessingChainData*,ProcessingChainData*>(tbb::filter::serial_in_order,
                                                                  [&](ProcessingChainData *pData)->ProcessingChainData*
      {
        double t1 = (double)cv::getTickCount();

        // find contours in the mask image
        cv::findContours(pData->morph1Img, pData->contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Only proceed if at least one contour was found
        if (pData->contours.size() > 0)
        {
          // [PROGRESS CHECK]
          //std::cout << "Contour found!" << std::endl;
          for (int idx = 0; idx < pData->contours.size(); idx++)
          {
            // Loop through contour array and only use the one within puck dimensions
            double objArea = cv::contourArea(pData->contours[idx]);

            if (objArea >= 100.0 && objArea < 500.0)
            {
              // [VALUE CHECK]
             //std::cout << "Contour area = " << objArea << std::endl;

              // Calculate the radius and center coordinates for the puck
              cv::minEnclosingCircle(pData->contours[idx], pData->center, pData->objRadius);
              // Create circle outline for visualisation
              cv::circle(pData->smallImg, pData->center, pData->objRadius, greenCol, 1);

              meas.at<float>(0) = pData->center.x;         // Centroid of the object (x)
              meas.at<float>(1) = pData->center.y;         // Centroid of the object (y)
              meas.at<float>(2) = (float)pData->objRadius; // Size of the object (x)
              meas.at<float>(3) = (float)pData->objRadius; // Size of the object (y)

              if (!FOUND) // First detection. Initialize Kalman filter
              {
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0); // Centroid of the object (x)
                state.at<float>(1) = meas.at<float>(1); // Centroid of the object (y)
                state.at<float>(2) = 0;                 // Velocity of the object (x)
                state.at<float>(3) = 0;                 // Velocity of the object (y)
                state.at<float>(4) = meas.at<float>(2); // Size of the object (x)
                state.at<float>(5) = meas.at<float>(3); // Size of the object (y)
                // <<<< Initialization

                kf.statePost = state;

                FOUND = true;
                //pData->_kal = 1;
              }
              else // Update Kalman
              {
                //pData->_kal = 1;
                kf.correct(meas); // Kalman Correction
              }
            }
          }
        }
        else // Update Kalman filter check
        {
          notFoundCount++;
          //pData->_kal = 0;

          if (notFoundCount >= 10)// Lost sight of puck
          {
            FOUND = false;    
          }
        }

        // [ KALMAN TRACKING ]
        /* [Note:] 
        [!] Set dT at each processing step!
          - Check if Kalman is already in action
          - If found:
	          - update matrix A
              - predict state
              - predict trajectory
          - Else, apply kalman correction
	      */

        double precTick = ticks;
        ticks = (double)cv::getTickCount();
        double dT = (ticks - precTick) / cv::getTickFrequency(); // in seconds

        if (FOUND) // If segmentation was successful
        {
          // Update matrix A
          kf.transitionMatrix.at<float>(2) = dT;
          kf.transitionMatrix.at<float>(9) = dT;

          // Update puck state
          state = kf.predict();

          // Show new circle for representation
          cv::circle(pData->smallImg, cv::Point2f(state.at<float>(0), state.at<float>(1)), state.at<float>(4), blueCol, 2);

          if (!isnan(state.at<float>(2)) && !isnan(state.at<float>(3)))
          {
            // visualise complete trajectory without intersection
            cv::Point2f targetPos((state.at<float>(0) + state.at<float>(2)), (state.at<float>(1) + state.at<float>(3)));

            //std::cout << "*** VISUALISE NO INTERSECTION ***" << std::endl;
            //std::cout << "*** ------------------------- ***" << std::endl << std::endl;
            cv::line(pData->smallImg, cv::Point2f(state.at<float>(0), state.at<float>(1)), targetPos, greenCol, 2);
          }
          
          pData->_kal = 1;
        }
        else // No puck detected through Kalman
        {
          // [PROGRESS CHECK]
          //std::cout << "/// NO PUCK DETECTED ///" << std::endl << std::endl;

          kf.correct(meas); // Kalman Correction
          pData->_kal = 0;
        }

        pData->se_T = ((double)cv::getTickCount() - t1)/cv::getTickFrequency(); 

        return pData;
      }
      )&
      // 12th filter
      tbb::make_filter<ProcessingChainData*,void>(tbb::filter::serial_in_order,
                                                 [&](ProcessingChainData *pData)
      {   // TBB NOTE: pipeline end point. dispatch to GUI
        if (! done)
        {
          try
          {
            guiQueue.push(pData);
          }
          catch (...)
          {
            std::cout << "Pipeline caught an exception on the queue" << std::endl;
            done = true;
          }
        }
      }
      )
      );
}



/////////////////////////////////////////////////////////////////////////////////////////////

//////////[ MAIN FUNCTION ]//////////////////////////////////////////////////////////////////

// -------[ INITIALISE DATA STRUCTS ]------------------------------------------

struct CameraProperties
{
  // [ GENERAL CAMERA PROPERTIES ]
  /* [NOTE]
    Any other properties may be added here, such as:
    
      - Exposure (shutter speed, auto exposure)
      - Gain (value, auto gain)
      - Format 

    Please refer to The Imaging Source documentation for getting and setting camera properties:
    https://www.theimagingsource.com/documentation/tiscamera/
  */
  const int rate = 120;                      // Frame Rate (FPS)
  const int width = 1440;                    // Width of image output (pixels)
  const int height = 1080;                   // Height of image output (pixels)
  const std::string serialNum = "25020018";  // Specific serial number for camera

  // [ IMAGE CROP PARAMETERS ]
  const int crop_x = 80;                     // x pixel-coordinate position where ROI rect is pinned (from top-left)
  const int crop_y = 0;                      // y pixel-coordinate position where ROI rect is pinned (from top-left)
};

struct ImageData
{
  /* [NOTE]

    Necessary to pre-allocate data parameters for function calls as memory allocation can be
    a cause for delay when using the GPU modules in OpenCV
  */

  cv::Mat frame, processedImg;
};

// -------[ MAIN FUNCTION ]------------------------------------------

int main(int argc, char **argv)
{
  std::cout << "---[ MAIN TESTING FUNCTION ]---" << std::endl;
  // -----[ INITIALISATION ] ---------------------------------------------

  std::cout << "---Starting: Main Initialisation---" << std::endl;

  // Initialise gstreamer
  gst_init(&argc, &argv);

  // Initialise camera properties 
  // [NOTE]
  // Video parameters/camera properties are initialised here for ease of access when changing and visibility
  // for reference. These will be passed on to the ImageProcessing object to initialise
  // the paramters so you only need to change them in the struct.
  CameraProperties cp;
  cv::Rect cropROI(cp.crop_x, cp.crop_y, (cp.width - cp.crop_x), (cp.height - cp.crop_y));
  ImageData d;

  // Initialise the TcamCamera object "cam" from The Imaging Source.
  // [NOTE]
  // This must be done with the serial number of the camera which is:
  // > 25020018 < 
  // [!] The camera feed is not possible to directly access with opencv functions
  TcamImage cam(cp.serialNum);

  // Set a color video format, resolution and frame rate
  cam.set_capture_format("BGRx", gsttcam::FrameSize{cp.width, cp.height}, gsttcam::FrameRate{cp.rate, 1});

  // Initialise the ImageProcessing object "camera" and initialise data parameters
  // [NOTE]
  // Optimising code for OpenCV CUDA modules requires that return arrays are pre-allocated
  // since memory allocation is a large cause for delay in running the OpenCV GPU modules
  //ImageProcessing camera(cp.width, cp.height, cropROI, CV_8UC4);

  cv::Mat new_frame;

  std::cout << "---Complete: Main Initialisation---" << std::endl;
  std::cout <<"--------------------------------------" << std::endl << std::endl;

  // ------[ TESTING ENVIRONMENT ] ----------------------------------------

  std::ofstream output_file(results_filename);
  if (!output_file)
  {
    std::cout << "Can't open log file. Aborting benchmark!" << std::endl;
    return 1;
  }
  else
  {
    std::string video_file = "Test_Video_236.avi";
    //std::string video_file = "Test_Video_full_200_CORRECT.avi";
    //std::string video_file = "Test_Video_full_100fps.avi";

    // Perform sequential test
    /*
    for (int i = 0; i < iterations; i++)
    {
      TimeData T;
      double fps = sequentialProgram(T);
      //double t_time = 1/fps;        // total time in seconds

      std::cout <<"Performance of Sequential Program: " << std::endl;
      std::cout <<"--------------------------------------" << std::endl;
      std::cout <<"Time for convert: " << T._cvT << " seconds per frame" << std::endl;
      std::cout <<"Time for undistort: " << T._unT << " seconds per frame" << std::endl;
      std::cout <<"Time for resize: " << T._reT << " seconds per frame" << std::endl;
      std::cout <<"Time for invert: " << T._inT << " seconds per frame" << std::endl;
      std::cout <<"Time for hsv: " << T._hsvT << " seconds per frame" << std::endl;
      std::cout <<"Time for thresh: " << T._thT << " seconds per frame" << std::endl;
      std::cout <<"Time for morph: " << T._moT << " seconds per frame" << std::endl;
      std::cout <<"Time for segment/track: " << T._seT << " seconds per frame" << std::endl;
      std::cout <<"Time for visualise: " << T._viT << " seconds per frame" << std::endl;
      std::cout <<"Average time for processing: " << T._capT << " seconds per frame" << std::endl;
      std::cout <<"Total time for processing: " << T.t_total << " seconds" << std::endl;
      std::cout <<"Total number of frames: " << T.frame_num << " frames" << std::endl;
      //std::cout <<"Time for everything: " << t_time << " seconds per frame" << std::endl;
      std::cout <<"FPS: " << fps <<std::endl;
      
      std::string result = video_file +
               ", C++, SEQUENTIAL TEST," 
               + " FPS," + std::to_string(fps) 
               + ", frames," + std::to_string(T.frame_num)
               + ", cvT," + std::to_string(T._cvT) 
               + ", unT," + std::to_string(T._unT) 
               + ", reT," + std::to_string(T._reT) 
               + ", inT," + std::to_string(T._inT) 
               + ", hsvT," + std::to_string(T._hsvT) 
               + ", thT," + std::to_string(T._thT)
               + ", moT," + std::to_string(T._moT)
               + ", seT," + std::to_string(T._seT)  
               + ", viT," + std::to_string(T._viT) 
               + ", avgT," + std::to_string(T._capT)
               + ", t_total," + std::to_string(T.t_total)
               + ", kalman total," + std::to_string(T.kal_total)
               + ", kalman true," + std::to_string(T.kal_true)
               + "\n";
      output_file << result;
      std::cout << result << std::endl;
    }
    */


    // Perform parallel test
    for (int i = 0; i < iterations; i++)
    {
      TimeData T;
      double fps = parallelProgram(T);
      //double t_time = 1/fps;        // total time in seconds

      std::cout <<"Performance of Parallel Program: " << std::endl;
      std::cout <<"--------------------------------------" << std::endl;
      std::cout <<"Time for get: " << T._getT << " seconds per frame" << std::endl;
      std::cout <<"Time for convert: " << T._cvT << " seconds per frame" << std::endl;
      std::cout <<"Time for GPU upload: " << T._upT << " seconds per frame" << std::endl;
      std::cout <<"Time for undistort: " << T._unT << " seconds per frame" << std::endl;
      std::cout <<"Time for CPU download: " << T._dnT << " seconds per frame" << std::endl;
      std::cout <<"Time for resize: " << T._reT << " seconds per frame" << std::endl;
      std::cout <<"Time for invert: " << T._inT << " seconds per frame" << std::endl;
      std::cout <<"Time for HSV: " << T._hsvT << " seconds per frame" << std::endl;
      std::cout <<"Time for thresh: " << T._thT << " seconds per frame" << std::endl;
      std::cout <<"Time for morph - Open: " << T._mo1T << " seconds per frame" << std::endl;
      std::cout <<"Time for segment/track: " << T._seT << " seconds per frame" << std::endl;
      std::cout <<"Time for visualise: " << T._viT << " seconds per frame" << std::endl;
      std::cout <<"Average time for processing: " << T._capT << " seconds per frame" << std::endl;
      std::cout <<"Total time for processing: " << T.t_total << " seconds" << std::endl;
      std::cout <<"Total number of frames: " << T.frame_num << " frames" << std::endl;
      std::cout <<"--------------------------------------" << std::endl;
      std::cout <<"Number of Kalman true: " << T.kal_total << " times" << std::endl;
      std::cout <<"Percentage of Kalman true: " << T.kal_true << " %" << std::endl;
      std::cout <<"FPS: " << fps <<std::endl;
      
      std::string result = video_file +
               ", C++, PARALLEL TEST,"
               + " FPS," + std::to_string(fps) 
               + ", frames," + std::to_string(T.frame_num)
               + ", cvT," + std::to_string(T._cvT) 
               + ", unT," + std::to_string(T._unT) 
               + ", reT," + std::to_string(T._reT) 
               + ", inT," + std::to_string(T._inT) 
               + ", hsvT," + std::to_string(T._hsvT) 
               + ", thT," + std::to_string(T._thT)
               + ", moT," + std::to_string(T._mo1T)
               + ", seT," + std::to_string(T._seT)  
               + ", viT," + std::to_string(T._viT) 
               + ", avgT," + std::to_string(T._capT)
               + ", t_total," + std::to_string(T.t_total)
               + ", kalman total," + std::to_string(T.kal_total)
               + ", kalman true," + std::to_string(T.kal_true)
               + ", upT," + std::to_string(T._upT)  
               + ", dnT," + std::to_string(T._dnT) 
               + ", getT," + std::to_string(T._getT) 
               + "\n";
      output_file << result;
      std::cout << result << std::endl;
    }

    output_file.close();
  }
  std::cout << "----------" << std::endl;
  std::cout << "Benchmark finished" << std::endl;
  std::cout << "==================================================="
              << "=========" << std::endl;
	
  return 0;
}
  



