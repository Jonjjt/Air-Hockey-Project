// //////[ SEQUENTIAL PROGRAM TEST ]//////////////////////////////////////

// double sequentialProgram(TimeData &stime)
// {
//   /* [ SEQUENTIAL PROGRAM TEST ]
//       To be used for performance comparison with TBB Test
//       Only uses OpenCV (no CUDA)

//       Pipeline:
//       - Capture image
//       - Convert to BGR (3)
//       - undistort / crop img
//       - smooth (Gauss)
//       - Invert image and thresh
//       - segment (contours)
//       - Kalman filter (tracking)
//       - Visualise image
//   */

//   // ----[ INITIALISE VARRIABLES ]------------------------------------------

//   std::cout << "---[ STARTING SEGMENTATION TEST INITIALISATION ]---" << std::endl;

//   // [ TIME ]
//   int frame_counter = 0;
//   //int frameMax = 1000;
//   double ticks = 0;
//   double dT = 0;
//   double t_start, t_end, t;

//   // [ CAMERA ]
//   TcamImage cam("25020018");

//   int frameRate_n = 2500000; // Frame rate numerator
//   int frameRate_d = 10593;   // Frame rate denumerator
//   cam.set_capture_format("RGBx", gsttcam::FrameSize{1440, 1080}, gsttcam::FrameRate{frameRate_n, frameRate_d});

//   // [ IMAGE ]
//   int frameHeight = cam.getHeight();
//   int frameWidth = cam.getWidth();
//   int pixls = cam.getBytesPerPixel();
//   int frameRate = (frameRate_n / frameRate_d);
//   int sampleRate = 2 * frameRate;

//   // [ VIDEO ]
//   cv::Size frameSize(frameWidth, frameHeight);
//   double videoFrameRate = 100;
//   cv::VideoWriter videoWriter("Output_Video_full_100.avi", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), videoFrameRate, frameSize, true);
//   std::vector<cv::Mat> videoFeed;
//   videoFeed.reserve(frameMax);

//   // [ GENERAL ]
//   cv::Mat img, undistortImg, cropImg, smallImg, notImg, smoothImg, biImg,
//       hsvImg, threshImg, morphImg, bgrImg, resultImg;

//   cv::Mat frame = cv::Mat(1080, 1440, CV_8UC4);

//    // [ CROP ]
//   int offset_x = 35;
//   int offset_y = 130;

//   int imgWidth = 1440;
//   int imgHeight = 1080;

//   cv::Rect roi;
//   roi.x = offset_x;
//   roi.y = offset_y;
//   roi.width = imgWidth - (offset_x*2);
//   roi.height = imgHeight - (offset_y*2);

//   // [ UNDISTORT ]
//   cv::Size imgSize = cv::Size(imgWidth, imgHeight);
//   cv::Mat cameraMatrix, distCoeffs, map1, map2;
//   // Read calibration file
//   cv::FileStorage fs("default.xml", cv::FileStorage::READ);
//   fs["camera_matrix"] >> cameraMatrix;
//   fs["distortion_coefficients"] >> distCoeffs;
//   fs.release();
//   // Get optimal camera matrix
//   cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize, 1, imgSize, 0);
//   // Create undistort maps for full image
//   cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//                               newCameraMatrix, imgSize,
//                               CV_16SC2, map1, map2);

//   // [ RESIZE ]
//   double scale = 2;
//   double fx = 1 / scale;

//   // [ GAUSSIAN SMOOTHING ]
//   int kSize = 15;
//   cv::Size kernel = cv::Size(kSize, kSize);
//   double sigma = 1;

//   // [ THRESHOLDING ]
//   // Create binary image using HSV color thresholding on inverted image
//   // -> Want to binarise the color cyan in inverted image instead of red
//   // Cyan is 90
//   const cv::Scalar minThresh = cv::Scalar(85, 40, 100);
//   const cv::Scalar maxThresh = cv::Scalar(95, 255, 255);

//   // [ MORPH ]
//   int morph_size = 3;
//   cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
//                                               cv::Point(morph_size, morph_size));

//   // [ SEGMENT ]
//   std::vector<std::vector<cv::Point>> contours;
//   cv::Point2f center; // center_check;
//   float objRadius;

//   // [ KALMAN FILTER ]
//   int stateSize = 6;
//   int measSize = 4;
//   int contrSize = 0;
//   unsigned int imgType = CV_32F;
//   cv::Mat state, meas;
//   cv::KalmanFilter kf;
//   bool FOUND = false;
//   int notFoundCount = 0;

//   // -----[ INITIALISE TEST ]------------------------------------------

//   // [ INITIALISE KALMAN FILTER ]
//   kf = cv::KalmanFilter(stateSize, measSize, contrSize, imgType);
//   // Initialize state matrix
//   state = cv::Mat(stateSize, 1, imgType); // [x,y,v_x,v_y,w,h]
//   /* [NOTE]
//       x,y          centroid position of the object (i.e. puck)
//       v_x,v_y      velocity of the object's centroid position (pixels/s)
//       w,h          size of the bounding box (i.e radius of puck)
//   */

//   // Initiralize measurement matrix
//   meas = cv::Mat(measSize, 1, imgType); // [z_x,z_y,z_w,z_h]
//   /* [NOTE]
//       z_x,z_y      measured centroid position of the object (i.e. puck)
//       z_w,z_h      measured size of the bounding box (i.e radius of puck)
//   */

//   // Initialise Mransition State Matrix A
//   // [Note: set dT at each processing step!]
//   cv::setIdentity(kf.transitionMatrix);

//   // Initialize Measure Matrix H
//   kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, imgType);
//   kf.measurementMatrix.at<float>(0) = 1.0f;
//   kf.measurementMatrix.at<float>(7) = 1.0f;
//   kf.measurementMatrix.at<float>(16) = 1.0f;
//   kf.measurementMatrix.at<float>(23) = 1.0f;

//   // Initialize Process Noise Covariance Matrix Q
//   kf.processNoiseCov.at<float>(0) = 1e-2;
//   kf.processNoiseCov.at<float>(7) = 1e-2;
//   kf.processNoiseCov.at<float>(14) = 5.0f;
//   kf.processNoiseCov.at<float>(21) = 5.0f;
//   kf.processNoiseCov.at<float>(28) = 1e-2;
//   kf.processNoiseCov.at<float>(35) = 1e-2;

//   // Initialize Measure Noise Covariance Matrix R
//   cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

//   // [ START TIME ]
//   int64 work_begin = cv::getTickCount();

//   // [ INITIALISE VIDEO CAPTURE ]

//   std::cout << "---[ SEGMENTATION TEST INITIALISATION COMPLETE ]---" << std::endl
//             << std::endl;

//   // -----[ RUN TEST ]-----------------------------------------------------
//   cam.start();
//   t_start = (double)cv::getTickCount();

//   //std::cout << "no frame" << std::endl;

//   while (true)
//   {
//     //std::cout << "no frame" << std::endl;
//     // [ CAPTURE IMAGE ]
    
//     if (cam.snapImage(sampleRate))
//     {
//       memcpy(frame.data, cam.getImageData(), cam.getImageDataSize());

//       //----------------------------------------------------------

//       // [ CONVERT IMAGE TO BGR ]
//       t = (double)cv::getTickCount();

//       cv::cvtColor(frame, bgrImg, cv::COLOR_RGBA2BGR, 3);
//       //img = frame.clone();

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.cvT.push_back(t);

//       //----------------------------------------------------------

//       // [ UNDISTORT IMAGE ]
//       t = (double)cv::getTickCount();

//       cv::remap(bgrImg, undistortImg, map1, map2, cv::INTER_LINEAR);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.unT.push_back(t);

//       //----------------------------------------------------------

//       // [ CROP IMAGE ]
//       t = (double)cv::getTickCount();

//       cropImg = undistortImg(roi);
//       cv::resize(cropImg, smallImg, cv::Size(), fx, fx, cv::INTER_LINEAR);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.reT.push_back(t);

//       //----------------------------------------------------------
//       /*
//       // [ RESIZE IMAGE ]
//       t = (double)cv::getTickCount();

//       cv::resize(cropImg, smallImg, cv::Size(), fx, fx, cv::INTER_LINEAR);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.reT.push_back(t);
//       */

//       //----------------------------------------------------------

//       // [ INVERT IMAGE ]
//       t = (double)cv::getTickCount();

//       cv::bitwise_not(smallImg, notImg);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.inT.push_back(t);

//       //----------------------------------------------------------

//       // [ CONVERT IMAGE TO HSV ]
//       t = (double)cv::getTickCount();

//       cv::cvtColor(notImg, hsvImg, cv::COLOR_BGR2HSV);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.hsvT.push_back(t);

//       //----------------------------------------------------------

//       // [ THRESH IMAGE ]
//       t = (double)cv::getTickCount();

//       cv::inRange(hsvImg, minThresh, maxThresh, threshImg);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.thT.push_back(t);

//       //----------------------------------------------------------

//       // [ MORPH IMAGE ]
//       t = (double)cv::getTickCount();

//       cv::morphologyEx(threshImg, morphImg, cv::MORPH_OPEN, element);
//       //cv::morphologyEx(morphImg, morphImg, cv::MORPH_CLOSE, element2);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.moT.push_back(t);

//       //----------------------------------------------------------

//       // [ SEGMENT PUCK ]
//       t = (double)cv::getTickCount();

//       // find contours in the mask image
//       cv::findContours(morphImg, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

//       // Only proceed if at least one contour was found
//       if (contours.size() > 0)
//       {
//         // [PROGRESS CHECK]
//         //std::cout << "Contour found!" << std::endl;
//         for (int idx = 0; idx < contours.size(); idx++)
//         {
//           // Loop through contour array and only use the one within puck dimensions
//           double objArea = cv::contourArea(contours[idx]);

//           if (objArea >= 300.0 && objArea < 380.0)
//           {
//             notFoundCount = 0;
//             // [VALUE CHECK]
//             //std::cout << "Contour area = " << objArea << std::endl;

//             // Calculate the radius and center coordinates for the puck
//             cv::minEnclosingCircle(contours[idx], center, objRadius);
//             // Create circle outline for visualisation
//             cv::circle(smallImg, center, objRadius, cv::Scalar(0, 255, 0), 1);

//             meas.at<float>(0) = center.x;         // Centroid of the object (x)
//             meas.at<float>(1) = center.y;         // Centroid of the object (y)
//             meas.at<float>(2) = (float)objRadius; // Size of the object (x)
//             meas.at<float>(3) = (float)objRadius; // Size of the object (y)

//             if (!FOUND) // First detection. Initialize Kalman filter
//             {
//               // >>>> Initialization
//               kf.errorCovPre.at<float>(0) = 1; // px
//               kf.errorCovPre.at<float>(7) = 1; // px
//               kf.errorCovPre.at<float>(14) = 1;
//               kf.errorCovPre.at<float>(21) = 1;
//               kf.errorCovPre.at<float>(28) = 1; // px
//               kf.errorCovPre.at<float>(35) = 1; // px

//               state.at<float>(0) = meas.at<float>(0); // Centroid of the object (x)
//               state.at<float>(1) = meas.at<float>(1); // Centroid of the object (y)
//               state.at<float>(2) = 0;                 // Velocity of the object (x)
//               state.at<float>(3) = 0;                 // Velocity of the object (y)
//               state.at<float>(4) = meas.at<float>(2); // Size of the object (x)
//               state.at<float>(5) = meas.at<float>(3); // Size of the object (y)
//               // <<<< Initialization

//               kf.statePost = state;

//               FOUND = true;
//             }
//             else // Update Kalman
//             {
//               kf.correct(meas); // Kalman Correction
//             }
//           }
//         }
//       }
//       else // Update Kalman filter check
//       {
//         notFoundCount++;

//         if (notFoundCount >= 100) // Lost sight of puck
//         {
//           FOUND = false;
//         }
//       }

//       // [ KALMAN TRACKING ]
//       /* [Note:] set dT at each processing step!
//       - Check if Kalman is already in action
//       - If found:
// 	      - update matrix A
//           - predict state
//           - predict trajectory
//       - Else, apply kalman correction
// 	  */

//       double precTick = ticks;
//       ticks = (double)cv::getTickCount();

//       double dT = (ticks - precTick) / cv::getTickFrequency(); // in seconds

//       if (FOUND) // If segmentation was successful
//       {
//         // Update matrix A
//         kf.transitionMatrix.at<float>(2) = dT;
//         kf.transitionMatrix.at<float>(9) = dT;

//         // Update puck state
//         state = kf.predict();

//         // Show new circle for representation
//         cv::circle(smallImg, cv::Point2f(state.at<float>(0), state.at<float>(1)), state.at<float>(4), cv::Scalar(255, 0, 0), 2);

//         if (!isnan(state.at<float>(2)) && !isnan(state.at<float>(3)))
//         {
//           // visualise complete trajectory without intersection
//           cv::Point2f targetPos((state.at<float>(0) + state.at<float>(2)), (state.at<float>(1) + state.at<float>(3)));

//           //std::cout << "*** VISUALISE NO INTERSECTION ***" << std::endl;
//           //std::cout << "*** ------------------------- ***" << std::endl << std::endl;
//           cv::line(smallImg, cv::Point2f(state.at<float>(0), state.at<float>(1)), targetPos, cv::Scalar(0, 255, 0), 2);
//         }

//         stime.kal_num.push_back(1);
//       }
//       else // No puck detected through Kalman
//       {
//         // [PROGRESS CHECK]
//         //std::cout << "/// NO PUCK DETECTED ///" << std::endl << std::endl;

//         kf.correct(meas); // Kalman Correction
//         stime.kal_num.push_back(0);
//       }

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.seT.push_back(t);

//       //----------------------------------------------------------

//       //videoFeed.push_back(smallImg);

//       // [ VISUALISE IMAGE ]
//       t = (double)cv::getTickCount();

//       cv::imshow("Undostort Image", smallImg);
//       //cv::imshow("Undostort Image", undistortImg);
//       cv::imshow("Thresh Image", threshImg);
//       //cv::imshow("Final Result", morphImg);
//       cv::waitKey(1);

//       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//       stime.viT.push_back(t);

//       frame_counter++;

//       if (frame_counter > (frameMax-1))
//       {
//         t_end = ((double)cv::getTickCount() - t_start) / cv::getTickFrequency();
//         break;
//       }
//     }
//     else
//     {
//       //std::cout << "no frame" << std::endl;
//       continue;
//     }
//   }

//   //t_end = ((double)cv::getTickCount() - t_start) / cv::getTickFrequency();
//   //stime.capT.push_back(t_end);

//   cam.stop();
//   cv::destroyAllWindows();

//   stime._cvT = (double)accumulate(stime.cvT.begin(), stime.cvT.end(), 0.0) / stime.cvT.size();
//   stime.capT.push_back(stime._cvT);
//   stime._reT = (double)accumulate(stime.reT.begin(), stime.reT.end(), 0.0) / stime.reT.size();
//   stime.capT.push_back(stime._reT);
//   stime._unT = (double)accumulate(stime.unT.begin(), stime.unT.end(), 0.0) / stime.unT.size();
//   stime.capT.push_back(stime._unT);
//   stime._inT = (double)accumulate(stime.inT.begin(), stime.inT.end(), 0.0) / stime.inT.size();
//   stime.capT.push_back(stime._inT);
//   stime._hsvT = (double)accumulate(stime.hsvT.begin(), stime.hsvT.end(), 0.0) / stime.hsvT.size();
//   stime.capT.push_back(stime._hsvT);
//   stime._thT = (double)accumulate(stime.thT.begin(), stime.thT.end(), 0.0) / stime.thT.size();
//   stime.capT.push_back(stime._thT);
//   stime._moT = (double)accumulate(stime.moT.begin(), stime.moT.end(), 0.0) / stime.moT.size();
//   stime.capT.push_back(stime._moT);
//   stime._seT = (double)accumulate(stime.seT.begin(), stime.seT.end(), 0.0) / stime.seT.size();
//   stime.capT.push_back(stime._seT);
//   stime._viT = (double)accumulate(stime.viT.begin(), stime.viT.end(), 0.0) / stime.viT.size();
//   stime.capT.push_back(stime._viT);

//   stime._capT = (double)accumulate(stime.capT.begin(), stime.capT.end(), 0.0);
//   double fps = 1.0 / stime._capT;


//   stime.t_total = t_end;
//   stime.frame_num = frame_counter;


//   // Determine Kalman number of times found
//   stime.kal_total = (double)accumulate(stime.kal_num.begin(), stime.kal_num.end(), 0.0); // compute sum
//   stime.kal_true = (stime.kal_total/frame_counter)*100;

//   return fps;
// }