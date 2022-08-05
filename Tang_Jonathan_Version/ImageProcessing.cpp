
#include "ImageProcessing.h"


ImageProcessing::ImageProcessing(int frameWidth, int frameHeight, cv::Rect frameROI, int type)
{
    // Initialise variables
    init();

    // Initialise Kalman
    initKalman();
}

ImageProcessing::~ImageProcessing()
{
}

void ImageProcessing::init()
{
    // Read calibration file
    cv::FileStorage fs("default.xml", cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // [VALUE CHECK]
    /*]
    std::cout << std::endl
              << "camera_matrix = " << std::endl
              << cameraMatrix << std::endl;
    std::cout << std::endl
              << "distortion_coefficients = " << std::endl
              << distCoeffs << std::endl;
    */

    // [PROGRESS CHECK]
    //std::cout << "Completed Variable Initialisation" << std::endl;
}

void ImageProcessing::initKalman()
{
    /**
	Function to initialise matrices and variables for the Kalman filter. 
    This step is done when a new MainSimulation (game object) is initialised.

	- Initialize Kalman filter
	    - Initialize state and measurement (cv::Mat) matrices
        - Initialize transition state matrix A
        - Initialise measure matrix H
        - Initialize process noise covariance matrix Q
        - Initialize measurement noise covariance matrix R

    - Initialize vector values for target position estimation
        - Initialize vectors for each side of the table
        - Initialize magnitudes
        - Initialize unit normal vectors

    - Initialize wall structs for target position estimation with reflection
        - Initialize right wall struct
        - Initialize left wall struct
        - Initialize Top wall struct
        - Initialize Bottom wall struct
        - Initialize no wall struct
	*/

    // >>>> Kalman Filter
    kf = cv::KalmanFilter(stateSize, measSize, contrSize, imgType);

    // Initialize state matrix
    state = cv::Mat(stateSize, 1, imgType); // [x,y,v_x,v_y,w,h]

    // [NOTE]
    // x,y          centroid position of the object (i.e. puck)
    // v_x,v_y      velocity of the object's centroid position (pixels/s)
    // w,h          size of the bounding box (i.e radius of puck)

    // Initiralize measurement matrix
    meas = cv::Mat(measSize, 1, imgType);   // [z_x,z_y,z_w,z_h]

    // [NOTE]
    // z_x,z_y      measured centroid position of the object (i.e. puck)
    // z_w,z_h      measured size of the bounding box (i.e radius of puck)

    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Initialise Mransition State Matrix A
    // [Note: set dT at each processing step!]
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Initialize Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, imgType);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Initialize Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Initialize Measure Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // [PROGRESS CHECK]
    //std::cout << "Completed Kalman Initialisation" << std::endl;
}

////////////////////////////////////////////////////////

// Acquire frame from camera
void ImageProcessing::camToOpenCV(cv::Mat frame)
{
    originalImg = frame.clone();
    //processedImg = originalImg.clone();
    cv::cvtColor(originalImg, processedImg, cv::COLOR_BGRA2BGR, 3);

    // [VALUE CHECK]
    //std::cout << "n channels (processedImg in camToOpenCV) = " << processedImg.channels() << std::endl;

    // [PROGRESS CHECK]
    //std::cout << "Completed cam to OpenCV conversion" << std::endl;
}



// Undistort image
void ImageProcessing::undistortImg(int cameraWidth, int cameraHeight)
{
    imageSize = cv::Size(cameraWidth, cameraHeight);
    // [VALUE CHECK]
    //std::cout << "check width = " << imageSize.width << " --- " << "check height = " << imageSize.height << std::endl;

    // Undistort the image
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                                CV_16SC2, map1, map2);

    // [PROGRESS CHECK]
    //std::cout << "Completed image undistort" << std::endl;

    // Remap the undistorted image
    cv::remap(processedImg.clone(), processedImg, map1, map2, cv::INTER_LINEAR);

    // [VALUE CHECK]
    //std::cout << "n channels (processedImg in undistortImg) = " << processedImg.channels() << std::endl;

    // [PROGRESS CHECK]
    //std::cout << "Completed image remap" << std::endl;
}



// [Optional] crop image
cv::Mat ImageProcessing::cropImg()
{
    // Setup a rectangle to define your region of interest

    // [FULL SIZE CROP]
    //cv::Rect myROI(470, 200, 440, 680);

    // [SMALL SIZE CROP]
    cv::Rect myROI(10, 50, 1400, 1000);

    //cv::Mat temp = processedImg.clone();

    //Crop the full image to that image contained by the rectangle myROI
    //Note that this doesn't copy the data
    croppedRef = processedImg.clone()(myROI);

    processedImg = croppedRef.clone();

    return processedImg;

    // [PROGRESS CHECK]
    //std::cout << "Completed image crop" << std::endl;
}



// Denoise image (preprocessing)
void ImageProcessing::denoise()
{
    denoisedImg = processedImg.clone();
    double sigma = 6.0;
    cv::GaussianBlur(processedImg.clone(), denoisedImg, cv::Size(0,0), sigma, sigma );

    processedImg = denoisedImg.clone();

    // [ALTERNATIVE OPTION - SLOW]
    //denoisedImg.create(3, processedImg.cols, CV_8UC3);
    //cv::fastNlMeansDenoising(processedImg.clone(), denoisedImg, 20, 7,21);

    // [VALUE CHECK]
    //std::cout << "n channels (processedImg in denoise) = " << processedImg.channels() << std::endl;

    // [PROGRESS CHECK]
    //std::cout << "Completed image denoise" << std::endl;
}



// HSV color segmentation
void ImageProcessing::segment()
{
    /**
	Function to continuously segment puck using HSV color thresholding and contours
	- Convert image frame to HSV color space

	- Create binary image using HSV color thresholding
    - Create mask using binary image
    - Find contours in the mask image 
    - Initialize the current (x, y) center of the puck, and radius
    - Initialize Kalman if not previously initialized
	*/

    // [VALUE CHECK]
    //std::cout << "n channels (processedImg in segment) = " << processedImg.channels() << std::endl;

    // Convert image frame to HSV color space
    cv::cvtColor(processedImg.clone(), thresholdImg, cv::COLOR_BGR2HSV);

    // Create binary image using HSV color thresholding
    cv::inRange(thresholdImg.clone(), cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), binaryImg);

    // Create mask using binary image
    cv::morphologyEx(binaryImg.clone(), maskImg, cv::MORPH_OPEN, element);
    cv::morphologyEx(binaryImg.clone(), maskImg, cv::MORPH_CLOSE, element);

    // find contours in the mask image
    cv::findContours(maskImg.clone(), contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Only proceed if at least one contour was found
    if (contours.size() > 0)
    {
        // [PROGRESS CHECK]
        //std::cout << "Contour found!" << std::endl;

        for (int idx = 0; idx < contours.size(); idx++)
        {
            // Loop through contour array and only use the one within puck dimensions
            double objArea = cv::contourArea(contours[idx]);
            // [VALUE CHECK]
            //std::cout << "Contour area = " << objArea << std::endl;

            if (objArea >= 500.0)
            {
                notFoundCount = 0;

                // Calculate the radius and center coordinates for the puck
                cv::minEnclosingCircle(contours[idx], center, objRadius);

                M = cv::moments(contours[idx]);
                center_check = cv::Point2f((M.m10 / M.m00), (M.m01 / M.m00));

                // Create circle outline for visualisation
                cv::circle(processedImg, center, objRadius, cv::Scalar(0, 0, 0), 2);
                //cv::circle(mat, center_check, 2, cv::Scalar(0, 0, 0), -1);

                meas.at<float>(0) = center.x;               // Centroid of the object (x)
                meas.at<float>(1) = center.y;               // Centroid of the object (y)
                meas.at<float>(2) = (float)objRadius;       // Size of the object (x)
                meas.at<float>(3) = (float)objRadius;       // Size of the object (y)

                if (!FOUND) // First detection. Initialize Kalman filter
                {
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(7) = 1; // px
                    kf.errorCovPre.at<float>(14) = 1;
                    kf.errorCovPre.at<float>(21) = 1;
                    kf.errorCovPre.at<float>(28) = 1; // px
                    kf.errorCovPre.at<float>(35) = 1; // px

                    state.at<float>(0) = meas.at<float>(0);     // Centroid of the object (x)
                    state.at<float>(1) = meas.at<float>(1);     // Centroid of the object (y)
                    state.at<float>(2) = 0;                     // Velocity of the object (x)
                    state.at<float>(3) = 0;                     // Velocity of the object (y)
                    state.at<float>(4) = meas.at<float>(2);     // Size of the object (x)
                    state.at<float>(5) = meas.at<float>(3);     // Size of the object (y)
                    // <<<< Initialization

                    kf.statePost = state;

                    FOUND = true;
                }
                else // Update Kalman
                {
                    kf.correct(meas); // Kalman Correction
                }

            }
        }
    }
    else // Update Kalman filter check
    {
        notFoundCount++;

        if (notFoundCount >= 100)
        {
            FOUND = false;
        }
    }
}



// Kalman Prediction if FOUND == TRUE
cv::Mat ImageProcessing::predictKalman()
{
     /**
	Function to continuously update Kalman filter estimation
    [Note:] set dT at each processing step!

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
        // >>>> Matrix A
        kf.transitionMatrix.at<float>(2) = dT;
        kf.transitionMatrix.at<float>(9) = dT;
        // <<<< Matrix A

        // [VALUE CHECK]
        //std::cout << "dT:" << std::endl << dT << std::endl;

        state = kf.predict();

        // Show new circle for representation
        cv::circle(processedImg, cv::Point2f(state.at<float>(0), state.at<float>(1)), state.at<float>(4), cv::Scalar(0, 255, 0), 2);

        // [VALUE CHECK]
        //std::cout << "State post:" << std::endl << state << std::endl;
        //std::cout << "State velocity:" << std::endl << state.at<float>(2) << ", " << state.at<float>(3) << std::endl;

        if (!isnan(state.at<float>(2)) && !isnan(state.at<float>(3)))
        {
            // visualise complete trajectory without intersection
            cv::Point2f targetPos((state.at<float>(0) + state.at<float>(2)), (state.at<float>(1) + state.at<float>(3)));

            //std::cout << "*** VISUALISE NO INTERSECTION ***" << std::endl;
            //std::cout << "*** ------------------------- ***" << std::endl << std::endl;
            cv::line(processedImg, cv::Point2f(state.at<float>(0), state.at<float>(1)), targetPos, cv::Scalar(0, 255, 0), 2);
        }
    }
    else // No puck detected through Kalman
    {
        // [PROGRESS CHECK]
        //std::cout << "/// NO PUCK DETECTED ///" << std::endl << std::endl;

        kf.correct(meas); // Kalman Correction
    }

    return processedImg.clone();
}



// Visualise frame
cv::Mat ImageProcessing::showFeed()
{
    // Output images for comparison
    //cv::imshow("Distorted input", originalImg);       // Distorted image
    cv::imshow("Undistorted output", processedImg);     // Undistorted image
    //cv::imshow("denoise Image", denoisedImg);           // Undistorted denoised img
    cv::imshow("mask Image", maskImg);                   // Undistorted binary img
    //cv::imshow("Crop Image", croppedRef);               // Cropped undistorted img
    
    // [VALUE CHECK]
    //std::cout << "n channels (processedImg in showFeed) = " << processedImg.channels() << std::endl;
    
    // wait for key press

    return processedImg;

    // [PROGRESS CHECK]
    //std::cout << "** Image Processing Complete **" << std::endl << std::endl;
}

