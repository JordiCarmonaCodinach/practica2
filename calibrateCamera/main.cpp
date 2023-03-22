#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/persistence.hpp>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
using namespace cv::aruco;

// Functions declarations
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags, const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) ;
void readParamsFile(string filename, Ptr<DetectorParameters> &parameters);


int main(int argc, char **argv)
{
    // Throws an error if wrong number of arguments
    if (argc <= 7 ) {
        cerr << "Insufficient parameters: (ID of the dictionary, Parameters file, Rows, Columns, Length of one side of the Aruco Marker, Distance between markers, Output file name): " << endl;
        return -1;
    }

    // Program parameters variables
    string filename = argv[2];
    int rows = stoi(argv[3]);
    int cols = stoi(argv[4]);
    float pixelSize = stof(argv[5]);
    float pixelSeparation = stof(argv[6]);
    string outputFile = argv[7];

    // Calibration variables
    Mat cameraMatrix, distCoeffs;
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector<vector < Point2f>> allCornersConcatenated;
    vector<int> allIdsConcatenated, markerCounterPerFrame, ids;
    vector<Mat> rvecs, tvecs;
    vector< vector< Point2f > > corners, rejected;
    Size imgSize;
    int aspectRatio = 1, calibrationFlags = 0;

    // Program variables
    char charCheckForESCKey = 0;
    Mat imgOriginal, imgOutput;
    int nCaptures = 1;
    double repError;


    // List of existent dictionaries
    map<string, PREDEFINED_DICTIONARY_NAME> dictionaryMap = {
        { "DICT_4X4_50", DICT_4X4_50},
        { "DICT_4X4_100", DICT_4X4_100},
        { "DICT_4X4_250", DICT_4X4_250},
        { "DICT_4X4_1000", DICT_4X4_1000},
        { "DICT_5X5_50", DICT_5X5_50},
        { "DICT_5X5_100", DICT_5X5_100},
        { "DICT_5X5_250", DICT_5X5_250},
        { "DICT_5X5_1000", DICT_5X5_1000},
        { "DICT_6X6_50", DICT_6X6_50},
        { "DICT_6X6_100", DICT_6X6_100},
        { "DICT_6X6_250", DICT_6X6_250},
        { "DICT_6X6_1000", DICT_6X6_1000},
        { "DICT_7X7_50", DICT_7X7_50},
        { "DICT_7X7_100", DICT_7X7_100},
        { "DICT_7X7_250", DICT_7X7_250},
        { "DICT_7X7_1000", DICT_7X7_1000},
        { "DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL},
        { "DICT_APRILTAG_16h5", DICT_APRILTAG_16h5},
        { "DICT_APRILTAG_25h9", DICT_APRILTAG_25h9},
        { "DICT_APRILTAG_36h10", DICT_APRILTAG_36h10},
        { "DICT_APRILTAG_36h11", DICT_APRILTAG_36h11}
    };

    // Select the choosen dictionary
    PREDEFINED_DICTIONARY_NAME dictionaryID = dictionaryMap.find(argv[1])->second;

    // Create the dictionary
    Ptr<Dictionary > dictionary = getPredefinedDictionary(dictionaryID);

    // Create the parameters variable
    Ptr<DetectorParameters > parameters = DetectorParameters::create();

    // Read paramters. Llegeix els paràmetres correctament però no detecta cap marca. En canvi quan no llegim els paràmetres i utilitzem els per defecte funciona correctament
    //readParamsFile(filename, parameters);

    // Create the Arcuo Board
    Ptr<GridBoard > gridBoard = GridBoard::create(cols, rows, pixelSize, pixelSeparation, dictionary);

    // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one
    VideoCapture webCam(0);

    // Check if the VideoCapture object has been correctly associated to the webcam
    if (webCam.isOpened() == false) {
        cerr << "error: Webcam could not be connected." << endl;
        return -1;
    }


    // VIDEO CATPURE
    // Loop until ESC key is pressed or webcam is lost
    while (charCheckForESCKey != 27 && webCam.isOpened()) {
        // Get next frame from input stream
        bool frameSuccess = webCam.read(imgOriginal);

        // If the frame was not read or read wrongly
        if (!frameSuccess || imgOriginal.empty()) {
            cerr << "error: Frame could not be read." << endl;
            break;
        }

        // Detect markers
        detectMarkers(imgOriginal, dictionary, corners, ids, parameters, rejected);
		
		cout << rejected.size()  << endl;

        // Draw results if at least 1 marker has been detected
        imgOriginal.copyTo(imgOutput);
        if(ids.size() > 0) drawDetectedMarkers(imgOutput, corners, ids);

        // Show the drawn markers
        imshow("Calibration", imgOutput);

        // If user click 'c' it captures a frame
        if(charCheckForESCKey == 99) {

            // All markers has been detected
            if(ids.size() >= (cols * rows)) {
                cout << "Frame " << nCaptures << " captured" << endl;
                nCaptures++;

                // Save corners, ids and image size
                allCorners.push_back(corners);
                allIds.push_back(ids);
                imgSize = imgOutput.size();
            }
            // Not all markers has been detected
            else {
                cout << "Invalid capture, all markers must be detected. Try again" << endl;
            }
        }

        // Wait for a key event to occur, or exit after 1 ms
        charCheckForESCKey = waitKey(1);
    }

    // At least 20 valid captures are needed for an optimal result, but for testing proposes we only demand 10
    if(allIds.size() < 10 ) {
        cerr << "Not enough captures for calibration, at least 10 captures are needed" << endl;
        return 0;
    }

    // Concatenate all the corners and all the ids of all the aruco markers detected in each capture
    markerCounterPerFrame.reserve(allCorners.size());
    for (unsigned int i = 0; i < allCorners.size(); i++)
    {
        markerCounterPerFrame.push_back((int) allCorners[i].size());
        for (unsigned int j = 0; j < allCorners[i].size(); j++)
        {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }
	
    // Calibrate camera, it returns the re-projection error
    repError = calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, gridBoard, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

    // Save the camera params
    bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags, cameraMatrix, distCoeffs, repError);

    if (!saveOk)
    {
        cerr << "Error at saving calibration file" << endl;
        return 0;
    }
    else cout << "Calibration saved to " << outputFile << endl;

    return 0;
}

static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags, const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {

    // Open file in write mode
    FileStorage fs(filename, FileStorage::WRITE);

    // If can't open return false
    if(!fs.isOpened()) return false;

    // Get current time and transform it into a string
    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    // Time
    fs << "calibration_time" << buf;
    // Image resolution
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    // Calibration parameters
    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

void readParamsFile(string filename, Ptr<DetectorParameters> &parameters) {
    // Read the params file
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened())
    {
        cerr << "error: " << filename << " could not be read." << endl;
    }
    else
    {
        fs["adaptiveThreshWinSizeMin"] >> parameters->adaptiveThreshWinSizeMin;
        fs["adaptiveThreshWinSizeMax"] >> parameters->adaptiveThreshWinSizeMax;
        fs["adaptiveThreshWinSizeStep"] >> parameters->adaptiveThreshWinSizeStep;
        fs["adaptiveThreshWinSize"] >> parameters->adaptiveThreshWinSizeMin;
        fs["adaptiveThreshConstant"] >> parameters->adaptiveThreshConstant;
        fs["minMarkerPerimeterRate"] >> parameters->minMarkerPerimeterRate;
        fs["maxMarkerPerimeterRate"] >> parameters->maxMarkerPerimeterRate;
        fs["polygonalApproxAccuracyRate"] >> parameters->polygonalApproxAccuracyRate;
        fs["minCornerDistance"] >> parameters->minCornerDistanceRate;
        fs["minDistanceToBorder"] >> parameters->minDistanceToBorder;
        fs["minMarkerDistance"] >> parameters->minMarkerDistanceRate;
        fs["minMarkerDistanceRate"] >> parameters->minMarkerDistanceRate;
        fs["cornerRefinementMethod"] >> parameters->cornerRefinementMethod;
        fs["cornerRefinementWinSize"] >> parameters->cornerRefinementWinSize;
        fs["cornerRefinementMaxIterations"] >> parameters->cornerRefinementMaxIterations;
        fs["cornerRefinementMinAccuracy"] >> parameters->cornerRefinementMinAccuracy;
        fs["markerBorderBits"] >> parameters->markerBorderBits;
        fs["perspectiveRemovePixelPerCell"] >> parameters->perspectiveRemovePixelPerCell;
        fs["perspectiveRemoveIgnoredMarginPerCell"] >> parameters->perspectiveRemoveIgnoredMarginPerCell;
        fs["maxErroneousBitsInBorderRate"] >> parameters->maxErroneousBitsInBorderRate;
        fs["minOtsuStdDev"] >> parameters->minOtsuStdDev;
        fs["errorCorrectionRate"] >> parameters->errorCorrectionRate;

        cout << filename << " readed\n";
    }
}