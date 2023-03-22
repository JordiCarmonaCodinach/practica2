#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace cv::aruco;

int main(int argc, char** argv)
{
    // Throws an error if wrong number of arguments
    if (argc <= 3 ) {
        cerr << "Insufficient parameters: (ID of the dictionary, ID of the mark, Length of one side of the Aruco Marker): " << endl;
        return -1;
    }

    // Program parameters variables
    int idMark = stoi(argv[2]);
    float markerLength = stof(argv[3]);

    // Program variables
    char charCheckForESCKey = 0;
    Mat markerImg;
    int borderBits = 1;
    Mat imgOriginal, imgOutput, cameraMatrix, distCoeffs;
    ostringstream vector_to_marker;
    vector<int> ids;
    vector<vector<Point2f> > corners;
    vector<Vec3d> rvecs, tvecs;

    // List of dictionaries
    map<string, PREDEFINED_DICTIONARY_NAME> dictionaryMap = {
        {"DICT_4X4_50", DICT_4X4_50},
        {"DICT_4X4_100", DICT_4X4_100},
        {"DICT_4X4_250", DICT_4X4_250},
        {"DICT_4X4_1000", DICT_4X4_1000},
        {"DICT_5X5_50", DICT_5X5_50},
        {"DICT_5X5_100", DICT_5X5_100},
        {"DICT_5X5_250", DICT_5X5_250},
        {"DICT_5X5_1000", DICT_5X5_1000},
        {"DICT_6X6_50", DICT_6X6_50},
        {"DICT_6X6_100", DICT_6X6_100},
        {"DICT_6X6_250", DICT_6X6_250},
        {"DICT_6X6_1000", DICT_6X6_1000},
        {"DICT_7X7_50", DICT_7X7_50},
        {"DICT_7X7_100", DICT_7X7_100},
        {"DICT_7X7_250", DICT_7X7_250},
        {"DICT_7X7_1000", DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL},
        {"DICT_APRILTAG_16h5", DICT_APRILTAG_16h5},
        {"DICT_APRILTAG_25h9", DICT_APRILTAG_25h9},
        {"DICT_APRILTAG_36h10", DICT_APRILTAG_36h10},
        {"DICT_APRILTAG_36h11", DICT_APRILTAG_36h11}
    };

    // Choose the dictionary
    PREDEFINED_DICTIONARY_NAME dictionaryID = dictionaryMap.find(argv[1])->second;

    // Create the specified dictionary
    Ptr<Dictionary> dictionary = getPredefinedDictionary(dictionaryID);

    // Read the calibrated Params file
    FileStorage fs("calibratedParams.yml", FileStorage::READ);

    // We only need camera matrix and distorntion coefficients
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

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

        // Copy the img
        imgOriginal.copyTo(imgOutput);

        // First we detect all the markers and save the corners and ids of them
        detectMarkers(imgOriginal, dictionary, corners, ids);

        // If at least one marker detected
        if (ids.size() > 0)
        {
            // We draw the detected markers
            drawDetectedMarkers(imgOutput, corners, ids);

            // Estimate the relative position of all detected markers
            estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            // Draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
				// Only display the axis for the specified ID
				if(ids[i] == idMark){
					 drawAxis(imgOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

					// Print the data for all the detected markers
					vector_to_marker.str(string());
					vector_to_marker << setprecision(4)  << "x: " << setw(8) << tvecs[0](0);
					putText(imgOutput, vector_to_marker.str(), Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 252, 124), 1, CV_AVX);

					vector_to_marker.str(string());
					vector_to_marker << setprecision(4) << "y: " << setw(8) << tvecs[0](1);
					putText(imgOutput, vector_to_marker.str(),  Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 252, 124), 1, CV_AVX);

					vector_to_marker.str(std::string());
					vector_to_marker << std::setprecision(4) << "z: " << setw(8) << tvecs[0](2);
					putText(imgOutput, vector_to_marker.str(),  Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 252, 124), 1, CV_AVX);

					// We finally draw the axis
					drawAxis(imgOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
				}
            }
        }

        // Show the drawn markers
        imshow("Pose Estimation", imgOutput);

        // Wait for a key event to occur, or exit after 1 ms
        charCheckForESCKey = waitKey(1);
    }

    return 0;
}
