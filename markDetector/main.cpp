#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace cv::aruco;

int main(int argc, char* argv[]) {

    // Throws an error if wrong number of arguments
    if (argc <= 1 ) {
        cerr << "Insufficient parameters: (ID of the dictionary): " << endl;
        return -1;
    }

    // VideoCapture object declaration. Usually 0 is the integrated, 2 is the first external USB one
    VideoCapture webCam(0);

    // Check if the VideoCapture object has been correctly associated to the webcam
    if (webCam.isOpened() == false) {
        cerr << "error: Webcam could not be connected." << endl;
        return -1;
    }

    // Variables
    Mat imgOriginal, imgOutput;
    char charCheckForESCKey = 0;
    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners;
    string message = "";

    // Loop until ESC key is pressed or webcam is lost
    while (charCheckForESCKey != 27 && webCam.isOpened()) {
        // Get next frame from input stream
        bool frameSuccess = webCam.read(imgOriginal);

        // If the frame was not read or read wrongly
        if (!frameSuccess || imgOriginal.empty()) {
            cerr << "error: Frame could not be read." << endl;
            break;
        }

        // List of existent dictionaries
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

        // Detect every marker in the image
        detectMarkers(imgOriginal, dictionary, markerCorners, markerIds);

        // We copy the image, so we can detect markers for every dictionary
        imgOutput = imgOriginal.clone();

        // Draw the detected markers
        drawDetectedMarkers(imgOutput, markerCorners, markerIds);

        // Show the drawn markers
        imshow("Aruco Markers Detection", imgOutput);

        // Wait for a key event to occur, or exit after 1 ms
        charCheckForESCKey = waitKey(1);
    }

    return 0;
}

