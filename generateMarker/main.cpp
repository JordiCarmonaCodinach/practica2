#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;
using namespace aruco;

int main(int argc, char* argv[]) {
    // Throws an error if wrong number of arguments.
    if (argc <= 4) {
        cerr << "Insuficient parameters: (Dictionary, ID of the mark, Pixel Size of the mark, Name of the output PNG): " << endl;
        return -1;
    }

    // Variables
    char charCheckForESCKey = 0;
    int idMark = stoi(argv[2]);
    int pixelSize = stoi(argv[3]);
    string pngName = argv[4];
    Mat markerImage;

    // Create a new window to display the generated Aruco Marker
    namedWindow("arcuoMarker", WINDOW_AUTOSIZE);

    //List of existent dictionaries
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

    // Get the specified dictionary
    Ptr<Dictionary> dictionary = getPredefinedDictionary(dictionaryID);

    // Draw the marker
    drawMarker(dictionary, idMark, pixelSize, markerImage, 1);

    // Create a white border of 50 pixels so that the contours of the mark can be detected
    copyMakeBorder(markerImage, markerImage, 50, 50, 50, 50, BORDER_CONSTANT, Scalar(255, 255, 255));

    // Save the marker image as png
    imwrite(pngName, markerImage);

    while (charCheckForESCKey != 27) {
        // Display the marker image in a window
        imshow("arcuoMarker", markerImage);

        // Program ends if we press the esc button
        charCheckForESCKey = waitKey(1);
    }

    return 0;
}

