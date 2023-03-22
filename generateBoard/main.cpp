#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;
using namespace cv::aruco;

int main(int argc, char* argv[]) {
    // Throws an error if wrong number of arguments
    if (argc <= 6 ) {
        cerr << "Insufficient parameters: (Rows, Columns, Dictionary, Pixel Size of the mark, Pixel Separation between marks, Name of the output PNG): " << endl;
        return -1;
    }

    // Variables
    char charCheckForESCKey = 0;
    int rows = stoi(argv[1]);
    int cols = stoi(argv[2]);
    int pixelSize = stoi(argv[4]);
    int pixelSeparation = stoi(argv[5]);
    string pngName = argv[6];
    int marginSize = 10;
    Mat markerBoard;

    // Create a new window to display the generated Aruco Board
    namedWindow("arcuoMarkers", WINDOW_AUTOSIZE);

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
    PREDEFINED_DICTIONARY_NAME dictionaryID = dictionaryMap.find(argv[3])->second;

    // Create the specified dictionary
    Ptr<Dictionary> dictionary = getPredefinedDictionary(dictionaryID);

    // Create the grid with the specified values
    Ptr<GridBoard> board = GridBoard::create(cols, rows, pixelSize, pixelSeparation, dictionary);

    // Calculate the total width and height of the board including pixel separation
    int width = cols * (pixelSize + pixelSeparation) - pixelSeparation;
    int height = rows * (pixelSize + pixelSeparation) - pixelSeparation;

    // Draw the board
    drawPlanarBoard(board, Size(width, height), markerBoard, marginSize);

    // Save the marker image as png
    imwrite(pngName, markerBoard);

    while (charCheckForESCKey != 27) {
        // Display the marker image in a window
        imshow("arcuoMarkers", markerBoard);

        // Program ends if we press the esc button
        charCheckForESCKey = waitKey(1);
    }

    return 0;
}

