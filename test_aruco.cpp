#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <iostream>

int main() {
    std::cout << "Getting dict..." << std::endl;
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::cout << "Dict size: " << dict.bytesList.rows << std::endl;
    return 0;
}
