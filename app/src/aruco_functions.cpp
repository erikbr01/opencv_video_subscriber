#include <aruco_functions.h>

#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

void ArucoFunctions::createArucoMarker(
    cv::Ptr<cv::aruco::Dictionary> dictionary) {
  cv::Mat markerImage;
  cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
  cv::imwrite("/Users/sarathmenon/Documents/c++_projects/opencv_examples/"
              "generated_images/1_waste.png",
              markerImage);
}

cv::Mat
ArucoFunctions::DetectArucoMarker(cv::Mat frame,
                                  cv::Ptr<cv::aruco::Dictionary> dictionary) {
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds,
                           parameters, rejectedCandidates);

  // Draw box on detected markers
  cv::Mat detectedImage = frame.clone();
  cv::aruco::drawDetectedMarkers(detectedImage, markerCorners, markerIds);

  return detectedImage;
}
