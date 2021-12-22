#include "include_helper.h"

using namespace cv;
using namespace std;

// OpenCV parameters ---------------------

// Physical dimension of the markers used in meters
const float arucoSquareDimension = 0.162f;

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R) {
  Mat Rt;
  transpose(R, Rt);
  Mat shouldBeIdentity = Rt * R;
  Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

  return norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R) {

  assert(isRotationMatrix(R));

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                  R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
    z = 0;
  }
  return Vec3f(x, y, z);
}

// Monitors the camera and estimates the pose of any detected markers
int startCameraMonitoring(DDSPublisher &mocap_pub, Mat &frame,
                          const Mat &cameraMatrix,
                          const Mat &distanceCoefficients,
                          float arucoSquareDimension) {
  cpp_msg::Mocap mocap_msg;

  mocap_msg.header.id = "AruCo detection camera";

  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  aruco::DetectorParameters parameters;

  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);

  //   namedWindow("Detector", WINDOW_AUTOSIZE);

  vector<Vec3d> rotationVectors, translationVectors;

  aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
  aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension,
                                   cameraMatrix, distanceCoefficients,
                                   rotationVectors, translationVectors);

  for (int i = 0; i < markerIds.size(); i++) {
    aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
                    rotationVectors[i], translationVectors[i], 0.1f);
  }

  if (markerIds.size() > 0) {
    Vec3d rvec = rotationVectors[0];
    Vec3d tvec = translationVectors[0];

    Mat rotationMatrix;
    Rodrigues(rvec, rotationMatrix);

    Vec3f rotation_euler_zyx =
        180 / M_PI * rotationMatrixToEulerAngles(rotationMatrix);

    mocap_msg.pose.position.x = tvec[0];
    mocap_msg.pose.position.y = tvec[1];
    mocap_msg.pose.position.z = tvec[2];

    mocap_msg.pose.orientation_euler.roll = rotation_euler_zyx[0];
    mocap_msg.pose.orientation_euler.pitch = rotation_euler_zyx[1];
    mocap_msg.pose.orientation_euler.yaw = rotation_euler_zyx[2];
    cout << "trying to publish" << endl;
    mocap_pub.publish(mocap_msg);
    cout << "published" << endl;
  }

  return 1;
}

int main() {

  DefaultParticipant dp(0, "video_subscriber_test");

  // Create subscri
  // Create subscriber with msg type DDSSubscriber
  DDSSubscriber img_sub(idl_msg::Image720pPubSubType(), &sub::img, "img_topic",
                        dp.participant());
  img_sub.init();

  constexpr static int rows = 480;
  constexpr static int cols = 640;
  constexpr static int img_size = rows * cols * 3;

  // Create opencv matrix of same diension as image, CV_8UC3 means 8 bit
  // unsigned characeter, 3 channels (RGB)
  cv::Mat frame = cv::Mat(rows, cols, CV_8UC3);

  // Set aruco marker type
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  // Class object
  Calibrator calib;

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;

  calib.loadCamerCalibration("camCalibration", cameraMatrix,
                             distanceCoefficients);

  DDSPublisher mocap_pub =
      DDSPublisher(idl_msg::MocapPubSubType(), "aruco_marker_relative_pos",
                   dp.participant());
  mocap_pub.init();

  ArucoFunctions detector;

  cpp_msg::Mocap mocap_msg;

  mocap_msg.header.id = "AruCo detection camera";

  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  aruco::DetectorParameters parameters;

  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);

  namedWindow("Detector", WINDOW_AUTOSIZE);

  vector<Vec3d> rotationVectors, translationVectors;

  for (;;) {
    // cout << "waiting for new data" << endl;
    // Wait for new data
    img_sub.listener->wait_for_data_for_ms(20);

    // Move data from C++ array to OpenCV matrix
    std::memcpy(frame.data, sub::img.frame().data(), img_size);

    // Detect aruco markers in image
    // startCameraMonitoring(mocap_pub, frame, cameraMatrix,
    // distanceCoefficients,
    //                       arucoSquareDimension);

    // frame = detector.DetectArucoMarker(frame, dictionary);

    // ArUco marker pose estimation
    // ---------------------------------

    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension,
                                     cameraMatrix, distanceCoefficients,
                                     rotationVectors, translationVectors);

    // Draws the axis for all detected markers
    for (int i = 0; i < markerIds.size(); i++) {
      aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
                      rotationVectors[i], translationVectors[i], 0.1f);
    }

    // If markers present, get the first one and publish its pose
    if (markerIds.size() > 0) {
      Vec3d rvec = rotationVectors[0];
      Vec3d tvec = translationVectors[0];

      // Convert rotation vector to rotation matrix
      Mat rotationMatrix;
      Rodrigues(rvec, rotationMatrix);

      // Convert rotation matrix to vector with Euler angles in RPY
      Vec3f rotation_euler_xyz =
          180 / M_PI * rotationMatrixToEulerAngles(rotationMatrix);

      mocap_msg.pose.position.x = tvec[0];
      mocap_msg.pose.position.y = tvec[1];
      mocap_msg.pose.position.z = tvec[2];

      mocap_msg.pose.orientation_euler.roll = rotation_euler_xyz[0];
      mocap_msg.pose.orientation_euler.pitch = rotation_euler_xyz[1];
      mocap_msg.pose.orientation_euler.yaw = rotation_euler_xyz[2];
      mocap_pub.publish(mocap_msg);
    }

    // ---------------------------------

    // Show image in window
    imshow("Detector", frame);

    if (waitKey(30) >= 0)
      break;
  }
}
