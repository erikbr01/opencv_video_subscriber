#include "include_helper.h"

int main() {

  DefaultParticipant dp(0, "video_subscriber_test");

  // Create subscri
  // Create subscriber with msg type DDSSubscriber
  DDSSubscriber img_sub(idl_msg::Image720pPubSubType(), &sub::img, "img_topic",
                        dp.participant());
  img_sub.init();

  constexpr static int rows = 720;
  constexpr static int cols = 1280;
  constexpr static int img_size = rows * cols * 3;

  // Create opencv matrix of same diension as image, CV_8UC3 means 8 bit
  // unsigned characeter, 3 channels (RGB)
  cv::Mat frame = cv::Mat(rows, cols, CV_8UC3);

  // Set aruco marker type
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  // Class object
  ArucoFunctions detector;

  for (;;) {

    // Wait for new data
    img_sub.listener->wait_for_data_for_ms(100);

    // Move data from C++ array to OpenCV matrix
    std::memcpy(frame.data, sub::img.frame().data(), img_size);

    // Detect aruco markers in image
    frame = detector.DetectArucoMarker(frame, dictionary);

    // Show image in window
    cv::imshow("Video subscriber", frame);

    // Show image for 1 ms, detect keypress
    int c = cv::waitKey(1);

    // Shut down upon pressing any key
    if (c != -1) {
      cv::destroyAllWindows();
      exit(EXIT_SUCCESS);
    }
  }
}
