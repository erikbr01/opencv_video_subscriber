#pragma once

#include "default_participant.h"
#include "default_subscriber.h"
#include "imghd_sub_callback.h"
#include "sensor_msgs/msgs/Image720p.h"

// C++ native
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>

// Opencv
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <aruco_functions.h>