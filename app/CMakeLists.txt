
add_executable(opencv_video_subscriber 
src/main.cpp
src/aruco_functions.cpp
)

target_include_directories(opencv_video_subscriber PUBLIC include)

target_link_libraries(opencv_video_subscriber PRIVATE
${OpenCV_LIBS}
comm_lib
fastcdr 
fastrtps
)


