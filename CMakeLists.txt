# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(qcam_calib 0.1)
rock_opencv_autodetect(OPENCV_PACKAGE)
rock_standard_layout()
