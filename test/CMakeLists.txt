rock_testsuite(stereoTools_test StereoTools_test.cpp
    DEPS QCamCalib        
    LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} 
    DEPS_PKGCONFIG opencv)
    
