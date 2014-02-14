set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(MOC_HDRS
    QCamCalib.hpp
    QCamCalibPlugin.hpp
)

SET(HDRS
    QCamCalib.hpp
)

FILE(GLOB SRCS
    *.cpp
)

FILE(GLOB UI_FILES
    *.ui
)

SET(QtApp_RCCS resources.qrc)

rock_library(QCamCalib
    SOURCES ${SRCS} ${QtApp_RCC_SRCS}
    HEADERS ${HDRS}
    MOC ${MOC_HDRS}
    DEPS_PKGCONFIG QtCore QtGui opencv cv_detectors
    UI ${UI_FILES}
)
#QT4_ADD_RESOURCES(QtApp_RCC_SRCS ${QtApp_RCCS})

rock_executable(QCameraCalib
    SOURCES Main.cpp
    LIBS QCamCalib
)

