cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(3D_Reconstruction LANGUAGES CXX)

set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE)

#======================= INCLUSION OF Packages =======================#
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_PREFIX_PATH $ENV{QTDIR})
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)


#=================== INCLUSION OF Project Files ====================#
set(FORMS_DIR "${CMAKE_SOURCE_DIR}/forms")
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")

include_directories(${FORMS_DIR})
include_directories(${INCLUDE_DIR})
include_directories(${SOURCE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories($ENV{OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCES
    "${FORMS_DIR}/*.ui"
    "${FORMS_DIR}/*.qrc"
    "${INCLUDE_DIR}/*.h"
    "${SOURCE_DIR}/*.cpp"
)

# Add the forms directory to the AUTOUIC search paths
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_AUTOUIC_SEARCH_PATHS} ${FORMS_DIR})

# Add the executable
add_executable(3D_Reconstruction WIN32 ${SOURCES})

# Add the target includes for 3D_Reconstruction 
target_include_directories(3D_Reconstruction PRIVATE ${FORMS_DIR})
target_include_directories(3D_Reconstruction PRIVATE ${INCLUDE_DIR})
target_include_directories(3D_Reconstruction PRIVATE ${SOURCE_DIR})

#===================== LINKING LIBRARIES =======================#
target_link_libraries(3D_Reconstruction Qt5::Widgets)
target_link_libraries(3D_Reconstruction Qt5::Gui)
target_link_libraries(3D_Reconstruction Qt5::Core)
target_link_libraries(3D_Reconstruction ${PCL_LIBRARIES})
target_link_libraries(3D_Reconstruction "$ENV{PCL_ROOT}/lib/pcl_visualizationd.lib")
target_link_libraries(3D_Reconstruction "$ENV{PCL_ROOT}/lib/pcl_filtersd.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_highgui455d.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_calib3d455d.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_cudaimgproc455d.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_cudafilters455d.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_core455d.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_features2d455d.lib")
target_link_libraries(3D_Reconstruction "$ENV{OPENCV_DIR}/lib/Debug/opencv_imgcodecs455d.lib")

# Link directory
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})