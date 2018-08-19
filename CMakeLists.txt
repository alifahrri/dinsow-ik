cmake_minimum_required(VERSION 2.8)
project(dinsow-ik)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -pthread")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Network REQUIRED)

file(GLOB SRC_FILES dinsowkinematic.cpp disowmotion.cpp jointentity.cpp jointsettingsdialog.cpp linkentity.cpp mainwindow.cpp meshentity.cpp scenemodifier.cpp)
add_library(${PROJECT_NAME}lib ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}lib 
    Qt5::Gui Qt5::Core Qt5::Widgets Qt5::PrintSupport Qt5::Network
)