cmake_minimum_required(VERSION 3.1)
project(test)
#CGAL_Qt5 is needed for the drawing and CGAL_Core is needed for this special Kernel.
set (CMAKE_CXX_STANDARD 17)
find_package(CGAL REQUIRED COMPONENTS Qt5 Core)
if(CGAL_FOUND)
  #required to use basic_viewer
  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
  #create the executable of the application
  add_executable(test_ test.cpp mesh.h)
  #link it with the required CGAL libraries
  target_link_libraries(test_ CGAL::CGAL CGAL::CGAL_Core)
else()
  message("ERROR: this program requires CGAL and CGAL_Qt5 and will not be compiled.")
endif()
