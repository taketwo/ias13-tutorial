cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set (CMAKE_BUILD_TYPE RelWithDebInfo)

project      (ias13-tutorial)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.2 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

include_directories (${PROJECT_SOURCE_DIR}/include)
include_directories (${CMAKE_BINARY_DIR})

set  (project_SOURCES src/main.cpp src/main_window.cpp src/seed_selection.cpp)
set  (project_HEADERS include/main_window.h include/seed_selection.h)
set  (project_FORMS   forms/main_window.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS}
              OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
                      -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

include         (${QT_USE_FILE})
add_definitions (${QT_DEFINITIONS} "-DQT_NO_KEYWORDS")

add_executable  (tutorial ${project_SOURCES}
                          ${project_FORMS_HEADERS}
                          ${project_HEADERS_MOC})

target_link_libraries (tutorial ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-fcolor-diagnostics)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()

