cmake_minimum_required(VERSION 3.0)
project(windows7)

find_package(OpenCV REQUIRED)
file(GLOB SOURCES "*.cpp")

foreach(SOURCES ${SOURCES})
    get_filename_component(EXE_NAME ${SOURCES} NAME_WE)
    add_executable(${EXE_NAME} ${SOURCES})
    target_link_libraries(${EXE_NAME} ${OpenCV_LIBS})
    target_include_directories(${EXE_NAME} PUBLIC ${OpenCV_LIB_DIR})
endforeach()
