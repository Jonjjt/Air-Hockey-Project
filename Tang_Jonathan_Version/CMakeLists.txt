project(cameracode)
 
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA QUIET REQUIRED)

find_package(PkgConfig REQUIRED)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(TBB "tbb")
    if (TBB_FOUND)
        add_definitions(${TBB_CFLAGS} ${TBB_CFLAGS_OTHER})
        MESSAGE(tbb version : ${TBB_VERSION})
        include_directories(
        ${TBB_INCLUDE_DIRS}
    )
  endif()
endif()

# Pass options to NVCC
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_22,code=sm_22
    )

#set(CMAKE_BUILD_TYPE Release)


pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(TCAMLIB tcam)

include_directories( ${CMAKE_CURRENT_BINARY_DIR}  ${GSTREAMER_INCLUDE_DIRS} ${TCAM_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} )

add_definitions(${GSTREAMER_CFLAGS_OTHER})  

add_executable(cameracode main.cpp tcamimage.cpp tcamcamera.cpp Socket.cpp ClientSocket.cpp udp_client_server.cpp)
target_link_libraries(cameracode ${TCAMLIB_LIBRARIES} ${GSTREAMER_LIBRARIES}  ${OpenCV_LIBS} Threads::Threads ${TBB_LIBRARIES})


install(TARGETS cameracode RUNTIME DESTINATION bin)
