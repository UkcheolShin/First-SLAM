cmake_minimum_required(VERSION 2.8)
project( mono-vo )
find_package( OpenCV 3.0 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package (Threads)

set (CMAKE_CXX_STANDARD 11)
file(GLOB viso
    "src/*.h"
    "src/*.cpp"
)
add_executable( vo ${viso} )
target_link_libraries( vo ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
