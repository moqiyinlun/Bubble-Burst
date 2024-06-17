cmake_minimum_required(VERSION 3.28)
project(HGF)

append_files (Headers "h" .)
append_files (Sources "cpp" .)

add_executable (HGF ${Headers} ${Sources})

TARGET_LINK_LIBRARIES(HGF LosTopos ${GLEW_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
