cmake_minimum_required(VERSION 3.28)
project(SZHSolver)

append_files (Headers "h" .)
append_files (Sources "cpp" .)

add_executable (SZHSolver ${Headers} ${Sources})

TARGET_LINK_LIBRARIES(SZHSolver LosTopos ${GLEW_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
