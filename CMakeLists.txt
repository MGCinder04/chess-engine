cmake_minimum_required(VERSION 3.16)
project(ChessEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ChessEngine src/main.cpp)

# Good warnings + optimize for release builds
if (MINGW)
  target_compile_options(ChessEngine PRIVATE -Wall -Wextra -Wpedantic)
endif()

