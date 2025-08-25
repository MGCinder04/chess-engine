cmake_minimum_required(VERSION 3.16)
project(ChessEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(ChessEngine
  src/main.cpp
  src/uci.cpp
  src/attacks.cpp
  src/position.cpp
  src/fen.cpp
  src/san.cpp         
  src/attacks.hpp
  src/position.hpp
  src/chess_types.hpp
  src/fen.hpp
  src/san.hpp          
  src/eval.cpp
  src/eval.hpp
  src/search.cpp
  src/zobrist.cpp
)

if (MINGW)
  target_compile_options(ChessEngine PRIVATE -Wall -Wextra -Wpedantic)
endif()
