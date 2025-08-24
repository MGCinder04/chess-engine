cmake_minimum_required(VERSION 3.16)
project(ChessEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release if not provided (good perf)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# (Optional) help VS Code intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(ChessEngine
  src/main.cpp
  src/uci.cpp
  src/attacks.cpp
  src/position.cpp
  src/fen.cpp
  src/san.cpp          # 👈 add
  # headers (optional)
  src/attacks.hpp
  src/position.hpp
  src/chess_types.hpp
  src/fen.hpp
  src/san.hpp          # 👈 add
  src/eval.cpp
  src/eval.hpp
  src/search.cpp
)




# Good warnings on MinGW
if (MINGW)
  target_compile_options(ChessEngine PRIVATE -Wall -Wextra -Wpedantic)
endif()
