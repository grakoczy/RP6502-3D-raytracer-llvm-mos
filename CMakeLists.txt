cmake_minimum_required(VERSION 3.18)
add_subdirectory(tools)
set(LLVM_MOS_PLATFORM rp6502)
find_package(llvm-mos-sdk REQUIRED)
project(MY-RP6502-PROJECT)
add_executable(raytracer)
rp6502_executable(raytracer)
target_sources(raytracer PRIVATE
    src/colors.c
    src/bitmap_graphics.c
    src/raytracer_float.c
)
