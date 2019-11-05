##########################################################################

cmake_minimum_required(VERSION 2.8)

##########################################################################

set(TARGET hal-fe310-digital-out-pin)

##########################################################################

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

##########################################################################

set(CMAKE_C_COMPILER riscv64-unknown-elf-gcc)
set(CMAKE_CXX_COMPILER riscv64-unknown-elf-g++)

##########################################################################

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-march=rv32imac -mabi=ilp32 -mcmodel=medlow -O1 -g3 -fno-exceptions")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections,-Map=hal-fe310-digital-out-pin.map -nostartfiles -T ../FE310-G002.ld")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-Wl,--relax,--gc-sections")

##########################################################################

add_executable(
  ${TARGET}
  hal-fe310-digital-out-pin.cpp
  startup.cpp
)

target_link_libraries(
  ${TARGET}
  snowfox-fe310
  "-Wl,--start-group -lc -lgcc -Wl,--end-group"
)

##########################################################################
