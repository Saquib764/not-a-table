# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/saquib/esp/esp-idf/components/bootloader/subproject"
  "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader"
  "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix"
  "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix/tmp"
  "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix/src"
  "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/saquib/Documents/PlatformIO/Projects/230331-223557-esp32dev/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
