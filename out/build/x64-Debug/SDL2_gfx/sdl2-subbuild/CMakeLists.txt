# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.26.4-msvc4)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(sdl2-populate NONE)



include(ExternalProject)
ExternalProject_Add(sdl2-populate
                     "URL" "https://github.com/libsdl-org/SDL/releases/download/release-2.24.2/SDL2-2.24.2.zip" "URL_HASH" "SHA256=7fae98ac4e7b39eb2511fc27c2e84b220ac69b5296ff41f833b967c891f9d2ac"
                    SOURCE_DIR          "C:/Users/bagro/OneDrive/Pulpit/tp_projekt_4/out/build/x64-Debug/SDL2_gfx/SDL-2.24.2"
                    BINARY_DIR          "C:/Users/bagro/OneDrive/Pulpit/tp_projekt_4/out/build/x64-Debug/SDL2_gfx/sdl2-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


