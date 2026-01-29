cmake_minimum_required(VERSION 3.30.5)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if("${CMAKE_GENERATOR}" STREQUAL "Ninja")
    set(WINDOWS TRUE)
    message("Ninja")
else()

endif()
# add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-O3;-ffast-math>")
# add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-O0;-g>")

set(CMAKE_CXX_FLAGS
    "-Wall -Wextra -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused\
    -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion\
    -Wnull-dereference -Wdouble-promotion -Wformat=2 -Wduplicated-cond\
    -Wduplicated-branches -Wlogical-op -Wuseless-cast"
)
if(NOT WINDOWS)
    add_compile_options("-fsanitize=undefined")
    # add_compile_options("-fsanitize=address")
endif()

# Change this to match the name of the current folder
project(
    physics3D
    LANGUAGES CXX C
    VERSION 0.1
)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

message("Build type: ${CMAKE_BUILD_TYPE}")

add_executable("${CMAKE_PROJECT_NAME}")

if(NOT WINDOWS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined")
    # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif()

set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET)
if(NOT raylib_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        raylib
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
        URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
            SYSTEM
    )
    FetchContent_GetProperties(raylib)
    if(NOT raylib_POPULATED)
        set(FETCHCONTENT_QUIET NO)
        FetchContent_MakeAvailable(raylib)
        set(BUILD_EXAMPLES
            OFF
            CACHE BOOL "" FORCE
        )
    endif()
endif()

set(IMGUI_VERSION 1.92.2)
find_package(ImGui ${IMGUI_VERSION} QUIET)
if(NOT ImGui_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        ImGui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG v${IMGUI_VERSION}
        SYSTEM
    )
    FetchContent_MakeAvailable(ImGui)
    FetchContent_GetProperties(ImGui SOURCE_DIR IMGUI_DIR)
    add_library(
        ImGui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
    )
    target_include_directories(ImGui SYSTEM INTERFACE ${imgui_SOURCE_DIR})
endif()

include(FetchContent)
FetchContent_Declare(
    rlImGui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui
    GIT_TAG 4d8a618
    SYSTEM
)
FetchContent_MakeAvailable(rlImGui)
FetchContent_GetProperties(rlImGui SOURCE_DIR RLIMGUI_DIR)

add_library(rlImGui STATIC ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
target_link_libraries(rlImGui PRIVATE ImGui raylib)
target_include_directories(rlImGui SYSTEM INTERFACE ${rlimgui_SOURCE_DIR})

include(FetchContent)

file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

if(EMSCRIPTEN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}\
 --shell-file ${CMAKE_CURRENT_SOURCE_DIR}/src/template.html"
    )
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}\
 --js-library ${CMAKE_CURRENT_SOURCE_DIR}/src/wasmUtils.js"
    )
    file(GLOB_RECURSE DATAFILES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/resources/*"
    )
    foreach(FILE ${DATAFILES})
        get_filename_component(TRIMMED ${FILE} NAME)
        set_target_properties(
            ${CMAKE_PROJECT_NAME}
            PROPERTIES
                LINK_FLAGS
                "--preload-file ../resources/${TRIMMED}@./resources/${TRIMMED}"
        )
    endforeach()
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set_target_properties(
        ${CMAKE_PROJECT_NAME}
        PROPERTIES LINK_FLAGS "--preload-file ../resources@./resources"
    )
endif()
target_compile_definitions(
    ${CMAKE_PROJECT_NAME} PUBLIC $<$<CONFIG:Release>:PRODUCTION_BUILD>
                                 NAME="${CMAKE_PROJECT_NAME}"
)

set_target_properties(
    "${CMAKE_PROJECT_NAME}"
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin"
               RUNTIME_OUTPUT_DIRECTORY_RELEASE
               "${CMAKE_CURRENT_SOURCE_DIR}/bin/\
${CMAKE_PROJECT_NAME}_${CMAKE_PROJECT_VERSION}"
)
if(EMSCRIPTEN)
    set_target_properties(
        ${CMAKE_PROJECT_NAME}
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE
                   "${CMAKE_CURRENT_SOURCE_DIR}/bin/\
${CMAKE_PROJECT_NAME}_${CMAKE_PROJECT_VERSION}_web"
    )
endif()

set(RELEASE_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin/\
${CMAKE_PROJECT_NAME}_${CMAKE_PROJECT_VERSION}/resources/"
)
set(DEBUG_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/")
target_compile_definitions(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
        RESOURCES_PATH=$<IF:$<CONFIG:Release>,"./resources/","${DEBUG_RESOURCE_DIR}">
)

target_compile_features("${CMAKE_PROJECT_NAME}" PRIVATE cxx_std_23)

target_sources("${CMAKE_PROJECT_NAME}" PRIVATE ${MY_SOURCES})

target_link_libraries(${PROJECT_NAME} raylib ImGui rlImGui)

target_include_directories(
    "${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/"
)
target_include_directories(
    "${CMAKE_PROJECT_NAME}" PUBLIC SYSTEM "${PROJECT_BINARY_DIR}"
)

if(NOT EMSCRIPTEN)
    add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/resources/"
            $<IF:$<CONFIG:Release>,"${RELEASE_RESOURCE_DIR}","${DEBUG_RESOURCE_DIR}">
    )
endif()
