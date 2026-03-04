cmake_minimum_required(VERSION 3.30.5)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message("Build type: ${CMAKE_BUILD_TYPE}")

if(NOT EMSCRIPTEN)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
else()
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
endif()

if(LINUX AND CONFIG_USE_UBSAN)
    add_compile_options("-fsanitize=undefined")
elseif(LINUX AND CONFIG_USE_ASAN)
    add_compile_options("-fsanitize=address")
endif()

project(
    physics3D
    LANGUAGES CXX C
    VERSION 0.1
)
add_executable("${CMAKE_PROJECT_NAME}")

if(LINUX AND CONFIG_USE_UBSAN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined")
elseif(LINUX AND CONFIG_USE_ASAN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif()

set(SHADERS_SOURCE ${CMAKE_SOURCE_DIR}/resources/shaders)
set(SHADERS_BIN ${CMAKE_BINARY_DIR}/shaders)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/engine-core/)

# include(${CMAKE_BINARY_DIR}/_deps/bgfx-src/cmake/bgfxToolUtils.cmake)
# build/_deps/bgfx-build/cmake/bgfx/shaderc -f resources/shaders/cubes.vert -o build/shaders/spirv/cubes.vert.bin --type v --varyingdef resources/shaders/varying.def.sc -p spirv && build/_deps/bgfx-build/cmake/bgfx/shaderc -f resources/shaders/cubes.frag -o build/shaders/spirv/cubes.frag.bin --type g --varyingdef resources/shaders/varying.def.sc -p spirv

if(NOT EMSCRIPTEN)
    message("Compiling Shaders")
    bgfx_compile_shaders(
        TYPE FRAGMENT
        SHADERS "${SHADERS_SOURCE}/cubes.frag"
        VARYING_DEF "${SHADERS_SOURCE}/varying.def.sc"
        OUTPUT_DIR "${SHADERS_BIN}"
    )
    bgfx_compile_shaders(
        TYPE VERTEX
        SHADERS "${SHADERS_SOURCE}/cubes.vert"
        VARYING_DEF "${SHADERS_SOURCE}/varying.def.sc"
        OUTPUT_DIR "${SHADERS_BIN}"
    )
    message("Done!")
endif()

set(CMAKE_CXX_FLAGS
    "-Wall -Wextra -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused\
    -Woverloaded-virtual -Wpedantic -Wconversion -Wsign-conversion\
    -Wnull-dereference -Wdouble-promotion -Wformat=2 -Wduplicated-cond\
    -Wduplicated-branches -Wlogical-op -Wuseless-cast"
)
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

# target_sources("${CMAKE_PROJECT_NAME}" PRIVATE ${MY_SOURCES})
target_sources(
    "${CMAKE_PROJECT_NAME}" PRIVATE
    src/main.cpp
    src/program.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC engine-core)

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
