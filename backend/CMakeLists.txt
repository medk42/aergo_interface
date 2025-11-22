cmake_minimum_required(VERSION 3.10)
project(aergo_adapter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Set path for linking with system dependencies
link_directories(/usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib $ENV{KR2_INSTALL_FOLDER}/lib)

# find all sources
file(GLOB_RECURSE PROJECT_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/cbun/src/*
)

# Compile CBun
set(KR2_API_LIBS kr2rc_api kr2_io_api kr2_program_api)
add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCES})
include_directories(${PROJECT_NAME} PRIVATE cbun/include $ENV{KR2_INSTALL_FOLDER}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE ${KR2_API_LIBS})

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)


# Install CBun
# "add_custom_command(...POST_BUILD" is not used, because it does not run if only bundle.xml changes for example
# The following solution should always run and always after the build is finished.
set(CBUN_BUILD_DIR ${PROJECT_NAME}.cbunbuild)
add_custom_target(always_run_target ALL
    COMMAND rm -rf ${CBUN_BUILD_DIR}
    COMMAND mkdir ${CBUN_BUILD_DIR}
    COMMAND cp "lib${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}" ${CBUN_BUILD_DIR}
    COMMAND cp ${PROJECT_SOURCE_DIR}/../bundle.xml ${CBUN_BUILD_DIR}
    COMMAND mkdir -p ${CBUN_BUILD_DIR}/include/aergo
    COMMAND cp ${PROJECT_SOURCE_DIR}/cbun/include/aergo/aergo_adapter.h ${CBUN_BUILD_DIR}/include/aergo
    COMMAND tar -cvzf ${PROJECT_NAME}.cbun -C ${CBUN_BUILD_DIR} .
)
# Ensure the custom target runs after the project is built
add_dependencies(always_run_target ${PROJECT_NAME})