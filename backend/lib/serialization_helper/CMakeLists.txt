set(LIB_NAME helpers___serialization_helper)


# find all sources
file(GLOB_RECURSE PROJECT_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
)

add_library(${LIB_NAME} INTERFACE)

target_include_directories(${LIB_NAME} INTERFACE include)