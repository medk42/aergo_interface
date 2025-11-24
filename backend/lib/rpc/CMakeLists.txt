set(LIB_NAME kassow_rpc)

file(GLOB_RECURSE RPC_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(${LIB_NAME} STATIC ${RPC_SOURCES})

target_include_directories(${LIB_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${LIB_NAME} PUBLIC helpers___robot_interface)

if (WIN32)
    target_link_libraries(${LIB_NAME} PRIVATE ws2_32)
endif()

if (MSVC)
    target_compile_options(${LIB_NAME} PRIVATE /MD$<$<CONFIG:Debug>:d>)
endif()
