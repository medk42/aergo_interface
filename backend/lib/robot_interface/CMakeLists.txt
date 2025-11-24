set(LIB_NAME helpers___robot_interface)

# find all sources
file(GLOB_RECURSE PROJECT_SOURCES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
)

add_library(${LIB_NAME} STATIC ${PROJECT_SOURCES})

target_include_directories(${LIB_NAME} PUBLIC include)

target_link_libraries(${LIB_NAME} PRIVATE helpers___serialization_helper)

if (TARGET module_common) # we only need module_common for message_types_definitions.h, we can avoid the dependency otherwise (use in external robot modules)
    target_link_libraries(${LIB_NAME} PUBLIC module_common)
    target_include_directories(${LIB_NAME} PUBLIC definitions_include)
endif()


# MSVC: force dynamic CRT
if (MSVC)
    target_compile_options(${LIB_NAME} PRIVATE /MD$<$<CONFIG:Debug>:d>)
endif()
# ELF: hide everything by default
set_target_properties(${LIB_NAME} PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1
)