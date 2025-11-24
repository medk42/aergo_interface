# Get immediate subdirectories
file(GLOB SUBDIRS LIST_DIRECTORIES true "${CMAKE_CURRENT_SOURCE_DIR}/*")

# Add each subdirectory that contains a CMakeLists.txt
foreach(SUBDIR ${SUBDIRS})
    if(IS_DIRECTORY ${SUBDIR})
        message(STATUS "[default_modules] Adding subdirectory: ${SUBDIR}")
        add_subdirectory("${SUBDIR}")
    endif()
endforeach()