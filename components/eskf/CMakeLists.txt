cmake_minimum_required(VERSION 3.5)

if(COMMAND idf_component_register)
    idf_component_register(
        SRCS eskf.c
        INCLUDE_DIRS include
    )
else()
    set(EXTRA_COMPONENT_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/basic
    )

    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(eskf)
endif()