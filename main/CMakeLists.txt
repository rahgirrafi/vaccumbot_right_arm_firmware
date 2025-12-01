file(GLOB_RECURSE SRC_FILES
     "${CMAKE_CURRENT_LIST_DIR}/*.c"
     "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

idf_component_register(SRCS ${SRC_FILES}
                       INCLUDE_DIRS "include")