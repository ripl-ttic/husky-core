add_definitions(-DBASE_PATH="${CMAKE_INSTALL_PREFIX}")

message(PATH ${CMAKE_INSTALL_PREFIX})
add_library(hr-path-util SHARED path_util.c)

# make the shared library public
pods_install_libraries(hr-path-util)
#make the header public
pods_install_headers(path_util.h DESTINATION hr_common)

pods_install_pkg_config_file(hr-path-util
    LIBS -lhr-path-util
    REQUIRES 
    VERSION 0.0.1)
