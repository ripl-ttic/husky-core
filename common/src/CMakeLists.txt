add_definitions(
    #-ggdb3
    -std=gnu99
    -msse2
)

# DO WE WANT THIS?
#pods_install_headers(lcm_utils.h DESTINATION lcm_utils)

file(GLOB c_files *.c)
file(GLOB h_files *.h)

file(GLOB cpp_files *.cpp)
file(GLOB hpp_files *.hpp)

include_directories(${GLIB2_INCLUDE_DIRS})

# Now for the remaining envoy common library
add_library(hr-common SHARED ${c_files})

# Set the library API version. Increment every time the public API changes
set_target_properties(hr-common PROPERTIES SOVERSION 1)

set(REQUIRED_LIBS bot2-core lcmtypes_hr-lcmtypes lcmtypes-gps18 geom-utils)

pods_use_pkg_config_packages(hr-common ${REQUIRED_LIBS})

#set(REQUIRED_LIBS_OPENCV bot2-core lcmtypes_hr-lcmtypes geom-utils opencv)

target_link_libraries(hr-common -lcarmen-utils)

pods_install_headers(${h_files} DESTINATION hr_common)

pods_install_headers(${hpp_files} DESTINATION hr_common)

pods_install_libraries(hr-common)

pods_install_pkg_config_file(hr-common
    LIBS -lhr-common
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)

pods_install_libraries(hr-common)
