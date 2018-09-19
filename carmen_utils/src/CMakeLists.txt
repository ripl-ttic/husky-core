add_definitions(
#    -ggdb3
    -std=gnu99
    )

# Create a shared library libhello.so with a single source file
add_library(carmen-utils SHARED
  geometry.c
  map_interface.c
  global.c
  carmen_stdio.c
  map_read.c
  map_write.c
  map3d_interface.c
  )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    )
target_link_libraries (carmen-utils z)

pods_use_pkg_config_packages (carmen-utils lcmtypes_hr-lcmtypes)
# make the header public
# install it to include/hello
pods_install_headers(carmen.h
  map.h
  map3d.h
  geometry.h
  global.h
  carmen_stdio.h
  map_interface.h
  map_messages.h
  base_messages.h
  map_io.h
  laser_messages.h
  robot_messages.h
  navigator_messages.h
  map_interface.h
  map3d_interface.h
  DESTINATION carmen_utils)

# make the library public
pods_install_libraries(carmen-utils)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(carmen-utils
    CFLAGS
    LIBS -lcarmen-utils
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
