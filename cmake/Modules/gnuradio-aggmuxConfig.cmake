find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_AGGMUX gnuradio-aggmux)

FIND_PATH(
    GR_AGGMUX_INCLUDE_DIRS
    NAMES gnuradio/aggmux/api.h
    HINTS $ENV{AGGMUX_DIR}/include
        ${PC_AGGMUX_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_AGGMUX_LIBRARIES
    NAMES gnuradio-aggmux
    HINTS $ENV{AGGMUX_DIR}/lib
        ${PC_AGGMUX_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-aggmuxTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_AGGMUX DEFAULT_MSG GR_AGGMUX_LIBRARIES GR_AGGMUX_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_AGGMUX_LIBRARIES GR_AGGMUX_INCLUDE_DIRS)
