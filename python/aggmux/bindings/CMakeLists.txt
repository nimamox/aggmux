# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT aggmux_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT aggmux_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND aggmux_python_files
    hold_rewrite_tags_cc_python.cc
    overlapping_stream_to_vec_cc_python.cc
    aggr_and_mux_vff_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(aggmux
   ../../..
   gr::aggmux
   "${aggmux_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(TARGET aggmux_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:aggmux_python>
    ${PROJECT_BINARY_DIR}/test_modules/gnuradio/aggmux/
)

install(TARGETS aggmux_python DESTINATION ${GR_PYTHON_DIR}/gnuradio/aggmux COMPONENT pythonapi)
