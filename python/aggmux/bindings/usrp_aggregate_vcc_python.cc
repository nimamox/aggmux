/*
 * Copyright 2023 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(usrp_aggregate_vcc.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(48e05e154d8c3819b70f2abebbe4771a)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/aggmux/usrp_aggregate_vcc.h>
// pydoc.h is automatically generated in the build directory
#include <usrp_aggregate_vcc_pydoc.h>

void bind_usrp_aggregate_vcc(py::module& m)
{

    using usrp_aggregate_vcc = ::gr::aggmux::usrp_aggregate_vcc;


    py::class_<usrp_aggregate_vcc,
               gr::block,
               gr::basic_block,
               std::shared_ptr<usrp_aggregate_vcc>>(
        m, "usrp_aggregate_vcc", D(usrp_aggregate_vcc))

        .def(py::init(&usrp_aggregate_vcc::make),
             py::arg("vector_size") = 1024,
             py::arg("out_vector_size") = 512,
             py::arg("max_update_freq") = 15,
             py::arg("modalities") = 7,
             py::arg("sid") = 0,
             py::arg("debug") = false,
             D(usrp_aggregate_vcc, make))


        ;
}
