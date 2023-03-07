/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "aggr_and_mux_vff_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace aggmux {

#pragma message("set the following appropriately and remove this warning")
using input_type = float;
#pragma message("set the following appropriately and remove this warning")
using output_type = float;
aggr_and_mux_vff::sptr aggr_and_mux_vff::make(int num_usrps,
                                              int num_shs,
                                              int usrp_vector_size,
                                              int sh_vector_size,
                                              int out_vector_size,
                                              int max_update_freq,
                                              int modalities,
                                              bool debug)
{
    return gnuradio::make_block_sptr<aggr_and_mux_vff_impl>(num_usrps,
                                                            num_shs,
                                                            usrp_vector_size,
                                                            sh_vector_size,
                                                            out_vector_size,
                                                            max_update_freq,
                                                            modalities,
                                                            debug);
}


/*
 * The private constructor
 */
aggr_and_mux_vff_impl::aggr_and_mux_vff_impl(int num_usrps,
                                             int num_shs,
                                             int usrp_vector_size,
                                             int sh_vector_size,
                                             int out_vector_size,
                                             int max_update_freq,
                                             int modalities,
                                             bool debug)
    : gr::block("aggr_and_mux_vff",
                gr::io_signature::make(
                    1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                gr::io_signature::make(
                    1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)))
{
}

/*
 * Our virtual destructor.
 */
aggr_and_mux_vff_impl::~aggr_and_mux_vff_impl() {}

void aggr_and_mux_vff_impl::forecast(int noutput_items,
                                     gr_vector_int& ninput_items_required)
{
#pragma message( \
    "implement a forecast that fills in how many items on each input you need to produce noutput_items and remove this warning")
    /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
}

int aggr_and_mux_vff_impl::general_work(int noutput_items,
                                        gr_vector_int& ninput_items,
                                        gr_vector_const_void_star& input_items,
                                        gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

#pragma message("Implement the signal processing in your block and remove this warning")
    // Do <+signal processing+>
    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace aggmux */
} /* namespace gr */
