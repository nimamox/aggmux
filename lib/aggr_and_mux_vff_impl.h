/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_AGGR_AND_MUX_VFF_IMPL_H
#define INCLUDED_AGGMUX_AGGR_AND_MUX_VFF_IMPL_H

#include <gnuradio/aggmux/aggr_and_mux_vff.h>

namespace gr {
namespace aggmux {

class aggr_and_mux_vff_impl : public aggr_and_mux_vff
{
private:
    // Nothing to declare in this block.

public:
    aggr_and_mux_vff_impl(int num_usrps,
                          int num_shs,
                          int usrp_vector_size,
                          int sh_vector_size,
                          int out_vector_size,
                          int max_update_freq,
                          int modalities,
                          bool debug);
    ~aggr_and_mux_vff_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_AGGR_AND_MUX_VFF_IMPL_H */
