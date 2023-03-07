/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_OVERLAPPING_STREAM_TO_VEC_CC_IMPL_H
#define INCLUDED_AGGMUX_OVERLAPPING_STREAM_TO_VEC_CC_IMPL_H

#include <gnuradio/aggmux/overlapping_stream_to_vec_cc.h>

namespace gr {
namespace aggmux {

class overlapping_stream_to_vec_cc_impl : public overlapping_stream_to_vec_cc
{
private:
    int d_vector_size;
    int d_overlap;
    bool d_debug;
    long d_counter;
    int d_sid;
    int d_unconsumed;
    std::deque<gr_complex> d_buf;

public:
    overlapping_stream_to_vec_cc_impl(int vector_size, int overlap, bool debug, int sid);
    ~overlapping_stream_to_vec_cc_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_OVERLAPPING_STREAM_TO_VEC_CC_IMPL_H */
