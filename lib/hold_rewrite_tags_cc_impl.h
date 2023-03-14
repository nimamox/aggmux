/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_HOLD_REWRITE_TAGS_CC_IMPL_H
#define INCLUDED_AGGMUX_HOLD_REWRITE_TAGS_CC_IMPL_H

#include <gnuradio/aggmux/hold_rewrite_tags_cc.h>

namespace gr {
namespace aggmux {

class hold_rewrite_tags_cc_impl : public hold_rewrite_tags_cc
{
private:
    long d_counter;
    bool d_tag_arrived;
    double d_last_freq, d_last_rate;
    int d_sid;
    bool d_tmp_flag;
public:
    hold_rewrite_tags_cc_impl(bool debug, int sid);
    ~hold_rewrite_tags_cc_impl();

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_HOLD_REWRITE_TAGS_CC_IMPL_H */
