/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "overlapping_stream_to_vec_cc_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace aggmux {

using input_type = gr_complex;
using output_type = gr_complex;
overlapping_stream_to_vec_cc::sptr
overlapping_stream_to_vec_cc::make(int vector_size, int overlap, bool debug, int sid)
{
    return gnuradio::make_block_sptr<overlapping_stream_to_vec_cc_impl>(
        vector_size, overlap, debug, sid);
}


/*
 * The private constructor
 */
overlapping_stream_to_vec_cc_impl::overlapping_stream_to_vec_cc_impl(int vector_size,
                                                                     int overlap,
                                                                     bool debug,
                                                                     int sid)
    : gr::block("overlapping_stream_to_vec_cc",
                gr::io_signature::make(1, 1, sizeof(input_type)),
                gr::io_signature::make(1, 1, sizeof(output_type) * vector_size)),
      d_vector_size(vector_size),
      d_overlap(overlap),
      d_counter(0)
{
    set_tag_propagation_policy(TPP_DONT);
    d_buffer.resize(overlap);
}

/*
 * Our virtual destructor.;
 */
overlapping_stream_to_vec_cc_impl::~overlapping_stream_to_vec_cc_impl() {}

void overlapping_stream_to_vec_cc_impl::forecast(int noutput_items,
                                                 gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] =
        d_vector_size + (d_vector_size - d_overlap) * (noutput_items - 1);
    std::cout << "forecast:" << noutput_items << "::" << ninput_items_required[0]
              << std::endl;
    if (d_buff_filled) {
        ninput_items_required[0] -= d_overlap;
    }
}

int overlapping_stream_to_vec_cc_impl::general_work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    d_counter++;

    int to_consume = ninput_items[0];

    std::vector<gr::tag_t> tagVector;
    get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + noutput_items);
    bool should_buffer = true;
    if (tagVector.size() > 0) {
        if (tagVector[0].offset == nitems_read(0)) {
            add_item_tag(0, nitems_written(0), tagVector[0].key, tagVector[0].value);
        } else {
            to_consume = tagVector[0].offset - nitems_read(0);
            should_buffer = false;
            d_buff_filled = false;
        }
    }

    int vecs_to_produce = noutput_items;
    while (vecs_to_produce > 0) {
        if (d_vector_size + (d_vector_size - d_overlap) * (vecs_to_produce - 1) >
            to_consume + (d_buff_filled ? d_overlap : 0)) {
            vecs_to_produce--;
        } else {
            break;
        }
    }



    if (should_buffer) {
        for (int i = 0; i < d_overlap; i++) {
            d_buffer[d_overlap - i] = in[noutput_items - i];
        }
        d_buff_filled = true;
    }


    consume_each(to_consume);

    return noutput_items;
}

} /* namespace aggmux */
} /* namespace gr */
