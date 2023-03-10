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
        overlapping_stream_to_vec_cc::make(int vector_size, int overlap, bool debug, int sid) {
            return gnuradio::make_block_sptr<overlapping_stream_to_vec_cc_impl>(
                    vector_size, overlap, debug, sid);
        }


        overlapping_stream_to_vec_cc_impl::overlapping_stream_to_vec_cc_impl(int vector_size,
                                                                             int overlap,
                                                                             bool debug,
                                                                             int sid)
                : gr::block("overlapping_stream_to_vec_cc",
                            gr::io_signature::make(1, 1, sizeof(input_type)),
                            gr::io_signature::make(1, 1, sizeof(output_type) * vector_size)),
                  d_vector_size(vector_size),
                  d_overlap(overlap),
                  d_debug(debug),
                  d_counter(0),
                  d_sid(sid),
                  d_unconsumed(0) {
            set_tag_propagation_policy(TPP_DONT);
        }

        overlapping_stream_to_vec_cc_impl::~overlapping_stream_to_vec_cc_impl() {}

        void overlapping_stream_to_vec_cc_impl::forecast(int noutput_items,
                                                         gr_vector_int &ninput_items_required) {
            ninput_items_required[0] =
                    std::max(d_vector_size + (d_vector_size - d_overlap) * (noutput_items - 1), 1);
            ninput_items_required[0] -= d_unconsumed;
            if (((int) d_buf.size() - d_unconsumed) >= d_overlap) {
                ninput_items_required[0] -= d_overlap;
            }
            if (noutput_items <= 1) {
                ninput_items_required[0] = std::min(ninput_items_required[0], 8191);
            }
        }

        int overlapping_stream_to_vec_cc_impl::general_work(
                int noutput_items,
                gr_vector_int &ninput_items,
                gr_vector_const_void_star &input_items,
                gr_vector_void_star &output_items) {
            auto in = static_cast<const input_type *>(input_items[0]);
            auto out = static_cast<output_type *>(output_items[0]);

            d_counter++;


            std::vector<gr::tag_t> tagVector;
            get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + ninput_items[0]);
            int to_consume = ninput_items[0];
            bool wrap_up = false;
            if (tagVector.size() > 0) {
                if (d_sid == 0)
                    std::cout << "OVRLAP: TAG RECEIVED " << d_counter << std::endl;
                if (tagVector[0].offset == nitems_read(0)) {
                    add_item_tag(0, nitems_written(0), tagVector[0].key, tagVector[0].value);
                    if (tagVector.size() > 1) {
                        to_consume = tagVector[1].offset - nitems_read(0);
                    }
                } else {
                    to_consume = tagVector[0].offset - nitems_read(0);
                    wrap_up = true;
                }
            }

            // Put input into internal buffer
            for (int i = 0; i < to_consume; i++) {
                d_buf.push_back(in[i]);
                d_unconsumed++;
            }
            consume_each(to_consume);

            bool has_overlap = ((int) d_buf.size() - d_unconsumed) >= d_overlap;
            int vecs_to_produce =
                    std::floor(((float) d_unconsumed + (has_overlap ? d_overlap : 0) - d_vector_size) /
                               (d_vector_size - d_overlap) +
                               1);
            if (!vecs_to_produce) {
                if (!wrap_up)
                    // Not enough data to produce a vector.
                    return 0;
                else {
                    // Retuning has occurred.
                    vecs_to_produce = std::floor(
                            ((float) d_buf.size() - d_vector_size) / (d_vector_size - d_overlap) + 1);
                    // Even with whole buffer we can not produce any vector.
                    if (!vecs_to_produce) {
                        d_unconsumed = 0;
                        d_buf.clear();
                        return 0;
                    }
                }
            }
            int onset = (int) d_buf.size() - d_unconsumed - (has_overlap ? d_overlap : 0);
            if (d_debug)
                std::cout << "\tbefore: unconsumed: " << d_unconsumed << "\tonset= " << onset
                          << std::endl;
            int ind;
            int kk = 0;
            for (int i = 0; i < vecs_to_produce; i++) {
                for (int j = 0; j < d_vector_size; j++) {
                    ind = i * d_vector_size + j - i * d_overlap;
                    if (!has_overlap || ind >= d_overlap)
                        d_unconsumed--;
                    out[i * d_vector_size + j] = d_buf[onset + ind];
                    kk++;
                }
            }
            if (d_debug)
                std::cout << "\tafter: unconsumed: " << d_unconsumed << "\tind= " << ind
                          << "\tkk: " << kk << std::endl;

            if (wrap_up) {
                d_unconsumed = 0;
                d_buf.clear();
            }
            if (d_debug)
                std::cout << "\tbefore removal: " << d_buf.size() << std::endl;
            while ((int) d_buf.size() >= d_vector_size)
                d_buf.pop_front();
            if (d_debug)
                std::cout << "\tafter removal: " << d_buf.size() << std::endl;

            if (d_debug)
                std::cout << "~ " << d_counter << "\tvecs produced: " << vecs_to_produce << std::endl;
            return vecs_to_produce;
        }

    } /* namespace aggmux */
} /* namespace gr */
