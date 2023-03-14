/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "overlapping_stream_to_vec_cc_impl.h"
#include <gnuradio/io_signature.h>

#include <fmt/format.h>

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
            set_output_multiple(2);
        }

        overlapping_stream_to_vec_cc_impl::~overlapping_stream_to_vec_cc_impl() {
            std::cout << fmt::format("\t** OVERLAP ({}) - DESTRUCTOR CALLED!", d_sid) << std::endl;
        }

        void overlapping_stream_to_vec_cc_impl::forecast(int noutput_items,
                                                         gr_vector_int &ninput_items_required) {
            ninput_items_required[0] = noutputs_to_ninputs(noutput_items);
//            std::cout << fmt::format("FORECAST: out:{} in:{} - bufsize: {}, unconsumed: {}", noutput_items, ninput_items_required[0], d_buf.size(), d_unconsumed)
//                      << std::endl;
        }

        int overlapping_stream_to_vec_cc_impl::noutputs_to_ninputs(int noutput_items) {
            int ninputs = std::max(d_vector_size + (d_vector_size - d_overlap) * (noutput_items - 1), 1);
            ninputs -= d_unconsumed;
            if (((int) d_buf.size() - d_unconsumed) >= d_overlap) {
                ninputs -= d_overlap;
            }
            ninputs = std::min(ninputs, 8191);
            return ninputs;
        }

        int overlapping_stream_to_vec_cc_impl::general_work(
                int noutput_items,
                gr_vector_int &ninput_items,
                gr_vector_const_void_star &input_items,
                gr_vector_void_star &output_items) {
            auto in = static_cast<const input_type *>(input_items[0]);
            auto out = static_cast<output_type *>(output_items[0]);

            d_counter++;

            int to_consume = noutputs_to_ninputs(noutput_items);
            assert(to_consume <= ninput_items[0]);
//            std::cout << fmt::format("********** CNT={}, inps={}, outps={}, to_consume={}", d_counter, ninput_items[0], noutput_items,
//                                     to_consume) << std::endl;
            std::vector<gr::tag_t> tagVector;
            get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + to_consume);
            bool wrap_up = false;
            if (tagVector.size() > 0) {
                if (tagVector[0].offset == nitems_read(0)) {
                    add_item_tag(0, nitems_written(0), tagVector[0].key, tagVector[0].value);
                    if (tagVector.size() > 1) {
//                        std::cout << "BBBBB" << std::endl;
                        to_consume = tagVector[1].offset - nitems_read(0);
                    }
                } else {
                    d_debug = true;
                    to_consume = tagVector[0].offset - nitems_read(0);
//                    std::cout << "CCCCC  " << to_consume << "   " << ninput_items[0] << std::endl;
                    wrap_up = true;
                }
            }

            // Put input into internal buffer
            for (int i = 0; i < to_consume; i++) {
                d_buf.push_back(in[i]);
                d_unconsumed++;
            }
//            std::cout << "Pushed " << to_consume << " items to buffer. Buffer size: " << d_buf.size()
//                      << std::endl;
            consume_each(to_consume);

            bool has_overlap = ((int) d_buf.size() - d_unconsumed) >= d_overlap;
            int vecs_to_produce =
                    std::floor(((float) d_unconsumed + (has_overlap ? d_overlap : 0) - d_vector_size) /
                               (d_vector_size - d_overlap) +
                               1);
//            std::cout << "2~>>" << bool(noutput_items >= vecs_to_produce) << std::endl;
            vecs_to_produce = std::min(vecs_to_produce, noutput_items);
//            std::cout << fmt::format("\t** has_overlap: {}, vecs_to_produce: {}, d_unconsumed: {}, d_buf.size(): {}",
//                                     has_overlap, vecs_to_produce, d_unconsumed, d_buf.size()) << std::endl;
            int onset;
            if (!vecs_to_produce) {
                if (!wrap_up) {
//                    std::cout << "DDDD" << std::endl;
                    // Not enough data to produce a vector.
                    return 0;
                } else {
//                    std::cout << "EEEEE" << std::endl;
                    // Retuning has occurred.
                    vecs_to_produce = std::floor(((float) d_buf.size() - d_vector_size) / (d_vector_size - d_overlap) + 1);
                    onset = d_buf.size() - d_vector_size;
                    // Even with whole buffer we can not produce any vector.
                    if (!vecs_to_produce) {
//                        std::cout << "FFFF" << std::endl;
                        d_unconsumed = 0;
                        d_buf.clear();
                        return 0;
                    }
                }
            } else {
                onset = (int) d_buf.size() - d_unconsumed - (has_overlap ? d_overlap : 0);
            }
//            std::cout << fmt::format("BEFORE: unconsumed: {}, onset: {}, vecs_to_produce: {}",
//                                     d_unconsumed, onset, vecs_to_produce) << std::endl;
            int ind;
            int kk = 0;
//            std::cout << "2~>>" << bool(noutput_items >= vecs_to_produce) << std::endl;
            for (int i = 0; i < vecs_to_produce; i++) {
                for (int j = 0; j < d_vector_size; j++) {
                    ind = i * d_vector_size + j - i * d_overlap;
                    out[i * d_vector_size + j] = d_buf[onset + ind];
                    kk++;
                }
            }
            d_unconsumed = d_buf.size() - (onset + ind + 1);
            assert(d_unconsumed < d_vector_size);
//            std::cout << fmt::format("AFTER: unconsumed: {}, onset: {}, vecs_to_produce: {}, kk: {}, ind: {}",
//                                     d_unconsumed, onset, vecs_to_produce, kk, ind) << std::endl;
            if (wrap_up) {
//                std::cout << "WRAP" << std::endl;
                d_unconsumed = 0;
                d_buf.clear();
            }
            while ((int) d_buf.size() >= d_vector_size)
                d_buf.pop_front();
//            std::cout << fmt::format("d_buf.size(): {}, d_unconsumed: {}, vecs_to_produce: {}",
//                                     d_buf.size(), d_unconsumed, vecs_to_produce) << std::endl;
            return vecs_to_produce;
        }

    } /* namespace aggmux */
} /* namespace gr */
