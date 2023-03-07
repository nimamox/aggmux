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
            return gnuradio::make_block_sptr<overlapping_stream_to_vec_cc_impl>(vector_size, overlap, debug, sid);
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
                  d_counter(0),
                  d_sid(sid),
                  d_unconsumed(0) {
            set_tag_propagation_policy(TPP_DONT);
        }

/*
 * Our virtual destructor.;
 */
        overlapping_stream_to_vec_cc_impl::~overlapping_stream_to_vec_cc_impl() {}

        void overlapping_stream_to_vec_cc_impl::forecast(int noutput_items,
                                                         gr_vector_int &ninput_items_required) {
            ninput_items_required[0] =
                    d_vector_size + (d_vector_size - d_overlap) * (noutput_items - 1);
            std::cout << "1forecast:" << noutput_items << "\t" << ninput_items_required[0]
                      << "\t" << d_buf.size() << std::endl;
            if ((int)d_buf.size() >= d_overlap) {
                ninput_items_required[0] -= d_overlap;
            }
            if (noutput_items <= 1)
                ninput_items_required[0] = std::min(ninput_items_required[0], 8191);
            std::cout << "2forecast:" << noutput_items << "\t" << ninput_items_required[0]
                      << std::endl;
            std::cout << "********"<< std::endl;
        }

        int overlapping_stream_to_vec_cc_impl::general_work(
                int noutput_items,
                gr_vector_int &ninput_items,
                gr_vector_const_void_star &input_items,
                gr_vector_void_star &output_items) {
            auto in = static_cast<const input_type *>(input_items[0]);
            auto out = static_cast<output_type *>(output_items[0]);

            d_counter++;

            std::cout << "~~~~~~~~>\n\tcnt: " << d_counter << "\tninp: " << ninput_items[0] << "\tnout:" << noutput_items
                      << std::endl;

            std::vector<gr::tag_t> tagVector;
            get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + noutput_items);
            int to_consume = ninput_items[0];
            bool wrap_up = false;
            if (tagVector.size() > 0) {
                std::cout << "\ttag received\t" << d_counter << "\t" << (tagVector[0].offset == nitems_read(0)) << std::endl;
                if (tagVector[0].offset == nitems_read(0)) {
                    add_item_tag(0, nitems_written(0), tagVector[0].key, tagVector[0].value);
                    if (tagVector.size() > 1){
                        to_consume = tagVector[1].offset - nitems_read(0);
                    }
                } else {
                    to_consume = tagVector[0].offset - nitems_read(0);
                    wrap_up = true;
                }
            }

            std::cout << "\tto_consume: " << to_consume << std::endl;

            // Put input into internal buffer
            for (int i = 0; i < to_consume; i++) {
                d_buf.push_back(in[i]);
                d_unconsumed++;
            }
            std::cout << "\td_unconsumed: " << d_unconsumed << std::endl;
            consume_each(to_consume);

            bool has_overlap = ((int)d_buf.size() - d_unconsumed) >= d_overlap;
            int vecs_to_produce = std::floor((d_unconsumed + (has_overlap ? d_overlap : 0) - d_vector_size) / (d_vector_size - d_overlap) + 1);
            std::cout << "\tvecs_to_produce: " << vecs_to_produce << "\thas_overlap: " << has_overlap << "\twrap_up: " << wrap_up<< std::endl;
            if (!vecs_to_produce) {
                if (!wrap_up)
                    // Not enough data to produce a vector.
                    return 0;
                else {
                    // Retuning has occurred.
                    vecs_to_produce = std::floor((d_buf.size() - d_vector_size) / (d_vector_size - d_overlap) + 1);
                    // Even with whole buffer we can not produce any vector.
                    if (!vecs_to_produce){
                        d_unconsumed = 0;
                        d_buf.clear();
                        return 0;
                    }
                }
            }
            int onset = (int) d_buf.size() - (d_vector_size + (d_vector_size - d_overlap) * (vecs_to_produce - 1));
            std::cout << "\tonset: " << onset << std::endl;
            int ind;
            for (int i = 0; i < vecs_to_produce; i++) {
                for (int j = 0; j < d_vector_size; j++) {
                    ind = i * d_vector_size + j - i * d_overlap;
                    out[i * d_vector_size + j] = d_buf[onset + ind];
                }
            }

            d_unconsumed = 0;
            if (wrap_up)
                d_buf.clear();
            while ((int)d_buf.size() >= d_vector_size)
                d_buf.pop_front();

            std::cout << "~ " << d_counter << "\tvecs produced: " << vecs_to_produce << std::endl;
            return vecs_to_produce;


            //    int vecs_to_produce = noutput_items;
            //    while (vecs_to_produce > 0) {
            //        if (d_vector_size + (d_vector_size - d_overlap) * (vecs_to_produce - 1) >
            //            to_consume + (d_buff_filled ? d_overlap : 0)) {
            //            vecs_to_produce--;
            //        } else {
            //            break;
            //        }
            //    }
            //
            //    int ind = 0;
            //    for (int i = 0; i < vecs_to_produce; i++) {
            //        for (int j = 0; j < d_vector_size; j++) {
            //            ind = i * d_vector_size + j - i * d_overlap;
            //            if (d_buff_filled && ind < d_overlap) {
            //                out[i * d_vector_size + j] = d_buffer[ind];
            //            } else {
            //                out[i * d_vector_size + j] = in[d_buff_filled?ind-d_overlap:ind];
            //            }
            //        }
            //    }
            //
            //    if (should_buffer) {
            //        for (int i = 0; i < d_overlap; i++) {
            //            d_buffer[d_overlap - i] = in[noutput_items - i];
            //        }
            //        d_buff_filled = true;
            //    }
            //
            //
            //    consume_each(to_consume);
            //    std::cout << "\tvecs produced: " << vecs_to_produce << std::endl;
            //    return vecs_to_produce;
        }

    } /* namespace aggmux */
} /* namespace gr */
