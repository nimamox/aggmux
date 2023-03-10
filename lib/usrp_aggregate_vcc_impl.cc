/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "usrp_aggregate_vcc_impl.h"
#include <gnuradio/io_signature.h>

#include <volk/volk.h>
#include <fmt/format.h>

namespace gr {
    namespace aggmux {

        using input_type = float;
        using output_type = float;

        usrp_aggregate_vcc::sptr usrp_aggregate_vcc::make(
                int vector_size, int out_vector_size, int max_update_freq, int modalities, int sid, bool debug) {
            return gnuradio::make_block_sptr<usrp_aggregate_vcc_impl>(
                    vector_size, out_vector_size, max_update_freq, modalities, sid, debug);
        }


/*
 * The private constructor
 */
        usrp_aggregate_vcc_impl::usrp_aggregate_vcc_impl(
                int vector_size, int out_vector_size, int max_update_freq, int modalities, int sid, bool debug)
                : gr::block("usrp_aggregate_vcc",
                            gr::io_signature::make(1, 1, sizeof(input_type) * vector_size),
                            gr::io_signature::make(1, 1, sizeof(output_type) * out_vector_size)),
                  d_vector_size(vector_size),
                  d_out_vector_size(out_vector_size),
                  d_max_update_freq(max_update_freq),
                  d_modalities(modalities),
                  d_sid(sid),
                  d_debug(debug),
                  d_counter(0),
                  d_FFT_which(0),
                  d_initFlag(true),
                  d_buf_filled_vecs(0) {
            d_AvgFlag = d_modalities & 1;
            d_MaxFlag = d_modalities & 2;
            d_ThreshFlag = d_modalities & 4;
            d_num_modalities = d_AvgFlag + d_MaxFlag + d_ThreshFlag;

            d_aggs = (float *) volk_malloc(sizeof(float) * d_out_vector_size * d_num_modalities, volk_get_alignment());

            set_tag_propagation_policy(TPP_DONT);

            d_tic = high_res_timer_now();
            gr::high_res_timer_type onesec = high_res_timer_tps();
            d_interval = onesec / (float) d_max_update_freq;

            d_last_FreqSrate.push_back(-1);
            for (int j = 0; j < 2; j++) {
                d_FFT_max.push_back((float *) volk_malloc(sizeof(float) * d_vector_size, volk_get_alignment()));
                memset(d_FFT_max[j], 0, sizeof(float) * d_vector_size);
                d_FFT_sum.push_back((float *) volk_malloc(sizeof(float) * d_vector_size, volk_get_alignment()));
                memset(d_FFT_sum[j], 0, sizeof(float) * d_vector_size);
                d_FFT_count.push_back(0);
                d_FFT_tune_info.push_back(pmt::make_dict());
                d_FFT_tune_info[j] = pmt::dict_add(d_FFT_tune_info[j], pmt::intern("rx_freq"), pmt::from_double(-1));
                d_FFT_tune_info[j] = pmt::dict_add(d_FFT_tune_info[j], pmt::intern("rx_rate"), pmt::from_double(-1));
            }
        }

/*
 * Our virtual destructor.
 */
        usrp_aggregate_vcc_impl::~usrp_aggregate_vcc_impl() {
            volk_free(d_aggs);
            for (int j = 0; j < 2; j++) {
                volk_free(d_FFT_max[j]);
                volk_free(d_FFT_sum[j]);
            }
        }

        void usrp_aggregate_vcc_impl::forecast(int noutput_items,
                                               gr_vector_int &ninput_items_required) {
            ninput_items_required[0] = noutput_items;
        }

        int usrp_aggregate_vcc_impl::general_work(int noutput_items,
                                                  gr_vector_int &ninput_items,
                                                  gr_vector_const_void_star &input_items,
                                                  gr_vector_void_star &output_items) {
            auto in = static_cast<const input_type *>(input_items[0]);
            auto out = static_cast<output_type *>(output_items[0]);

            d_counter++;

            std::vector<gr::tag_t> tagVector;
            get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + ninput_items[0]);
            int remainderOffset = 0;
            int curOffset = 0;
            if (tagVector.size() > 0) {
                if (d_sid == 0)
                    std::cout << fmt::format("AGG: TAG RECEIVED {} BOOM BOOM!", d_counter) << std::endl;
//                    std::cout << "AGG: TAG RECEIVED " << d_counter << std::endl;
                for (int i = 0; i < (int) tagVector.size(); i++) {
                    auto &tagval = tagVector[i].value;
                    bool freq_changed = pmt::to_double(
                            pmt::dict_ref(d_FFT_tune_info[d_FFT_which], pmt::intern("rx_freq"), pmt::from_double(-2))) !=
                                        pmt::to_double(pmt::dict_ref(tagval, pmt::intern("rx_freq"), pmt::from_double(-2)));
                    bool rate_changed = pmt::to_double(
                            pmt::dict_ref(d_FFT_tune_info[d_FFT_which], pmt::intern("rx_rate"), pmt::from_double(-2))) !=
                                        pmt::to_double(pmt::dict_ref(tagval, pmt::intern("rx_rate"), pmt::from_double(-2)));
                    if (freq_changed || rate_changed) {
                        curOffset = tagVector[i].offset - nitems_read(0);
                        if ((curOffset - remainderOffset) > 0) {
                            d_FFT_count[d_FFT_which] += curOffset - remainderOffset;
                            if (d_AvgFlag) {
                                volk_32f_x2_add_32f(d_FFT_sum[d_FFT_which], d_FFT_sum[d_FFT_which], in + remainderOffset,
                                                    curOffset - remainderOffset);
                            }
                            if (d_MaxFlag) {
                                volk_32f_x2_max_32f(d_FFT_max[d_FFT_which], d_FFT_max[d_FFT_which], in + remainderOffset,
                                                    curOffset - remainderOffset);
                            }
                        }
                        d_FFT_which = (d_FFT_which + 1) % 2;
                        d_FFT_tune_info[d_FFT_which] = tagval;
                        remainderOffset = curOffset;
                    }
                }
            }
            curOffset = ninput_items[0];
            if ((curOffset - remainderOffset) > 0) {
                d_FFT_count[d_FFT_which] += curOffset - remainderOffset;
                if (d_AvgFlag) {
                    volk_32f_x2_add_32f(d_FFT_sum[d_FFT_which], d_FFT_sum[d_FFT_which], in + remainderOffset,
                                        curOffset - remainderOffset);
                }
                if (d_MaxFlag) {
                    volk_32f_x2_max_32f(d_FFT_max[d_FFT_which], d_FFT_max[d_FFT_which], in + remainderOffset,
                                        curOffset - remainderOffset);
                }
            }
            consume_each(ninput_items[0]);
            gr::high_res_timer_type toc = high_res_timer_now();
            if ((toc - d_tic) > d_interval) {
                d_tic = toc;
                int selected = d_FFT_which;
                if (((float) d_FFT_count[selected] / (d_FFT_count[0] + d_FFT_count[1])) < .25) {
                    selected = (selected + 1) % 2;
                }
                double new_freq = pmt::to_double(
                        pmt::dict_ref(d_FFT_tune_info[selected], pmt::intern("rx_freq"), pmt::from_double(-2)));
                double new_rate = pmt::to_double(
                        pmt::dict_ref(d_FFT_tune_info[selected], pmt::intern("rx_freq"), pmt::from_double(-2)));
                if ((d_last_FreqSrate[0] != new_freq) || (d_last_FreqSrate[1] != new_rate)) {
                    d_last_FreqSrate[0] = new_freq;
                    d_last_FreqSrate[1] = new_rate;
                    auto new_tag = pmt::make_dict();
                    new_tag = pmt::dict_add(new_tag, pmt::intern("rx_freq"), pmt::from_double(new_freq));
                    new_tag = pmt::dict_add(new_tag, pmt::intern("rx_rate"), pmt::from_double(new_rate));
                    add_item_tag(0, nitems_written(0), pmt::intern("tune"), new_tag);
                    std::cout << "AGG - Tune: " << new_freq << " " << new_rate << std::endl;
                }
//                unsigned int alignment = volk_get_alignment();
                if (d_AvgFlag) {
//                    float *agg = (float *) volk_malloc(sizeof(float) * d_vector_size, alignment);
                    float *agg = d_aggs;
                    volk_32f_s32f_multiply_32f(agg, d_FFT_sum[selected], 1.0 / d_FFT_count[selected], d_vector_size);
                    for (int i = 0; i < d_vector_size; i++) {
                        agg[i] = 10 * log10(agg[i] + 1e-20);
                    }
                    agg[0] = d_counter;
                    d_out_queue.push_back(agg);
                }
                if (d_ThreshFlag) {
                    float *agg = (float *) volk_malloc(sizeof(float) * d_vector_size, alignment);
                    volk_32f_s32f_multiply_32f(agg, d_FFT_sum[selected], 1.0 / d_FFT_count[selected], d_vector_size);
                    for (int i = 0; i < d_vector_size; i++) {
                        agg[i] = (10 * log10(agg[i] + 1e-20)) > -96;
                    }
                    agg[0] = d_counter;
                    d_out_queue.push_back(agg);
                }
                if (d_MaxFlag) {
                    float *agg = (float *) volk_malloc(sizeof(float) * d_vector_size, alignment);
                    memcpy(agg, d_FFT_max[selected], sizeof(float) * d_vector_size);
                    for (int i = 0; i < d_vector_size; i++) {
                        agg[i] = 10 * log10(agg[i] + 1e-20);
                    }
                    agg[0] = d_counter;
                    d_out_queue.push_back(agg);
                }
                if (d_AvgFlag || d_ThreshFlag)
                    memset(d_FFT_sum[selected], 0, sizeof(float) * d_vector_size);
                if (d_MaxFlag)
                    memset(d_FFT_max[selected], -1, sizeof(float) * d_vector_size);
                d_FFT_count[selected] = 0;
            }
            int written_packets = 0;
            while ((d_out_queue.size() > 0) && written_packets < noutput_items) {
                float *agg = d_out_queue.front();
                memcpy(out + written_packets * d_vector_size, agg, sizeof(float) * d_vector_size);
                volk_free(agg);
                d_out_queue.pop_front();
                written_packets++;
            }
            return written_packets;
        }

    } /* namespace aggmux */
} /* namespace gr */
