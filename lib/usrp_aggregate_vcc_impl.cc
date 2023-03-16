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
                int vector_size, int max_update_freq, int modalities, int sid, bool debug) {
            return gnuradio::make_block_sptr<usrp_aggregate_vcc_impl>(
                    vector_size, max_update_freq, modalities, sid, debug);
        }


/*
 * The private constructor
 */
        usrp_aggregate_vcc_impl::usrp_aggregate_vcc_impl(
                int vector_size, int max_update_freq, int modalities, int sid, bool debug)
                : gr::block("usrp_aggregate_vcc",
                            gr::io_signature::make(1, 1, sizeof(input_type) * vector_size),
                            gr::io_signature::make(1, 1, sizeof(output_type) * std::bitset<32>(modalities).count() * vector_size)),
                  d_vector_size(vector_size),
                  d_max_update_freq(max_update_freq),
                  d_modalities(modalities),
                  d_sid(sid),
                  d_debug(debug),
                  d_counter(0),
                  d_FFT_which(0),
                  d_calibRunning(false),
                  d_calibDataFile(nullptr),
                  d_calibMetadata(nullptr),
                  d_calibCounter(0),
                  d_calibFreqCounter(0){
            d_AvgFlag = d_modalities & 1;
            d_MaxFlag = d_modalities & 2;
            d_ThreshFlag = d_modalities & 4;
            d_num_modalities = d_AvgFlag + d_MaxFlag + d_ThreshFlag;

            d_aggs = (float *) volk_malloc(sizeof(float) * d_vector_size * d_num_modalities, volk_get_alignment());
            d_tmp_agg = (float *) volk_malloc(sizeof(float) * d_vector_size, volk_get_alignment());

            set_tag_propagation_policy(TPP_DONT);

            d_calibPath = "/tmp/usrp_calib_started.flag";

            d_onesec = high_res_timer_tps();
            d_interval = d_onesec / (float) d_max_update_freq;

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

            d_next_update = (high_res_timer_now() + 2 * d_onesec) / d_onesec * d_onesec;
            std::cout << d_sid << " SDR FIRST UPDATE" << d_next_update << std::endl;
        }

/*
 * Our virtual destructor.
 */
        usrp_aggregate_vcc_impl::~usrp_aggregate_vcc_impl() {
            std::cout << fmt::format("\t** AGG: ({}) - DESTRUCTOR CALLED!", d_sid) << std::endl;
            volk_free(d_aggs);
            volk_free(d_tmp_agg);
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
//            if (d_counter == 1) {
//                d_next_update = (high_res_timer_now() + 2 * d_onesec) / d_onesec * d_onesec;
//                std::cout << d_sid << " SDR FIRST UPDATE" << d_next_update << std::endl;
//            }
            std::vector<gr::tag_t> tagVector;
            get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + ninput_items[0]);
            int remainderOffset = 0;
            int curOffset = 0;

            bool calibFlag = std::filesystem::exists(d_calibPath);
            if (calibFlag && !d_calibRunning) {
                d_calibRunning = true;
                d_calibDataFile = new std::ofstream(fmt::format("/tmp/usrp_calib_data_{}__{}.bin", d_sid, d_vector_size),
                                                    std::ios::binary | std::ios::binary);
                d_calibMetadata = new std::ofstream(fmt::format("/tmp/usrp_calib_metadata_{}__{}.bin", d_sid, d_vector_size),
                                                    std::ios::binary | std::ios::binary);
            } else if (!calibFlag && d_calibRunning) {
                d_calibRunning = false;
                d_calibDataFile->close();
                d_calibMetadata->close();
                delete d_calibDataFile;
                delete d_calibMetadata;
            }

            if (tagVector.size() > 0) {
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
                                for (int j = remainderOffset; j < curOffset; j++) {
                                    volk_32f_x2_add_32f(d_FFT_sum[d_FFT_which], d_FFT_sum[d_FFT_which], in + j * d_vector_size,
                                                        d_vector_size);
                                }
                            }
                            if (d_MaxFlag) {
                                for (int j = remainderOffset; j < curOffset; j++) {
                                    volk_32f_x2_max_32f(d_FFT_max[d_FFT_which], d_FFT_max[d_FFT_which], in + j * d_vector_size,
                                                        d_vector_size);
                                }
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
                if (d_calibRunning) {
                    double freq = pmt::to_double(
                            pmt::dict_ref(d_FFT_tune_info[d_FFT_which], pmt::intern("rx_freq"), pmt::from_double(-2)));
                    double rate = pmt::to_double(
                            pmt::dict_ref(d_FFT_tune_info[d_FFT_which], pmt::intern("rx_rate"), pmt::from_double(-2)));
                    if (d_calibLastFreq != freq) {
                        d_calibLastFreq = freq;
                        d_calibFreqCounter = 0;
                    }
                    for (int j = remainderOffset; j < curOffset; j++) {
                        if (d_calibFreqCounter > 50)
                            break;
                        d_calibDataFile->write(reinterpret_cast<const char *>(in + j * d_vector_size),
                                               d_vector_size * sizeof(float));
                        d_calibMetadata->write(reinterpret_cast<const char *>(&d_calibCounter), sizeof(int));
                        d_calibMetadata->write(reinterpret_cast<const char *>(&freq), sizeof(double));
                        d_calibMetadata->write(reinterpret_cast<const char *>(&rate), sizeof(double));
                        d_calibCounter++;
                        d_calibFreqCounter++;
                    }
//                    d_calibDataFile->write(reinterpret_cast<const char *>(in + remainderOffset * d_vector_size),
//                                           (curOffset - remainderOffset) * d_vector_size * sizeof(gr_complex));
//                    d_calibMetadata->write(reinterpret_cast<const char *>(&d_FFT_tune_info[d_FFT_which]),
//                                           sizeof(pmt::pmt_t));
                }
                d_FFT_count[d_FFT_which] += curOffset - remainderOffset;
                if (d_AvgFlag) {
                    for (int j = remainderOffset; j < curOffset; j++) {
                        volk_32f_x2_add_32f(d_FFT_sum[d_FFT_which], d_FFT_sum[d_FFT_which], in + j * d_vector_size, d_vector_size);
                    }
                }
                if (d_MaxFlag) {
                    for (int j = remainderOffset; j < curOffset; j++) {
                        volk_32f_x2_max_32f(d_FFT_max[d_FFT_which], d_FFT_max[d_FFT_which], in + j * d_vector_size, d_vector_size);
                    }
                }
            }
            consume_each(ninput_items[0]);
            gr::high_res_timer_type toc = high_res_timer_now();
//            std::cout << "Time: " << toc << "next_update" << d_next_update << "cond: " << ((toc - d_next_update) > 0) << std::endl;
            if ((toc - d_next_update) > 0) {
                int missed = -1;
                while ((toc - d_next_update) > 0) {
                    d_next_update += d_interval;
                    missed++;
                }
                if (missed > 0) {
                    std::cout << "Missed " << missed << " updates" << std::endl;
                }
                int selected = d_FFT_which;
                if (((float) d_FFT_count[selected] / (d_FFT_count[0] + d_FFT_count[1])) < .25) {
                    selected = (selected + 1) % 2;
                }
                double new_freq = pmt::to_double(
                        pmt::dict_ref(d_FFT_tune_info[selected], pmt::intern("rx_freq"), pmt::from_double(-2)));
                double new_rate = pmt::to_double(
                        pmt::dict_ref(d_FFT_tune_info[selected], pmt::intern("rx_rate"), pmt::from_double(-2)));
                if ((d_last_FreqSrate[0] != new_freq) || (d_last_FreqSrate[1] != new_rate)) {
                    d_last_FreqSrate[0] = new_freq;
                    d_last_FreqSrate[1] = new_rate;
                    auto new_tag = pmt::make_dict();
                    new_tag = pmt::dict_add(new_tag, pmt::intern("rx_freq"), pmt::from_double(new_freq));
                    new_tag = pmt::dict_add(new_tag, pmt::intern("rx_rate"), pmt::from_double(new_rate));
                    add_item_tag(0, nitems_written(0), pmt::intern("tune"), new_tag);
                    std::cout << fmt::format("\t** AGG ({}) - NEW TAG SENT freq: {}", d_sid, new_freq) << std::endl;
                }
                int m = 0;
                if (d_AvgFlag || d_ThreshFlag) {
                    volk_32f_s32f_multiply_32f(d_FFT_sum[selected], d_FFT_sum[selected], 1.0 / d_FFT_count[selected], d_vector_size);
                }
                if (d_AvgFlag) {
                    for (int i = 0; i < d_vector_size; i++) {
                        out[m * d_vector_size + i] = 10 * log10(d_FFT_sum[selected][i] + 1e-20);
                    }
                    m++;
                }
                if (d_MaxFlag) {
                    for (int i = 0; i < d_vector_size; i++) {
                        out[m * d_vector_size + i] = 10 * log10(d_FFT_max[selected][i] + 1e-20);
                    }
                    m++;
                }
                if (d_ThreshFlag) {
                    for (int i = 0; i < d_vector_size; i++) {
                        out[m * d_vector_size + i] = 10 * log10(d_FFT_sum[selected][i] + 1e-20);
                    }
                    m++;
                }
                if (d_AvgFlag || d_ThreshFlag)
                    memset(d_FFT_sum[selected], 0, sizeof(float) * d_vector_size);
                if (d_MaxFlag)
                    memset(d_FFT_max[selected], -1, sizeof(float) * d_vector_size);
                d_FFT_count[selected] = 0;
                return 1;
            }
            return 0;
        }

    } /* namespace aggmux */
} /* namespace gr */
