/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_USRP_AGGREGATE_VCC_IMPL_H
#define INCLUDED_AGGMUX_USRP_AGGREGATE_VCC_IMPL_H

#include <gnuradio/high_res_timer.h>
#include <gnuradio/aggmux/usrp_aggregate_vcc.h>
#include <filesystem>

namespace gr {
namespace aggmux {

class usrp_aggregate_vcc_impl : public usrp_aggregate_vcc
{
private:
    int d_vector_size;
    int d_max_update_freq;
    int d_modalities;
    int d_sid;
    bool d_debug;
    long d_counter;
    int d_num_modalities;
    float *d_aggs;
    float *d_tmp_agg;
    float d_default_thresh_val;

    std::vector<float*> d_FFT_max;
    std::vector<float*> d_FFT_sum;
    std::vector<float> d_FFT_count;
    std::vector<float*> d_FFT_thresh_count;
    std::vector<pmt::pmt_t> d_FFT_tune_info;
    std::vector<double> d_last_FreqSrate;
    int d_FFT_which;

    bool d_AvgFlag, d_MaxFlag, d_ThreshFlag;

    gr::high_res_timer_type d_interval;
    gr::high_res_timer_type d_next_update;
    gr::high_res_timer_type d_onesec;

    std::deque<float*> d_out_queue;

    bool d_calibRunning;
    std::filesystem::path d_calibPath;
    std::ofstream *d_calibDataFile;
    std::ofstream *d_calibMetadata;
    double d_calibLastFreq;
    int d_calibCounter, d_calibFreqCounter;

public:
    usrp_aggregate_vcc_impl(
        int vector_size, int max_update_freq, int modalities, int sid, bool debug);
    ~usrp_aggregate_vcc_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_USRP_AGGREGATE_VCC_IMPL_H */
