/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_SIGNALHOUND_SOURCE_IMPL_H
#define INCLUDED_AGGMUX_SIGNALHOUND_SOURCE_IMPL_H

#include <gnuradio/aggmux/signalhound_source.h>

#include <filesystem>
#include <gnuradio/high_res_timer.h>
#include "zmq.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>

namespace gr {
    namespace aggmux {

        class signalhound_source_impl : public signalhound_source {
        private:
            typedef struct {
                uint64_t startFrequency;
                uint64_t endFrequency;
                int64_t nsSinceEpoch;
                uint32_t numFftBins;
                float fftData[];

            } fftSweepMsg;

            int d_fft_bins;
            int d_modalities;
            int d_max_update_freq;
            int d_counter;
            bool d_debug;
            bool d_initFlag;

            bool d_AvgFlag, d_MaxFlag, d_ThreshFlag;

            uint64_t d_startFreq;
            uint64_t d_endFreq;
            uint32_t d_numBins;

//            fftSweepMsg *d_fftMsg;

            int64_t d_interval;
            int64_t d_next_update;
            gr::high_res_timer_type d_onesec;
            gr::high_res_timer_type d_toc;

            zmq::context_t *d_context;
            zmq::socket_t *d_socket;
            zmq::message_t *d_msg;

            std::vector<float> linspace(float a, float b, uint32_t N);

            std::vector<float> d_orig_pos;
            int d_interm_interval;
            std::vector<float> d_interm_pos;
            std::vector<float> d_out_pos;

//            std::vector<float> d_interm_bins;
            std::vector<float> * interm_bins_ptr;
            std::vector<float> d_iter_max, d_iter_mean, d_iter_thresh;
            std::vector<float> d_frame_max, d_frame_mean, d_frame_thresh;

            std::vector<float> *d_thresholds;

            bool d_calibRunning;
            std::filesystem::path d_calibPath;
            std::ofstream *d_calibDataFile;
            std::ofstream *d_calibMetadata;
            int d_calibCounter;

            int d_iter_counter;

            float lerp(float x, float x1, float x2, float y1, float y2);

            void interp_indices(const std::vector<float> &x, const std::vector<float> &xp, std::vector<int> &ind);
            void interp2(const std::vector<int> &ind, const std::vector<float> &xp, const std::vector<float> &fp, const std::vector<float> &x, std::vector<float> &fp_interp);

            void interp(const std::vector<float> &xp, const std::vector<float> &fp, const std::vector<float> &x, std::vector<float> &fp_interp);

            bool m_stop_request;
            bool m_running;
            std::mutex m_mutex;
            std::deque<std::vector<float>*> m_binQueue;
            float *binData;
            void producerThread();
            std::thread m_producerThread;
            std::condition_variable m_cond;

        public:
            signalhound_source_impl(const std::string &address,
                                    int fft_bins,
                                    int modalities,
                                    int max_update_freq,
                                    bool debug);

            ~signalhound_source_impl();

            int general_work(int noutput_items,
                             gr_vector_int &ninput_items,
                             gr_vector_const_void_star &input_items,
                             gr_vector_void_star &output_items);
        };

    } // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_SIGNALHOUND_SOURCE_IMPL_H */
