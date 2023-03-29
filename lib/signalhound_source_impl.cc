/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "signalhound_source_impl.h"
#include <gnuradio/io_signature.h>

#include <fmt/format.h>
#include <bitset>
#include <numeric>
#include <immintrin.h>

namespace gr {
    namespace aggmux {

        using input_type = float;
        using output_type = float;

        signalhound_source::sptr signalhound_source::make(const std::string &address,
                                                          int fft_bins,
                                                          int modalities,
                                                          int max_update_freq,
                                                          bool debug) {
            return gnuradio::make_block_sptr<signalhound_source_impl>(
                    address, fft_bins, modalities, max_update_freq, debug);
        }


/*
 * The private constructor
 */
        signalhound_source_impl::signalhound_source_impl(const std::string &address,
                                                         int fft_bins,
                                                         int modalities,
                                                         int max_update_freq,
                                                         bool debug)
                : gr::block("signalhound_source",
                            gr::io_signature::make(0, 0, 0),
                            gr::io_signature::make(1, 1, sizeof(output_type) * std::bitset<32>(modalities).count() * fft_bins)),
                  d_fft_bins(fft_bins),
                  d_modalities(modalities),
                  d_max_update_freq(max_update_freq),
                  d_counter(0),
                  d_debug(debug),
                  d_initFlag(true),
                  d_thresholds(nullptr),
                  d_calibDataFile(nullptr),
                  d_calibMetadata(nullptr),
                  d_calibCounter(0) {
            d_AvgFlag = d_modalities & 1;
            d_MaxFlag = d_modalities & 2;
            d_ThreshFlag = d_modalities & 4;

            d_calibPath = "/tmp/sh_calib_started.flag";

            d_context = new zmq::context_t(1);
            d_socket = new zmq::socket_t(*d_context, ZMQ_SUB);
            d_socket->connect(address);
            d_socket->setsockopt(ZMQ_SUBSCRIBE, "", 0);

            d_onesec = high_res_timer_tps();
            d_interval = d_onesec / (float) d_max_update_freq;

            d_iter_mean = std::vector<float>(d_fft_bins, 0);
            d_iter_max = std::vector<float>(d_fft_bins, 0);
            d_iter_thresh = std::vector<float>(d_fft_bins, 0);

            d_frame_mean = std::vector<float>(d_fft_bins, 0);
            d_frame_max = std::vector<float>(d_fft_bins, 0);
            d_frame_thresh = std::vector<float>(d_fft_bins, 0);

            d_next_update = (high_res_timer_now() + 3 * d_onesec) / d_onesec * d_onesec;
            std::cout << "SH FIRST UPDATE" << d_next_update << std::endl;

            m_producerThread = std::thread(&signalhound_source_impl::producerThread, this);
        }

/*
 * Our virtual destructor.
 */
        signalhound_source_impl::~signalhound_source_impl() {
            delete d_socket;
            delete d_context;
            delete d_thresholds;
        }

        std::vector<float> signalhound_source_impl::linspace(float a, float b, uint32_t N) {
            float h = (b - a) / static_cast<float>(N - 1);
            std::vector<float> xs(N);
            typename std::vector<float>::iterator x;
            float val;
            for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
                *x = val;
            return xs;
        }

        float signalhound_source_impl::lerp(float x, float x1, float x2, float y1, float y2) {
            return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
        }

        void signalhound_source_impl::interp_indices(const std::vector<float> &x, const std::vector<float> &xp,
                                                     std::vector<int> &ind) {
            ind.reserve(x.size());
            for (size_t i = 0; i < x.size(); i++) {
                auto it = std::upper_bound(xp.begin(), xp.end(), x[i]);
                if (it == xp.begin()) {
                    ind.push_back(-1);
                } else if (it == xp.end()) {
                    ind.push_back(-2);
                } else {
                    ind.push_back(std::distance(xp.begin(), it));
                }
            }
        }

        void signalhound_source_impl::interp2(const std::vector<int> &ind, const std::vector<float> &xp,
                                              const std::vector<float> &fp, const std::vector<float> &x,
                                              std::vector<float> &fp_interp) {
            fp_interp.reserve(x.size());
            for (size_t i = 0; i < x.size(); i++) {
                if (ind[i] == -1) {
                    fp_interp.push_back(fp.front());
                } else if (ind[i] == -2) {
                    fp_interp.push_back(fp.back());
                } else {
                    fp_interp.push_back(lerp(x[i], xp[ind[i] - 1], xp[ind[i]], fp[ind[i] - 1], fp[ind[i]]));
                }
            }
        }

        void signalhound_source_impl::producerThread() {
            zmq::message_t msg;
            fftSweepMsg *fftMsg;
            std::vector<float> *interm_bins;
            std::vector<int> interp_ind;
            while (true) {
                auto res = d_socket->recv(msg);
                if (res) {
                    fftMsg = (fftSweepMsg *) msg.data();
                    std::vector<float> orig_bins(fftMsg->fftData, fftMsg->fftData + fftMsg->numFftBins);

                    if (d_initFlag) {
                        d_initFlag = false;
                        d_startFreq = fftMsg->startFrequency;
                        d_endFreq = fftMsg->endFrequency;
                        d_numBins = fftMsg->numFftBins;
                        float offset = (d_endFreq - d_startFreq) / (float) d_numBins / 2.0;
                        d_orig_pos = linspace(d_startFreq + offset, d_endFreq - offset, d_numBins);
                        d_out_pos = linspace(100000000, 6000000000, d_fft_bins);
                        d_interm_interval = std::ceil((float) d_numBins / (float) d_fft_bins);
                        d_interm_pos = linspace(100000000, 6000000000, d_interm_interval * d_fft_bins);
                        interp_indices(d_interm_pos, d_orig_pos, interp_ind);
                        std::cout << "SH PRODUCER INITIALIZED" << std::endl;
                    }
                    interm_bins = new std::vector<float>();
                    interp2(interp_ind, d_orig_pos, orig_bins, d_interm_pos, *interm_bins);
//                    interp(d_orig_pos, orig_bins, d_interm_pos, *interm_bins);
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_binQueue.push_back(interm_bins);
                    m_cond.notify_one();
//                    std::cout << "SH PRODUCER THREAD PRODUCED " << fftMsg->numFftBins << " " << interm_bins->size() << std::endl;
                } else {
                    std::cout << "SH RECV ERROR" << std::endl;
                    return;
                }
            }
        }

        int signalhound_source_impl::general_work(int noutput_items,
                                                  gr_vector_int &ninput_items,
                                                  gr_vector_const_void_star &input_items,
                                                  gr_vector_void_star &output_items) {
            auto out = static_cast<output_type *>(output_items[0]);
            d_counter++;
//            if (d_counter == 1) {
//                d_next_update = (high_res_timer_now() + 2 * d_onesec) / d_onesec * d_onesec;
//                std::cout << "SH FIRST UPDATE" << d_next_update << std::endl;
//            }
            zmq::message_t msg;
            std::fill(d_frame_mean.begin(), d_frame_mean.end(), 0);
            std::fill(d_frame_max.begin(), d_frame_max.end(), 0);
            std::fill(d_frame_thresh.begin(), d_frame_thresh.end(), 0);
            d_iter_counter = 0;
            while (1) {
                gr::high_res_timer_type toc = high_res_timer_now();
                if (toc - d_next_update > 0) {
                    int missed = -1;
                    while (toc - d_next_update > 0) {
                        d_next_update += d_interval;
                        missed++;
                    }
                    if (missed > 0) {
                        std::cout << "SH Missed " << missed << " updates" << std::endl;
                    }
                    break;
                }
                d_iter_counter++;
//                std::vector<float> *bins;
                std::unique_lock<std::mutex> lock(m_mutex);
                m_cond.wait(lock, [this] { return !m_binQueue.empty(); });
//                std::cout << fmt::format("{} SH CONSUMER THREAD CONSUMED! \t qsize: {}", d_iter_counter, m_binQueue.size())
//                          << std::endl;
                interm_bins_ptr = m_binQueue.front();
                m_binQueue.pop_front();
                lock.unlock();
//                auto res = d_socket->recv(msg);
//                if (!res) {
//                    throw std::runtime_error("Error receiving data from SignalHound");
//                }
//                d_fftMsg = (fftSweepMsg *) msg.data();
//                std::vector<float> orig_bins(d_fftMsg->fftData, d_fftMsg->fftData + d_fftMsg->numFftBins);
//                std::vector<float> orig_bins = *bins;
//                if (d_initFlag) {
//                    d_initFlag = false;
//                    d_startFreq = d_fftMsg->startFrequency;
//                    d_endFreq = d_fftMsg->endFrequency;
//                    d_numBins = d_fftMsg->numFftBins;
//                    float offset = (d_endFreq - d_startFreq) / (float) d_numBins / 2.0;
//                    d_orig_pos = linspace(d_startFreq + offset, d_endFreq - offset, d_numBins);
//                    d_out_pos = linspace(100000000, 6000000000, d_fft_bins);
//                    d_interm_interval = std::ceil((float) d_numBins / (float) d_fft_bins);
//                    d_interm_pos = linspace(100000000, 6000000000, d_interm_interval * d_fft_bins);
//
//                }
                bool calibFlag = std::filesystem::exists(d_calibPath);
                if (calibFlag && !d_calibRunning) {
                    d_calibRunning = true;
                    // TODO: I need to add an ID for SignalHound as there may be more than one!
                    d_calibDataFile = new std::ofstream(fmt::format("/tmp/sh_calib_data_{}.bin", d_fft_bins),
                                                        std::ios::binary | std::ios::binary);
                    d_calibMetadata = new std::ofstream(fmt::format("/tmp/sh_calib_metadata_{}.bin", d_fft_bins),
                                                        std::ios::binary | std::ios::binary);
                } else if (!calibFlag && d_calibRunning) {
                    d_calibRunning = false;
                    d_calibDataFile->close();
                    d_calibMetadata->close();
                    delete d_calibDataFile;
                    delete d_calibMetadata;
                }
                if (d_thresholds == nullptr) {
                    std::cout << "Initializing thresholds" << std::endl;
                    d_thresholds = static_cast<std::vector<float> *>(malloc(sizeof(std::vector<float>)));
                    new(d_thresholds) std::vector<float>(d_interm_pos.size(), -97);
                    if (std::filesystem::exists(
                            fmt::format("/home/nima/PermanentDir/signalhound_thresholds_{}.bin", d_fft_bins))) { ;
                        std::cout << "Getting thresholds from file" << std::endl;
                        try {
                            std::ifstream ifs(fmt::format("/home/nima/PermanentDir/signalhound_thresholds_{}.bin", d_fft_bins),
                                              std::ios::binary);
                            float f;
                            int i = 0;
                            while (ifs.read(reinterpret_cast<char *>(&f), sizeof(float))) {
                                d_thresholds->at(i) = f;
                            }
                            std::cout << "DONEDONEDONE" << std::endl;
                            ifs.close();
                        } catch (std::exception &e) {
                            std::cout << "WHATDA FUCK!" << e.what() << std::endl;
                        }
                    }
                }
//                if (d_calibRunning) {
//                    if (d_iter_counter % 3 == 0) {
//                        d_calibDataFile->write((char *) d_fftMsg->fftData, d_fftMsg->numFftBins * sizeof(float));
//                        d_calibMetadata->write((char *) &d_fftMsg->startFrequency, sizeof(uint64_t));
//                        d_calibMetadata->write((char *) &d_fftMsg->endFrequency, sizeof(uint64_t));
//                        d_calibMetadata->write((char *) &d_fftMsg->numFftBins, sizeof(uint32_t));
//                        d_calibCounter++;
//                    }
//                    if (d_calibCounter > 1000) {
//                        try {
//                            std::filesystem::remove(d_calibPath);
//                        } catch (std::filesystem::filesystem_error &e) {
//                            std::cout << "Error removing calibration flag file: " << e.what() << std::endl;
//                        }
//                    }
//                }
//                d_interm_bins = interp(d_orig_pos, orig_bins, d_interm_pos);
                for (int i = 0; i < d_fft_bins; i++) {
                    auto start = interm_bins_ptr->begin() + i * d_interm_interval;
                    auto end = std::next(start, d_interm_interval);
                    auto thresholdStart = d_thresholds->begin() + i * d_interm_interval;
                    d_iter_max[i] = *std::max_element(start, end);
                    d_iter_mean[i] = std::accumulate(start, end, 0.0) / d_interm_interval;
                    int count = std::inner_product(start, end, thresholdStart, 0.0, std::plus<int>(),
                                                   [](float a, float b) { return a >= b; });
                    d_iter_thresh[i] = static_cast<float>(count) / d_interm_interval;
                }
                delete interm_bins_ptr;
                std::transform(d_frame_mean.begin(), d_frame_mean.end(), d_iter_mean.begin(), d_frame_mean.begin(),
                               std::plus<float>());
                std::transform(d_frame_max.begin(), d_frame_max.end(), d_iter_max.begin(), d_frame_max.begin(),
                               std::plus<float>());
                std::transform(d_frame_thresh.begin(), d_frame_thresh.end(), d_iter_thresh.begin(), d_frame_thresh.begin(),
                               std::plus<float>());
            }
            int m = 0;
            if (d_AvgFlag) {
                for (int i = 0; i < d_fft_bins; i++) {
                    out[m * d_fft_bins + i] = d_frame_mean[i] / d_iter_counter;
                }
                m += 1;
            }
//            std::cout << ">>" << out[0] << "\t" << out[1] << "\t" << out[2] << "\t" << out[3] << "\t" << std::endl;
            if (d_MaxFlag) {
                for (int i = 0; i < d_fft_bins; i++) {
                    out[m * d_fft_bins + i] = d_frame_max[i] / d_iter_counter;
                }
                m += 1;
            }
            if (d_ThreshFlag) {
                for (int i = 0; i < d_fft_bins; i++) {
                    out[m * d_fft_bins + i] = d_frame_thresh[i] / d_iter_counter;
                }
                m += 1;
            }
//            std::cout << "d_counter: " << d_counter << "noutput_items: " << noutput_items << std::endl;
            return 1;
        }
    } /* namespace aggmux */
} /* namespace gr */
