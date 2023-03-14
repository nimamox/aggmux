/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_SIGNALHOUND_SOURCE_H
#define INCLUDED_AGGMUX_SIGNALHOUND_SOURCE_H

#include <gnuradio/aggmux/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace aggmux {

/*!
 * \brief <+description of block+>
 * \ingroup aggmux
 *
 */
class AGGMUX_API signalhound_source : virtual public gr::block
{
public:
    typedef std::shared_ptr<signalhound_source> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of aggmux::signalhound_source.
     *
     * To avoid accidental use of raw pointers, aggmux::signalhound_source's
     * constructor is in a private implementation
     * class. aggmux::signalhound_source::make is the public interface for
     * creating new instances.
     */
    static sptr make(const std::string& address = "tcp://0.0.0.0:51665",
                     int fft_bins = 1024,
                     int modalities = 7,
                     int max_update_freq = 15,
                     bool debug = false);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_SIGNALHOUND_SOURCE_H */
