/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_OVERLAPPING_STREAM_TO_VEC_CC_H
#define INCLUDED_AGGMUX_OVERLAPPING_STREAM_TO_VEC_CC_H

#include <gnuradio/aggmux/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace aggmux {

/*!
 * \brief <+description of block+>
 * \ingroup aggmux
 *
 */
class AGGMUX_API overlapping_stream_to_vec_cc : virtual public gr::block
{
public:
    typedef std::shared_ptr<overlapping_stream_to_vec_cc> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of
     * aggmux::overlapping_stream_to_vec_cc.
     *
     * To avoid accidental use of raw pointers, aggmux::overlapping_stream_to_vec_cc's
     * constructor is in a private implementation
     * class. aggmux::overlapping_stream_to_vec_cc::make is the public interface for
     * creating new instances.
     */
    static sptr make(int vector_size = 1024, int overlap=1, bool debug = false, int sid =
                                                                                    0);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_OVERLAPPING_STREAM_TO_VEC_CC_H */
