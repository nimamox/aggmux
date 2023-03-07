/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_AGGR_AND_MUX_VFF_H
#define INCLUDED_AGGMUX_AGGR_AND_MUX_VFF_H

#include <gnuradio/aggmux/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace aggmux {

/*!
 * \brief <+description of block+>
 * \ingroup aggmux
 *
 */
class AGGMUX_API aggr_and_mux_vff : virtual public gr::block
{
public:
    typedef std::shared_ptr<aggr_and_mux_vff> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of aggmux::aggr_and_mux_vff.
     *
     * To avoid accidental use of raw pointers, aggmux::aggr_and_mux_vff's
     * constructor is in a private implementation
     * class. aggmux::aggr_and_mux_vff::make is the public interface for
     * creating new instances.
     */
    static sptr make(int num_usrps = 1,
                     int num_shs = 0,
                     int usrp_vector_size = 1024,
                     int sh_vector_size = 4096,
                     int out_vector_size = 512,
                     int max_update_freq = 15,
                     int modalities = 7,
                     bool debug = false);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_AGGR_AND_MUX_VFF_H */
