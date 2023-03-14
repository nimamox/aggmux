/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_USRP_AGGREGATE_VCC_H
#define INCLUDED_AGGMUX_USRP_AGGREGATE_VCC_H

#include <gnuradio/aggmux/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace aggmux {

/*!
 * \brief <+description of block+>
 * \ingroup aggmux
 *
 */
class AGGMUX_API usrp_aggregate_vcc : virtual public gr::block
{
public:
    typedef std::shared_ptr<usrp_aggregate_vcc> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of aggmux::usrp_aggregate_vcc.
     *
     * To avoid accidental use of raw pointers, aggmux::usrp_aggregate_vcc's
     * constructor is in a private implementation
     * class. aggmux::usrp_aggregate_vcc::make is the public interface for
     * creating new instances.
     */
    static sptr make(int vector_size = 1024,
                     int max_update_freq = 15,
                     int modalities = 7,
                     int sid = 0,
                     bool debug = false);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_USRP_AGGREGATE_VCC_H */
