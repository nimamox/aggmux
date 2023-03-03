/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_AGGMUX_HOLD_REWRITE_TAGS_CC_H
#define INCLUDED_AGGMUX_HOLD_REWRITE_TAGS_CC_H

#include <gnuradio/aggmux/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace aggmux {

/*!
 * \brief <+description of block+>
 * \ingroup aggmux
 *
 */
class AGGMUX_API hold_rewrite_tags_cc : virtual public gr::block
{
public:
    typedef std::shared_ptr<hold_rewrite_tags_cc> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of aggmux::hold_rewrite_tags_cc.
     *
     * To avoid accidental use of raw pointers, aggmux::hold_rewrite_tags_cc's
     * constructor is in a private implementation
     * class. aggmux::hold_rewrite_tags_cc::make is the public interface for
     * creating new instances.
     */
    static sptr make(bool debug, int sid);
};

} // namespace aggmux
} // namespace gr

#endif /* INCLUDED_AGGMUX_HOLD_REWRITE_TAGS_CC_H */
