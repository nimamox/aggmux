/* -*- c++ -*- */
/*
 * Copyright 2023 Nima Mohammadi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hold_rewrite_tags_cc_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace aggmux {

using input_type = gr_complex;
using output_type = gr_complex;
hold_rewrite_tags_cc::sptr hold_rewrite_tags_cc::make(bool debug, int sid)
{
    return gnuradio::make_block_sptr<hold_rewrite_tags_cc_impl>(debug, sid);
}


/*
 * The private constructor
 */
hold_rewrite_tags_cc_impl::hold_rewrite_tags_cc_impl(bool debug, int sid)
    : gr::block("hold_rewrite_tags_cc",
                gr::io_signature::make(1, 1, sizeof(input_type)),
                gr::io_signature::make(1, 1, sizeof(output_type))),
      d_counter(0),
      d_tag_arrived(false),
      d_last_freq(0),
      d_last_rate(0)
{
    set_tag_propagation_policy(TPP_DONT);
}

/*
 * Our virtual destructor.
 */
hold_rewrite_tags_cc_impl::~hold_rewrite_tags_cc_impl() {}

// void hold_rewrite_tags_cc_impl::forecast(int noutput_items,
//                                          gr_vector_int& ninput_items_required)
//{
//     ninput_items_required[0] = noutput_items;
// }

int hold_rewrite_tags_cc_impl::general_work(int noutput_items,
                                            gr_vector_int& ninput_items,
                                            gr_vector_const_void_star& input_items,
                                            gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    d_counter++;

    std::vector<gr::tag_t> tagVector;
    get_tags_in_range(tagVector, 0, nitems_read(0), nitems_read(0) + noutput_items);
    if (!d_tag_arrived) {
        uint64_t thrown_away = noutput_items;
        if (tagVector.size() > 0) {
            d_tag_arrived = true;
            thrown_away = std::min(thrown_away, tagVector[0].offset - nitems_read(0));
        }
        consume_each(thrown_away);
        return 0;
    }

    uint64_t to_consume = noutput_items;
    if (tagVector.size() > 0) {
        if (tagVector[0].offset == nitems_read(0)) {
            auto new_tag = pmt::make_dict();
            std::vector<uint64_t> offsets;
            for (const auto& tag : tagVector) {
                if (tag.offset == tagVector[0].offset) {
                    new_tag = pmt::dict_add(new_tag, tag.key, tag.value);
                } else {
                    offsets.push_back(tag.offset);
                }
            }
            bool freq_changed =
                pmt::to_double(pmt::dict_ref(
                    new_tag, pmt::intern("rx_freq"), pmt::PMT_NIL)) != d_last_freq;
            bool rate_changed =
                pmt::to_double(pmt::dict_ref(
                    new_tag, pmt::intern("rx_rate"), pmt::PMT_NIL)) != d_last_rate;
            if (freq_changed || rate_changed) {
                d_last_freq = pmt::to_double(
                    pmt::dict_ref(new_tag, pmt::intern("rx_freq"), pmt::PMT_NIL));
                d_last_rate = pmt::to_double(
                    pmt::dict_ref(new_tag, pmt::intern("rx_rate"), pmt::PMT_NIL));
                add_item_tag(0, nitems_written(0), pmt::intern("tuneInfo"), new_tag);
            }
            if (offsets.size() > 0) {
                std::sort(offsets.begin(), offsets.end());
                to_consume = std::min(offsets[0] - nitems_read(0), to_consume);
            }
        } else {
            to_consume = std::min(tagVector[0].offset - nitems_read(0), to_consume);
        }
    }

    for (uint64_t i = 0; i < to_consume; i++)
        out[i] = in[i];
    consume_each(to_consume);
    return to_consume;
}

} /* namespace aggmux */
} /* namespace gr */
