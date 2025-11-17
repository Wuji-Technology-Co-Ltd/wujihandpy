#pragma once

#include "utility/tdigest.hpp"
#include <stop_token>
#include <utility>

namespace wujihandcpp::utility {

struct TickContext {
    using clock_t = std::chrono::steady_clock;

    clock_t::duration update_period;

    clock_t::time_point begin_time, next_iteration_time, now;
    uint64_t frame_index;

    mutable bool enable_statistics = false;
    mutable uint64_t skipped_frame_count;
    mutable TDigest<> jitter_statistics{100};
};

template <typename Functor>
requires requires(Functor& functor, const TickContext& context) {
    { functor(context) };
} class TickExecutor {
public:
    using clock_t = TickContext::clock_t;

    constexpr explicit TickExecutor(Functor callback)
        : callback_(std::move(callback)) {}

    void spin(double update_rate, const std::stop_token& stop_token) {
        context_.update_period = std::chrono::duration_cast<clock_t::duration>(
            std::chrono::duration<double>(1.0 / update_rate));
        context_.begin_time = clock_t::now();
        context_.next_iteration_time = context_.begin_time;

        context_.frame_index = context_.skipped_frame_count = 0;
        while (!stop_token.stop_requested()) {
            context_.now = clock_t::now();
            callback_(static_cast<const TickContext&>(context_));

            if (context_.enable_statistics) {
                double jitter_us = std::abs(
                    std::chrono::duration<double, std::micro>(
                        context_.now - context_.next_iteration_time)
                        .count());
                context_.jitter_statistics.insert(jitter_us);
            }

            context_.next_iteration_time += context_.update_period;
            context_.frame_index++;

            while (context_.now > context_.next_iteration_time) {
                context_.next_iteration_time += context_.update_period;
                context_.frame_index++;
                context_.skipped_frame_count += context_.enable_statistics;
            }

            std::this_thread::sleep_until(context_.next_iteration_time);
        }
    }

private:
    Functor callback_;
    TickContext context_;
};

} // namespace wujihandcpp::utility