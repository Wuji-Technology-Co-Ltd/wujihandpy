#pragma once

#include <cmath>

#include <chrono>
#include <concepts>
#include <stop_token>
#include <thread>
#include <type_traits>
#include <utility>

#include "utility/tdigest.hpp"

namespace wujihandcpp::utility {

struct TickContext {
    TickContext() = default;

    TickContext(const TickContext&) = delete;
    TickContext& operator=(const TickContext&) = delete;
    TickContext(TickContext&&) = delete;
    TickContext& operator=(TickContext&&) = delete;

    using clock_t = std::chrono::steady_clock;

    clock_t::duration update_period;

    clock_t::time_point begin_time, scheduled_update_time, now;
    uint64_t frame_index;

    mutable bool enable_statistics = false;
    mutable uint64_t skipped_frame_count;
    mutable TDigest<> jitter_statistics{100};
};

template <typename Functor>
requires requires(Functor& functor, const TickContext& context) {
    { functor(context) };
}
      && (std::same_as<std::invoke_result_t<Functor, const TickContext&>, void>
          || std::convertible_to<std::invoke_result_t<Functor, const TickContext&>, bool>)
class TickExecutor {
public:
    using clock_t = TickContext::clock_t;
    using callback_return_t = std::invoke_result_t<Functor, const TickContext&>;

    constexpr explicit TickExecutor(Functor callback)
        : callback_(std::move(callback)) {}

    void spin(double update_rate, const std::stop_token& stop_token) {
        context_.update_period = std::chrono::duration_cast<clock_t::duration>(
            std::chrono::duration<double>(1.0 / update_rate));
        context_.begin_time = clock_t::now();
        context_.scheduled_update_time = context_.begin_time;

        context_.frame_index = context_.skipped_frame_count = 0;
        while (!stop_token.stop_requested()) {
            context_.now = clock_t::now();

            bool should_continue;
            if constexpr (std::same_as<callback_return_t, void>) {
                callback_(static_cast<const TickContext&>(context_));
                should_continue = true;
            } else {
                should_continue =
                    static_cast<bool>(callback_(static_cast<const TickContext&>(context_)));
            }

            if (context_.enable_statistics) {
                double jitter_us = std::abs(
                    std::chrono::duration<double, std::micro>(
                        context_.now - context_.scheduled_update_time)
                        .count());
                context_.jitter_statistics.insert(jitter_us);
            }

            context_.scheduled_update_time += context_.update_period;
            context_.frame_index++;

            while (context_.now > context_.scheduled_update_time) {
                context_.scheduled_update_time += context_.update_period;
                context_.frame_index++;
                context_.skipped_frame_count += context_.enable_statistics;
            }

            std::this_thread::sleep_until(context_.scheduled_update_time);

            if (!should_continue)
                break;
        }
    }

private:
    Functor callback_;
    TickContext context_;
};

} // namespace wujihandcpp::utility
