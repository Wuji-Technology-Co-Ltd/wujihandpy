#pragma once

#include <bitset>
#include <chrono>
#include <map>
#include <memory_resource>
#include <new>

#include "logging/logging.hpp"
#include "protocol/frame_builder.hpp"
#include "protocol/protocol.hpp"
#include "utility/ring_buffer.hpp"
#include "utility/tdigest.hpp"
#include "utility/tick_executor.hpp"

namespace wujihandcpp::protocol {

class LatencyTester {
public:
    explicit LatencyTester(FrameBuilder& builder)
        : logger_(logging::get_logger())
        , builder_(builder) {}

    void spin(const std::stop_token& stop_token) {
        logger_.info("Starting latency test with {} Frames warmup...", warmup_frames_);
        uint64_t next_log_frame = warmup_frames_;

        utility::TickExecutor{[&](const utility::TickContext& context) {
            send_latency_probe();

            auto now = std::chrono::steady_clock::now();
            if (!pending_requests_.emplace_back(next_id_, now))
                logger_.error(
                    "Pending requests queue is full, which should not happen. Test results may be "
                    "distorted.");

            builder_.finalize();
            advance_id();

            if (context.frame_index >= next_log_frame) {
                if (next_log_frame == warmup_frames_) {
                    context.enable_statistics = true;
                    logger_.info("Warmup complete, collecting statistics...");
                } else {
                    log_thread_statistics(context);
                }
                next_log_frame += log_frames_;

                dropped_frame_count_.store(
                    context.skipped_frame_count + builder_.dropped_frame_count(),
                    std::memory_order::relaxed);
                frame_index_.store(context.frame_index, std::memory_order::release);
            }
        }}.spin(update_rate_, stop_token);
    }

    void log_thread_statistics(const utility::TickContext& context) {
        uint64_t frame_count = context.frame_index - warmup_frames_;

        logger_.info(
            "======== Latency Statistics ({} Frames, {:.1f}s test) ========", frame_count,
            static_cast<double>(frame_count) / update_rate_);

        double update_period_us =
            std::chrono::duration<double, std::micro>(context.update_period).count();
        logger_.info(
            "RT Thread Scheduling ({:.0f}Hz, {:.0f}us period):", update_rate_, update_period_us);

        context.jitter_statistics.merge();
        double min_us = context.jitter_statistics.min(),
               med_us = context.jitter_statistics.quantile(50),
               p90_us = context.jitter_statistics.quantile(90),
               p99_us = context.jitter_statistics.quantile(99),
               max_us = context.jitter_statistics.max();
        logger_.info(
            "  Timing Jitter: Min {:.0f}us ({:.0f}%), Med {:.0f}us ({:.0f}%), P90 "
            "{:.0f}us ({:.0f}%), P99 {:.0f}us ({:.0f}%), Max {:.0f}us ({:.0f}%)",
            min_us, min_us / update_period_us * 100.0, med_us, med_us / update_period_us * 100.0,
            p90_us, p90_us / update_period_us * 100.0, p99_us, p99_us / update_period_us * 100.0,
            max_us, max_us / update_period_us * 100.0);

        uint64_t dropped_frame_count = context.skipped_frame_count + builder_.dropped_frame_count();
        logger_.info(
            "  Frame Drop: {}/{} ({:.3f}%, {} Deadline Miss + {} Buffer Full)", dropped_frame_count,
            frame_count,
            static_cast<double>(dropped_frame_count) / static_cast<double>(frame_count) * 100.0,
            context.skipped_frame_count, builder_.dropped_frame_count());
    }

    void read_result(const pdo::LatencyTestResult& package) {
        auto now = std::chrono::steady_clock::now();

        pending_requests_.pop_front_n([this](Request&& request) {
            auto [it, success] = result_map_.try_emplace(request.id, request.transmit_timestamp);
            if (!success) [[unlikely]]
                throw std::runtime_error("Id overlap in latency test");
        });

        auto frame_index = frame_index_.load(std::memory_order::acquire);

        uint32_t id = 0;
        decltype(result_map_)::iterator it = result_map_.end();

        for (int i = 0; i < joint_count_; i++) {
            const auto& data = package.joint_datas[i];

            if (data.id == 0)
                continue;
            if (id != data.id) {
                auto found = result_map_.find(data.id);
                if (found == result_map_.end())
                    continue;
                it = found;
                id = data.id;
            }

            auto& result = it->second;
            if (result.received.test(i))
                continue;

            result.received.set(i);
            result.round_trip_times[i] = now - result.transmit_timestamp;

            if (++result.received_count == joint_count_) {
                id = 0;
                if (frame_index >= warmup_frames_)
                    record_all_values(result);
                result_map_.erase(it);
                it = result_map_.end();
            }
        }

        using namespace std::chrono_literals;
        it = result_map_.begin();
        while (it != result_map_.end() && now - it->second.transmit_timestamp > request_timeout_) {
            if (frame_index >= warmup_frames_)
                record_received_values(it->second);
            it = result_map_.erase(it);
        }

        if (frame_index >= next_log_frame_) {
            next_log_frame_ += log_frames_;
            log_device_statistics(
                frame_index, dropped_frame_count_.load(std::memory_order::relaxed));
        }
    }

    void log_device_statistics(uint64_t frame_index, uint64_t dropped_frame_count) {
        rtt_statistics_.merge();

        logger_.info("Device Communication:");
        logger_.info(
            "  Round Trip Time: Min {:.3f}ms, Med {:.3f}ms, P90 {:.3f}ms, P99 {:.3f}ms, Max "
            "{:.3f}ms",
            rtt_statistics_.min(), rtt_statistics_.quantile(50), rtt_statistics_.quantile(90),
            rtt_statistics_.quantile(99), rtt_statistics_.max());

        uint64_t frame_send_count = frame_index - warmup_frames_ - dropped_frame_count;
        uint64_t frame_loss_count = (timeout_count_ + joint_count_ - 1) / joint_count_;
        logger_.info(
            "  Packet Loss: {}/{} ({:.3f}%), In-flight: {}", frame_loss_count, frame_send_count,
            static_cast<double>(frame_loss_count) / static_cast<double>(frame_send_count) * 100.0,
            result_map_.size());
    }

private:
    static constexpr int joint_count_ = 20;

    static constexpr auto request_timeout_ = std::chrono::milliseconds{500};
    static constexpr double update_rate_ = 500.0;

    static constexpr uint64_t warmup_frames_ = 2000, log_frames_ = 2500;

    struct Request {
        uint32_t id;
        std::chrono::steady_clock::time_point transmit_timestamp;
    };

    struct Result {
        explicit Result(const std::chrono::steady_clock::time_point& transmit_timestamp) noexcept
            : transmit_timestamp(transmit_timestamp) {}

        std::chrono::steady_clock::time_point transmit_timestamp;

        std::chrono::steady_clock::duration round_trip_times[joint_count_];

        std::bitset<joint_count_> received = 0;
        int received_count = 0;
    };

    void send_latency_probe() {
        std::byte* buffer = builder_.allocate(sizeof(protocol::pdo::LatencyTest));
        new (buffer) protocol::pdo::LatencyTest{.id = next_id_};
    }

    void advance_id() {
        if (++next_id_ == 0)
            next_id_ = 1;
    }

    void record_all_values(const Result& result) {
        for (const auto& round_trip_time : result.round_trip_times)
            record_value(round_trip_time);
    }

    void record_received_values(const Result& result) {
        for (int i = 0; i < joint_count_; i++) {
            if (result.received.test(i))
                record_value(result.round_trip_times[i]);
            else
                timeout_count_++;
        }
    }

    void record_value(const std::chrono::steady_clock::duration& round_trip_time) {
        double rtt_ms = std::chrono::duration<double, std::milli>(round_trip_time).count();
        rtt_statistics_.insert(rtt_ms);
    }

    logging::Logger& logger_;

    FrameBuilder& builder_;

    uint32_t next_id_ = 1; // 1 ~ uint32_max
    utility::RingBuffer<Request> pending_requests_{64};

    std::atomic<uint64_t> frame_index_{0}, dropped_frame_count_{0};
    uint64_t next_log_frame_ = warmup_frames_ + log_frames_;

    std::pmr::unsynchronized_pool_resource pool_;
    std::pmr::map<uint32_t, Result> result_map_{&pool_};

    utility::TDigest<double> rtt_statistics_{1000};
    uint64_t timeout_count_ = 0;
};

} // namespace wujihandcpp::protocol
