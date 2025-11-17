#pragma once

#include <chrono>
#include <map>
#include <memory_resource>

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
        logger_.info("Starting latency test with {} Frames warmup...", warmup_frames);
        uint64_t next_log_frame = warmup_frames;

        auto executor = utility::TickExecutor{[&](const utility::TickContext& context) {
            std::byte* buffer = builder_.allocate(sizeof(protocol::pdo::LatencyTest));
            new (buffer) protocol::pdo::LatencyTest{.id = next_id_};

            auto now = std::chrono::steady_clock::now();
            if (!pending_requests_.emplace_back(next_id_, now))
                logger_.error(
                    "Pending requests queue is full, which should not happen. Test results may be "
                    "distorted.");

            builder_.finalize();

            if (++next_id_ == 0)
                next_id_ = 1;

            if (context.frame_index >= next_log_frame) {
                if (next_log_frame == warmup_frames) {
                    context.enable_statistics = true;
                    logger_.info("Warmup complete, collecting statistics...");
                } else
                    log_statistics(context);

                next_log_frame += log_frames;
            }
        }};
        executor.spin(update_rate, stop_token);
    }

    void log_statistics(const utility::TickContext& context) {
        uint64_t frame_count = context.frame_index - warmup_frames;

        logger_.info(
            "======== Latency Statistics ({} Frames, {:.1f}s test) ========", frame_count,
            static_cast<double>(frame_count) / update_rate);

        double update_period_us =
            std::chrono::duration<double, std::micro>(context.update_period).count();
        logger_.info(
            "RT Thread Scheduling ({:.0f}Hz, {:.0f}us period):", update_rate, update_period_us);

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

        dropped_frame_count_.store(
            context.skipped_frame_count + builder_.dropped_frame_count(),
            std::memory_order::relaxed);
        frame_index_.store(context.frame_index, std::memory_order::release);
    }

    void read_result(const pdo::LatencyTestResult& package) {
        auto now = std::chrono::steady_clock::now();

        pending_requests_.pop_front_n([this](Request&& request) {
            auto [it, success] = result_map_.try_emplace(request.id, request.transmit_timestamp);
            if (!success) [[unlikely]]
                throw std::runtime_error("Id overlap in latency test");
        });

        uint32_t id = 0;
        decltype(result_map_)::iterator it = result_map_.end();

        for (int i = 0; i < 20; i++) {
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

            if (it == result_map_.end())
                std::terminate();

            auto& result = it->second;
            result.round_trip_times[i] = now - result.transmit_timestamp;

            if (++result.received_count == 20) {
                id = 0;
                record_all_values(result);
                result_map_.erase(it);
                it = result_map_.end();
                success_count_++;
            }
        }

        using namespace std::chrono_literals;
        it = result_map_.begin();
        while (it != result_map_.end() && now - it->second.transmit_timestamp > 500ms) {
            it = result_map_.erase(it);
            timeout_count_++;
        }

        auto frame_index = frame_index_.load(std::memory_order::acquire);
        if (frame_index >= next_log_frame_) {
            next_log_frame_ += log_frames;
            log_statistics2(frame_index, dropped_frame_count_.load(std::memory_order::relaxed));
        }
    }

    void log_statistics2(uint64_t frame_index, uint64_t dropped_frame_count) {
        logger_.info("Device Communication:");
        logger_.info(
            "  Round Trip Time: Min {:.3f}ms, Med {:.3f}ms, P90 {:.3f}ms, P99 {:.3f}ms, Max "
            "{:.3f}ms",
            tdigest_.min(), tdigest_.quantile(50), tdigest_.quantile(90), tdigest_.quantile(99),
            tdigest_.max());

        uint64_t frame_send_count = frame_index - warmup_frames - dropped_frame_count;
        logger_.info(
            "  Packet Loss: {}/{} ({:.3f}%), In-flight: {}", timeout_count_, frame_send_count,
            static_cast<double>(timeout_count_) / static_cast<double>(frame_send_count) * 100.0,
            result_map_.size());
    }

private:
    struct Request {
        uint32_t id;
        std::chrono::steady_clock::time_point transmit_timestamp;
    };

    struct Result {
        explicit Result(const std::chrono::steady_clock::time_point& transmit_timestamp) noexcept
            : transmit_timestamp(transmit_timestamp) {}

        std::chrono::steady_clock::time_point transmit_timestamp;
        std::chrono::steady_clock::duration round_trip_times[20];
        int received_count = 0;
    };

    void record_all_values(const Result& result) {
        for (const auto& round_trip_time : result.round_trip_times) {
            double rtt_ms = std::chrono::duration<double, std::milli>(round_trip_time).count();
            tdigest_.insert(rtt_ms);
        }
    }

    static constexpr double update_rate = 500.0;

    static constexpr uint64_t warmup_frames = 1000, log_frames = 2500;

    logging::Logger& logger_;

    FrameBuilder& builder_;

    uint32_t next_id_ = 1; // 1 ~ uint32_max

    utility::RingBuffer<Request> pending_requests_{64};

    std::atomic<uint64_t> frame_index_, dropped_frame_count_;
    uint64_t next_log_frame_ = warmup_frames + log_frames;

    std::pmr::unsynchronized_pool_resource pool_;
    std::pmr::map<uint32_t, Result> result_map_{&pool_};

    utility::TDigest<double> tdigest_{1000};

    uint64_t success_count_ = 0, timeout_count_ = 0;
};

} // namespace wujihandcpp::protocol