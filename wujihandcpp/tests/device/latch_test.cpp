#include <atomic>
#include <format>
#include <thread>
#include <vector>

#define private public
#include "wujihandcpp/device/latch.hpp"
#undef private

#include <gtest/gtest.h>

using namespace std::chrono_literals;

namespace wujihandcpp::device {

TEST(LatchTest, WaitReturnsAfterSuccessfulCompletion) {
    Latch latch;
    latch.count_up();

    std::thread worker([&]() {
        std::this_thread::sleep_for(10ms);
        latch.count_down(true);
    });

    EXPECT_NO_THROW(latch.wait());
    worker.join();
    EXPECT_TRUE(latch.try_wait());
}

TEST(LatchTest, WaitThrowsWhenAnOperationFails) {
    Latch latch;
    latch.count_up();

    std::thread worker([&]() {
        std::this_thread::sleep_for(10ms);
        latch.count_down(false);
    });

    EXPECT_THROW(latch.wait(), TimeoutError);
    worker.join();
    EXPECT_TRUE(latch.try_wait());
}

TEST(LatchTest, WaitAggregatesMultipleFailures) {
    Latch latch;
    latch.count_up();
    latch.count_up();

    std::thread worker([&]() {
        std::this_thread::sleep_for(10ms);
        latch.count_down(false);
        latch.count_down(false);
    });

    try {
        latch.wait();
        FAIL() << "TimeoutError expected";
    } catch (const TimeoutError& error) {
        EXPECT_STREQ("2 operations timed out while waiting for completion", error.what());
    }

    worker.join();
    EXPECT_TRUE(latch.try_wait());
}

TEST(LatchTest, ReuseAfterFailureSucceeds) {
    Latch latch;
    latch.count_up();

    std::thread failing([&]() {
        std::this_thread::sleep_for(5ms);
        latch.count_down(false);
    });

    EXPECT_THROW(latch.wait(), TimeoutError);
    failing.join();
    EXPECT_TRUE(latch.try_wait());

    latch.count_up();
    std::thread succeeding([&]() {
        std::this_thread::sleep_for(5ms);
        latch.count_down(true);
    });

    EXPECT_NO_THROW(latch.wait());
    succeeding.join();
    EXPECT_TRUE(latch.try_wait());
}

TEST(LatchTest, HandlesHighVolumeSuccessfulOperations) {
    Latch latch;
    constexpr int kThreads = 8;
    constexpr int kIterationsPerThread = 100000;
    std::atomic<bool> start{false};
    std::vector<std::thread> workers;
    workers.reserve(kThreads);

    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&]() {
            while (!start.load(std::memory_order_acquire))
                std::this_thread::yield();
            for (int i = 0; i < kIterationsPerThread; ++i)
                latch.count_down(true);
        });
    }

    for (int i = 0; i < kThreads * kIterationsPerThread; ++i)
        latch.count_up();

    start.store(true, std::memory_order_release);

    EXPECT_NO_THROW(latch.wait());
    for (auto& worker : workers)
        worker.join();
    EXPECT_TRUE(latch.try_wait());
}

TEST(LatchTest, HandlesHighVolumeFailures) {
    Latch latch;
    constexpr int kThreads = 8;
    constexpr int kIterationsPerThread = 100000;
    std::atomic<bool> start{false};
    std::vector<std::thread> workers;
    workers.reserve(kThreads);

    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&]() {
            while (!start.load(std::memory_order_acquire))
                std::this_thread::yield();
            for (int i = 0; i < kIterationsPerThread; ++i)
                latch.count_down(i % 2 == 0);
        });
    }

    for (int i = 0; i < kThreads * kIterationsPerThread; ++i)
        latch.count_up();

    start.store(true, std::memory_order_release);

    try {
        latch.wait();
        FAIL() << "TimeoutError expected";
    } catch (const TimeoutError& error) {
        EXPECT_STREQ(
            std::format(
                "{} operations timed out while waiting for completion",
                kThreads * kIterationsPerThread / 2)
                .c_str(),
            error.what());
    }

    for (auto& worker : workers)
        worker.join();
    EXPECT_TRUE(latch.try_wait());
}

} // namespace wujihandcpp::device
