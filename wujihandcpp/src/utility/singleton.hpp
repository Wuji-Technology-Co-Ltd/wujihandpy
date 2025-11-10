#pragma once

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstddef>

#include <atomic>
#include <memory>
#include <mutex>

template <typename T>
class Singleton {
public:
    Singleton() = delete;

    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;
    Singleton(Singleton&&) = delete;
    Singleton& operator=(Singleton&&) = delete;

    static std::unique_lock<std::mutex> acquire_instance_mutex() noexcept {
        return singleton_.acquire_instance_mutex();
    }
    static bool has_instance() noexcept { return singleton_.has_instance(); }
    static T& get_instance() noexcept { return singleton_.get_instance(); }

private:
    template <typename U>
    class SingletonUnit {
    public:
        ~SingletonUnit() {
            std::lock_guard guard{mutex_};
            if (has_instance())
                std::destroy_at(get());
        }

        std::unique_lock<std::mutex> acquire_instance_mutex() noexcept {
            return std::unique_lock<std::mutex>{mutex_};
        };

        bool has_instance() noexcept { return constructed_.load(std::memory_order::acquire); }

        U& get_instance() noexcept {
            if (has_instance()) [[likely]]
                return *get();

            std::lock_guard guard{mutex_};
            if (!has_instance()) {
                new (aligned_storage_) U{};
                constructed_.store(true, std::memory_order::release);
            }

            return *get();
        }

    private:
        U* get() { return reinterpret_cast<U*>(aligned_storage_); }

        std::mutex mutex_;
        std::atomic<bool> constructed_{false};
        alignas(U) std::byte aligned_storage_[sizeof(U)];
    };
    static inline SingletonUnit<T> singleton_;
};