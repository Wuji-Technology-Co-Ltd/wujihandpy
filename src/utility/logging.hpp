#pragma once

#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <atomic>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#ifdef _WIN32
# include <process.h>
#else
# include <pwd.h>
# include <unistd.h>
#endif

#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "utility/singleton.hpp"

namespace wujihandcpp::logging {

using Level = spdlog::level::level_enum;

class Config {
public:
    static Config& get_instance() {
        static Config instance;
        return instance;
    }

    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(Config&&) = delete;

    bool log_to_console() const noexcept {
        return log_to_console_.load(std::memory_order::relaxed);
    }
    bool log_to_file() const noexcept { return log_to_file_.load(std::memory_order::relaxed); }
    Level log_level() const noexcept { return log_level_.load(std::memory_order::relaxed); }
    std::filesystem::path log_path() const {
        std::lock_guard guard{log_path_mutex_};
        return log_path_;
    }

    void set_log_to_console(bool value) noexcept;
    void set_log_to_file(bool value) noexcept;
    void set_log_level(Level value) noexcept;
    void set_log_path(std::filesystem::path value);

private:
    Config() {
        if (!parse_bool(log_to_console_, std::getenv("WUJI_LOG_TO_CONSOLE")))
            log_to_console_.store(true, std::memory_order::relaxed);

        if (!parse_bool(log_to_file_, std::getenv("WUJI_LOG_TO_FILE")))
            log_to_file_.store(true, std::memory_order::relaxed);

        if (!parse_level(log_level_, std::getenv("WUJI_LOG_LEVEL")))
            log_level_.store(spdlog::level::info, std::memory_order::relaxed);

        if (const char* value = std::getenv("WUJI_LOG_PATH"))
            log_path_ = std::filesystem::path{value};
        else if (!default_log_path())
            log_to_file_.store(false, std::memory_order::relaxed);
    }

    static bool parse_bool(std::atomic<bool>& destination, const char* value) {
        if (!value || !*value)
            return false;

        auto str = to_lower_string(value);
        if (str == "1" || str == "true" || str == "on" || str == "yes")
            destination.store(true, std::memory_order::relaxed);
        else if (str == "0" || str == "false" || str == "off" || str == "no")
            destination.store(false, std::memory_order::relaxed);
        else
            return false;

        return true;
    }

    bool default_log_path() {
        std::filesystem::path home;

#ifdef _WIN32
        const char* user_profile = getenv("USERPROFILE");
        if (user_profile) {
            home = user_profile;
        } else {
            const char* home_drive = getenv("HOMEDRIVE");
            const char* home_path_env = getenv("HOMEPATH");
            if (home_drive && home_path_env)
                home = std::filesystem::path(home_drive) / home_path_env;
        }
#else
        const char* home_env = getenv("HOME");
        if (home_env) {
            home = home_env;
        } else {
            struct passwd* pwd = getpwuid(getuid());
            if (pwd)
                home = pwd->pw_dir;
        }
#endif

        if (!home.empty()) {
            std::error_code err;
            auto path =
                std::filesystem::weakly_canonical(std::filesystem::path(home) / ".wuji/log/", err);
            if (!err) {
                log_path_ = std::move(path);
                return true;
            }
        }

        std::fprintf(stderr, "[wuji] Unable to resolve home path, log file will not be created.");
        return false;
    }

    static bool parse_level(std::atomic<Level>& destination, const char* value) {
        if (!value || !*value)
            return false;

        auto str = to_lower_string(value);
        if (str == "trace")
            destination.store(spdlog::level::trace, std::memory_order::relaxed);
        else if (str == "debug")
            destination.store(spdlog::level::debug, std::memory_order::relaxed);
        else if (str == "info" || str == "information")
            destination.store(spdlog::level::info, std::memory_order::relaxed);
        else if (str == "warn" || str == "warning")
            destination.store(spdlog::level::warn, std::memory_order::relaxed);
        else if (str == "err" || str == "error")
            destination.store(spdlog::level::err, std::memory_order::relaxed);
        else if (str == "critical" || str == "crit")
            destination.store(spdlog::level::critical, std::memory_order::relaxed);
        else if (str == "off")
            destination.store(spdlog::level::off, std::memory_order::relaxed);
        else
            return false;

        return true;
    }

    static std::string to_lower_string(const char* value) {
        std::string result{value};
        for (auto& c : result)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return result;
    }

    std::atomic<bool> log_to_console_, log_to_file_;
    std::atomic<Level> log_level_;
    static_assert(decltype(log_level_)::is_always_lock_free);

    std::filesystem::path log_path_;
    mutable std::mutex log_path_mutex_;
};

class Logger {
public:
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

public: // Singleton
    friend class Singleton<Logger>;

    static std::unique_lock<std::mutex> acquire_instance_mutex() noexcept {
        return Singleton<Logger>::acquire_instance_mutex();
    }

    static bool has_instance() noexcept { return Singleton<Logger>::has_instance(); }

    static Logger& get_instance() noexcept { return Singleton<Logger>::get_instance(); }

public: // Config
    void set_log_to_console(bool value) noexcept {
        if (console_sink_) [[likely]]
            console_sink_->set_level(value ? spdlog::level::trace : spdlog::level::off);
    }

    void set_log_to_file(bool value) noexcept {
        if (file_sink_)
            file_sink_->set_level(value ? spdlog::level::trace : spdlog::level::off);
    }

    void set_log_level(Level value) noexcept {
        if (logger_) [[likely]]
            logger_->set_level(value);
    }

    Level level() const {
        if (logger_) [[likely]]
            return logger_->level();
        else
            return spdlog::level::off;
    }

    void set_level(Level lvl) {
        if (logger_) [[likely]]
            logger_->set_level(lvl);
    }

public: // Logging
    bool should_log(Level lvl) const {
        if (logger_) [[likely]]
            return logger_->should_log(lvl);
        else
            return false;
    }

    void flush() {
        if (logger_) [[likely]]
            logger_->flush();
    }

public: // Logging.Formatted
    template <typename... Args>
    void trace(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        log(spdlog::level::trace, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        log(spdlog::level::debug, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        log(spdlog::level::info, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        log(spdlog::level::warn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        log(spdlog::level::err, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        log(spdlog::level::critical, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log(Level lvl, spdlog::format_string_t<Args...> fmt, Args&&... args) {
        if (logger_) [[likely]]
            logger_->log(lvl, fmt, std::forward<Args>(args)...);
    }

public: // Logging.Raw
    template <typename T>
    void trace(const T& msg) {
        log(spdlog::level::trace, msg);
    }

    template <typename T>
    void debug(const T& msg) {
        log(spdlog::level::debug, msg);
    }

    template <typename T>
    void info(const T& msg) {
        log(spdlog::level::info, msg);
    }

    template <typename T>
    void warn(const T& msg) {
        log(spdlog::level::warn, msg);
    }

    template <typename T>
    void error(const T& msg) {
        log(spdlog::level::err, msg);
    }

    template <typename T>
    void critical(const T& msg) {
        log(spdlog::level::critical, msg);
    }

    template <typename T>
    void log(Level lvl, const T& msg) {
        if (logger_) [[likely]]
            logger_->log(lvl, msg);
    }

private:
    Logger() noexcept {
        try {
            // Only one thread can be used here, otherwise the _mt sink must be used
            thread_pool_ = std::make_shared<spdlog::details::thread_pool>(8192, 1);

            const auto& config = Config::get_instance();
            std::vector<std::shared_ptr<spdlog::sinks::sink>> sinks;

            console_sink_ = std::make_shared<LevelBasedConsoleSink>();
            console_sink_->set_level(spdlog::level::trace);
            console_sink_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [wuji] [%^%l%$] %v");
            if (!config.log_to_console())
                console_sink_->set_level(spdlog::level::off);
            sinks.emplace_back(console_sink_);

            constexpr size_t max_size = 1024 * 1024 * 10; // 10 MB
            constexpr size_t max_files = 5;               // 5 Files
            auto log_path = config.log_path();
            std::string log_path_error;

            if (config.log_to_file()) {
                if (log_path.empty())
                    log_path_error =
                        "Got an empty log path. If you don’t want to generate logs, please set "
                        "WUJI_LOG_TO_FILE=0, or call logger::set_log_to_file(false) before "
                        "instantiating any objects";
                else
                    try {
                        log_path = std::filesystem::weakly_canonical(log_path);
                        log_path /= make_timestamped_log_filename();
                        file_sink_ = std::make_shared<spdlog::sinks::rotating_file_sink_st>(
                            log_path.string(), max_size, max_files);
                        file_sink_->set_level(spdlog::level::trace);
                        file_sink_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%t] [%l] %v");
                        sinks.emplace_back(file_sink_);
                    } catch (const std::exception& ex) {
                        log_path_error = ex.what();
                    }
            }

            logger_ = std::make_shared<spdlog::async_logger>(
                "logger", sinks.begin(), sinks.end(), thread_pool_,
                spdlog::async_overflow_policy::overrun_oldest);
            logger_->set_level(config.log_level());
            logger_->flush_on(spdlog::level::warn);

            if (log_path_error.empty()) {
                if (config.log_to_file())
                    logger_->info("Log files can be found at {}", log_path.string());
            } else {
                logger_->warn("Failed to create log file:");
                logger_->warn("  {}.", log_path_error);
            }

        } catch (const std::exception& ex) {
            std::fprintf(stderr, "[wuji] Unable to init logging system: %s.\n", ex.what());
        } catch (...) {
            std::fprintf(stderr, "[wuji] Unable to init logging system: Unknown exception.\n");
        }
    }

    static std::uint64_t current_process_id() noexcept {
#if defined(_WIN32)
        return static_cast<std::uint64_t>(::_getpid());
#else
        return static_cast<std::uint64_t>(::getpid());
#endif
    }

    static std::string make_timestamped_log_filename() {
        const auto timestamp =
            std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
        return std::format("{:%Y%m%d_%H%M%S}_{}.log", timestamp, current_process_id());
    }

    class LevelBasedConsoleSink : public spdlog::sinks::sink {
    public:
        LevelBasedConsoleSink() = default;

        void log(const spdlog::details::log_msg& msg) override {
            if (msg.level >= spdlog::level::err)
                stderr_sink_.log(msg);
            else
                stdout_sink_.log(msg);
        };

        void flush() override {
            stdout_sink_.flush();
            stderr_sink_.flush();
        }

        void set_pattern(const std::string& pattern) override {
            set_formatter(std::make_unique<spdlog::pattern_formatter>(pattern));
        }

        void set_formatter(std::unique_ptr<spdlog::formatter> sink_formatter) override {
            stdout_sink_.set_formatter(sink_formatter->clone());
            stderr_sink_.set_formatter(std::move(sink_formatter));
        };

    private:
        spdlog::sinks::stdout_color_sink_st stdout_sink_;
        spdlog::sinks::stderr_color_sink_st stderr_sink_;
    };

    std::shared_ptr<spdlog::sinks::sink> console_sink_, file_sink_;

    std::shared_ptr<spdlog::details::thread_pool> thread_pool_;
    std::shared_ptr<spdlog::logger> logger_;
};

void Config::set_log_to_console(bool value) noexcept {
    auto guard = Logger::acquire_instance_mutex();
    if (Logger::has_instance())
        Logger::get_instance().set_log_to_console(value);
    log_to_console_.store(value, std::memory_order::relaxed);
}

void Config::set_log_to_file(bool value) noexcept {
    auto guard = Logger::acquire_instance_mutex();
    if (Logger::has_instance())
        Logger::get_instance().set_log_to_file(value);
    log_to_file_.store(value, std::memory_order::relaxed);
}

void Config::set_log_level(Level value) noexcept {
    auto guard = Logger::acquire_instance_mutex();
    if (Logger::has_instance())
        Logger::get_instance().set_level(value);
    log_level_.store(value, std::memory_order::relaxed);
}

void Config::set_log_path(std::filesystem::path value) {
    auto guard = Logger::acquire_instance_mutex();
    if (Logger::has_instance())
        throw std::logic_error("It is illegal to set log path after the Logger is constructed");
    std::lock_guard path_guard{log_path_mutex_};
    log_path_ = std::move(value);
}

Config& get_config() { return Config::get_instance(); }

Logger& get_logger() { return Logger::get_instance(); }

} // namespace wujihandcpp::logging
