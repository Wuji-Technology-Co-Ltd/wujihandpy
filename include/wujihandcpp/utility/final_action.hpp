#pragma once

#include <utility>

namespace wujihandcpp::utility {

template <typename Functor>
struct FinalAction {
    constexpr explicit FinalAction(Functor action)
        : enabled_(true)
        , action_{std::move(action)} {}

    constexpr FinalAction(const FinalAction&) = delete;
    constexpr FinalAction& operator=(const FinalAction&) = delete;
    constexpr FinalAction(FinalAction&&) = delete;
    constexpr FinalAction& operator=(FinalAction&&) = delete;

    ~FinalAction() {
        if (enabled_)
            action_();
    }

    void disable() { enabled_ = false; };

private:
    bool enabled_;
    Functor action_;
};

} // namespace wujihandcpp::utility