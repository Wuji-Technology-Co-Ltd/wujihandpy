#include <memory>

class HandtrackingClient final {
public:
    explicit HandtrackingClient(const std::string& target, bool ros2_broadcast = true);

    ~HandtrackingClient();

    void wait_data_ready();

    auto joint_angles() -> std::atomic<double> (&)[5][4];

private:
    class HandtrackingClientImpl;
    std::unique_ptr<HandtrackingClientImpl> impl_;
};