#include <memory>

class HandtrackingClient {
public:
    explicit HandtrackingClient(const std::string& target, bool ros2_broadcast = true);

    virtual ~HandtrackingClient();

    void stream_hand_updates();

    virtual void joint_angles_update_callback(const double (&joint_angles)[5][4]) {
        (void)joint_angles;
    }

private:
    class HandtrackingClientImpl;
    std::unique_ptr<HandtrackingClientImpl> impl_;
};