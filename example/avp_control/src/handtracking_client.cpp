#include <cmath>

#include <atomic>
#include <format>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <wujihandcpp/utility/fps_counter.hpp>

#include "handtracking.grpc.pb.h"
#include "handtracking_client.hpp"

class HandtrackingClient::HandtrackingClientImpl : public rclcpp::Node {
public:
    explicit HandtrackingClientImpl(
        const std::shared_ptr<grpc::Channel>& channel, bool ros2_broadcast)
        : Node("handtracking_tf_broadcaster")
        , stub_(handtracking::HandTrackingService::NewStub(channel))
        , ros2_broadcast_(ros2_broadcast) {}

    ~HandtrackingClientImpl() { running_.store(false, std::memory_order::relaxed); }

    std::atomic<double> joint_angles[5][4];

    void wait_data_ready() { data_ready_.wait(false, std::memory_order::acquire); }

private:
    void stream_hand_updates() {
        handtracking::HandUpdate request;
        grpc::ClientContext context;

        auto reader = stub_->StreamHandUpdates(&context, request);

        handtracking::HandUpdate response;
        while (reader->Read(&response)) {
            process_hand_update(response);
            if (ros2_broadcast_)
                broadcast_hand_update(response);
            if (!running_.load(std::memory_order::relaxed)) {
                std::cout << "Stream exited correctly." << std::endl;
                return;
            }
        }

        grpc::Status status = reader->Finish();
        if (!status.ok()) {
            throw std::runtime_error{
                std::format("StreamHandUpdates rpc failed: {}", status.error_message())};
        } else {
            std::cout << "Stream completed successfully." << std::endl;
        }
    }

    void process_hand_update(const handtracking::HandUpdate& update) {
        if (!update.has_right_hand())
            return;
        const auto& right_hand = update.right_hand();

        if (!right_hand.has_wristmatrix())
            return;
        if (!right_hand.has_skeleton())
            return;
        const auto& jointmatrices = right_hand.skeleton().jointmatrices();
        solve_hand_angles(jointmatrices.cbegin());

        ready_data();

        if (fps_counter_.count())
            std::cout << "Handtracking Fps: " << fps_counter_.fps() << '\n';
    }

    using MatrixIterator =
        google::protobuf::RepeatedPtrField<handtracking::Matrix4x4>::const_iterator;

    void solve_hand_angles(MatrixIterator matrix_iterator) {
        solve_finger_angles(matrix_iterator + 5, joint_angles[1]);
        solve_finger_angles(matrix_iterator + 10, joint_angles[2]);
        solve_finger_angles(matrix_iterator + 15, joint_angles[3]);
        solve_finger_angles(matrix_iterator + 20, joint_angles[4]);
    }

    void ready_data() {
        if (data_ready_.load(std::memory_order::relaxed))
            return;

        data_ready_.store(true, std::memory_order::release);
        data_ready_.notify_all();
    }

    static void
        solve_finger_angles(MatrixIterator matrix_iterator, std::atomic<double> (&destination)[4]) {
        double angles[4];

        for (int i = 0; i < 4; i++)
            angles[i] = solve_joint_angle(matrix_iterator[i]);

        for (int i = 4; i-- > 1;) {
            angles[i] -= angles[i - 1];
            if (angles[i] > std::numbers::pi)
                angles[i] -= 2 * std::numbers::pi;
            else if (angles[i] < -std::numbers::pi)
                angles[i] += 2 * std::numbers::pi;
        }

        angles[0] = angles[1];
        angles[1] = 0;

        for (int i = 0; i < 4; i++)
            destination[i].store(angles[i], std::memory_order::relaxed);
    }

    static double solve_joint_angle(const handtracking::Matrix4x4& matrix) {
        return std::atan2(matrix.m10(), matrix.m00());
    }

    void broadcast_hand_update(const handtracking::HandUpdate& update) {
        if (!update.has_right_hand())
            return;
        const auto& right_hand = update.right_hand();

        if (right_hand.has_wristmatrix()) {
            publish_transform(
                "world", "right_wrist", to_ros_transform_matrix(right_hand.wristmatrix()));
        }

        if (right_hand.has_skeleton()) {
            const auto& jointmatrices = right_hand.skeleton().jointmatrices();
            publish_transform(
                "right_wrist", "right_middle_1", to_transform_matrix(jointmatrices[10]));
            publish_transform(
                "right_wrist", "right_middle_2", to_transform_matrix(jointmatrices[11]));
            publish_transform(
                "right_wrist", "right_middle_3", to_transform_matrix(jointmatrices[12]));
            publish_transform(
                "right_wrist", "right_middle_4", to_transform_matrix(jointmatrices[13]));
            publish_transform(
                "right_wrist", "right_middle_5", to_transform_matrix(jointmatrices[14]));
        }
    }

    static Eigen::Matrix4d to_transform_matrix(const handtracking::Matrix4x4& matrix) {
        Eigen::Matrix4d result;
        result << matrix.m00(), matrix.m01(), matrix.m02(), matrix.m03(),     //
            matrix.m10(), matrix.m11(), matrix.m12(), matrix.m13(),           //
            matrix.m20(), matrix.m21(), matrix.m22(), matrix.m23(),           //
            0, 0, 0, 1;
        return result;
    }

    static Eigen::Matrix4d to_ros_transform_matrix(const handtracking::Matrix4x4& matrix) {
        Eigen::Matrix4d result;
        result << -matrix.m20(), -matrix.m21(), -matrix.m22(), -matrix.m23(), //
            -matrix.m00(), -matrix.m01(), -matrix.m02(), -matrix.m03(),       //
            matrix.m10(), matrix.m11(), matrix.m12(), matrix.m13(),           //
            0, 0, 0, 1;
        return result;
    }

    void publish_transform(
        const std::string& header_id, const std::string& child_id,
        const Eigen::Matrix4d& transform_matrix) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = header_id;
        transform.child_frame_id = child_id;

        transform.transform.translation.x = transform_matrix(0, 3);
        transform.transform.translation.y = transform_matrix(1, 3);
        transform.transform.translation.z = transform_matrix(2, 3);

        Eigen::Quaterniond q(transform_matrix.block<3, 3>(0, 0));
        q.normalize();

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }
    std::unique_ptr<handtracking::HandTrackingService::Stub> stub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{
        std::make_unique<tf2_ros::TransformBroadcaster>(this)};
    bool ros2_broadcast_;

    wujihandcpp::utility::FpsCounter fps_counter_;
    std::atomic<bool> running_ = true;
    std::atomic<bool> data_ready_ = false;
    std::jthread streaming_thread_{&HandtrackingClientImpl::stream_hand_updates, this};
};

HandtrackingClient::HandtrackingClient(const std::string& target, bool ros2_broadcast) {
    auto channel = grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
    impl_ = std::make_unique<HandtrackingClient::HandtrackingClientImpl>(channel, ros2_broadcast);
};

HandtrackingClient::~HandtrackingClient() = default;

void HandtrackingClient::wait_data_ready() { impl_->wait_data_ready(); }

auto HandtrackingClient::joint_angles() -> std::atomic<double> (&)[5][4] {
    return impl_->joint_angles;
}