#include <cmath>

#include <iostream>
#include <memory>
#include <numbers>
#include <string>

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
        HandtrackingClient& outer, const std::shared_ptr<grpc::Channel>& channel,
        bool ros2_broadcast)
        : Node("handtracking_tf_broadcaster")
        , outer_(outer)
        , stub_(handtracking::HandTrackingService::NewStub(channel))
        , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(this))
        , ros2_broadcast_(ros2_broadcast) {}

    void stream_hand_updates() {
        handtracking::HandUpdate request;
        grpc::ClientContext context;

        auto reader = stub_->StreamHandUpdates(&context, request);

        handtracking::HandUpdate response;
        while (reader->Read(&response)) {
            process_hand_update(response);
            if (ros2_broadcast_)
                broadcast_hand_update(response);
        }

        grpc::Status status = reader->Finish();
        if (!status.ok()) {
            std::cerr << "StreamHandUpdates rpc failed: " << status.error_message() << std::endl;
        } else {
            std::cout << "Stream completed successfully." << std::endl;
        }
    }

private:
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
        outer_.joint_angles_update_callback(joint_angles_);

        if (fps_counter_.count())
            std::cout << "Handtracking Fps: " << fps_counter_.fps() << '\n';
    }

    using MatrixIterator =
        google::protobuf::RepeatedPtrField<handtracking::Matrix4x4>::const_iterator;

    void solve_hand_angles(MatrixIterator matrix_iterator) {
        solve_finger_angles(matrix_iterator + 5, joint_angles_[1]);
        solve_finger_angles(matrix_iterator + 10, joint_angles_[2]);
        solve_finger_angles(matrix_iterator + 15, joint_angles_[3]);
        solve_finger_angles(matrix_iterator + 20, joint_angles_[4]);
    }

    static void solve_finger_angles(MatrixIterator matrix_iterator, double (&destination)[4]) {
        for (int i = 0; i < 4; i++)
            destination[i] = solve_joint_angle(matrix_iterator[i]);

        for (int i = 4; i-- > 1;) {
            destination[i] -= destination[i - 1];
            if (destination[i] > std::numbers::pi)
                destination[i] -= 2 * std::numbers::pi;
            else if (destination[i] < -std::numbers::pi)
                destination[i] += 2 * std::numbers::pi;
        }

        destination[0] = destination[1];
        destination[1] = 0;
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

    HandtrackingClient& outer_;

    std::unique_ptr<handtracking::HandTrackingService::Stub> stub_;

    double joint_angles_[5][4];

    utility::FpsCounter fps_counter_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool ros2_broadcast_;
};

HandtrackingClient::HandtrackingClient(const std::string& target, bool ros2_broadcast) {
    auto channel = grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
    impl_ = std::make_unique<HandtrackingClient::HandtrackingClientImpl>(
        *this, channel, ros2_broadcast);
};

HandtrackingClient::~HandtrackingClient() = default;

void HandtrackingClient::stream_hand_updates() { impl_->stream_hand_updates(); }