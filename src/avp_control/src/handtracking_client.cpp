#include <exception>
#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <wujihandcpp/utility/fps_counter.hpp>

#include "handtracking.grpc.pb.h"

class HandtrackingClient : public rclcpp::Node {
public:
    explicit HandtrackingClient(const std::shared_ptr<grpc::Channel>& channel)
        : Node("handtracking_tf_broadcaster")
        , stub_(handtracking::HandTrackingService::NewStub(channel))
        , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(this)) {}

    void stream_hand_updates() {
        handtracking::HandUpdate request;
        grpc::ClientContext context;

        auto reader = stub_->StreamHandUpdates(&context, request);

        utility::FpsCounter counter;
        handtracking::HandUpdate response;
        while (reader->Read(&response)) {
            if (counter.count())
                std::cout << "Fps: " << counter.fps() << '\n';
            process_hand_update(response);
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
        if (update.has_right_hand()) {
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
        std::terminate();
    });
    auto channel = grpc::CreateChannel("192.168.10.123:12345", grpc::InsecureChannelCredentials());
    HandtrackingClient client(channel);
    client.stream_hand_updates();
}
