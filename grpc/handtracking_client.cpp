#include <iostream>
#include <memory>

#include <grpcpp/grpcpp.h>

#include "handtracking.grpc.pb.h"

using handtracking::HandTrackingService;
using handtracking::HandUpdate;

class HandtrackingClient {
public:
    explicit HandtrackingClient(const std::shared_ptr<grpc::Channel>& channel)
        : stub_(HandTrackingService::NewStub(channel)) {}

    void stream_hand_updates() {
        HandUpdate request;
        grpc::ClientContext context;

        std::unique_ptr<grpc::ClientReader<HandUpdate>> reader =
            stub_->StreamHandUpdates(&context, request);

        HandUpdate response;
        while (reader->Read(&response)) {
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
    static void process_hand_update(const HandUpdate& update) {
        // 处理左手数据
        if (update.has_left_hand()) {
            const auto& left_hand = update.left_hand();
            std::cout << "Left Hand Data:" << std::endl;

            // 处理手腕矩阵
            if (left_hand.has_wristmatrix()) {
                const auto& wrist_matrix = left_hand.wristmatrix();
                std::cout << "  Wrist Matrix: " << wrist_matrix.m00() << ", " << wrist_matrix.m01()
                          << ", ..." << std::endl;
            }

            // 处理骨骼数据
            if (left_hand.has_skeleton()) {
                const auto& skeleton = left_hand.skeleton();
                std::cout << "  Skeleton Joints: " << skeleton.jointmatrices_size() << " joints"
                          << std::endl;
            }
        }

        // 处理右手数据
        if (update.has_right_hand()) {
            const auto& right_hand = update.right_hand();
            std::cout << "Right Hand Data:" << std::endl;

            // 处理手腕矩阵
            if (right_hand.has_wristmatrix()) {
                const auto& wrist_matrix = right_hand.wristmatrix();
                std::cout << "  Wrist Matrix: " << wrist_matrix.m00() << ", " << wrist_matrix.m01()
                          << ", ..." << std::endl;
            }

            // 处理骨骼数据
            if (right_hand.has_skeleton()) {
                const auto& skeleton = right_hand.skeleton();
                std::cout << "  Skeleton Joints: " << skeleton.jointmatrices_size() << " joints"
                          << std::endl;
            }
        }

        // 处理头部数据
        if (update.has_head()) {
            const auto& head_matrix = update.head();
            std::cout << "Head Matrix: " << head_matrix.m00() << ", " << head_matrix.m01()
                      << ", ..." << std::endl;
        }

        std::cout << "--------------------------------" << std::endl;
    }

    std::unique_ptr<HandTrackingService::Stub> stub_;
};

int main(int argc, char** argv) {
    auto channel = grpc::CreateChannel("192.168.10.123:12345", grpc::InsecureChannelCredentials());
    HandtrackingClient client(channel);
    client.stream_hand_updates();
}
