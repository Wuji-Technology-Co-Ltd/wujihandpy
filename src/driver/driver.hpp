#pragma once

#include <cstdint>

#include <atomic>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <libusb.h>

#include "utility/cross_os.hpp"
#include "utility/final_action.hpp"
#include "utility/logging.hpp"

namespace wujihandcpp::driver {

template <typename TransferPrefill>
concept is_legal_transfer_prefill =
    std::is_same_v<TransferPrefill, void> || alignof(TransferPrefill) == 1;

template <typename Device>
class Driver {
public:
    template <typename TransferPrefill = void>
    class AsyncTransmitBuffer;

    explicit Driver(uint16_t usb_vid, int32_t usb_pid, const char* serial_number) {
        if (!init(usb_vid, usb_pid, serial_number)) {
            throw std::runtime_error{"Failed to init."};
        }
    }

    ~Driver() {
        libusb_free_transfer(libusb_receive_transfer_);
        libusb_release_interface(libusb_device_handle_, target_interface_);
        libusb_attach_kernel_driver(libusb_device_handle_, 0);
        libusb_close(libusb_device_handle_);
        libusb_exit(libusb_context_);
    }

    int usb_transmit(const std::byte* data, int length) {
        int actual_length = -1;
        auto libusb_data = reinterpret_cast<unsigned char*>(const_cast<std::byte*>(data));
        int ret = libusb_bulk_transfer(
            libusb_device_handle_, out_endpoint_, libusb_data, length, &actual_length, 500);
        if (ret != 0) [[unlikely]] {
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                LOG_ERROR("Failed to submit transmit transfer: Device disconnected.");
            else
                LOG_ERROR("Failed to submit transmit transfer: %d (%s).", ret, libusb_errname(ret));
        }

        return actual_length;
    }

    void handle_events() {
        while (receive_transfer_busy_) {
            libusb_handle_events(libusb_context_);
        }
    }

    void stop_handling_events() {
        handling_events_.store(false, std::memory_order::relaxed);
        libusb_cancel_transfer(libusb_receive_transfer_);
        // TODO: Very low probability of race condition.
    }

private:
    bool init(uint16_t vendor_id, int32_t product_id, const char* serial_number) noexcept {
        int ret;

        ret = libusb_init(&libusb_context_);
        if (ret != 0) [[unlikely]] {
            LOG_ERROR("Failed to init libusb: %d (%s)", ret, libusb_errname(ret));
            return false;
        }
        utility::FinalAction exit_libusb{[this]() { libusb_exit(libusb_context_); }};

        if (!select_device(vendor_id, product_id, serial_number))
            return false;
        utility::FinalAction close_device_handle{[this]() { libusb_close(libusb_device_handle_); }};

        if constexpr (utility::is_linux()) {
            ret = libusb_detach_kernel_driver(libusb_device_handle_, target_interface_);
            if (ret != LIBUSB_ERROR_NOT_FOUND && ret != 0) [[unlikely]] {
                LOG_ERROR("Failed to detach kernel driver: %d (%s)", ret, libusb_errname(ret));
                return false;
            }
        }

        ret = libusb_claim_interface(libusb_device_handle_, target_interface_);
        if (ret != 0) [[unlikely]] {
            LOG_ERROR("Failed to claim interface: %d (%s)", ret, libusb_errname(ret));
            return false;
        }
        utility::FinalAction release_interface{
            [this]() { libusb_release_interface(libusb_device_handle_, target_interface_); }};

        libusb_receive_transfer_ = libusb_alloc_transfer(0);
        if (!libusb_receive_transfer_) [[unlikely]] {
            LOG_ERROR("Failed to alloc receive transfer");
            return false;
        }

        libusb_fill_bulk_transfer(
            libusb_receive_transfer_, libusb_device_handle_, in_endpoint_,
            reinterpret_cast<unsigned char*>(receive_buffer_), max_receive_length_,
            [](libusb_transfer* transfer) {
                static_cast<Driver*>(transfer->user_data)->usb_receive_complete_callback(transfer);
            },
            this, 0);
        ret = libusb_submit_transfer(libusb_receive_transfer_);
        if (ret != 0) [[unlikely]] {
            LOG_ERROR("Failed to submit receive transfer: %d (%s)", ret, libusb_errname(ret));
            return false;
        }

        // Libusb successfully initialized.
        release_interface.disable();
        close_device_handle.disable();
        exit_libusb.disable();
        return true;
    }

    bool select_device(uint16_t vendor_id, int32_t product_id, const char* serial_number) {
        libusb_device** device_list = nullptr;
        const ssize_t device_count = libusb_get_device_list(libusb_context_, &device_list);
        if (device_count < 0) {
            LOG_ERROR(
                "Failed to get device list: %zd (%s)", device_count,
                libusb_errname(static_cast<int>(device_count)));
            return false;
        }

        utility::FinalAction free_device_list{
            [&device_list]() { libusb_free_device_list(device_list, 1); }};

        auto device_descriptors = new libusb_device_descriptor[device_count];
        utility::FinalAction free_device_descriptors{
            [&device_descriptors]() { delete[] device_descriptors; }};

        std::vector<libusb_device_handle*> devices_opened;

        for (ssize_t i = 0; i < device_count; i++) {
            int ret = libusb_get_device_descriptor(device_list[i], &device_descriptors[i]);
            if (ret != 0 || device_descriptors[i].bLength == 0) {
                LOG_WARN("A device descriptor failed to get: %d (%s)", ret, libusb_errname(ret));
                continue;
            }
            auto& descriptors = device_descriptors[i];

            if (descriptors.idVendor != vendor_id)
                continue;
            if (descriptors.iSerialNumber == 0)
                continue;
            if (product_id >= 0 && descriptors.idProduct != product_id)
                continue;

            libusb_device_handle* handle;
            ret = libusb_open(device_list[i], &handle);
            if (ret != 0)
                continue;
            utility::FinalAction close_device{[&handle]() { libusb_close(handle); }};

            if (serial_number) {
                unsigned char serial_buf[256];
                int n = libusb_get_string_descriptor_ascii(
                    handle, descriptors.iSerialNumber, serial_buf, sizeof(serial_buf) - 1);
                if (n < 0)
                    continue;
                serial_buf[n] = '\0';

                if (strcmp(reinterpret_cast<char*>(serial_buf), serial_number) != 0)
                    continue;
            }

            close_device.disable();
            devices_opened.push_back(handle);
        }

        if (devices_opened.size() != 1) {
            for (auto& device : devices_opened)
                libusb_close(device);

            std::fprintf(stderr, "[ERROR] ");
            if (devices_opened.size())
                std::fprintf(stderr, "%zu devices ", devices_opened.size());
            else
                std::fprintf(stderr, "No device ");
            std::fprintf(stderr, "found with specified vendor id (0x%04x)", vendor_id);

            if (product_id >= 0)
                std::fprintf(stderr, ", product id (0x%04x)", product_id);
            if (serial_number)
                std::fprintf(stderr, ", serial number (%s)", serial_number);
            std::fprintf(stderr, "\n");

            int relaxing_count = print_matched_unmatched_devices(
                device_list, device_count, device_descriptors, vendor_id, product_id,
                serial_number);

            if (devices_opened.size()) {
                if (!serial_number)
                    LOG_ERROR(
                        "To ensure correct device selection, please specify the Serial Number");
                else
                    LOG_ERROR(
                        "Multiple devices found, which is unusual. Consider using a device "
                        "with a unique Serial Number");
            } else {
                if (relaxing_count)
                    LOG_ERROR("Consider relaxing some filters");
            }

            return false;
        }

        libusb_device_handle_ = devices_opened[0];
        return true;
    }

    int print_matched_unmatched_devices(
        libusb_device** device_list, ssize_t device_count,
        libusb_device_descriptor* device_descriptors, uint16_t vendor_id, int32_t product_id,
        const char* serial_number) {

        int j = 0, k = 0;
        for (ssize_t i = 0; i < device_count; i++) {
            auto& descriptors = device_descriptors[i];
            bool matched = true;

            if (descriptors.idVendor != vendor_id)
                continue;
            if (descriptors.iSerialNumber == 0)
                continue;
            if (product_id >= 0 && descriptors.idProduct != product_id)
                matched = false;

            std::fprintf(
                stderr, "[ERROR]   Device %d (%04x:%04x): ", ++j, descriptors.idVendor,
                descriptors.idProduct);

            libusb_device_handle* handle;
            int ret = libusb_open(device_list[i], &handle);
            if (ret != 0) {
                std::fprintf(
                    stderr, "Ignored because device could not be opened: %d (%s)\n", ret,
                    libusb_errname(ret));
                continue;
            }
            utility::FinalAction close_device{[&handle]() { libusb_close(handle); }};

            unsigned char serial_buf[256];
            int n = libusb_get_string_descriptor_ascii(
                handle, descriptors.iSerialNumber, serial_buf, sizeof(serial_buf) - 1);
            if (n < 0) {
                std::fprintf(
                    stderr, "Ignored because descriptor could not be read: %d (%s) ignored\n", n,
                    libusb_errname(n));
                continue;
            }
            serial_buf[n] = '\0';
            std::fprintf(stderr, "Serial Number = %s", serial_buf);

            if (serial_number && strcmp(reinterpret_cast<char*>(serial_buf), serial_number) != 0)
                matched = false;

            if (matched)
                std::fprintf(stderr, " <-- Matched #%d\n", ++k);
            else
                std::fprintf(stderr, "\n");
        }
        return j;
    }

    void usb_receive_complete_callback(libusb_transfer* transfer) {
        if (!handling_events_.load(std::memory_order::relaxed)) [[unlikely]] {
            receive_transfer_busy_ = false;
            return;
        }

        static_cast<Device*>(this)->receive_transfer_completed_callback(transfer);

        int ret = libusb_submit_transfer(transfer);
        if (ret != 0) [[unlikely]] {
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                LOG_ERROR(
                    "Failed to re-submit receive transfer: Device disconnected. "
                    "Terminating...");
            else
                LOG_ERROR(
                    "Failed to re-submit receive transfer: %d (%s). Terminating...", ret,
                    libusb_errname(ret));
            std::terminate();
        }
    }

    static constexpr const char* libusb_errname(int number) {
        switch (number) {
        case LIBUSB_ERROR_IO: return "ERROR_IO";
        case LIBUSB_ERROR_INVALID_PARAM: return "ERROR_INVALID_PARAM";
        case LIBUSB_ERROR_ACCESS: return "ERROR_ACCESS";
        case LIBUSB_ERROR_NO_DEVICE: return "ERROR_NO_DEVICE";
        case LIBUSB_ERROR_NOT_FOUND: return "ERROR_NOT_FOUND";
        case LIBUSB_ERROR_BUSY: return "ERROR_BUSY";
        case LIBUSB_ERROR_TIMEOUT: return "ERROR_TIMEOUT";
        case LIBUSB_ERROR_OVERFLOW: return "ERROR_OVERFLOW";
        case LIBUSB_ERROR_PIPE: return "ERROR_PIPE";
        case LIBUSB_ERROR_INTERRUPTED: return "ERROR_INTERRUPTED";
        case LIBUSB_ERROR_NO_MEM: return "ERROR_NO_MEM";
        case LIBUSB_ERROR_NOT_SUPPORTED: return "ERROR_NOT_SUPPORTED";
        case LIBUSB_ERROR_OTHER: return "ERROR_OTHER";
        default: return "UNKNOWN";
        }
    }

    static constexpr int target_interface_ = 0x01;

    static constexpr unsigned char out_endpoint_ = 0x01;
    static constexpr unsigned char in_endpoint_ = 0x81;

    static constexpr int max_transmit_length_ = 512;
    static constexpr int max_receive_length_ = 512;

    libusb_context* libusb_context_;
    libusb_device_handle* libusb_device_handle_;

    libusb_transfer* libusb_receive_transfer_;
    std::byte receive_buffer_[max_receive_length_];

    std::atomic<bool> handling_events_ = true;
    bool receive_transfer_busy_ = true;
};

} // namespace wujihandcpp::driver
