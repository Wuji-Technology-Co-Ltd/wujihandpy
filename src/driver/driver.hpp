#pragma once

#include <cstdint>

#include <atomic>
#include <stdexcept>

#include <libusb.h>

#include "utility/cross_os.hpp"
#include "utility/final_action.hpp"
#include "utility/logging.hpp"

namespace driver {

template <typename TransferPrefill>
concept is_legal_transfer_prefill =
    std::is_same_v<TransferPrefill, void> || alignof(TransferPrefill) == 1;

template <typename Client>
class Driver {
public:
    template <is_legal_transfer_prefill TransferPrefill = void>
    class AsyncTransmitBuffer;

    explicit Driver(uint16_t usb_vid, int32_t usb_pid) {
        if (!init(usb_vid, usb_pid)) {
            throw std::runtime_error{"Failed to init."};
        }
    }

    virtual ~Driver() {
        libusb_free_transfer(libusb_receive_transfer_);
        libusb_release_interface(libusb_device_handle_, target_interface_);
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
                LOG_ERROR("Failed to submit transmit transfer: %d.", ret);
        }

        return actual_length;
    }

    void handle_events() {
        auto ret = libusb_submit_transfer(libusb_receive_transfer_);
        if (ret != 0) [[unlikely]] {
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                LOG_ERROR(
                    "Failed to submit receive transfer: Device disconnected. "
                    "Terminating...");
            else
                LOG_ERROR("Failed to submit receive transfer: %d. Terminating...", ret);
            std::terminate();
            return;
        }

        handling_events_.store(true, std::memory_order::relaxed);
        receive_transfer_busy_ = true;
        while (receive_transfer_busy_) {
            libusb_handle_events(libusb_context_);
        }
    }

    void stop_handling_events() { handling_events_.store(false, std::memory_order::relaxed); }

private:
    bool init(uint16_t vendor_id, int32_t product_id) noexcept {
        int ret;

        ret = libusb_init(&libusb_context_);
        if (ret != 0) {
            LOG_ERROR("Failed to init libusb: %d", ret);
            return false;
        }
        utility::FinalAction exit_libusb{[this]() { libusb_exit(libusb_context_); }};

        auto selected_device = select_device(vendor_id, product_id);
        if (!selected_device)
            return false;

        ret = libusb_open(selected_device, &libusb_device_handle_);
        if (ret != 0) {
            LOG_ERROR(
                "Device with vendor id: 0x%x and product id: 0x%x was successfully detected but "
                "could not be opened: %d",
                vendor_id, product_id, ret);
            return false;
        }
        utility::FinalAction close_device_handle{[this]() { libusb_close(libusb_device_handle_); }};

        if constexpr (utility::is_linux()) {
            ret = libusb_set_auto_detach_kernel_driver(libusb_device_handle_, true);
            if (ret != 0) {
                LOG_ERROR("Failed to set auto detach kernel driver: %d", ret);
                return false;
            }
        }

        ret = libusb_claim_interface(libusb_device_handle_, target_interface_);
        if (ret != 0) {
            LOG_ERROR("Failed to claim interface: %d", ret);
            return false;
        }
        utility::FinalAction release_interface{
            [this]() { libusb_release_interface(libusb_device_handle_, target_interface_); }};

        libusb_receive_transfer_ = libusb_alloc_transfer(0);
        if (!libusb_receive_transfer_) {
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

        // Libusb successfully initialized.
        release_interface.disable();
        close_device_handle.disable();
        exit_libusb.disable();
        return true;
    }

    libusb_device* select_device(uint16_t vendor_id, int32_t product_id) {
        libusb_device** device_list;
        int device_count = static_cast<int>(libusb_get_device_list(libusb_context_, &device_list));
        utility::FinalAction free_device_list{[&device_list, &device_count]() {
            libusb_free_device_list(device_list, device_count);
        }};

        auto device_descriptors = new libusb_device_descriptor[device_count];
        utility::FinalAction free_device_descriptors{
            [&device_descriptors]() { delete[] device_descriptors; }};

        libusb_device* selected_device = nullptr;
        int valid_device_count = 0;

        for (int i = 0; i < device_count; i++) {
            int ret = libusb_get_device_descriptor(device_list[i], &device_descriptors[i]);
            if (ret != 0) {
                device_descriptors[i].bLength = 0;
                LOG_WARN("A device descriptor failed to get: %d", ret);
                continue;
            }

            if (device_descriptors[i].idVendor != vendor_id)
                continue;
            if (product_id >= 0 && device_descriptors[i].idProduct != product_id)
                continue;

            selected_device = device_list[i];
            valid_device_count++;
        }
        if (valid_device_count == 0) {
            if (product_id >= 0) {
                LOG_ERROR(
                    "No devices found with specified vendor id (0x%x) and product id (0x%x)",
                    vendor_id, product_id);
                for (int i = 0, j = 0; i < device_count; i++) {
                    if (device_descriptors[i].idVendor != vendor_id)
                        continue;
                    LOG_ERROR(
                        "  Unmatched device #%d: product id = 0x%x", j++,
                        device_descriptors[i].idProduct);
                }
            } else {
                LOG_ERROR("No devices found with vendor id: 0x%x", vendor_id);
            }
            return nullptr;
        } else if (valid_device_count != 1) {
            if (product_id >= 0) {
                LOG_ERROR(
                    "%d devices found with specified vendor id (0x%x) and product id (0x%x)",
                    valid_device_count, vendor_id, product_id);
                LOG_ERROR(
                    "Multiple devices found, which is unusual. Consider using a device with "
                    "a unique serial number");
            } else {
                LOG_ERROR("%d devices found with vendor id: 0x%x", valid_device_count, vendor_id);
                for (int i = 0, j = 0; i < device_count; i++) {
                    if (device_descriptors[i].idVendor != vendor_id)
                        continue;
                    LOG_ERROR(
                        "  Device #%d: product id = 0x%x", j++, device_descriptors[i].idProduct);
                }
                LOG_ERROR("To ensure correct device selection, please specify the product id");
            }
            return nullptr;
        }

        return selected_device;
    }

    void usb_receive_complete_callback(libusb_transfer* transfer) {
        if (!handling_events_.load(std::memory_order::relaxed)) [[unlikely]] {
            receive_transfer_busy_ = false;
            return;
        }

        static_cast<Client*>(this)->receive_transfer_completed_callback(transfer);

        int ret = libusb_submit_transfer(transfer);
        if (ret != 0) [[unlikely]] {
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                LOG_ERROR(
                    "Failed to re-submit receive transfer: Device disconnected. "
                    "Terminating...");
            else
                LOG_ERROR("Failed to re-submit receive transfer: %d. Terminating...", ret);
            std::terminate();
        }
    }

    static constexpr int target_interface_ = 0x01;

    static constexpr unsigned char out_endpoint_ = 0x01;
    static constexpr unsigned char in_endpoint_ = 0x81;

    static constexpr int max_transmit_length_ = 1024;
    static constexpr int max_receive_length_ = 1024;

    libusb_context* libusb_context_;
    libusb_device_handle* libusb_device_handle_;

    libusb_transfer* libusb_receive_transfer_;
    std::byte receive_buffer_[max_receive_length_];

    std::atomic<bool> handling_events_ = false;
    bool receive_transfer_busy_ = false;
};

} // namespace driver
