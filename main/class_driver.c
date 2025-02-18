/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"

#define CLIENT_NUM_EVENT_MSG 5

typedef enum
{
    ACTION_OPEN_DEV = (1 << 0),
    // ACTION_GET_DEV_INFO = (1 << 1),
    ACTION_GET_DEV_DESC = (1 << 1),
    ACTION_CLAIM_INTERFACE = (1 << 2),
    ACTION_PARSE_CONFIG_DESC = (1 << 3),
    ACTION_TRANSFER = (1 << 4),
    // ACTION_GET_CONFIG_DESC = (1 << 3),
    // ACTION_GET_STR_DESC = (1 << 4),
    ACTION_CLOSE_DEV = (1 << 5),
} action_t;

#define DEV_MAX_COUNT 128

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    action_t actions;
} usb_device_t;

typedef struct
{
    struct
    {
        union
        {
            struct
            {
                uint8_t unhandled_devices : 1; /**< Device has unhandled devices */
                uint8_t shutdown : 1;          /**<  */
                uint8_t reserved6 : 6;         /**< Reserved */
            };
            uint8_t val;                    /**< Class drivers' flags value */
        } flags;                            /**< Class drivers' flags */
        usb_device_t device[DEV_MAX_COUNT]; /**< Class drivers' static array of devices */
    } mux_protected;                        /**< Mutex protected members. Must be protected by the Class mux_lock when accessed */

    struct
    {
        usb_host_client_handle_t client_hdl;
        SemaphoreHandle_t mux_lock; /**< Mutex for protected members */
    } constant;                     /**< Constant members. Do not change after installation thus do not require a critical section or mutex */
} class_driver_t;

static const char *TAG = "CLASS";
static class_driver_t *s_driver_obj;
static usb_transfer_t *rx_xfer;
static usb_transfer_t *tx_xfer;

// Bulk IN/OUT endpoints
uint8_t bulk_in_ep = 0;
uint8_t bulk_out_ep = 0;

// Ethernet packet buffer
#define PACKET_SIZE 64
uint8_t rx_buffer[PACKET_SIZE];

void usb_rx_callback(usb_transfer_t *rx_xfer);

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        // Save the device address
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_addr = event_msg->new_dev.address;
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_hdl = NULL;
        // Open the device next
        driver_obj->mux_protected.device[event_msg->new_dev.address].actions |= ACTION_OPEN_DEV;
        // Set flag
        driver_obj->mux_protected.flags.unhandled_devices = 1;
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        // Cancel any other actions and close the device next
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
        {
            if (driver_obj->mux_protected.device[i].dev_hdl == event_msg->dev_gone.dev_hdl)
            {
                driver_obj->mux_protected.device[i].actions = ACTION_CLOSE_DEV;
                // Set flag
                driver_obj->mux_protected.flags.unhandled_devices = 1;
            }
        }
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;
    default:
        // Should never occur
        abort();
    }
}

static void action_open_dev(usb_device_t *device_obj)
{
    assert(device_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", device_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(device_obj->client_hdl, device_obj->dev_addr, &device_obj->dev_hdl));
    // Get the device's information next
    // device_obj->actions |= ACTION_GET_DEV_INFO;

    // Get the device's descriptors next
    device_obj->actions |= ACTION_GET_DEV_DESC;
}

// static void action_get_info(usb_device_t *device_obj)
// {
//     assert(device_obj->dev_hdl != NULL);
//     ESP_LOGI(TAG, "Getting device information");
//     usb_device_info_t dev_info;
//     ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));

//     ESP_LOGI(TAG, "\t%s speed", (char *[]){"Low", "Full", "High"}[dev_info.speed]);
//     ESP_LOGI(TAG, "\tParent info:");
//     if (dev_info.parent.dev_hdl)
//     {
//         usb_device_info_t parent_dev_info;
//         ESP_ERROR_CHECK(usb_host_device_info(dev_info.parent.dev_hdl, &parent_dev_info));
//         ESP_LOGI(TAG, "\t\tBus addr: %d", parent_dev_info.dev_addr);
//         ESP_LOGI(TAG, "\t\tPort: %d", dev_info.parent.port_num);
//     }
//     else
//     {
//         ESP_LOGI(TAG, "\t\tPort: ROOT");
//     }
//     ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
//     //        if (dev_info.desc.idVendor == 0x0BDA && dev_info.desc.idProduct == 0x8152)
//     // {
//     //     printf("Realtek RTL8152 detected! Initializing driver...\n");
//     //     // Implement further initialization
//     // }

//     // Get the device descriptor next
//     device_obj->actions |= ACTION_GET_DEV_DESC;
// }

static void action_get_dev_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    // ESP_LOGI(TAG, "Checking device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(device_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    // Get the device's config descriptor next
    uint16_t vendor_id = dev_desc->idVendor;
    uint16_t product_id = dev_desc->idProduct;
    if (vendor_id == 0x0BDA && product_id == 0x8152)
    {
        ESP_LOGI(TAG, "Realtek RTL8152/RTL8153 detected!");
        device_obj->actions |= ACTION_CLAIM_INTERFACE;
    }
    else
    {
        ESP_LOGW(TAG, "Unrecognized device, deregistering...");
        device_obj->actions |= ACTION_CLOSE_DEV;
        // set flag
    }
    // device_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_claim_interface(usb_device_t *device_obj)
{
    assert(device_obj->client_hdl != NULL);
    ESP_LOGI(TAG, "Claiming interface");
    if (usb_host_interface_claim(device_obj->client_hdl, device_obj->dev_hdl, 1, 1) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to claim interface!");
        return;
    }

    // // Parse Configuration Descriptor
    // parse_config_descriptor(device_obj);

    // usb_print_interface_descriptor(interface_desc);
    // Get the device's config descriptor next
    device_obj->actions |= ACTION_PARSE_CONFIG_DESC;
}

static void action_parse_config_desc(usb_device_t *device_obj)
{
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(device_obj->dev_hdl, &config_desc));

    const uint8_t *p = (const uint8_t *)config_desc;
    const uint8_t *end = p + config_desc->wTotalLength;

    while (p < end)
    {
        const usb_standard_desc_t *desc = (const usb_standard_desc_t *)p;

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE)
        {
            const usb_intf_desc_t *intf_desc = (const usb_intf_desc_t *)desc;
            ESP_LOGI(TAG, "Interface %d, Class: 0x%02X, Subclass: 0x%02X, Protocol: 0x%02X",
                     intf_desc->bInterfaceNumber, intf_desc->bInterfaceClass,
                     intf_desc->bInterfaceSubClass, intf_desc->bInterfaceProtocol);
        }

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT)
        {
            const usb_ep_desc_t *ep_desc = (const usb_ep_desc_t *)desc;
            if ((ep_desc->bmAttributes & 0x03) == 0x02)
            { // Bulk transfer type
                if (ep_desc->bEndpointAddress & 0x80)
                {
                    bulk_in_ep = ep_desc->bEndpointAddress;
                    ESP_LOGI(TAG, "Found Bulk IN endpoint: 0x%02X", bulk_in_ep);
                }
                else
                {
                    bulk_out_ep = ep_desc->bEndpointAddress;
                    ESP_LOGI(TAG, "Found Bulk OUT endpoint: 0x%02X", bulk_out_ep);
                }
            }
        }

        p += desc->bLength;
    }

    if (bulk_in_ep && bulk_out_ep)
    {
        // ESP_LOGI(TAG, "Bulk endpoints found, starting transfer");
        device_obj->actions |= ACTION_TRANSFER;
    }
    else
    {
        ESP_LOGW(TAG, "Bulk endpoints not found, deregistering...");
        device_obj->actions |= ACTION_CLOSE_DEV;
    }
}

static void action_transfer(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Starting transfer");

    if (bulk_in_ep && bulk_out_ep)
    {
        if (usb_host_transfer_alloc(PACKET_SIZE, 0, &rx_xfer) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to allocate USB transfer!");
            return;
        }
        rx_xfer->num_bytes = PACKET_SIZE;
        rx_xfer->device_handle = device_obj->dev_hdl;
        rx_xfer->bEndpointAddress = bulk_in_ep;
        rx_xfer->callback = usb_rx_callback;
        rx_xfer->context = (void *)&device_obj;
        usb_host_transfer_submit(rx_xfer);
    }
    else
    {
        ESP_LOGW(TAG, "Bulk endpoints not found, deregistering...");
        device_obj->actions |= ACTION_CLOSE_DEV;
    }
}
// static void action_get_config_desc(usb_device_t *device_obj)
// {
//     assert(device_obj->dev_hdl != NULL);
//     ESP_LOGI(TAG, "Getting config descriptor");
//     const usb_config_desc_t *config_desc;
//     ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(device_obj->dev_hdl, &config_desc));
//     // usb_print_config_descriptor(config_desc, NULL);
//     // Get the device's string descriptors next
//     device_obj->actions |= ACTION_GET_STR_DESC;
// }

// static void action_get_str_desc(usb_device_t *device_obj)
// {
//     assert(device_obj->dev_hdl != NULL);
//     usb_device_info_t dev_info;
//     ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));
//     if (dev_info.str_desc_manufacturer)
//     {
//         ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
//         usb_print_string_descriptor(dev_info.str_desc_manufacturer);
//     }
//     if (dev_info.str_desc_product)
//     {
//         ESP_LOGI(TAG, "Getting Product string descriptor");
//         usb_print_string_descriptor(dev_info.str_desc_product);
//     }
//     if (dev_info.str_desc_serial_num)
//     {
//         ESP_LOGI(TAG, "Getting Serial Number string descriptor");
//         usb_print_string_descriptor(dev_info.str_desc_serial_num);
//     }
// }

static void action_close_dev(usb_device_t *device_obj)
{
    usb_host_transfer_free(rx_xfer);
    usb_host_transfer_free(tx_xfer);
    usb_host_interface_release(device_obj->client_hdl, device_obj->dev_hdl, 1);
    ESP_ERROR_CHECK(usb_host_device_close(device_obj->client_hdl, device_obj->dev_hdl));
    device_obj->dev_hdl = NULL;
    device_obj->dev_addr = 0;
    rx_xfer = NULL;
    tx_xfer = NULL;
    ESP_LOGI(TAG, "Device closed");
}

// USB Receive Callback
void usb_rx_callback(usb_transfer_t *rx_xfer)
{
    if (rx_xfer->status == USB_TRANSFER_STATUS_COMPLETED)
    {
        ESP_LOGI(TAG, "Received %d bytes", rx_xfer->actual_num_bytes);
        // Process Ethernet packet (example: print first few bytes)
        ESP_LOG_BUFFER_HEX(TAG, rx_xfer->data_buffer, rx_xfer->actual_num_bytes < 16 ? rx_xfer->actual_num_bytes : 16);
    }
    // Re-submit transfer
    usb_host_transfer_submit(rx_xfer);
}

// // // USB Send Packet Function
// void send_ethernet_packet(uint8_t *packet, size_t len)
// {
//     if (!bulk_out_ep)
//     {
//         ESP_LOGE(TAG, "Device not ready");
//         return;
//     }

//     if (usb_host_transfer_alloc(len, 0, &tx_xfer) != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to allocate USB transfer!");
//         return;
//     }
//     memcpy(tx_xfer->data_buffer, packet, len);
//     tx_xfer->device_handle = s_driver_obj->constant.dev_hdl;
//     tx_xfer->bEndpointAddress = bulk_out_ep;
//     tx_xfer->num_bytes = len;
//     tx_xfer->callback = NULL; // No callback needed for sending
//     usb_host_transfer_submit(tx_xfer);

//     ESP_LOGI(TAG, "Sent %d bytes", len);
// }

static void class_driver_device_handle(usb_device_t *device_obj)
{
    uint8_t actions = device_obj->actions;
    device_obj->actions = 0;

    while (actions)
    {
        if (actions & ACTION_OPEN_DEV)
        {
            action_open_dev(device_obj);
        }
        // if (actions & ACTION_GET_DEV_INFO)
        // {
        //     action_get_info(device_obj);
        // }
        if (actions & ACTION_GET_DEV_DESC)
        {
            action_get_dev_desc(device_obj);
        }
        if (actions & ACTION_CLAIM_INTERFACE)
        {
            action_claim_interface(device_obj);
        }
        if (actions & ACTION_PARSE_CONFIG_DESC)
        {
            action_parse_config_desc(device_obj);
        }
        if (actions & ACTION_TRANSFER)
        {
            action_transfer(device_obj);
        }
        // if (actions & ACTION_GET_CONFIG_DESC)
        // {
        //     action_get_config_desc(device_obj);
        // }
        // if (actions & ACTION_GET_STR_DESC)
        // {
        //     action_get_str_desc(device_obj);
        // }
        if (actions & ACTION_CLOSE_DEV)
        {
            action_close_dev(device_obj);
        }

        actions = device_obj->actions;
        device_obj->actions = 0;
    }
}

void class_driver_task(void *arg)
{
    class_driver_t driver_obj = {0};
    usb_host_client_handle_t class_driver_client_hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");

    SemaphoreHandle_t mux_lock = xSemaphoreCreateMutex();
    if (mux_lock == NULL)
    {
        ESP_LOGE(TAG, "Unable to create class driver mutex");
        vTaskSuspend(NULL);
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false, // Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &class_driver_client_hdl));

    driver_obj.constant.mux_lock = mux_lock;
    driver_obj.constant.client_hdl = class_driver_client_hdl;

    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
    {
        driver_obj.mux_protected.device[i].client_hdl = class_driver_client_hdl;
    }

    s_driver_obj = &driver_obj;

    while (1)
    {
        // Driver has unhandled devices, handle all devices first
        if (driver_obj.mux_protected.flags.unhandled_devices)
        {
            xSemaphoreTake(driver_obj.constant.mux_lock, portMAX_DELAY);
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
            {
                if (driver_obj.mux_protected.device[i].actions)
                {
                    class_driver_device_handle(&driver_obj.mux_protected.device[i]);
                }
            }
            driver_obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(driver_obj.constant.mux_lock);
        }
        else
        {
            // Driver is active, handle client events
            if (driver_obj.mux_protected.flags.shutdown == 0)
            {
                usb_host_client_handle_events(class_driver_client_hdl, portMAX_DELAY);
            }
            else
            {
                // Shutdown the driver
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(class_driver_client_hdl));
    if (mux_lock != NULL)
    {
        vSemaphoreDelete(mux_lock);
    }
    vTaskSuspend(NULL);
}

void class_driver_client_deregister(void)
{
    // Mark all opened devices
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
    {
        if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL)
        {
            // Mark device to close
            s_driver_obj->mux_protected.device[i].actions |= ACTION_CLOSE_DEV;
            // Set flag
            s_driver_obj->mux_protected.flags.unhandled_devices = 1;
        }
    }
    s_driver_obj->mux_protected.flags.shutdown = 1;
    xSemaphoreGive(s_driver_obj->constant.mux_lock);

    // Unblock, exit the loop and proceed to deregister client
    ESP_ERROR_CHECK(usb_host_client_unblock(s_driver_obj->constant.client_hdl));
}
