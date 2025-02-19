/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_eth.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/init.h"

#define HOST_LIB_TASK_PRIORITY 2
#define CLASS_TASK_PRIORITY 3
#define APP_QUIT_PIN CONFIG_APP_QUIT_PIN

#ifdef CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
#define ENABLE_ENUM_FILTER_CALLBACK
#endif // CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK

extern void class_driver_task(void *arg);
extern void class_driver_client_deregister(void);

static const char *TAG = "USB host lib";

esp_eth_handle_t eth_handle = NULL;
static uint8_t s_sta_mac[6];

extern usb_device_handle_t usb_dev;

QueueHandle_t app_event_queue = NULL;

/**
 * @brief APP event group
 *
 * APP_EVENT            - General event, which is APP_QUIT_PIN press event in this example.
 */
typedef enum
{
    APP_EVENT = 0,
} app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering events from callback to a task.
 */
typedef struct
{
    app_event_group_t event_group;
} app_event_queue_t;

/**
 * @brief BOOT button pressed callback
 *
 * Signal application to exit the Host lib task
 *
 * @param[in] arg Unused
 */
static void gpio_cb(void *arg)
{
    const app_event_queue_t evt_queue = {
        .event_group = APP_EVENT,
    };

    BaseType_t xTaskWoken = pdFALSE;

    if (app_event_queue)
    {
        xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
    }

    if (xTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id)
    {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

typedef struct
{
    esp_eth_phy_t parent; // Base structure
} usb_eth_phy_t;

typedef struct
{
    esp_eth_mac_t parent;
    uint8_t mac_addr[6];
    usb_device_handle_t usb_dev; // TinyUSB device handle
} usb_eth_mac_t;

// PHY start (not much needed for USB)
static esp_err_t usb_phy_start(esp_eth_phy_t *phy)
{
    return ESP_OK;
}

// PHY Reset (No-op for USB)
static esp_err_t usb_phy_reset(esp_eth_phy_t *phy)
{
    return ESP_OK;
}

// PHY Power Control (No-op for USB)
static esp_err_t usb_phy_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    return ESP_OK;
}

// Get PHY Link Status (Return always UP)
static esp_err_t usb_phy_get_link(esp_eth_phy_t *phy)
{
    return ESP_OK; // Always return success (Realtek USB PHY manages its link internally)
}

// Create dummy PHY instance
esp_eth_phy_t *esp_eth_phy_new_usb()
{
    usb_eth_phy_t *phy = calloc(1, sizeof(usb_eth_phy_t));

    if (!phy)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for USB PHY");
        return NULL;
    }

    phy->parent.reset = usb_phy_reset;
    phy->parent.pwrctl = usb_phy_pwrctl;
    phy->parent.get_link = usb_phy_get_link;
    return &(phy->parent);
}

// Get MAC address from Realtek adapter
static esp_err_t usb_eth_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    usb_eth_mac_t *usb_mac = __containerof(mac, usb_eth_mac_t, parent);
    memcpy(addr, usb_mac->mac_addr, 6);
    return ESP_OK;
}

// Start function (if needed for initialization)
static esp_err_t usb_eth_start(esp_eth_mac_t *mac)
{
    // Configure USB communication (e.g., enable RX transfers)
    return ESP_OK;
}

// Send a packet over USB
static esp_err_t usb_eth_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    usb_eth_mac_t *usb_mac = __containerof(mac, usb_eth_mac_t, parent);

    // Use TinyUSB bulk transfer to send data
    ESP_LOGI(TAG, "Transmitting %ld bytes", length);
    // tusb_host_xfer_submit(usb_mac->usb_dev, BULK_OUT_ENDPOINT, buf, length);

    return ESP_OK;
}

// Receive packets from USB
static esp_err_t usb_eth_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    usb_eth_mac_t *usb_mac = __containerof(mac, usb_eth_mac_t, parent);

    // Perform a bulk transfer to receive packets
    ESP_LOGI(TAG, "Receiving packet");
    // tusb_host_xfer_submit(usb_mac->usb_dev, BULK_IN_ENDPOINT, buf, *length);

    return ESP_OK;
}

// Define MAC driver
esp_eth_mac_t *esp_eth_mac_new_usb(usb_device_handle_t usb_dev)
{
    usb_eth_mac_t *mac = calloc(1, sizeof(usb_eth_mac_t));
    mac->parent.get_addr = usb_eth_get_addr;
    mac->parent.start = usb_eth_start;
    mac->parent.transmit = usb_eth_transmit;
    mac->parent.receive = usb_eth_receive;
    mac->usb_dev = usb_dev;
    return &(mac->parent);
}

// This function will be called when the USB Ethernet device is initialized
static void usb_ethernet_init()
{

    esp_netif_init();                                 // Initialize TCP/IP network interface (should be called only once in application)
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH(); // apply default network interface configuration for Ethernet
    esp_netif_t *eth_netif = esp_netif_new(&cfg);     // create network interface for Ethernet driver

    // Initialize the Ethernet driver (you'll need to fill in your configuration here)
    if (!usb_dev)
    {
        ESP_LOGE(TAG, "USB device not initialized");
        return;
    }
    esp_eth_mac_t *mac = esp_eth_mac_new_usb(usb_dev); // todo: link usb_dev to usb host device handle.
    esp_eth_phy_t *phy = esp_eth_phy_new_usb();

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy); // Default configuration for LAN8720

    esp_err_t err = esp_eth_driver_install(&eth_config, &eth_handle); // Install the Ethernet driver and get the handle
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ethernet driver installation failed: %s", esp_err_to_name(err));
        return;
    }

    esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)); // attach Ethernet driver to TCP/IP stack

    // Register Ethernet event handler
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_eth_start(eth_handle); // start Ethernet driver state machine
}

/**
 * @brief Set configuration callback
 *
 * Set the USB device configuration during the enumeration process, must be enabled in the menuconfig

 * @note bConfigurationValue starts at index 1
 *
 * @param[in] dev_desc device descriptor of the USB device currently being enumerated
 * @param[out] bConfigurationValue configuration descriptor index, that will be user for enumeration
 *
 * @return bool
 * - true:  USB device will be enumerated
 * - false: USB device will not be enumerated
 */
#ifdef ENABLE_ENUM_FILTER_CALLBACK
static bool set_config_cb(const usb_device_desc_t *dev_desc, uint8_t *bConfigurationValue)
{
    // If the USB device has more than one configuration, set the second configuration
    if (dev_desc->bNumConfigurations > 1)
    {
        *bConfigurationValue = 2;
    }
    else
    {
        *bConfigurationValue = 1;
    }

    // Return true to enumerate the USB device
    return true;
}
#endif // ENABLE_ENUM_FILTER_CALLBACK

/**
 * @brief Start USB Host install and handle common USB host library events while app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
#ifdef ENABLE_ENUM_FILTER_CALLBACK
        .enum_filter_cb = set_config_cb,
#endif // ENABLE_ENUM_FILTER_CALLBACK
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Signalize the app_main, the USB host library has been installed
    xTaskNotifyGive(arg);

    bool has_clients = true;
    bool has_devices = false;
    while (has_clients)
    {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_LOGI(TAG, "Get FLAGS_NO_CLIENTS");
            if (ESP_OK == usb_host_device_free_all())
            {
                ESP_LOGI(TAG, "All devices marked as free, no need to wait FLAGS_ALL_FREE event");
                has_clients = false;
            }
            else
            {
                ESP_LOGI(TAG, "Wait for the FLAGS_ALL_FREE");
                has_devices = true;
            }
        }
        if (has_devices && event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "Get FLAGS_ALL_FREE");
            has_clients = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices, uninstall USB Host library");

    // Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskSuspend(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB host library example");

    // Init BOOT button: Pressing the button simulates app request to exit
    // It will uninstall the class driver and USB Host Lib
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

    app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));
    app_event_queue_t evt_queue;

    TaskHandle_t host_lib_task_hdl, class_driver_task_hdl;

    // Create usb host lib task
    BaseType_t task_created;
    task_created = xTaskCreatePinnedToCore(usb_host_lib_task,
                                           "usb_host",
                                           4096,
                                           xTaskGetCurrentTaskHandle(),
                                           HOST_LIB_TASK_PRIORITY,
                                           &host_lib_task_hdl,
                                           0);
    assert(task_created == pdTRUE);

    // Wait until the USB host library is installed
    ulTaskNotifyTake(false, 1000);

    // Create class driver task
    task_created = xTaskCreatePinnedToCore(class_driver_task,
                                           "class",
                                           5 * 1024,
                                           NULL,
                                           CLASS_TASK_PRIORITY,
                                           &class_driver_task_hdl,
                                           0);
    assert(task_created == pdTRUE);
    // Add a short delay to let the tasks run
    vTaskDelay(1000);

    usb_ethernet_init();

    while (1)
    {
        if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
        {
            if (APP_EVENT == evt_queue.event_group)
            {
                // User pressed button
                usb_host_lib_info_t lib_info;
                ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
                if (lib_info.num_devices != 0)
                {
                    ESP_LOGW(TAG, "Shutdown with attached devices.");
                }
                // End while cycle
                break;
            }
        }
    }

    // Deregister client
    class_driver_client_deregister();
    vTaskDelay(10);

    // Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(host_lib_task_hdl);

    // Delete interrupt and queue
    gpio_isr_handler_remove(APP_QUIT_PIN);
    xQueueReset(app_event_queue);
    vQueueDelete(app_event_queue);
    ESP_LOGI(TAG, "End of the example");
}
