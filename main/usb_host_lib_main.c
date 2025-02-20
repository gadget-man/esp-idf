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
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/init.h"
#include "lwip/ip_addr.h"

#include "class_driver.h"

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

esp_netif_t *usb_netif = NULL;

void usb_transmit(uint8_t *packet, size_t len);

// esp_err_t usb_eth_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length);
// esp_err_t usb_eth_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length);

/**
 * @brief APP event group
 *
 * APP_EVENT            - General event, which is APP_QUIT_PIN press event in this example.
 */
typedef enum
{
    APP_EVENT = 0,
} app_event_group_t;

// Define the USB Ethernet Driver Struct
typedef struct
{
    esp_netif_driver_base_t base;
    bool link_up;
    uint8_t mac_addr[6];
} usb_eth_driver_t;

usb_eth_driver_t *usb_driver = NULL;

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

    if (event_base == ETH_EVENT)
    {
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
            ESP_LOGI(TAG, "Unhandled Ethernet Event: %ld", event_id);
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
        case IP_EVENT_ETH_GOT_IP:
        {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Ethernet got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Subnet Mask: " IPSTR, IP2STR(&event->ip_info.netmask));
            ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
            break;
        }

        case IP_EVENT_ETH_LOST_IP:
            ESP_LOGI(TAG, "Ethernet lost IP address");
            break;

        default:
            ESP_LOGW(TAG, "Unhandled IP Event: %ld", event_id);
            break;
        }
    }
}

typedef struct
{
    esp_eth_phy_t parent; // Base structure
} usb_eth_phy_t;

// PHY Reset (No-op for USB)
static esp_err_t usb_phy_reset(esp_eth_phy_t *phy)
{
    return ESP_OK;
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
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

esp_err_t usb_phy_del(esp_eth_phy_t *phy)
{
    free(phy);
    return ESP_OK;
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
    phy->parent.del = usb_phy_del;
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

// Receive packets from USB
// static esp_err_t usb_eth_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
// {
//     usb_eth_mac_t *usb_mac = __containerof(mac, usb_eth_mac_t, parent);

//     // Perform a bulk transfer to receive packets
//     ESP_LOGI(TAG, "Receiving packet");
//     // tusb_host_xfer_submit(usb_mac->usb_dev, BULK_IN_ENDPOINT, buf, *length);

//     return ESP_OK;
// }

esp_err_t usb_eth_mac_del(esp_eth_mac_t *mac)
{
    free(mac);
    return ESP_OK;
}

esp_err_t usb_eth_mac_stop(esp_eth_mac_t *mac)
{
    return ESP_OK; // Stop USB Ethernet (if needed)
}

// Define MAC driver
// esp_eth_mac_t *esp_eth_mac_new_usb(usb_device_handle_t usb_dev)
// {
//     usb_eth_mac_t *mac = calloc(1, sizeof(usb_eth_mac_t));

//     // Set the MAC address (use a valid address or one you define)
//     uint8_t mac_addr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55}; // Example MAC address
//     memcpy(mac->mac_addr, mac_addr, 6);

//     mac->parent.get_addr = usb_eth_get_addr;
//     mac->parent.start = usb_eth_start;
//     // mac->parent.transmit = usb_eth_transmit;
//     // mac->parent.receive = usb_eth_receive;
//     mac->parent.del = usb_eth_mac_del;
//     mac->parent.stop = usb_eth_mac_stop;
//     mac->usb_dev = usb_dev;
//     return &(mac->parent);
// }

esp_err_t my_stack_input_function(void *arg, uint8_t *data, uint32_t length, void *ctx)
{
    // esp_eth_mac_t *mac = (esp_eth_mac_t *)arg; // Cast the 'arg' to the MAC type if needed

    ESP_LOGI(TAG, "Received packet of length %ld", length);
    // ethernet_input(buf, length); // Pass to lwIP or your IP stack
    return ESP_OK;
}

esp_err_t usb_eth_transmit(void *h, void *buffer, size_t len)
{
    ESP_LOGI(TAG, "Transmitting packet");
    // Send the Ethernet frame over USB
    usb_eth_driver_t *driver = (usb_eth_driver_t *)h;
    if (!driver->link_up)
    {
        ESP_LOGI(TAG, "Link is down, cannot send packet");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending packet of length %d", len);
    // USB Send Implementation (TinyUSB)
    usb_transmit(buffer, len);
    // esp_err_t ret = usb_send_ethernet_packet(buffer, len); // Custom function
    // return ret == ESP_OK ? ESP_OK : ESP_FAIL;
    return ESP_OK;
}

// Function to be called after attaching the netif
static esp_err_t usb_eth_post_attach(esp_netif_t *netif, void *args)
{
    usb_eth_driver_t *driver = (usb_eth_driver_t *)args;
    if (!driver || !netif)
    {
        ESP_LOGE(TAG, "Invalid driver or netif in post_attach");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Post attach: Linking network interface with driver");

    // Store netif handle in driver
    // driver->netif = netif;

    // Optional: Set up additional event handlers
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);

    return ESP_OK;
}

void create_usb_netif()
{
    esp_netif_init();
    esp_event_loop_create_default();

    usb_driver = calloc(1, sizeof(usb_eth_driver_t));
    // usb_driver->base.post_attach = NULL; // Optional callback
    usb_driver->link_up = true; // Assume link is up initially

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    usb_netif = esp_netif_new(&netif_config);

    // Get factory MAC and set it
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_ETH);
    esp_netif_set_mac(usb_netif, mac);
    memcpy(usb_driver->mac_addr, mac, 6);
    ESP_LOGI(TAG, "Set MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_netif_driver_ifconfig_t driver_config = {
        .handle = usb_driver,
        .transmit = usb_eth_transmit,
        .driver_free_rx_buffer = NULL, // Not needed for USB
        // .post_attach = usb_eth_post_attach,
    };

    // esp_netif_set_driver_config(usb_netif, &driver_config);

    if (esp_netif_set_driver_config(usb_netif, &driver_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set driver config");
    }
    // Cast to esp_netif_iodriver_handle_t and attach
    if (esp_netif_attach(usb_netif, (esp_netif_iodriver_handle)usb_driver) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to attach netif");
    }

    esp_netif_attach(usb_netif, usb_driver);

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);

    // esp_netif_glue_t glue = driver_glue(usb_driver);

    // esp_netif_attach(esp_netif, glue);

    ESP_LOGI(TAG, "USB Ethernet netif created");
    esp_err_t err = esp_netif_dhcpc_stop(usb_netif);
    if (err == ESP_OK || err == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
    {
        err = esp_netif_dhcpc_start(usb_netif);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start DHCP: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "DHCP client started successfully.");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to stop DHCP: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "DHCP client started");

    // esp_netif_ip_info_t ip_info;
    // IP4_ADDR(&ip_info.ip, 192, 168, 0, 220);
    // IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);
    // IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    // esp_netif_dhcpc_stop(usb_netif);
    // esp_netif_set_ip_info(usb_netif, &ip_info);
    // ESP_LOGI(TAG, "Static IP set: 192.168.0.220");
    // esp_netif_action_connected(usb_netif, NULL, 0, NULL);
}

// This function will be called when the USB Ethernet device is initialized
// static void usb_ethernet_init()
// {
//     ESP_LOGI(TAG, "Initializing USB Ethernet");

//     esp_netif_init();                                 // Initialize TCP/IP network interface (should be called only once in application)
//     esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH(); // apply default network interface configuration for Ethernet
//     esp_netif_t *eth_netif = esp_netif_new(&cfg);     // create network interface for Ethernet driver
//     esp_event_loop_create_default();

//     // Initialize the Ethernet driver (you'll need to fill in your configuration here)
//     if (!usb_dev)
//     {
//         ESP_LOGE(TAG, "USB device not initialized");
//         return;
//     }

//     eth_handle = calloc(1, sizeof(esp_eth_handle_t));
//     if (!eth_handle)
//     {
//         ESP_LOGE(TAG, "Failed to allocate memory for Ethernet handle");
//         return;
//     }

//     // eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();

//     // Define ESP32-specific MAC configuration
//     // eth_esp32_emac_config_t vendor_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

//     // eth_esp32_mac_config_t vendor_config = ETH_ESP32_MAC_DEFAULT_CONFIG();
//     // esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&vendor_config, &mac_config);

//     // eth_esp32_emac_config_t esp32s3_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
//     // esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32se_emac_config, &mac_config);

//     esp_eth_mac_t *mac = esp_eth_mac_new_usb(usb_dev);
//     if (!mac)
//     {
//         ESP_LOGE(TAG, "Failed to create MAC for USB Ethernet");
//         return;
//     }

//     eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

//     // esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);

//     esp_eth_phy_t *phy = esp_eth_phy_new_usb();
//     if (!phy)
//     {
//         ESP_LOGE(TAG, "Failed to create PHY for USB Ethernet");
//         return;
//     }

//     ESP_LOGI(TAG, "MAC pointer: %p, PHY pointer: %p", mac, phy);

//     esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
//     eth_config.stack_input = my_stack_input_function;

//     ESP_LOGI(TAG, "ETH config: MAC=%p, PHY=%p, stack_input=%p",
//              eth_config.mac, eth_config.phy, eth_config.stack_input);

//     ESP_LOGI(TAG, "MAC and PHY ops initialized correctly");

//     ESP_LOGI(TAG, "eth_handle: %p", eth_handle);

//     esp_err_t ret = esp_eth_driver_install(&eth_config, &eth_handle);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Ethernet driver init failed: %d", ret);
//         return;
//     }

//     esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)); // attach Ethernet driver to TCP/IP stack

//     // Register Ethernet event handler
//     esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
//     esp_eth_start(eth_handle); // start Ethernet driver state machine
// }

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
void test_transmit()
{
    uint8_t test_packet[64] = {0}; // Dummy Ethernet frame
    memset(test_packet, 0xFF, 6);  // Broadcast MAC

    if (usb_netif)
    {
        ESP_LOGI(TAG, "Manually sending test packet");
        usb_eth_transmit(usb_netif, test_packet, sizeof(test_packet));
    }
    else
    {
        ESP_LOGE(TAG, "usb_netif is NULL!");
    }
}

void set_link_status(bool link_up)
{
    usb_driver->link_up = link_up;
    if (link_up)
    {
        esp_netif_action_connected(usb_netif, NULL, 0, NULL);
    }
    else
    {
        esp_netif_action_disconnected(usb_netif, NULL, 0, NULL);
    }
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

    // usb_ethernet_init();
    ESP_LOGI(TAG, "calling create_usb_netif");
    create_usb_netif();

    esp_netif_t *test_netif = esp_netif_next(NULL);
    while (test_netif)
    {
        ESP_LOGI("DEBUG", "Interface: %s", esp_netif_get_desc(test_netif));
        test_netif = esp_netif_next(test_netif);
    }
    set_link_status(true);

    // test_transmit();

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
