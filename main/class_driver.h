#ifndef CLASS_DRIVER_H
#define CLASS_DRIVER_H

typedef struct
{
    esp_eth_mac_t parent;
    uint8_t mac_addr[6];
    usb_device_handle_t usb_dev; // TinyUSB device handle
} usb_eth_mac_t;

#endif