#ifndef _esp32_w5500_h
#define _esp32_w5500_h

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "lwip/dns.h"

extern void tcpipInit();

esp_netif_ip_info_t info_t;
bool w5500_static_ip_mode = false;
uint32_t W5500_SPI_HOST      = 1;
uint32_t W5500_SPI_CLOCK_MHZ = 12000000;
char *dns1;
char *dns2;

void w5500_set_statc_ip( char* ip,char* gateway,char* netmask,char* _dns1, char* _dns2)
{
  esp_netif_str_to_ip4((const char *)ip,&info_t.ip);
  esp_netif_str_to_ip4((const char *)gateway,&info_t.gw);
  esp_netif_str_to_ip4((const char *)netmask,&info_t.netmask);
  dns1 = _dns1;
  dns2 = _dns2;
  w5500_static_ip_mode = true;
}
void w5500_set_spi(uint32_t spi_host, uint32_t spi_clk)
{
  W5500_SPI_HOST = spi_host;
  W5500_SPI_CLOCK_MHZ = spi_clk;
}
void w5500_begin(int mosi_pin,int miso_pin,int sclk_pin,int cs_pin,int rst_pin,int intr_pin)
{
  tcpipInit();

  esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
  if(w5500_static_ip_mode)esp_netif_config.ip_info = &info_t;

  esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };

  esp_netif_t *eth_netif_spi;
  char if_key_str[10];
  char if_desc_str[10];
  char num_str[3];
  itoa(0, num_str, 10);
  strcat(strcpy(if_key_str, "ETH_SPI_"), num_str);
  strcat(strcpy(if_desc_str, "eth"), num_str);
  esp_netif_config.if_key = if_key_str;
  esp_netif_config.if_desc = if_desc_str;
  esp_netif_config.route_prio = 30;
  eth_netif_spi = esp_netif_new(&cfg_spi);
  if(w5500_static_ip_mode)
  {
    esp_netif_dhcpc_stop(eth_netif_spi);
    esp_netif_set_ip_info(eth_netif_spi, &info_t);
    IPAddress ns1,ns2;
    ns1.fromString(String(dns1));
    ns2.fromString(String(dns2));

    ip_addr_t d;
    d.type = IPADDR_TYPE_V4;
    if(static_cast<uint32_t>(ns1) != 0) {
          d.u_addr.ip4.addr = static_cast<uint32_t>(ns1);
          dns_setserver(0, &d);
      }
    if(static_cast<uint32_t>(ns2) != 0) {
          d.u_addr.ip4.addr = static_cast<uint32_t>(ns2);
          dns_setserver(1, &d);
      }
  }

  // Init MAC and PHY configs to default
  eth_mac_config_t mac_config_spi = {                     \
        .sw_reset_timeout_ms = 100,                       \
        .rx_task_stack_size = 2048,                       \
        .rx_task_prio = 15,                               \
        .smi_mdc_gpio_num = -1,                           \
        .smi_mdio_gpio_num = -1,                          \
        .flags = 0,                                       \
        .interface = EMAC_DATA_INTERFACE_RMII,            \
        .clock_config =                                   \
        {                                                 \
            .rmii =                                       \
            {                                             \
                .clock_mode = EMAC_CLK_DEFAULT,           \
                .clock_gpio = EMAC_CLK_IN_GPIO            \
            }                                             \
        }                                                 \
    };
  eth_phy_config_t phy_config_spi = {      \
        .phy_addr = ESP_ETH_PHY_ADDR_AUTO, \
        .reset_timeout_ms = 100,           \
        .autonego_timeout_ms = 4000,       \
        .reset_gpio_num = 5,               \
    };

  // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
  gpio_install_isr_service(0);

  spi_device_handle_t spi_handle;
  spi_bus_config_t buscfg = {
      .mosi_io_num = mosi_pin,
      .miso_io_num = miso_pin,
      .sclk_io_num = sclk_pin,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)W5500_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
              

  // Configure SPI interface and Ethernet driver for specific SPI module
  esp_eth_mac_t *mac_spi;
  esp_eth_phy_t *phy_spi;
  esp_eth_handle_t eth_handle_spi;
  
  spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = W5500_SPI_CLOCK_MHZ,
        .queue_size = 20
    };

  // Set SPI module Chip Select GPIO
  devcfg.spics_io_num = cs_pin;

  ESP_ERROR_CHECK(spi_bus_add_device((spi_host_device_t)W5500_SPI_HOST, &devcfg, &spi_handle));
  // w5500 ethernet driver is based on spi driver
  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);

  // Set remaining GPIO numbers and configuration used by the SPI module
  w5500_config.int_gpio_num = intr_pin;
  phy_config_spi.phy_addr = 1;
  phy_config_spi.reset_gpio_num = rst_pin;

  mac_spi = esp_eth_mac_new_w5500(&w5500_config, &mac_config_spi);
  phy_spi = esp_eth_phy_new_w5500(&phy_config_spi);


  esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi, phy_spi);
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi));

  uint8_t mac_data[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
  ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle_spi, ETH_CMD_S_MAC_ADDR,mac_data));

  // attach Ethernet driver to TCP/IP stack
  ESP_ERROR_CHECK(esp_netif_attach(eth_netif_spi, esp_eth_new_netif_glue(eth_handle_spi)));
  ESP_ERROR_CHECK(esp_eth_start(eth_handle_spi));
}

#endif