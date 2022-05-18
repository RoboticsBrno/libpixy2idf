#include <esp_log.h>
#include <driver/spi_master.h>
#include <freertos/task.h>
#include <vector>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "pixy2/pixy2.hpp"
#include "pixy2/spi.hpp"
#include "pixy2/i2c.hpp"

using namespace pixy2;

extern "C" void app_main_spi()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));


    const spi_bus_config_t busCfg = {
        .mosi_io_num = GPIO_NUM_13,
        .miso_io_num = GPIO_NUM_12,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busCfg, 0));

    auto linkRes = LinkSpi::addSpiDevice(SPI2_HOST);
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    LinkSpi link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkSpi>(std::move(link));

    ESP_ERROR_CHECK(pixy.waitForStartup());

    GetBlocksContext blocksCtx;
    while(true) {
        auto err = pixy.getColorBlocks(1, 4, blocksCtx);
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
        uart_write_bytes(UART_NUM_0, lorrisHeader, sizeof(lorrisHeader));
        if(blocksCtx.blocks.size() > 0) {
            uart_write_bytes(UART_NUM_0, blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    } 
}



extern "C" void app_main_i2c()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));


    auto linkRes = LinkI2C::withBusInit(I2C_NUM_0, 0x54, GPIO_NUM_13, GPIO_NUM_12);
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    auto link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkI2C>(std::move(link));

    ESP_ERROR_CHECK(pixy.waitForStartup());

    GetBlocksContext blocksCtx;
    while(true) {
        auto err = pixy.getColorBlocks(1, 4, blocksCtx);
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
        uart_write_bytes(UART_NUM_0, lorrisHeader, sizeof(lorrisHeader));
        if(blocksCtx.blocks.size() > 0) {
            uart_write_bytes(UART_NUM_0, blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
