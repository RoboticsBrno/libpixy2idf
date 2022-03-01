#include <esp_log.h>
#include <driver/spi_master.h>
#include <freertos/task.h>
#include <vector>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "pixy2/pixy2.hpp"
#include "pixy2/spi.hpp"

using namespace pixy2;

extern "C" void app_main()
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



    LineFeaturesContext ctx;

    while(true) {
        auto err = pixy.getLineFeatures(ctx);
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        const uint8_t lorrisHeader[] = {
            0xFF,
            0x02,
            (uint8_t)(ctx.vectors.size() * sizeof(LineVector)),
        };
        uart_write_bytes(UART_NUM_0, lorrisHeader, sizeof(lorrisHeader));
        uart_write_bytes(UART_NUM_0, ctx.vectors.data(), ctx.vectors.size()*sizeof(LineVector));

        /*printf("vectors: %d intersections: %d barcodes: %d\n", ctx.vectors.size(), ctx.intersections.size(), ctx.barcodes.size());

        for(const auto& vec : ctx.vectors) {
            printf("   %3d: %3dx%3d %3dx%3d\n", vec.index, vec.x0, vec.y0, vec.x1, vec.y1);
        }*/

        vTaskDelay(pdMS_TO_TICKS(10));
    } 
}

