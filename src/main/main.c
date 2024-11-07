#include <stdio.h>
#include <string.h>
/* Peryferia */
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
/* Free rtos*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
/* MICS */
#include "esp_log.h"
#include "TMC5072.h"
/* definy pinów */
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define MOTOR_CURRENT_SCALE 32
/* zmienne */
static const char* TAG = "tmc driver";
spi_device_handle_t tmc5072_spi;

/* deklaracje funkcji */
static void spi_init();
void tmc5072_writeInt(uint8_t address, int32_t value);
int32_t tmc5072_readInt(uint8_t address);
void tmc5072_setTargetVelocity(int motor, int32_t velocity);
void tmc5072_setAcceleration(int motor, int32_t acceleration);
void tmc5072_setTargetPosition(int motor, int32_t position);
void tmc5072_setTargetPosition(int motor, int32_t position);
int32_t tmc5072_getActualVelocity(int motor);
void tmc5072_setCurrent(int motor, uint8_t current_scale);
void tmc5072_rotateFullTurn(int motor);

/* main */

void app_main(void)
{   

	spi_init();
	tmc5072_setCurrent(0, MOTOR_CURRENT_SCALE);
	tmc5072_setTargetVelocity(0, 500);
	tmc5072_rotateFullTurn(0); 
}

/* definicje funkcji */
static void spi_init(){

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,           // Częstotliwość 1 MHz
        .mode = 3,                               // Tryb SPI 3 (CPOL=1, CPHA=1)
        .spics_io_num = PIN_NUM_CS,              // Pin CS
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &tmc5072_spi));
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

// Funkcja zapisu danych
void tmc5072_writeInt(uint8_t address, int32_t value) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Wyzerowanie struktury
    t.length = 8 * 5;          // 5 bajtów = 1 bajt adresu + 4 bajty danych
    uint8_t tx_buffer[5];
    tx_buffer[0] = address | 0x80;   // Ustawienie bitu zapisu
    tx_buffer[1] = (value >> 24) & 0xFF;
    tx_buffer[2] = (value >> 16) & 0xFF;
    tx_buffer[3] = (value >> 8) & 0xFF;
    tx_buffer[4] = value & 0xFF;

    t.tx_buffer = tx_buffer;
    t.rx_buffer = NULL;

    spi_device_transmit(tmc5072_spi, &t);  // Wysyłanie przez SPI
}

// Funkcja odczytu danych
int32_t tmc5072_readInt(uint8_t address) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Wyzerowanie struktury
    t.length = 8 * 5;          // 5 bajtów = 1 bajt adresu + 4 bajty danych
    uint8_t tx_buffer[5];
    uint8_t rx_buffer[5];
    tx_buffer[0] = address & 0x7F;   // Ustawienie bitu odczytu
    memset(&tx_buffer[1], 0, 4);     // Zerowanie pozostałych bajtów

    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;

    spi_device_transmit(tmc5072_spi, &t);  // Wysyłanie i odbiór przez SPI

    int32_t result = (rx_buffer[1] << 24) | (rx_buffer[2] << 16) | (rx_buffer[3] << 8) | rx_buffer[4];
    return result;
}

void tmc5072_setTargetVelocity(int motor, int32_t velocity) {
    uint8_t address = TMC5072_VMAX(motor);
    tmc5072_writeInt(address, velocity);
}

void tmc5072_setAcceleration(int motor, int32_t acceleration) {
    uint8_t address = TMC5072_AMAX(motor);
    tmc5072_writeInt(address, acceleration);
}

void tmc5072_setTargetPosition(int motor, int32_t position) {
    uint8_t address = TMC5072_XTARGET(motor);
    tmc5072_writeInt(address, position);
}

int32_t tmc5072_getActualVelocity(int motor) {
    uint8_t address = TMC5072_VACTUAL(motor);
    return tmc5072_readInt(address);
}

void tmc5072_setCurrent(int motor, uint8_t current_scale) {
    uint8_t address = TMC5072_IHOLD_IRUN(motor);
    int32_t value = (current_scale << 8) | current_scale; // Same scale for hold and run current
    tmc5072_writeInt(address, value);
}

void tmc5072_rotateFullTurn(int motor) {
    // Set target position for 200 steps forward
    tmc5072_setTargetPosition(motor, 200);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1 second

    // Set target position for 200 steps backward
    tmc5072_setTargetPosition(motor, -200);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1 second
}
