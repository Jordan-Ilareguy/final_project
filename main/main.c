#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "string.h"
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/event_groups.h" 
#include "esp_log.h"
#include "esp_wifi.h"                               // Wi-Fi functions
#include "esp_event.h"                              // Wi-Fi/MQTT events
#include "esp_netif.h"                              // Network interface
#include "nvs_flash.h"  

// PIN DEFINITIONS 

// LEDs
#define RED_LED 2
#define GREEN_LED 21

#define ROWS 4
#define COLS 4
int rowPins[ROWS] = {11, 12, 13, 14};      // Row pins
int colPins[COLS] = {18, 8, 9, 10};        // Column pins

// I2C for LCD and MPU6050
#define I2C_MASTER_SCL_IO 37
#define I2C_MASTER_SDA_IO 38
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU_TEMP                    0x41       // Temperature register
#define MPU6050_ADDR                0x68       // AD0 = GND
#define MPU6050_WHO_AM_I_REG        0x75
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B       // Start of accel data
#define I2C_TIMEOUT_MS              1000

static const char *TAG = "MPU6050";

// I2C Master Init
esp_err_t i2c_master_init()
{
    // Configure I2C master mode, pins, and speed
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

     // Install I2C driver
    esp_err_t ret = i2c_driver_install(I2C_MASTER_NUM,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(ret)); // Log error
        return ret;
    }

    // Wake MPU6050 from sleep
    uint8_t wake[2] = {MPU6050_PWR_MGMT_1, 0x00}; // PWR_MGMT_1 = 0x00 (wake up)
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM,
                                               MPU6050_ADDR,
                                                wake, 
                                               sizeof(wake),
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS)));

    if (ret!= ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 woken up successfully");
    
    // Give sensor time to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return ESP_OK;
}

// KEYPAD LAYOUT
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// VARIABLES
char pin[5] = {0};        // Buffer to store entered PIN (4 digits + null terminator)
int position = 0;         // Current position in PIN entry
int blueCounter = 0;      // Simple counter for system status
bool systemReady = true;  // System status

// FUNCTION DECLARATIONS
void setupGPIO();
char scanKeypad();
bool checkPin();
void correctPINAction();
void incorrectPINAction();
void resetPINEntry();
esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t start_reg, uint8_t *buffer, size_t length);

// MAIN APPLICATION
void app_main() {
  setupGPIO();  // Initialize GPIO pins
  ESP_ERROR_CHECK(i2c_master_init()); // Initialize I2C
  ESP_LOGI(TAG, "I2C initialized"); // I2C init done

  // Phase 1: Keypad login loop (do-while)
  bool authenticated = false;
  do {
    // Prompt user to enter key code (each attempt)
    printf("Enter Key Code:\n");
    resetPINEntry();

    while (1) {
      char key = scanKeypad();
      if (key != 0) {
        if (key >= '0' && key <= '9' && position < 4) {
          pin[position] = key;
          position++;
          printf("*");
          fflush(stdout);

          if (position == 4) {
            pin[4] = 0; // Null-terminate
            if (checkPin()) {
              correctPINAction();
              authenticated = true;
            } else {
              incorrectPINAction();
            }
            // Finish this attempt (either success or retry)
            break;
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  } while (!authenticated);

  // Phase 2: MPU data loop after successful PIN
  while (1) {
    uint8_t data[14];
    if (i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, sizeof(data)) == ESP_OK) {
      int16_t ax = (data[0] << 8) | data[1];
      int16_t ay = (data[2] << 8) | data[3];
      int16_t az = (data[4] << 8) | data[5];

      int16_t temp_raw = (data[6] << 8) | data[7];
      float temperature = (temp_raw / 340.0f) + 36.53f;

      int16_t gx = (data[8] << 8)  | data[9];
      int16_t gy = (data[10] << 8) | data[11];
      int16_t gz = (data[12] << 8) | data[13];

      char payload[160];
      snprintf(payload, sizeof(payload),
              "{\"ax\":%d,\"ay\":%d,\"az\":%d,\"temp_c\":%.2f,\"gx\":%d,\"gy\":%d,\"gz\":%d}",
              ax, ay, az, temperature, gx, gy, gz);

      ESP_LOGI(TAG, "MPU TX: %s", payload);
    }

    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

//  GPIO SETUP 

void setupGPIO() {
  // Set up LEDs as outputs
  gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);

  // Set initial state (all off)
  // With wiring VCC -> resistor -> LED -> GPIO, LEDs are active-low.
  // Drive HIGH to turn them OFF initially.
  gpio_set_level(RED_LED, 1);
  gpio_set_level(GREEN_LED, 1);

  // Set up keypad rows as outputs
  for (int i = 0; i < ROWS; i++) {
    gpio_set_direction(rowPins[i], GPIO_MODE_OUTPUT);
    gpio_set_level(rowPins[i], 1);
  }

  // Set up keypad columns as inputs with pull-ups
  for (int i = 0; i < COLS; i++) {
    gpio_set_direction(colPins[i], GPIO_MODE_INPUT);
    gpio_set_pull_mode(colPins[i], GPIO_PULLUP_ONLY);
  }
}

//  KEYPAD SCANNING 

char scanKeypad() {
  for (int row = 0; row < ROWS; row++) {
    // Set current row LOW
    gpio_set_level(rowPins[row], 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Check each column
    for (int col = 0; col < COLS; col++) {
      // If column is LOW, a key is pressed
      if (gpio_get_level(colPins[col]) == 0) {
        char keyPressed = keys[row][col];

        // Wait for key release (debounce)
        while (gpio_get_level(colPins[col]) == 0) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Set row back to HIGH
        gpio_set_level(rowPins[row], 1);

        // Return the key value
        return keyPressed;
      }
    }

    // Set row back to HIGH
    gpio_set_level(rowPins[row], 1);
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }

  return 0; // No key pressed
}

//  PIN VALIDATION 

bool checkPin() {
  // Manual comparison instead of using strcmp
  if (pin[0] == '1' &&
      pin[1] == '2' &&
      pin[2] == '3' &&
      pin[3] == '4') {
    return true;
  } else {
    return false;
  }
}

//  PIN ENTRY ACTIONS 

void correctPINAction() {
  // ACCESS GRANTED
  printf("\nAccess Granted!\n");

  // Turn off any previous LED state
  // Active-low: drive HIGH to turn red OFF
  gpio_set_level(RED_LED, 1);

  // Turn on green LED
  // Active-low: drive LOW to turn green ON
  gpio_set_level(GREEN_LED, 0);
  // Leave green LED on to indicate persistent success state
}

void incorrectPINAction() {
  // ACCESS DENIED
  printf("\nAccess Denied!\n");

  // Turn off any previous LED state
  // Active-low: drive HIGH to turn green OFF
  gpio_set_level(GREEN_LED, 1);

  // Turn on red LED
  // Active-low: drive LOW to turn red ON
  gpio_set_level(RED_LED, 0);

  // Keep red LED on for 2 seconds
  vTaskDelay(pdMS_TO_TICKS(2000));
  // Active-low: drive HIGH to turn red OFF
  gpio_set_level(RED_LED, 1);
}

void resetPINEntry() {
  // Reset PIN buffer and position
  for (int i = 0; i < 5; i++) {
    pin[i] = 0;  // Clear PIN buffer manually
  }
  position = 0;
}

/* ---------- I2C read function definition from Lab 8----------*/

/* Function Name - i2c_read_bytes

* Description - This function reads a block of data from an I2C device.

* Return type - The return type is esp_err_t, which indicates the success or failure of the operation.

* Parameters - 

- parameter1 - uint8_t device_addr: The I2C address of the device to read from.

- parameter2 - uint8_t start_reg: The starting register address to read from.

- parameter3 - uint8_t *buffer: A pointer to the buffer to store the read data.

- parameter4 - size_t length: The number of bytes to read.

*/

esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t start_reg, uint8_t *buffer, size_t length) // I2C read function definition
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        device_addr,
                                        &start_reg,
                                        1,   
                                        buffer,
                                        length,                                                                                                     
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}