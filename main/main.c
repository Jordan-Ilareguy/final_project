#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "string.h"
#include <math.h>

// ==================== PIN DEFINITIONS ====================

// LEDs
#define RED_LED 2
#define GREEN_LED 21

// Keypad - USING YOUR PREFERRED PIN ASSIGNMENTS
#define ROWS 4
#define COLS 4
int rowPins[ROWS] = {11, 12, 13, 14};      // Row pins
int colPins[COLS] = {18, 8, 9, 10};        // Column pins

// I2C for LCD and MPU6050
#define I2C_MASTER_SCL_IO 14
#define I2C_MASTER_SDA_IO 13
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// LCD1602 I2C address (0x27 or 0x3F - will auto-detect)
#define LCD_ADDR 0x27

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// ==================== KEYPAD LAYOUT ====================

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// ==================== VARIABLES ====================

char pin[5] = {0};        // Buffer to store entered PIN (4 digits + null terminator)
int position = 0;         // Current position in PIN entry
int blueCounter = 0;      // Simple counter for system status
bool systemReady = true;  // System status

// ==================== FUNCTION DECLARATIONS ====================

void setupGPIO();                    // Initialize GPIO pins
char scanKeypad();                   // Scan keypad for key press
bool checkPin();                     // Check if entered PIN matches correct PIN
void correctPINAction();             // Execute when PIN is correct
void incorrectPINAction();           // Execute when PIN is incorrect
void resetPINEntry();                // Reset PIN buffer and position

// ==================== MAIN APPLICATION ====================

void app_main() {
  
  setupGPIO();  // Initialize GPIO pins

  // Prompt user to enter key code
  printf("Enter Key Code:\n");

  while (1) {

    // ===== KEYPAD SCANNING - Checking for key press =====
    char key = scanKeypad();

    // If a key is pressed, then...
    if (key != 0) {
      // Check if it's a digit AND we haven't filled 4 digits yet
      if (key >= '0' && key <= '9' && position < 4) {
        // Store digit in PIN buffer
        pin[position] = key;
        position++;

        printf("*");
        fflush(stdout);  // Flushes out the buffer and immediately prints asterisk to the same line.

        // If 4 digits entered, verify PIN
        if (position == 4) {
          pin[4] = 0;  // Null-terminate

          // Check if PIN is correct
          if (checkPin()) {
            correctPINAction();
          } else {
            incorrectPINAction();
          }

          // Reset PIN buffer and position
          resetPINEntry();

          // Prompt for next PIN
          printf("\nEnter Key Code:\n");
        }
      }
    }

    // Print system status periodically
    if (blueCounter >= 100) {  // Every 1 second at 10ms delays
        blueCounter = 0;
    } else {
      blueCounter++;
    }

    // Delay for timing (10ms)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== GPIO SETUP ====================

void setupGPIO() {
  // Set up LEDs as outputs
  gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);

  // Set initial state (all off)
  gpio_set_level(RED_LED, 0);
  gpio_set_level(GREEN_LED, 0);

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

// ==================== KEYPAD SCANNING ====================

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

// ==================== PIN VALIDATION ====================

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

// ==================== PIN ENTRY ACTIONS ====================

void correctPINAction() {
  // ACCESS GRANTED
  printf("\nAccess Granted!\n");

  // Turn off any previous LED state
  gpio_set_level(RED_LED, 0);

  // Turn on green LED
  gpio_set_level(GREEN_LED, 1);

  // Keep green LED on for 5 seconds
  vTaskDelay(pdMS_TO_TICKS(5000));
  gpio_set_level(GREEN_LED, 0);
}

void incorrectPINAction() {
  // ACCESS DENIED
  printf("\nAccess Denied!\n");

  // Turn off any previous LED state
  gpio_set_level(GREEN_LED, 0);

  // Turn on red LED
  gpio_set_level(RED_LED, 1);

  // Keep red LED on for 2 seconds
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(RED_LED, 0);
}

void resetPINEntry() {
  // Reset PIN buffer and position
  for (int i = 0; i < 5; i++) {
    pin[i] = 0;  // Clear PIN buffer manually
  }
  position = 0;
}
