/**
 * @file      Final_Project.ino
 * @brief     ESP32 controller node for capturing joystick input and transmitting 8x8 matrix frames via BLE.
 * @author    Alex Williams
 * @version   1.0
 */


// ================================================================ Includes ================================================================
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <MD_MAX72xx.h>
#include <SPI.h>


// ================================================================ Macros ================================================================
// Define physical GPIO pin numbers
#define JOYSTICK_X       5  
#define JOYSTICK_Y       6  
#define JOYSTICK_BUTTON  7  

#define FREQ 128
#define PERIOD_MS (1000 / FREQ)

#define MATRIX_SIZE 8

// Class-provided UUID strings
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define TARGET_DEVICE_NAME          "CARLOS_ESP32"
#define TARGET_SERVICE_UUID        (uint16_t)0x00FF
#define TARGET_CHARACTERISTIC_UUID (uint16_t)0xFF01

#define MATRIX_DIN   12  // Data In pin
#define MATRIX_CS    11  // Chip Select pin
#define MATRIX_CLK   10  // Clock pin
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES   1


// ================================================================ Global Variables ================================================================
// Instantiate the control object using hardware SPI pins
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, MATRIX_DIN, MATRIX_CLK, MATRIX_CS, MAX_DEVICES);

static BLEAdvertisedDevice* myDevice = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static bool doConnect = false;
static bool connected = false;

/**
 * @struct  GridFrame
 * @brief   Holds an 8-byte buffer representing the entire 8x8 canvas state.
 * @details Each byte represents a row, and each bit inside that byte represents a column (1 = ON, 0 = OFF).
 */
typedef struct {
  uint8_t rows[MATRIX_SIZE];
} GridFrame;

// Task and Queue Handles
static QueueHandle_t FrameQueue_Handle = NULL; 
static TaskHandle_t Joystick_Handle = NULL; 
static TaskHandle_t BLE_Handle = NULL; 

// Global internal canvas tracking the 8x8 matrix state
static uint8_t canvas[MATRIX_SIZE] = {0};
static int16_t currentY = 7;
static int16_t currentX = 7;


// ================================================================ Forward Declarations ================================================================
void initHardware();
void JoystickCheck_Task(void *pvParameter);
void BleTransmit_Task(void *pvParameter);
void updateLedMatrix(uint8_t x, uint8_t y, bool clear);


// ================================================================ Classes ================================================================
/**
 * @class   MyAdvertisedDeviceCallbacks
 * @brief   Callback class for handling BLE scan results.
 * @details Evaluates advertised devices against the target device name and locks onto the correct robotic arm peripheral.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  
  /**
   * @brief   Called automatically when a BLE device is found during a scan.
   * @param   advertisedDevice: The BLE device discovered during the scan.
   * @return  None
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == TARGET_DEVICE_NAME) {
      advertisedDevice.getScan()->stop();
      
      // Save the device reference so the transmit task can connect to it
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};


// ================================================================ Main Execution ================================================================
/**
 * @brief   Standard Arduino setup initialization.
 * @details Initializes serial communication, hardware peripherals, FreeRTOS queues, and spawns the joystick and BLE application tasks.
 * @return  None
 */
void setup() {
  Serial.begin(115200);

  initHardware(); 

  // Queue holds completed frame arrays ready for transmission
  FrameQueue_Handle = xQueueCreate(5, sizeof(GridFrame));
  if (FrameQueue_Handle == NULL) {
    Serial.println("ERROR: Failed to create Frame Queue");
  }

  xTaskCreatePinnedToCore(JoystickCheck_Task, "JoystickCheck", 4096, NULL, 3, &Joystick_Handle, 1); 
  xTaskCreatePinnedToCore(BleTransmit_Task, "BleTransmit", 4096, NULL, 2, &BLE_Handle, 0); 
}


/**
 * @brief   Standard Arduino infinite loop.
 * @details Task execution is handled by FreeRTOS, so the main loop is permanently delayed to yield CPU time.
 * @return  None
 */
void loop() {
  vTaskDelay(portMAX_DELAY);
}


// ================================================================ Function Implementations ================================================================
/**
 * @brief   Initializes GPIO pins and the MAX7219 LED matrix hardware.
 * @return  None
 */
void initHardware() {
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);

  mx.begin();
  mx.clear();

  Serial.println("Hardware initialized.");
}


/**
 * @brief   FreeRTOS task running on Core 1 to monitor joystick inputs and update the drawing matrix.
 * @details Reads analog inputs from the joystick, updates the internal coordinate trackers, illuminates 
 * the corresponding LED, and packages the final matrix frame into a queue upon button press.
 * @param   pvParameter: Pointer to FreeRTOS task parameters (unused).
 * @return  None
 */
void JoystickCheck_Task(void *pvParameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(100); 

  xLastWakeTime = xTaskGetTickCount();
  bool buttonWasPressed = false;

  const int CENTER_LOW = 1800;
  const int CENTER_HIGH = 2300;

  canvas[currentY] |= (1 << currentX);
  updateLedMatrix(currentX, currentY, false);

  while (1) {
    int rawX = analogRead(JOYSTICK_X);
    int rawY = analogRead(JOYSTICK_Y);
    bool btnPressed = (digitalRead(JOYSTICK_BUTTON) == LOW); 
    
    // Check X-Axis (Left / Right movement)
    if (rawX > CENTER_HIGH) {
      currentX--; // Push Right
    } else if (rawX < CENTER_LOW) {
      currentX++; // Push Left
    }

    // Check Y-Axis (Up / Down movement)
    if (rawY > CENTER_HIGH) {
      currentY++; // Push Down
    } else if (rawY < CENTER_LOW) {
      currentY--; // Push Up
    }

    // Constrain the cursor positions so it cannot wander off the 8x8 boundaries (0 to 7)
    if (currentX < 0) currentX = 0;
    if (currentX > 7) currentX = 7;
    if (currentY < 0) currentY = 0;
    if (currentY > 7) currentY = 7;

    canvas[currentY] |= (1 << currentX);
    updateLedMatrix(currentX, currentY, false);

    if (btnPressed && !buttonWasPressed) {
      GridFrame completedFrame;
      memcpy(completedFrame.rows, canvas, MATRIX_SIZE);

      if (xQueueSend(FrameQueue_Handle, &completedFrame, (TickType_t)0) == pdPASS) {        
        updateLedMatrix(0, 0, true); 

        currentX = 7;
        currentY = 7;
        
        // Illuminate the starting pixel for the next drawing session
        canvas[currentY] |= (1 << currentX);
        mx.setPoint(currentY, currentX, true);
        mx.update();

      } else {
        Serial.println("[WARN] Frame queue saturated.");
      }
    }
    
    buttonWasPressed = btnPressed; 
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


/**
 * @brief   FreeRTOS task running on Core 0 acting as a BLE Client to transmit matrix data.
 * @details Scans for the target BLE peripheral, establishes a connection, adjusts the MTU size, 
 * and waits for frames in the FreeRTOS queue to transmit as a 64-byte ASCII payload.
 * @param   pvParameter: Pointer to FreeRTOS task parameters (unused).
 * @return  None
 */
void BleTransmit_Task(void *pvParameter) {
  BLEDevice::init("ESP32_Matrix_Remote"); 
  
  // Start the Scanner
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0);
  
  GridFrame outgoingFrame;

  for (;;) {
    if (doConnect && myDevice != nullptr) {
      BLEClient* pClient = BLEDevice::createClient();
      
      pClient->setMTU(500); 
      
      if (pClient->connect(myDevice)) {
        BLERemoteService* pRemoteService = pClient->getService(BLEUUID(TARGET_SERVICE_UUID));
        if (pRemoteService != nullptr) {
          pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(TARGET_CHARACTERISTIC_UUID));
          if (pRemoteCharacteristic != nullptr) {
            connected = true;
          }
        }
      }
      doConnect = false; 
    }

    // Wait for Joystick Frame and Transmit
    if (connected) {
      if (xQueueReceive(FrameQueue_Handle, &outgoingFrame, portMAX_DELAY) == pdTRUE) {
        
        char ascii_payload[65]; // 64 characters + 1 null terminator
        int index = 0;
        
        for (int row = 0; row < MATRIX_SIZE; row++) {
          for (int col = 0; col < MATRIX_SIZE; col++) {
            // Read the individual bit. If high, append '1', else append '0'
            if (outgoingFrame.rows[row] & (1 << col)) {
              ascii_payload[index] = '1';
            } else {
              ascii_payload[index] = '0';
            }
            index++;
          }
        }
        // Null terminate the string safely
        ascii_payload[64] = '\0'; 
        
        // Write the 64-byte ASCII string to the ESP-IDF Server
        pRemoteCharacteristic->writeValue(ascii_payload, 64);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}


/**
 * @brief   Helper function to update the local MAX7219 LED matrix display.
 * @details Can either illuminate a specific coordinate or execute a full hardware and software reset of the canvas.
 * @param   x: X-coordinate (column) to illuminate.
 * @param   y: Y-coordinate (row) to illuminate.
 * @param   clear: Boolean flag. If true, the matrix hardware and local tracker are wiped clean.
 * @return  None
 */
void updateLedMatrix(uint8_t x, uint8_t y, bool clear) {
  if (clear) {
    mx.clear();
    memset(canvas, 0, sizeof(canvas));
  } else {
    mx.setPoint(y, x, true);
    mx.update();
  }
}
