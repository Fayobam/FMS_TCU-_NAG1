// ============================================================================
// FILE: main.cpp
// VERSION: 7.1
// UPDATES: Restored FreeRTOS dual-core architecture and SpeedReader integration.
// ============================================================================
#include <Arduino.h>

// --- Our Custom Headers ---
#include "TCU_Data.h"
#include "SolenoidDriver.h"
#include "SpeedReader.h"
#include "InputManager.h"
#include "AdaptiveMemory.h"
#include "ShiftScheduler.h"
#include "WebManager.h"

// REMOVED the conflicting #define PIN_TEMP_SENSOR 34 here!

// ============================================================================
// 1. GLOBAL OBJECT INSTANTIATION
// ============================================================================
// Create the global telemetry struct
TCU_Telemetry telemetry;

// Instantiate the Hardware Modules
SolenoidDriver solenoids(PIN_MPC, PIN_SPC, PIN_TCC, PIN_Y3, PIN_Y4, PIN_Y5);
SpeedReader speedReader(PIN_N2_SPEED, PIN_N3_SPEED, PIN_OUT_SPEED, PIN_ENG_SPEED);

// FIX: Pass the correct temperature pin from TCU_Data.h!
InputManager inputManager(PIN_ATF_TEMP); 

AdaptiveMemory adaptives;
WebManager webManager;

// The Scheduler requires pointers to BOTH the Solenoids and the Adaptive Memory
ShiftScheduler shiftScheduler(&solenoids, &adaptives);

// ============================================================================
// 2. FREERTOS TASK PROTOTYPES
// ============================================================================
void core1PhysicsTask(void *pvParameters);
void core0DashboardTask(void *pvParameters);

// ============================================================================
// 3. SETUP (Runs exactly once at boot)
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Booting 722.6 Standalone TCU (W5A330 / M111 Supercharged)...");

    // Initialize all modules
    solenoids.begin();      
    speedReader.begin();    
    inputManager.begin();   
    adaptives.begin();      
    shiftScheduler.begin(); 
    
    // Initialize the Web Server and Access Point
    webManager.setAdaptiveMemory(&adaptives);
    webManager.begin(); 

    // Launch FreeRTOS Tasks
    xTaskCreatePinnedToCore(core1PhysicsTask, "PhysicsTask", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(core0DashboardTask, "DashboardTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL); // Free up the loop task since FreeRTOS handles everything
}

// ============================================================================
// 5. THE PHYSICS LOOP (Core 1) - 1000Hz (1ms)
// ============================================================================
void core1PhysicsTask(void *pvParameters) {
    const TickType_t xFrequency = 1; 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint8_t speed_update_counter = 0;

    while (true) {
        // 1. Read Inputs
        inputManager.update();              // Reads Digital PRND pins
        inputManager.updateAnalogSensors(); // NEW: Reads Analog Temp/Multiplex pin
        
        speed_update_counter++;
        if (speed_update_counter >= 50) {
            speedReader.update();
            speed_update_counter = 0; 
        }

        // 2. Run Main Shift Logic
        shiftScheduler.update();

        // 3. Keep the Y4 valve active in Park/Neutral for smooth engagements
        if (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N') {
            solenoids.startGarageShiftJiggle(); 
        } else {
            solenoids.stopGarageShiftJiggle();
        }

        // 4. Command Hardware
        solenoids.update();

        // 5. Sleep exactly until the next 1ms tick
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// ============================================================================
// 6. THE DASHBOARD LOOP (Core 0) - 100Hz (10ms)
// ============================================================================
void core0DashboardTask(void *pvParameters) {
    while (true) {
        // This broadcasts the data to your phone's browser via WebSockets
        webManager.broadcastTelemetry();
        
        // Yield CPU to the ESP32's internal WiFi radio
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}