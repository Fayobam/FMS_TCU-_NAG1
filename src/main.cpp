// ============================================================================
// FILE: main.cpp
// VERSION: 9.0
// UPDATES: SpeedReader is now period-based (MCPWM capture) and self-gates to
//          200Hz — called every loop, no more 50ms counting window. InputManager
//          owns all its sampling internally (staggered ADC round-robin).
// ============================================================================
#include <Arduino.h>

#include "TCU_Data.h"
#include "EngineProfile.h"
#include "SolenoidDriver.h"
#include "SpeedReader.h"
#include "InputManager.h"
#include "AdaptiveMemory.h"
#include "ShiftScheduler.h"
#include "WebManager.h"

// ============================================================================
// 1. GLOBAL OBJECTS
// ============================================================================
TCU_Telemetry telemetry;
ShiftTrace    shiftTrace;       // high-rate per-shift datalog ring (bench tuning)
EngineProfile engineProfile;   // per-engine torque table + limits + sensor cal (NVS)

SolenoidDriver solenoids(PIN_MPC, PIN_SPC, PIN_TCC, PIN_Y3, PIN_Y4, PIN_Y5, PIN_RP_LOCK);
SpeedReader speedReader(PIN_N2_SPEED, PIN_N3_SPEED, PIN_OUT_SPEED, PIN_ENG_SPEED);
InputManager inputManager(PIN_ATF_TEMP, PIN_TPS, PIN_MAP);   // <-- now gets load pins
AdaptiveMemory adaptives;
WebManager webManager;
ShiftScheduler shiftScheduler(&solenoids, &adaptives);

// ============================================================================
// 2. TASK PROTOTYPES
// ============================================================================
void core1PhysicsTask(void *pvParameters);
void core0DashboardTask(void *pvParameters);

// ============================================================================
// 3. SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Booting FMS 722.6 TCU V9 (W5A330 / M111.985 + TVS1320)...");

    engineProfile.begin();   // before inputs (TPS/MAP cal) and scheduler (torque model)
    solenoids.begin();
    speedReader.begin();
    inputManager.begin();
    adaptives.begin();
    shiftScheduler.begin();

    webManager.setAdaptiveMemory(&adaptives);
    webManager.begin();

    xTaskCreatePinnedToCore(core1PhysicsTask, "PhysicsTask", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(core0DashboardTask, "DashboardTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}

// ============================================================================
// 4. PHYSICS LOOP (Core 1) - 1000Hz
// ============================================================================
void core1PhysicsTask(void *pvParameters) {
    const TickType_t xFrequency = 1;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        inputManager.update();     // PRND + paddles + one ADC channel (round-robin)
        speedReader.update();      // period-capture readout, self-gated to 200Hz

        shiftScheduler.update();   // owns standby/garage Y4 windowing now (ATSG-correct)

        solenoids.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================================================
// 5. DASHBOARD LOOP (Core 0) - 100Hz
// ============================================================================
void core0DashboardTask(void *pvParameters) {
    while (true) {
        webManager.broadcastTelemetry();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
