// Startup and task ownership. Control starts before networking.
// See docs/CONTROL_ARCHITECTURE.md for the firmware reading order.
#include <Arduino.h>
#include "esp_task_wdt.h"

#include "TCU_Data.h"
#include "EngineProfile.h"
#include "SolenoidDriver.h"
#include "SpeedReader.h"
#include "InputManager.h"
#include "AdaptiveMemory.h"
#include "ShiftScheduler.h"
#include "WebManager.h"
#include "DtcManager.h"
#include "TuneOverlay.h"
#include "ControlBridge.h"
#include <Preferences.h>

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
    Serial.println("Booting FMS 722.6 TCU...");

    engineProfile.begin();   // before inputs (TPS/MAP cal) and scheduler (torque model)
    tuneOverlay.begin();     // before the scheduler: it reads line/apply/inertia/backstop
    Preferences selectorPrefs;
    if (selectorPrefs.begin("tcu_selector", false)) {
        if (selectorPrefs.isKey("atf_only"))
            telemetry.atf_only_selector = selectorPrefs.getBool("atf_only", false);
        selectorPrefs.end();
    }
    solenoids.begin();
    speedReader.begin();
    inputManager.begin();
    adaptives.begin();
    dtcManager.begin();
    shiftScheduler.begin();

    webManager.setAdaptiveMemory(&adaptives);
    if (xTaskCreatePinnedToCore(core1PhysicsTask, "PhysicsTask", 8192, NULL, 5, NULL, 1) != pdPASS) {
        Serial.println("Control task allocation failed; outputs retain their safe boot state.");
        return; // Never allocate networking ahead of a failed control task.
    }
    xTaskCreatePinnedToCore(core0DashboardTask, "DashboardTask", 16384, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}

// ============================================================================
// 4. PHYSICS LOOP (Core 1) - 1000Hz
// ============================================================================
void core1PhysicsTask(void *pvParameters) {
    // Hardware task watchdog. If this loop ever stalls — a wedged driver call, an
    // accidental infinite loop, priority-10 WiFi work starving core 1 — the chip
    // panics (register dump on serial) and reboots into the safe boot state: all
    // solenoids de-energized, then drive latch + gear resync recover the gear and
    // block shifting until the label is ratio-verified. Without this, a stall
    // mid-shift holds a routing coil at its 80% kick and SPC mid-ramp forever.
    // 250 ms is ~250 loop periods and well above the worst legitimate stall
    // (an NVS commit's flash-cache suspension is tens of ms). The dashboard/WiFi
    // tasks and the idle tasks are deliberately NOT watched — a WiFi stall must
    // never reboot the car.
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 250,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    if (esp_task_wdt_init(&wdt_cfg) == ESP_ERR_INVALID_STATE)
        esp_task_wdt_reconfigure(&wdt_cfg);   // Arduino core had already started the TWDT
    esp_task_wdt_add(NULL);                   // watch THIS task

    const TickType_t xFrequency = 1;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long lastOverrunLog = 0;

    while (true) {
        esp_task_wdt_reset();
        uint32_t t0 = micros();

        controlBridge.consume(adaptives); // bounded typed commands; no network or NVS work
        inputManager.update();     // PRND + paddles + one ADC channel (round-robin)
        speedReader.update();      // period-capture readout, recomputed every control tick

        shiftScheduler.update();   // owns standby/garage Y4 windowing now (ATSG-correct)

        solenoids.update();
        dtcManager.poll();         // edge the fault flags into the DTC store
        controlBridge.publish(adaptives); // coherent read-only snapshot at 10 Hz

        // Loop-overrun accounting. The 1 kHz budget is 1000 us; running long silently
        // stretches every shift phase's wall-clock. The DTC stays on the hard case and
        // stays rate-limited, but the counters are cheap and unthrottled, so a stall
        // that happens once an hour is still visible after the fact.
        uint32_t elapsed = micros() - t0;
        if (elapsed > telemetry.loop_max_us) telemetry.loop_max_us = elapsed;
        if (elapsed > 1500) {
            telemetry.loop_overrun_hard++;
            if ((millis() - lastOverrunLog) > 1000) {
                dtcManager.trip(DTC_LOOP_OVERRUN);
                lastOverrunLog = millis();
            }
        } else if (elapsed > 1000) {
            telemetry.loop_overrun_soft++;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================================================
// 5. DASHBOARD LOOP (Core 0) - 200Hz service, rate-limited telemetry
// ============================================================================
void core0DashboardTask(void *pvParameters) {
    webManager.begin(); // Control is already running before any Wi-Fi/filesystem work.
    while (true) {
        webManager.update();               // Core 0: DNS, cmd queue, WS telemetry, NVS
        dtcManager.processFlush();          // persist DTC counts to NVS (throttled)
        vTaskDelay(pdMS_TO_TICKS(5));       // 200Hz service loop; broadcast gate sets the real rate
    }
}
