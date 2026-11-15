/*
 =====================================================================================
   ESP32 Smart BMS & Inverter Controller (Local Web Dashboard)
   Battery Pack: LiFePO4 12.8V 4S 100Ah (Power4S100A / SmarTEC-BMS)
   Connectivity: NimBLE-Arduino + Local Wi-Fi Web Server & mDNS (http://bms.local)
   Relay Automation: Active-LOW Relays with Hysteresis (Charger: GPIO 18, Inverter: GPIO 19)
 =====================================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <NimBLEDevice.h>

// =============================================================================
// 1. HARDWARE PINS & CONFIGURATION
// =============================================================================
#define RELAY_CHARGER_PIN    18   // Controls AC/Solar Charger Relay
#define RELAY_INVERTER_PIN   19   // Controls Inverter Output/Power Relay
#define BUTTON_RESET_PIN     0    // ESP32 BOOT button (hold 5s to reset Wi-Fi)

// Most relay modules are Active-LOW (LOW = Relay Energized / ON)
#define RELAY_ACTIVE_LOW     true

// =============================================================================
// 2. WI-FI & WEB DASHBOARD CONFIGURATION
// =============================================================================
// You can set your home Wi-Fi here OR set it directly from the Web Dashboard!
const char* DEFAULT_STA_SSID = "test";          // Local 2.4GHz Wi-Fi SSID
const char* DEFAULT_STA_PASS = "test123";       // Wi-Fi Password
const char* AP_SSID          = "ESP32-BMS-AP";  // SoftAP broadcast name
const char* AP_PASS          = "12345678";      // SoftAP password (min 8 characters)
const char* MDNS_HOST        = "bms";           // Accessible at http://bms.local
const uint16_t WEB_PORT      = 80;

// =============================================================================
// 3. BLE (Power4S100A SmarTEC BMS)
// =============================================================================
#define BMS_DEVICE_NAME             "Power4S100A"
#define BMS_MAC_ADDRESS             "bc:f3:10:10:31:37"  // Target BMS Bluetooth MAC address
#define BMS_SERVICE_UUID            "0000ffe0-0000-1000-8000-00805f9b34fb"
#define BMS_CHARACTERISTIC_UUID     "0000ffe1-0000-1000-8000-00805f9b34fb"
#define BMS_POLL_INTERVAL_MS        1000

// =============================================================================
// 4. LiFePO4 12.8V (4S) AUTOMATIC VOLTAGE PROTECTION CONFIGURATION
// =============================================================================
// Default protection thresholds (user-configurable from Web UI & stored in NVS)
#define DEFAULT_CHG_CUT_CELL_MV   3650   // 3.65V per cell: Stop Charging
#define DEFAULT_CHG_RES_CELL_MV   3400   // 3.40V per cell: Resume Charging
#define DEFAULT_CHG_CUT_PACK_V    14.40f // 14.4V pack: Stop Charging
#define DEFAULT_CHG_RES_PACK_V    13.60f // 13.6V pack: Resume Charging

#define DEFAULT_INV_CUT_CELL_MV   2800   // 2.80V per cell: Cut Inverter Load
#define DEFAULT_INV_RES_CELL_MV   3100   // 3.10V per cell: Resume Inverter Load
#define DEFAULT_INV_CUT_PACK_V    11.20f // 11.2V pack: Cut Inverter Load
#define DEFAULT_INV_RES_PACK_V    12.50f // 12.5V pack: Resume Inverter Load

struct ProtectionConfig {
  uint16_t cellChgCutoffMv = DEFAULT_CHG_CUT_CELL_MV;
  uint16_t cellChgResumeMv = DEFAULT_CHG_RES_CELL_MV;
  float    packChgCutoffV  = DEFAULT_CHG_CUT_PACK_V;
  float    packChgResumeV  = DEFAULT_CHG_RES_PACK_V;

  uint16_t cellInvCutoffMv = DEFAULT_INV_CUT_CELL_MV;
  uint16_t cellInvResumeMv = DEFAULT_INV_RES_CELL_MV;
  float    packInvCutoffV  = DEFAULT_INV_CUT_PACK_V;
  float    packInvResumeV  = DEFAULT_INV_RES_PACK_V;
};

ProtectionConfig g_cfg;

// =============================================================================
// 5. GLOBAL BMS DATA STRUCTURE
// =============================================================================
struct BmsMetrics {
  bool connected = false;

  // Frame 0x02: Cell Voltages (mV)
  uint16_t cellVoltages[4] = {0, 0, 0, 0};
  uint16_t minCellMv = 0;
  uint16_t maxCellMv = 0;
  uint16_t diffCellMv = 0;
  float packVoltage = 0.0f;

  // Frame 0x04: Gauges & Metrics
  uint8_t soc = 100;                 // State of Charge %
  uint8_t soh = 100;                 // State of Health %
  float current = 0.0f;              // Signed Current (Amps)
  float remainingCapacityAh = 99.9f; // Remaining Ah
  float totalCapacityAh = 100.0f;    // Full capacity (100.0 Ah)
  uint32_t cycleCount = 2;

  // Thermal sensors (°C)
  float cellTemp = 21.0f;
  float cellTempMin = 21.0f;
  float cellTempMax = 21.0f;
  float ambientTemp = 20.0f;
  float mosTemp = 20.0f;

  // Timers & Charging Intervals
  uint16_t curChargeIntervalH = 1;
  uint16_t maxChargeIntervalH = 372;
  uint16_t remChargeMin = 0;
  uint16_t remDischargeMin = 0xFFFF; // 0xFFFF = N/A / Idle

  // MOSFET Switches
  bool chargingState = true;
  bool dischargingState = true;

  uint32_t lastUpdateMs = 0;
};

BmsMetrics g_bms;

// Relay States & Safety Trip Flags
bool g_chargerRelay = true;
bool g_inverterRelay = true;
bool g_autoMode = true;
bool g_chargerTripped = false;
bool g_inverterTripped = false;

// Web Server & Preferences
WebServer g_server(WEB_PORT);
Preferences g_prefs;
bool g_staConnected = false;

// BLE Objects
static const uint8_t CMD_CELLS[]   = {0xEA, 0xD1, 0x01, 0x04, 0xFF, 0x02, 0xF9, 0xF5}; // Frame 0x02 (Cells)
static const uint8_t CMD_STATUS[]  = {0xEA, 0xD1, 0x01, 0x04, 0xFF, 0x03, 0xF8, 0xF5}; // Frame 0x03 (Temps & Status)
static const uint8_t CMD_METRICS[] = {0xEA, 0xD1, 0x01, 0x04, 0xFF, 0x04, 0xFF, 0xF5}; // Frame 0x04 (Pack Metrics)

NimBLEAdvertisedDevice* g_advDevice = nullptr;
NimBLEClient* g_client = nullptr;
NimBLERemoteCharacteristic* g_pChr = nullptr;
bool g_doConnect = false;
uint32_t g_lastPollMs = 0;
uint32_t g_lastScanMs = 0;

// Forward Declarations
void writeHardwareRelay(uint8_t pin, bool state);
void setChargerRelay(bool enable, bool manual = false);
void setInverterRelay(bool enable, bool manual = false);
void evaluateAutoProtection();
void loadProtectionConfig();
void saveProtectionConfig();
bool connectToBms();
void requestMetrics();

// =============================================================================
// 6. EMBEDDED WEB DASHBOARD HTML / CSS / JAVASCRIPT
// =============================================================================
static const char PAGE_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Smart BMS - Power4S100A</title>
  <style>
    :root {
      --bg: #0b1120;
      --card-bg: #1e293b;
      --card-border: #334155;
      --text: #f8fafc;
      --text-muted: #94a3b8;
      --emerald: #10b981;
      --amber: #f59e0b;
      --red: #ef4444;
      --blue: #3b82f6;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      background: var(--bg);
      color: var(--text);
      min-height: 100vh;
      padding: 1rem;
    }
    .container { max-width: 900px; margin: 0 auto; display: flex; flex-direction: column; gap: 1rem; }
    header {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 1rem;
      padding: 1.25rem;
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 0.75rem;
    }
    .brand h1 { font-size: 1.35rem; font-weight: 700; color: #fff; display: flex; align-items: center; gap: 0.5rem; }
    .brand p { font-size: 0.85rem; color: var(--text-muted); }
    .badges { display: flex; gap: 0.5rem; flex-wrap: wrap; }
    .badge {
      font-size: 0.75rem;
      font-weight: 600;
      padding: 0.35rem 0.65rem;
      border-radius: 2rem;
      background: #0f172a;
      border: 1px solid var(--card-border);
      display: inline-flex;
      align-items: center;
      gap: 0.35rem;
    }
    .badge.active { background: rgba(16, 185, 129, 0.15); border-color: var(--emerald); color: var(--emerald); }
    .badge.warn { background: rgba(245, 158, 11, 0.15); border-color: var(--amber); color: var(--amber); }
    .badge.danger { background: rgba(239, 68, 68, 0.15); border-color: var(--red); color: var(--red); }
    .dot { width: 8px; height: 8px; border-radius: 50%; background: currentColor; }

    .grid-2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 1rem; }
    .grid-4 { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 0.75rem; }

    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 1rem;
      padding: 1.25rem;
      display: flex;
      flex-direction: column;
      gap: 1rem;
    }
    .card-title {
      font-size: 0.95rem;
      font-weight: 600;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .big-stat { font-size: 2.2rem; font-weight: 800; color: #fff; line-height: 1; }
    .unit { font-size: 1rem; font-weight: 500; color: var(--text-muted); margin-left: 0.2rem; }
    .stat-sub { font-size: 0.85rem; color: var(--text-muted); margin-top: 0.25rem; }

    .progress-bar-container {
      width: 100%;
      height: 14px;
      background: #0f172a;
      border-radius: 8px;
      overflow: hidden;
      border: 1px solid var(--card-border);
    }
    .progress-fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, #10b981, #3b82f6);
      transition: width 0.5s ease;
    }

    .cells-container { display: flex; flex-direction: column; gap: 0.75rem; }
    .cell-row { display: flex; align-items: center; gap: 0.75rem; }
    .cell-label { font-size: 0.85rem; font-weight: 600; width: 50px; }
    .cell-bar-bg {
      flex: 1;
      height: 22px;
      background: #0f172a;
      border-radius: 6px;
      overflow: hidden;
      border: 1px solid var(--card-border);
    }
    .cell-bar-fill {
      height: 100%;
      width: 0%;
      background: var(--blue);
      transition: width 0.4s ease;
      border-radius: 5px;
    }
    .cell-val { font-size: 0.9rem; font-weight: 700; width: 75px; text-align: right; }

    .relay-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 0.85rem;
      background: #0f172a;
      border: 1px solid var(--card-border);
      border-radius: 0.75rem;
      gap: 0.5rem;
    }
    .relay-info h4 { font-size: 1rem; font-weight: 600; color: #fff; }
    .relay-info p { font-size: 0.75rem; color: var(--text-muted); }
    .btn {
      padding: 0.55rem 1.1rem;
      font-size: 0.85rem;
      font-weight: 600;
      border: none;
      border-radius: 0.5rem;
      cursor: pointer;
      transition: 0.2s;
    }
    .btn-emerald { background: var(--emerald); color: #000; }
    .btn-emerald:hover { opacity: 0.9; }
    .btn-danger { background: var(--red); color: #fff; }
    .btn-danger:hover { opacity: 0.9; }
    .btn-ghost { background: #334155; color: #fff; }
    .btn-ghost:hover { background: #475569; }

    .input-field {
      width: 100%;
      padding: 0.6rem;
      background: #0f172a;
      border: 1px solid var(--card-border);
      border-radius: 0.5rem;
      color: #fff;
      font-size: 0.9rem;
      outline: none;
    }
    .input-field:focus { border-color: var(--blue); }

    footer {
      text-align: center;
      font-size: 0.75rem;
      color: var(--text-muted);
      padding: 1rem;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div class="brand">
        <h1>⚡ LiFePO4 Smart BMS</h1>
        <p>Power4S100A (12.8V 100Ah) &middot; ESP32 Controller</p>
      </div>
      <div class="badges">
        <div id="ble-badge" class="badge warn"><span class="dot"></span><span id="ble-text">BLE Scanning...</span></div>
        <div id="cond-badge" class="badge active"><span class="dot"></span><span id="cond-text">Condition: Normal</span></div>
        <div id="wifi-badge" class="badge active"><span class="dot"></span><span id="wifi-text">Wi-Fi Online</span></div>
      </div>
    </header>

    <!-- Top Power & Energy Stats -->
    <div class="grid-2">
      <!-- Battery Pack Overview -->
      <div class="card">
        <div class="card-title">
          <span>Battery Pack</span>
          <span id="power-stat" style="font-weight:700; color:var(--emerald);">0.0 W</span>
        </div>
        <div style="display:flex; justify-content:space-between; align-items:baseline;">
          <div>
            <div class="big-stat"><span id="pack-voltage">0.00</span><span class="unit">V</span></div>
            <div class="stat-sub">Pack Voltage</div>
          </div>
          <div style="text-align:right;">
            <div class="big-stat" id="current-color"><span id="pack-current">0.0</span><span class="unit">A</span></div>
            <div class="stat-sub" id="current-dir">Idle / Standby</div>
          </div>
        </div>

        <div>
          <div style="display:flex; justify-content:space-between; margin-bottom:0.35rem; font-size:0.85rem;">
            <span>State of Charge (SOC)</span>
            <span style="font-weight:700;" id="soc-text">0%</span>
          </div>
          <div class="progress-bar-container">
            <div class="progress-fill" id="soc-bar"></div>
          </div>
        </div>

        <div class="grid-4" style="margin-top:0.25rem;">
          <div>
            <div style="font-size:1.15rem; font-weight:700; color:var(--emerald);" id="cap-rem">99.9 Ah</div>
            <div class="stat-sub">Remaining Cap</div>
          </div>
          <div>
            <div style="font-size:1.15rem; font-weight:700;" id="cap-full">100.0 Ah</div>
            <div class="stat-sub">Full Capacity</div>
          </div>
          <div>
            <div style="font-size:1.15rem; font-weight:700; color:var(--emerald);" id="soh-text">100%</div>
            <div class="stat-sub">Health (SOH)</div>
          </div>
          <div>
            <div style="font-size:1.15rem; font-weight:700; color:var(--emerald);" id="batt-condition">Normal</div>
            <div class="stat-sub">Condition</div>
          </div>
          <div>
            <div style="font-size:1.15rem; font-weight:700;" id="cycles-text">2</div>
            <div class="stat-sub">Cycle Count</div>
          </div>
          <div>
            <div style="font-size:1.15rem; font-weight:700;" id="delta-text">0 mV</div>
            <div class="stat-sub">Cell Delta</div>
          </div>
        </div>
      </div>

      <!-- 4S Individual Cells -->
      <div class="card">
        <div class="card-title">
          <span>4S Individual Cell Voltages</span>
          <span class="badge" id="balance-badge">Checking...</span>
        </div>
        <div class="cells-container">
          <div class="cell-row">
            <span class="cell-label">Cell 1</span>
            <div class="cell-bar-bg"><div class="cell-bar-fill" id="c1-bar"></div></div>
            <span class="cell-val" id="c1-val">0.000 V</span>
          </div>
          <div class="cell-row">
            <span class="cell-label">Cell 2</span>
            <div class="cell-bar-bg"><div class="cell-bar-fill" id="c2-bar"></div></div>
            <span class="cell-val" id="c2-val">0.000 V</span>
          </div>
          <div class="cell-row">
            <span class="cell-label">Cell 3</span>
            <div class="cell-bar-bg"><div class="cell-bar-fill" id="c3-bar"></div></div>
            <span class="cell-val" id="c3-val">0.000 V</span>
          </div>
          <div class="cell-row">
            <span class="cell-label">Cell 4</span>
            <div class="cell-bar-bg"><div class="cell-bar-fill" id="c4-bar"></div></div>
            <span class="cell-val" id="c4-val">0.000 V</span>
          </div>
        </div>
        <div style="display:flex; justify-content:space-between; font-size:0.8rem; color:var(--text-muted); border-top:1px solid var(--card-border); padding-top:0.6rem;">
          <span>Min Cell: <strong style="color:#fff;" id="min-cell">0.000 V</strong></span>
          <span>Max Cell: <strong style="color:#fff;" id="max-cell">0.000 V</strong></span>
        </div>
      </div>
    </div>

    <!-- Temperatures & Relays -->
    <div class="grid-2">
      <!-- Thermal Monitoring -->
      <div class="card">
        <div class="card-title"><span>Thermal Monitoring</span></div>
        <div class="grid-4">
          <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
            <div style="font-size:1.3rem; font-weight:700; color:var(--emerald);" id="temp-cell-max">-- &deg;C</div>
            <div class="stat-sub">Cell Temp (Max)</div>
          </div>
          <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
            <div style="font-size:1.3rem; font-weight:700; color:var(--emerald);" id="temp-cell-min">-- &deg;C</div>
            <div class="stat-sub">Cell Temp (Min)</div>
          </div>
          <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
            <div style="font-size:1.3rem; font-weight:700; color:var(--blue);" id="temp-amb">-- &deg;C</div>
            <div class="stat-sub">Ambient Temp</div>
          </div>
          <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
            <div style="font-size:1.3rem; font-weight:700; color:var(--amber);" id="temp-mos">-- &deg;C</div>
            <div class="stat-sub">MOSFET Temp</div>
          </div>
        </div>
        <div style="font-size:0.85rem; color:var(--text-muted); display:flex; justify-content:space-between; margin-top:0.5rem;">
          <span>BMS Charge MOS: <strong id="bms-chg-mos" style="color:#fff;">--</strong></span>
          <span>BMS Discharge MOS: <strong id="bms-dsg-mos" style="color:#fff;">--</strong></span>
        </div>
      </div>

      <!-- Relays & Protection Controls -->
      <div class="card">
        <div class="card-title">
          <span>Relay Automation & Controls</span>
          <button class="btn btn-ghost" id="auto-btn" onclick="toggleRelay('auto')" style="font-size:0.75rem; padding:0.25rem 0.6rem;">Auto Mode: ON</button>
        </div>

        <!-- Charger Relay -->
        <div class="relay-item">
          <div class="relay-info">
            <div style="display:flex; align-items:center; gap:0.5rem; margin-bottom:0.2rem;">
              <h4 id="chg-relay-title">Charger Relay (GPIO 18)</h4>
              <span id="chg-status-badge" class="badge active" style="font-size:0.7rem; padding:0.15rem 0.5rem;">ON</span>
            </div>
            <p id="chg-thresh-text">Cutoff: &ge; 14.40V (3650mV) &middot; Resume: &le; 13.60V (3400mV)</p>
          </div>
          <button class="btn btn-danger" id="chg-relay-btn" onclick="toggleRelay('charger')">Turn OFF</button>
        </div>

        <!-- Inverter Relay -->
        <div class="relay-item">
          <div class="relay-info">
            <div style="display:flex; align-items:center; gap:0.5rem; margin-bottom:0.2rem;">
              <h4 id="inv-relay-title">Inverter Relay (GPIO 19)</h4>
              <span id="inv-status-badge" class="badge active" style="font-size:0.7rem; padding:0.15rem 0.5rem;">ON</span>
            </div>
            <p id="inv-thresh-text">Cutoff: &le; 11.20V (2800mV) &middot; Resume: &ge; 12.50V (3100mV)</p>
          </div>
          <button class="btn btn-danger" id="inv-relay-btn" onclick="toggleRelay('inverter')">Turn OFF</button>
        </div>
      </div>
    </div>

    <!-- Cycle Life & Charging Metrics Card -->
    <div class="card">
      <div class="card-title"><span>Cycle Life &amp; Charging Intervals</span></div>
      <div class="grid-4">
        <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
          <div style="font-size:1.3rem; font-weight:700; color:var(--emerald);" id="cur-interval">-- h</div>
          <div class="stat-sub">Cur Charge Interval</div>
        </div>
        <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
          <div style="font-size:1.3rem; font-weight:700; color:var(--blue);" id="max-interval">-- h</div>
          <div class="stat-sub">Max Charge Interval</div>
        </div>
        <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
          <div style="font-size:1.3rem; font-weight:700; color:var(--amber);" id="rem-charge">-- min</div>
          <div class="stat-sub">Rem. Charge Time</div>
        </div>
        <div style="background:#0f172a; padding:0.85rem; border-radius:0.75rem; border:1px solid var(--card-border); text-align:center;">
          <div style="font-size:1.3rem; font-weight:700; color:var(--text-muted);" id="rem-discharge">- min</div>
          <div class="stat-sub">Rem. Discharge Time</div>
        </div>
      </div>
    </div>

    <!-- Protection Thresholds Configuration -->
    <div class="card">
      <div class="card-title">
        <span>⚡ Protection Thresholds Configuration</span>
        <button type="button" class="btn btn-ghost" onclick="resetThresholdDefaults()" style="font-size:0.75rem; padding:0.25rem 0.6rem;">Reset Defaults</button>
      </div>
      <form id="thresh-form" onsubmit="saveThresholds(event)" style="display:flex; flex-direction:column; gap:1rem;">
        <div class="grid-2">
          <!-- Charger Protection Settings -->
          <div style="background:#0f172a; padding:1rem; border-radius:0.75rem; border:1px solid var(--card-border); display:flex; flex-direction:column; gap:0.75rem;">
            <div style="font-weight:700; color:var(--emerald); font-size:0.95rem;">⚡ Charger Protection (Overvoltage)</div>
            <div style="display:grid; grid-template-columns:1fr 1fr; gap:0.6rem;">
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Cell Cutoff (mV)</label>
                <input type="number" id="cfg-chg-cut-cell" class="input-field" min="3400" max="3700" step="10" required />
              </div>
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Cell Resume (mV)</label>
                <input type="number" id="cfg-chg-res-cell" class="input-field" min="3200" max="3650" step="10" required />
              </div>
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Pack Cutoff (V)</label>
                <input type="number" id="cfg-chg-cut-pack" class="input-field" min="13.60" max="14.80" step="0.05" required />
              </div>
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Pack Resume (V)</label>
                <input type="number" id="cfg-chg-res-pack" class="input-field" min="13.00" max="14.60" step="0.05" required />
              </div>
            </div>
            <div style="font-size:0.75rem; color:var(--text-muted);">
              Cutoff triggered if Cell &ge; Cutoff OR Pack &ge; Cutoff.<br/>
              Resumes when Cell &le; Resume AND Pack &le; Resume.
            </div>
          </div>

          <!-- Inverter Protection Settings -->
          <div style="background:#0f172a; padding:1rem; border-radius:0.75rem; border:1px solid var(--card-border); display:flex; flex-direction:column; gap:0.75rem;">
            <div style="font-weight:700; color:var(--amber); font-size:0.95rem;">🔌 Inverter Protection (Undervoltage)</div>
            <div style="display:grid; grid-template-columns:1fr 1fr; gap:0.6rem;">
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Cell Cutoff (mV)</label>
                <input type="number" id="cfg-inv-cut-cell" class="input-field" min="2400" max="3100" step="10" required />
              </div>
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Cell Resume (mV)</label>
                <input type="number" id="cfg-inv-res-cell" class="input-field" min="2500" max="3400" step="10" required />
              </div>
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Pack Cutoff (V)</label>
                <input type="number" id="cfg-inv-cut-pack" class="input-field" min="10.00" max="12.40" step="0.05" required />
              </div>
              <div>
                <label style="font-size:0.75rem; color:var(--text-muted); display:block; margin-bottom:0.25rem;">Pack Resume (V)</label>
                <input type="number" id="cfg-inv-res-pack" class="input-field" min="10.50" max="13.40" step="0.05" required />
              </div>
            </div>
            <div style="font-size:0.75rem; color:var(--text-muted);">
              Cutoff triggered if Cell &le; Cutoff OR Pack &le; Cutoff.<br/>
              Resumes when Cell &ge; Resume AND Pack &ge; Resume.
            </div>
          </div>
        </div>

        <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:0.75rem;">
          <div id="thresh-save-msg" style="font-size:0.85rem; color:var(--emerald); display:none; font-weight:600;">
            ✓ Protection thresholds saved to ESP32 Flash memory!
          </div>
          <button type="submit" class="btn btn-emerald" style="margin-left:auto;">Save Protection Thresholds</button>
        </div>
      </form>
    </div>

    <!-- Wi-Fi Setup Section -->
    <div class="card">
      <div class="card-title"><span>Wi-Fi Network Configuration</span></div>
      <form id="wifi-form" onsubmit="saveWifi(event)" style="display:flex; flex-wrap:wrap; gap:0.75rem;">
        <input type="text" id="wifi-ssid" class="input-field" placeholder="2.4GHz Wi-Fi SSID" style="flex:1; min-width:180px;" required />
        <input type="password" id="wifi-pass" class="input-field" placeholder="Wi-Fi Password" style="flex:1; min-width:180px;" />
        <button type="submit" class="btn btn-emerald" style="white-space:nowrap;">Save &amp; Connect</button>
      </form>
      <div id="wifi-save-msg" style="font-size:0.8rem; color:var(--emerald); display:none;">Credentials saved! ESP32 is reconnecting...</div>
    </div>

    <!-- OTA Firmware Update Card -->
    <div class="card">
      <div class="card-title">
        <span>📡 Wireless Firmware Update (OTA)</span>
        <span class="badge" id="ota-badge">Ready</span>
      </div>
      <form id="ota-form" onsubmit="submitOta(event)">
        <p style="font-size:0.85rem; color:var(--text-muted); margin-bottom:0.75rem;">
          Flash new firmware wirelessly over Wi-Fi without USB cables. In Arduino IDE, click <b>Sketch &rarr; Export Compiled Binary (Ctrl+Alt+S)</b>, then select the <code>.bin</code> file below.
        </p>
        <div style="display:flex; flex-wrap:wrap; gap:0.75rem; align-items:center;">
          <input type="file" id="ota-file" accept=".bin" class="input-field" style="flex:1; min-width:200px;" required />
          <button type="submit" id="ota-btn" class="btn btn-emerald" style="white-space:nowrap;">Flash Firmware</button>
        </div>
        <div id="ota-prog-box" style="margin-top:0.75rem; display:none;">
          <div class="progress-bar-container" style="height:12px;">
            <div class="progress-fill" id="ota-bar" style="width:0%;"></div>
          </div>
          <div style="display:flex; justify-content:space-between; font-size:0.8rem; margin-top:0.35rem;">
            <span id="ota-status" style="color:var(--emerald);">Uploading...</span>
            <span id="ota-pct" style="font-weight:700;">0%</span>
          </div>
        </div>
      </form>
    </div>

    <footer>
      ESP32 Power4S100A BMS Controller &bull; Built with NimBLE &amp; Local Web Server
    </footer>
  </div>

  <script>
    let threshLoaded = false;

    function updateMetrics() {
      fetch('/api/metrics')
        .then(res => res.json())
        .then(d => {
          // BLE Badge
          const bleBadge = document.getElementById('ble-badge');
          const bleText = document.getElementById('ble-text');
          if (d.ble_connected) {
            bleBadge.className = 'badge active';
            bleText.textContent = 'BMS Connected';
          } else {
            bleBadge.className = 'badge warn';
            bleText.textContent = 'BLE Scanning...';
          }

          // Pack Stats
          document.getElementById('pack-voltage').textContent = d.pack_voltage.toFixed(2);
          const curr = d.current;
          document.getElementById('pack-current').textContent = (curr > 0 ? '+' : '') + curr.toFixed(1);
          const currCol = document.getElementById('current-color');
          const currDir = document.getElementById('current-dir');
          if (curr >= 0.3) {
            currCol.style.color = 'var(--emerald)';
            currDir.textContent = 'Charging';
          } else if (curr <= -0.3) {
            currCol.style.color = 'var(--amber)';
            currDir.textContent = 'Discharging';
          } else {
            currCol.style.color = '#fff';
            currDir.textContent = 'Idle / Standby';
          }

          const power = Math.abs(d.pack_voltage * curr);
          document.getElementById('power-stat').textContent = (curr >= 0.3 ? '+' : (curr <= -0.3 ? '-' : '')) + power.toFixed(1) + ' W';
          document.getElementById('power-stat').style.color = curr >= 0.3 ? 'var(--emerald)' : (curr <= -0.3 ? 'var(--amber)' : '#fff');

          // SOC
          const soc = d.soc;
          document.getElementById('soc-text').textContent = soc + '%';
          document.getElementById('soc-bar').style.width = soc + '%';
          if (soc < 20) {
            document.getElementById('soc-bar').style.background = 'var(--red)';
          } else if (soc < 40) {
            document.getElementById('soc-bar').style.background = 'var(--amber)';
          } else {
            document.getElementById('soc-bar').style.background = 'linear-gradient(90deg, #10b981, #3b82f6)';
          }

          // Capacities & Health
          document.getElementById('cap-rem').textContent = (Math.floor(d.remaining_ah * 10) / 10).toFixed(1) + ' Ah';
          document.getElementById('cap-full').textContent = d.total_ah.toFixed(1) + ' Ah';
          document.getElementById('soh-text').textContent = d.soh + '%';
          document.getElementById('cycles-text').textContent = d.cycles;
          document.getElementById('delta-text').textContent = d.delta_mv + ' mV';

          // Battery Condition
          const chgCutV = (d.cfg && d.cfg.chg_cut_pack) ? d.cfg.chg_cut_pack : 14.4;
          const invCutV = (d.cfg && d.cfg.inv_cut_pack) ? d.cfg.inv_cut_pack : 11.2;
          const condText = (d.delta_mv > 80) ? 'Imbalance' : (d.pack_voltage >= chgCutV ? 'Overvoltage' : (d.pack_voltage <= invCutV ? 'Undervoltage' : 'Normal'));
          document.getElementById('batt-condition').textContent = condText;
          document.getElementById('batt-condition').style.color = (condText === 'Normal') ? 'var(--emerald)' : 'var(--amber)';
          document.getElementById('cond-text').textContent = 'Condition: ' + condText;
          const condBadge = document.getElementById('cond-badge');
          condBadge.className = (condText === 'Normal') ? 'badge active' : 'badge warn';

          // Cells
          const cells = d.cells || [0,0,0,0];
          for (let i = 0; i < 4; i++) {
            const mv = cells[i];
            const v = (mv / 1000.0).toFixed(3);
            document.getElementById('c' + (i+1) + '-val').textContent = v + ' V';
            let pct = ((mv - 2500) / (3650 - 2500)) * 100;
            pct = Math.max(0, Math.min(100, pct));
            const bar = document.getElementById('c' + (i+1) + '-bar');
            bar.style.width = pct + '%';
            if (mv <= 2800) bar.style.background = 'var(--red)';
            else if (mv >= 3600) bar.style.background = 'var(--amber)';
            else bar.style.background = 'var(--blue)';
          }

          document.getElementById('min-cell').textContent = (d.min_cell / 1000.0).toFixed(3) + ' V';
          document.getElementById('max-cell').textContent = (d.max_cell / 1000.0).toFixed(3) + ' V';

          const balBadge = document.getElementById('balance-badge');
          if (d.delta_mv <= 30) {
            balBadge.className = 'badge active';
            balBadge.textContent = 'Balanced (' + d.delta_mv + 'mV)';
          } else if (d.delta_mv <= 80) {
            balBadge.className = 'badge warn';
            balBadge.textContent = 'Normal (' + d.delta_mv + 'mV)';
          } else {
            balBadge.className = 'badge danger';
            balBadge.textContent = 'Imbalance (' + d.delta_mv + 'mV)';
          }

          // Thermal Sensors (Max, Min, Ambient, MOS)
          document.getElementById('temp-cell-max').textContent = d.cell_temp_max.toFixed(1) + ' °C';
          document.getElementById('temp-cell-min').textContent = d.cell_temp_min.toFixed(1) + ' °C';
          document.getElementById('temp-amb').textContent = d.ambient_temp.toFixed(1) + ' °C';
          document.getElementById('temp-mos').textContent = d.mos_temp.toFixed(1) + ' °C';

          // Intervals & Timers
          if (document.getElementById('cur-interval')) {
            document.getElementById('cur-interval').textContent = d.cur_charge_interval + ' h';
            document.getElementById('max-interval').textContent = d.max_charge_interval + ' h';
            document.getElementById('rem-charge').textContent = d.rem_charge_min + ' min';
            document.getElementById('rem-discharge').textContent = (d.rem_discharge_min === 65535 || d.rem_discharge_min === 0) ? '- min' : (d.rem_discharge_min + ' min');
          }

          document.getElementById('bms-chg-mos').textContent = d.charging_state ? 'ENABLED' : 'OFF';
          document.getElementById('bms-chg-mos').style.color = d.charging_state ? 'var(--emerald)' : 'var(--red)';
          document.getElementById('bms-dsg-mos').textContent = d.discharging_state ? 'ENABLED' : 'OFF';
          document.getElementById('bms-dsg-mos').style.color = d.discharging_state ? 'var(--emerald)' : 'var(--red)';

          // Relays
          const autoBtn = document.getElementById('auto-btn');
          autoBtn.textContent = 'Auto Mode: ' + (d.auto_mode ? 'ON' : 'OFF');
          autoBtn.style.color = d.auto_mode ? 'var(--emerald)' : 'var(--text-muted)';

          // Charger Relay
          const chgBtn = document.getElementById('chg-relay-btn');
          const chgBadge = document.getElementById('chg-status-badge');
          if (d.charger_relay) {
            chgBtn.textContent = 'Turn OFF';
            chgBtn.className = 'btn btn-danger';
            if (chgBadge) {
              chgBadge.className = 'badge active';
              chgBadge.textContent = 'ON';
            }
          } else {
            chgBtn.textContent = 'Turn ON';
            chgBtn.className = 'btn btn-emerald';
            if (chgBadge) {
              if (d.charger_tripped) {
                chgBadge.className = 'badge danger';
                chgBadge.textContent = 'TRIPPED (Overvoltage)';
              } else {
                chgBadge.className = 'badge';
                chgBadge.textContent = 'OFF';
              }
            }
          }

          // Inverter Relay
          const invBtn = document.getElementById('inv-relay-btn');
          const invBadge = document.getElementById('inv-status-badge');
          if (d.inverter_relay) {
            invBtn.textContent = 'Turn OFF';
            invBtn.className = 'btn btn-danger';
            if (invBadge) {
              invBadge.className = 'badge active';
              invBadge.textContent = 'ON';
            }
          } else {
            invBtn.textContent = 'Turn ON';
            invBtn.className = 'btn btn-emerald';
            if (invBadge) {
              if (d.inverter_tripped) {
                invBadge.className = 'badge danger';
                invBadge.textContent = 'TRIPPED (Undervoltage)';
              } else {
                invBadge.className = 'badge';
                invBadge.textContent = 'OFF';
              }
            }
          }

          // Protection Thresholds & Relays Subtitles
          if (d.cfg) {
            if (!threshLoaded) {
              document.getElementById('cfg-chg-cut-cell').value = d.cfg.chg_cut_cell;
              document.getElementById('cfg-chg-res-cell').value = d.cfg.chg_res_cell;
              document.getElementById('cfg-chg-cut-pack').value = d.cfg.chg_cut_pack.toFixed(2);
              document.getElementById('cfg-chg-res-pack').value = d.cfg.chg_res_pack.toFixed(2);
              document.getElementById('cfg-inv-cut-cell').value = d.cfg.inv_cut_cell;
              document.getElementById('cfg-inv-res-cell').value = d.cfg.inv_res_cell;
              document.getElementById('cfg-inv-cut-pack').value = d.cfg.inv_cut_pack.toFixed(2);
              document.getElementById('cfg-inv-res-pack').value = d.cfg.inv_res_pack.toFixed(2);
              threshLoaded = true;
            }
            const chgThresh = document.getElementById('chg-thresh-text');
            if (chgThresh) {
              chgThresh.innerHTML = 'Cutoff: &ge; ' + d.cfg.chg_cut_pack.toFixed(2) + 'V (' + d.cfg.chg_cut_cell + 'mV) &middot; Resume: &le; ' + d.cfg.chg_res_pack.toFixed(2) + 'V (' + d.cfg.chg_res_cell + 'mV)';
            }
            const invThresh = document.getElementById('inv-thresh-text');
            if (invThresh) {
              invThresh.innerHTML = 'Cutoff: &le; ' + d.cfg.inv_cut_pack.toFixed(2) + 'V (' + d.cfg.inv_cut_cell + 'mV) &middot; Resume: &ge; ' + d.cfg.inv_res_pack.toFixed(2) + 'V (' + d.cfg.inv_res_cell + 'mV)';
            }
          }
        })
        .catch(err => {
          console.error("Metrics fetch error:", err);
        });
    }

    function toggleRelay(target) {
      if (target === 'charger') {
        const btn = document.getElementById('chg-relay-btn');
        if (btn) btn.textContent = 'Switching...';
      } else if (target === 'inverter') {
        const btn = document.getElementById('inv-relay-btn');
        if (btn) btn.textContent = 'Switching...';
      } else if (target === 'auto') {
        const btn = document.getElementById('auto-btn');
        if (btn) btn.textContent = 'Updating...';
      }
      fetch('/api/relay?target=' + target + '&action=toggle', { method: 'POST' })
        .then(res => res.json())
        .then(() => {
          setTimeout(updateMetrics, 150);
        })
        .catch(console.error);
    }

    function saveThresholds(e) {
      e.preventDefault();
      const chgCutCell = parseInt(document.getElementById('cfg-chg-cut-cell').value);
      const chgResCell = parseInt(document.getElementById('cfg-chg-res-cell').value);
      const chgCutPack = parseFloat(document.getElementById('cfg-chg-cut-pack').value);
      const chgResPack = parseFloat(document.getElementById('cfg-chg-res-pack').value);

      const invCutCell = parseInt(document.getElementById('cfg-inv-cut-cell').value);
      const invResCell = parseInt(document.getElementById('cfg-inv-res-cell').value);
      const invCutPack = parseFloat(document.getElementById('cfg-inv-cut-pack').value);
      const invResPack = parseFloat(document.getElementById('cfg-inv-res-pack').value);

      if (chgCutCell <= chgResCell) {
        alert("Charger Overvoltage Cutoff (" + chgCutCell + " mV) must be strictly greater than Resume threshold (" + chgResCell + " mV)!");
        return;
      }
      if (chgCutPack <= chgResPack) {
        alert("Charger Overvoltage Pack Cutoff (" + chgCutPack.toFixed(2) + " V) must be strictly greater than Resume threshold (" + chgResPack.toFixed(2) + " V)!");
        return;
      }
      if (invCutCell >= invResCell) {
        alert("Inverter Undervoltage Cutoff (" + invCutCell + " mV) must be strictly less than Resume threshold (" + invResCell + " mV)!");
        return;
      }
      if (invCutPack >= invResPack) {
        alert("Inverter Undervoltage Pack Cutoff (" + invCutPack.toFixed(2) + " V) must be strictly less than Resume threshold (" + invResPack.toFixed(2) + " V)!");
        return;
      }

      const params = new URLSearchParams({
        chg_cut_cell: chgCutCell,
        chg_res_cell: chgResCell,
        chg_cut_pack: chgCutPack,
        chg_res_pack: chgResPack,
        inv_cut_cell: invCutCell,
        inv_res_cell: invResCell,
        inv_cut_pack: invCutPack,
        inv_res_pack: invResPack
      });

      fetch('/api/thresholds?' + params.toString(), { method: 'POST' })
        .then(res => res.json())
        .then(data => {
          const msg = document.getElementById('thresh-save-msg');
          msg.style.display = 'block';
          setTimeout(() => { msg.style.display = 'none'; }, 5000);
          updateMetrics();
        })
        .catch(err => {
          console.error(err);
          alert("Failed to save thresholds to ESP32: " + err);
        });
    }

    function resetThresholdDefaults() {
      document.getElementById('cfg-chg-cut-cell').value = 3650;
      document.getElementById('cfg-chg-res-cell').value = 3400;
      document.getElementById('cfg-chg-cut-pack').value = "14.40";
      document.getElementById('cfg-chg-res-pack').value = "13.60";
      document.getElementById('cfg-inv-cut-cell').value = 2800;
      document.getElementById('cfg-inv-res-cell').value = 3100;
      document.getElementById('cfg-inv-cut-pack').value = "11.20";
      document.getElementById('cfg-inv-res-pack').value = "12.50";
    }

    function saveWifi(e) {
      e.preventDefault();
      const ssid = document.getElementById('wifi-ssid').value;
      const pass = document.getElementById('wifi-pass').value;
      fetch('/api/wifi?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass), { method: 'POST' })
        .then(() => {
          document.getElementById('wifi-save-msg').style.display = 'block';
          setTimeout(() => { document.getElementById('wifi-save-msg').style.display = 'none'; }, 5000);
        })
        .catch(console.error);
    }

    function submitOta(e) {
      e.preventDefault();
      const fileInput = document.getElementById('ota-file');
      if (!fileInput.files.length) return;
      const file = fileInput.files[0];
      if (!file.name.endsWith('.bin')) {
        alert("Please select a compiled .bin file!");
        return;
      }
      if (!confirm("Are you sure you want to flash '" + file.name + "' wirelessly? The ESP32 will reboot automatically after update.")) {
        return;
      }

      const progBox = document.getElementById('ota-prog-box');
      const bar = document.getElementById('ota-bar');
      const status = document.getElementById('ota-status');
      const pct = document.getElementById('ota-pct');
      const btn = document.getElementById('ota-btn');
      const badge = document.getElementById('ota-badge');

      progBox.style.display = 'block';
      btn.disabled = true;
      btn.textContent = 'Flashing...';
      badge.className = 'badge warn';
      badge.textContent = 'Flashing OTA...';

      const formData = new FormData();
      formData.append('update', file);

      const xhr = new XMLHttpRequest();
      xhr.open('POST', '/update', true);

      xhr.upload.onprogress = function(evt) {
        if (evt.lengthComputable) {
          const p = Math.round((evt.loaded / evt.total) * 100);
          bar.style.width = p + '%';
          pct.textContent = p + '%';
          status.textContent = 'Uploading: ' + (evt.loaded / 1024).toFixed(0) + ' / ' + (evt.total / 1024).toFixed(0) + ' KB';
        }
      };

      xhr.onload = function() {
        if (xhr.status === 200) {
          bar.style.width = '100%';
          pct.textContent = '100%';
          status.textContent = 'Firmware flashed successfully! ESP32 is rebooting... Page will reload in 10s.';
          badge.className = 'badge active';
          badge.textContent = 'Rebooting...';
          setTimeout(() => { window.location.reload(); }, 10000);
        } else {
          status.textContent = 'OTA Error: ' + xhr.responseText;
          status.style.color = 'var(--red)';
          badge.className = 'badge danger';
          badge.textContent = 'Failed';
          btn.disabled = false;
          btn.textContent = 'Flash Firmware';
        }
      };

      xhr.onerror = function() {
        status.textContent = 'Network error during OTA upload!';
        status.style.color = 'var(--red)';
        badge.className = 'badge danger';
        badge.textContent = 'Network Error';
        btn.disabled = false;
        btn.textContent = 'Flash Firmware';
      };

      xhr.send(formData);
    }

    setInterval(updateMetrics, 1000);
    updateMetrics();
  </script>
</body>
</html>
)rawliteral";

// =============================================================================
// 7. RELAY CONTROLLER IMPLEMENTATION
// =============================================================================
void writeHardwareRelay(uint8_t pin, bool state) {
#if RELAY_ACTIVE_LOW
  digitalWrite(pin, state ? LOW : HIGH);
#else
  digitalWrite(pin, state ? HIGH : LOW);
#endif
}

void setChargerRelay(bool enable, bool manual) {
  if (g_chargerRelay != enable) {
    g_chargerRelay = enable;
    writeHardwareRelay(RELAY_CHARGER_PIN, g_chargerRelay);
    Serial.printf("[Relay] Charger turned %s %s\n", enable ? "ON" : "OFF", manual ? "(Manual)" : "(Auto)");
  }
}

void setInverterRelay(bool enable, bool manual) {
  if (g_inverterRelay != enable) {
    g_inverterRelay = enable;
    writeHardwareRelay(RELAY_INVERTER_PIN, g_inverterRelay);
    Serial.printf("[Relay] Inverter turned %s %s\n", enable ? "ON" : "OFF", manual ? "(Manual)" : "(Auto)");
  }
}

// =============================================================================
// NVS PREFERENCES FOR PROTECTION THRESHOLDS
// =============================================================================
void loadProtectionConfig() {
  g_prefs.begin("bms_cfg", false);
  g_cfg.cellChgCutoffMv = g_prefs.getUShort("chg_cut_cell", DEFAULT_CHG_CUT_CELL_MV);
  g_cfg.cellChgResumeMv = g_prefs.getUShort("chg_res_cell", DEFAULT_CHG_RES_CELL_MV);
  g_cfg.packChgCutoffV  = g_prefs.getFloat("chg_cut_pack", DEFAULT_CHG_CUT_PACK_V);
  g_cfg.packChgResumeV  = g_prefs.getFloat("chg_res_pack", DEFAULT_CHG_RES_PACK_V);

  g_cfg.cellInvCutoffMv = g_prefs.getUShort("inv_cut_cell", DEFAULT_INV_CUT_CELL_MV);
  g_cfg.cellInvResumeMv = g_prefs.getUShort("inv_res_cell", DEFAULT_INV_RES_CELL_MV);
  g_cfg.packInvCutoffV  = g_prefs.getFloat("inv_cut_pack", DEFAULT_INV_CUT_PACK_V);
  g_cfg.packInvResumeV  = g_prefs.getFloat("inv_res_pack", DEFAULT_INV_RES_PACK_V);
  g_prefs.end();

  Serial.println("[Config] Loaded Protection Thresholds from Flash NVS:");
  Serial.printf("  Charger: Cutoff >= %umV / %.2fV | Resume <= %umV / %.2fV\n",
                g_cfg.cellChgCutoffMv, g_cfg.packChgCutoffV,
                g_cfg.cellChgResumeMv, g_cfg.packChgResumeV);
  Serial.printf("  Inverter: Cutoff <= %umV / %.2fV | Resume >= %umV / %.2fV\n",
                g_cfg.cellInvCutoffMv, g_cfg.packInvCutoffV,
                g_cfg.cellInvResumeMv, g_cfg.packInvResumeV);
}

void saveProtectionConfig() {
  g_prefs.begin("bms_cfg", false);
  g_prefs.putUShort("chg_cut_cell", g_cfg.cellChgCutoffMv);
  g_prefs.putUShort("chg_res_cell", g_cfg.cellChgResumeMv);
  g_prefs.putFloat("chg_cut_pack", g_cfg.packChgCutoffV);
  g_prefs.putFloat("chg_res_pack", g_cfg.packChgResumeV);

  g_prefs.putUShort("inv_cut_cell", g_cfg.cellInvCutoffMv);
  g_prefs.putUShort("inv_res_cell", g_cfg.cellInvResumeMv);
  g_prefs.putFloat("inv_cut_pack", g_cfg.packInvCutoffV);
  g_prefs.putFloat("inv_res_pack", g_cfg.packInvResumeV);
  g_prefs.end();

  Serial.println("[Config] Saved new Protection Thresholds to Flash NVS.");
}

void evaluateAutoProtection() {
  if (!g_autoMode) return;
  if (g_bms.cellVoltages[0] == 0) return; // Wait until initial telemetry arrives

  // Safety Timeout: If no BLE packet for >15s, disengage charger
  if (millis() - g_bms.lastUpdateMs > 15000 && g_bms.lastUpdateMs > 0) {
    if (g_chargerRelay) {
      Serial.println("[Safety] BMS telemetry stale (>15s)! Shutting off Charger.");
      setChargerRelay(false);
      g_chargerTripped = true;
    }
    return;
  }

  // 1. CHARGER OVERVOLTAGE CUTOFF & RESUME
  if (g_bms.maxCellMv >= g_cfg.cellChgCutoffMv || g_bms.packVoltage >= g_cfg.packChgCutoffV) {
    if (g_chargerRelay) {
      Serial.printf("[Safety Cutoff] Overvoltage! Max Cell: %umV (limit %u), Pack: %.2fV (limit %.2f). Stopping Charger.\n", 
                    g_bms.maxCellMv, g_cfg.cellChgCutoffMv, g_bms.packVoltage, g_cfg.packChgCutoffV);
      setChargerRelay(false);
      g_chargerTripped = true;
    }
  } else if (g_chargerTripped) {
    // Only automatically resume if it was previously tripped by safety!
    if (g_bms.maxCellMv <= g_cfg.cellChgResumeMv && g_bms.packVoltage <= g_cfg.packChgResumeV) {
      Serial.printf("[Safety Restore] Voltage settled below resume threshold (%umV / %.2fV). Resuming Charger.\n",
                    g_cfg.cellChgResumeMv, g_cfg.packChgResumeV);
      setChargerRelay(true);
      g_chargerTripped = false;
    }
  }

  // 2. INVERTER UNDERVOLTAGE CUTOFF & RESUME
  if (g_bms.minCellMv <= g_cfg.cellInvCutoffMv || g_bms.packVoltage <= g_cfg.packInvCutoffV) {
    if (g_inverterRelay) {
      Serial.printf("[Safety Cutoff] Undervoltage! Min Cell: %umV (limit %u), Pack: %.2fV (limit %.2f). Cutting Inverter Load!\n", 
                    g_bms.minCellMv, g_cfg.cellInvCutoffMv, g_bms.packVoltage, g_cfg.packInvCutoffV);
      setInverterRelay(false);
      g_inverterTripped = true;
    }
  } else if (g_inverterTripped) {
    // Only automatically resume if it was previously tripped by safety!
    if (g_bms.minCellMv >= g_cfg.cellInvResumeMv && g_bms.packVoltage >= g_cfg.packInvResumeV) {
      Serial.printf("[Safety Restore] Battery recharged above safe threshold (%umV / %.2fV). Re-enabling Inverter Load.\n",
                    g_cfg.cellInvResumeMv, g_cfg.packInvResumeV);
      setInverterRelay(true);
      g_inverterTripped = false;
    }
  }
}

// =============================================================================
// 8. BLE SCANNING, CLIENT & BMS FRAME PARSING
// =============================================================================
void bmsNotifyCallback(NimBLERemoteCharacteristic* pRemoteCharacteristic, 
                        uint8_t* pData, size_t length, bool isNotify) {
  if (length < 8 || pData[0] != 0xEA || pData[1] != 0xD1) return;

  uint8_t frameType = pData[5];

  // Frame 0x02: Individual Cell Voltages (19 bytes)
  if (frameType == 0x02 && length >= 17) {
    g_bms.cellVoltages[0] = (uint16_t(pData[9]) << 8)  | pData[10];
    g_bms.cellVoltages[1] = (uint16_t(pData[11]) << 8) | pData[12];
    g_bms.cellVoltages[2] = (uint16_t(pData[13]) << 8) | pData[14];
    g_bms.cellVoltages[3] = (uint16_t(pData[15]) << 8) | pData[16];

    uint16_t minMv = g_bms.cellVoltages[0];
    uint16_t maxMv = g_bms.cellVoltages[0];
    uint32_t sumMv = 0;

    for (int i = 0; i < 4; i++) {
      uint16_t v = g_bms.cellVoltages[i];
      if (v < minMv) minMv = v;
      if (v > maxMv) maxMv = v;
      sumMv += v;
    }

    g_bms.minCellMv = minMv;
    g_bms.maxCellMv = maxMv;
    g_bms.diffCellMv = maxMv - minMv;
    g_bms.packVoltage = sumMv / 1000.0f;
    g_bms.lastUpdateMs = millis();

    Serial.printf("[BMS Cells] C1: %u | C2: %u | C3: %u | C4: %u mV | Pack: %.2fV | Delta: %u mV\n",
                  g_bms.cellVoltages[0], g_bms.cellVoltages[1],
                  g_bms.cellVoltages[2], g_bms.cellVoltages[3],
                  g_bms.packVoltage, g_bms.diffCellMv);
  }

  // ---------------------------------------------------------------------------
  // Frame 0x03: Temperatures & Protection Status (29 bytes)
  // ---------------------------------------------------------------------------
  else if (frameType == 0x03 && length >= 17) {
    uint8_t numProbes = pData[13];
    // Formula: Temperature (°C) = Raw_Byte - 40
    if (numProbes >= 1 && pData[14] >= 40 && pData[14] <= 140) {
      g_bms.cellTemp = float(pData[14]) - 40.0f;
      g_bms.cellTempMin = g_bms.cellTemp;
      g_bms.cellTempMax = g_bms.cellTemp;
    }
    if (numProbes >= 2 && pData[15] >= 40 && pData[15] <= 140) {
      g_bms.ambientTemp = float(pData[15]) - 40.0f;
    }
    if (numProbes >= 3 && pData[16] >= 40 && pData[16] <= 140) {
      g_bms.mosTemp = float(pData[16]) - 40.0f;
    }

    // PACK CURRENT (Frame 0x03, Bytes [7..8]):
    // Unit is centiamperes (0.01A units -> 100 units = 1A)
    // Positive = Charging, Negative = Discharging
    uint16_t uCur = (uint16_t(pData[7]) << 8) | pData[8];
    if (uCur == 0) {
      g_bms.current = 0.0f;
    } else if (uCur & 0x8000) {
      // Negative / Discharging current
      int16_t sCur = (int16_t)uCur;
      if (sCur >= -15000 && sCur < 0) {
        g_bms.current = float(sCur) / 100.0f;
      } else {
        g_bms.current = -float(uCur & 0x7FFF) / 100.0f;
      }
    } else {
      // Positive / Charging current (e.g. 1000 -> 10.00A, 1200 -> 12.00A)
      g_bms.current = float(uCur) / 100.0f;
    }

    // Deadband filter to suppress noise near zero
    if (fabs(g_bms.current) < 0.15f) {
      g_bms.current = 0.0f;
    }

    g_bms.lastUpdateMs = millis();
    Serial.printf("[BMS Status 0x03] Current: %.2fA (Raw 7-8: 0x%02X%02X) | Cell Max: %.1fC | Cell Min: %.1fC | Ambient: %.1fC | MOS: %.1fC\n",
                  g_bms.current, pData[7], pData[8], g_bms.cellTempMax, g_bms.cellTempMin, g_bms.ambientTemp, g_bms.mosTemp);
  }

  // ---------------------------------------------------------------------------
  // Frame 0x04: Gauges & Metrics (Cycle Count, SOC, SOH, Capacities, Intervals)
  // ---------------------------------------------------------------------------
  else if (frameType == 0x04 && length >= 50) {
    // 1. SOH (Tag 0x01, byte 7)
    if (pData[6] == 0x01 && pData[7] > 0 && pData[7] <= 100) {
      g_bms.soh = pData[7];
    }

    // 2. Dynamic Cycle Count (Tag 0x02, bytes [9..10])
    if (pData[8] == 0x02) {
      uint16_t cycles = (uint16_t(pData[9]) << 8) | pData[10];
      if (cycles > 0) g_bms.cycleCount = cycles;
    }

    // 3. MOSFET switches
    g_bms.dischargingState = (pData[13] != 0); // Tag 0x03
    g_bms.chargingState    = (pData[19] != 0); // Tag 0x05

    // 4. Capacities (32-bit mAh -> Ah)
    // Full Capacity: [12..13] (high) & [15..16] (low)
    uint32_t fullMah = (uint32_t(pData[12]) << 24) | (uint32_t(pData[13]) << 16) | (uint32_t(pData[15]) << 8) | pData[16];
    if (fullMah >= 1000 && fullMah <= 500000) {
      g_bms.totalCapacityAh = fullMah / 1000.0f;
    } else {
      g_bms.totalCapacityAh = 100.0f;
    }

    // Remaining Capacity: [18..19] (high) & [21..22] (low)
    uint32_t remMah = (uint32_t(pData[18]) << 24) | (uint32_t(pData[19]) << 16) | (uint32_t(pData[21]) << 8) | pData[22];
    if (remMah > 0 && remMah <= 500000) {
      // SuperBMS uses integer tenths truncation (99960 mAh -> 99.9 Ah, not rounded to 100.0)
      g_bms.remainingCapacityAh = float(remMah / 100) / 10.0f;
    } else {
      g_bms.remainingCapacityAh = 99.9f;
    }

    // Accurate SOC calculated from remaining / full Ah
    if (g_bms.totalCapacityAh > 0) {
      float calcSoc = (g_bms.remainingCapacityAh / g_bms.totalCapacityAh) * 100.0f;
      g_bms.soc = (uint8_t)constrain((int)round(calcSoc), 0, 100);
    }

    // 5. Charging Intervals: Current & Maximum
    g_bms.curChargeIntervalH = (uint16_t(pData[36]) << 8) | pData[37]; // e.g. 1h
    g_bms.maxChargeIntervalH = (uint16_t(pData[38]) << 8) | pData[39]; // e.g. 372h

    // 6. Timers: Remaining Discharge & Charge time
    g_bms.remDischargeMin = (uint16_t(pData[30]) << 8) | pData[31];
    g_bms.remChargeMin    = (uint16_t(pData[33]) << 8) | pData[34]; // Tag 0x0A: remChargeMin

    g_bms.lastUpdateMs = millis();

    Serial.printf("[BMS Pack 0x04] SOC: %u%% | SOH: %u%% | Current: %.2fA | Cap: %.1f/%.1fAh | Cycles: %u | Chg Int: %uh (Max %uh)\n",
                  g_bms.soc, g_bms.soh, g_bms.current,
                  g_bms.remainingCapacityAh, g_bms.totalCapacityAh,
                  g_bms.cycleCount,
                  g_bms.curChargeIntervalH, g_bms.maxChargeIntervalH);
  }
}

class BmsScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
    bool matchName = advertisedDevice->haveName() && advertisedDevice->getName() == BMS_DEVICE_NAME;
    bool matchMac = (strlen(BMS_MAC_ADDRESS) > 0) &&
                    (strcasecmp(advertisedDevice->getAddress().toString().c_str(), BMS_MAC_ADDRESS) == 0);

    if (matchName || matchMac) {
      Serial.printf("[BLE] Discovered BMS Target: %s [%s] (RSSI: %d dBm)\n", 
                    advertisedDevice->getName().c_str(), 
                    advertisedDevice->getAddress().toString().c_str(),
                    advertisedDevice->getRSSI());
      NimBLEDevice::getScan()->stop();
      g_advDevice = const_cast<NimBLEAdvertisedDevice*>(advertisedDevice);
      g_doConnect = true;
    }
  }

  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    onResult(const_cast<const NimBLEAdvertisedDevice*>(advertisedDevice));
  }
};

class BmsClientCallbacks : public NimBLEClientCallbacks {
public:
  void onConnect(NimBLEClient* pClient) {
    Serial.println("[BLE] Connected to BMS successfully!");
  }
  void onDisconnect(NimBLEClient* pClient) {
    Serial.println("[BLE] Disconnected from BMS");
    g_bms.connected = false;
    g_advDevice = nullptr;
  }
  void onDisconnect(NimBLEClient* pClient, int reason) {
    Serial.printf("[BLE] Disconnected from BMS, reason: %d\n", reason);
    g_bms.connected = false;
    g_advDevice = nullptr;
  }
};

bool connectToBms() {
  if (!g_client) {
    g_client = NimBLEDevice::createClient();
    g_client->setClientCallbacks(new BmsClientCallbacks(), false);
    g_client->setConnectionParams(12, 24, 0, 200); // Fast connection interval
    g_client->setConnectTimeout(4000);              // 4 second timeout
  }

  bool connected = false;

  // 1. If device was found during scan, connect directly to it
  if (g_advDevice) {
    Serial.printf("[BLE] Connecting to discovered BMS: %s [%s]...\n",
                  g_advDevice->getName().c_str(),
                  g_advDevice->getAddress().toString().c_str());
    connected = g_client->connect(g_advDevice);
  }
  // 2. Direct connect using hardcoded MAC address (Instant, skips scan!)
  else if (strlen(BMS_MAC_ADDRESS) > 0) {
    Serial.printf("[BLE] Direct connect to BMS MAC: %s...\n", BMS_MAC_ADDRESS);
    NimBLEAddress bmsAddr(std::string(BMS_MAC_ADDRESS), 0);
    connected = g_client->connect(bmsAddr);
  }

  if (!connected) {
    Serial.println("[BLE] Connection attempt failed");
    return false;
  }

  Serial.println("[BLE] Connected! Discovering GATT services...");

  NimBLERemoteService* pService = g_client->getService(BMS_SERVICE_UUID);
  if (!pService) {
    Serial.println("[BLE] BMS Service 0xFFE0 not found");
    g_client->disconnect();
    return false;
  }

  g_pChr = pService->getCharacteristic(BMS_CHARACTERISTIC_UUID);
  if (!g_pChr) {
    Serial.println("[BLE] Characteristic 0xFFE1 not found");
    g_client->disconnect();
    return false;
  }

  if (g_pChr->canNotify()) {
    if (!g_pChr->subscribe(true, bmsNotifyCallback)) {
      Serial.println("[BLE] Failed to subscribe to notifications");
      g_client->disconnect();
      return false;
    }
    Serial.println("[BLE] Subscribed to BMS telemetry notifications!");
  }

  g_bms.connected = true;
  return true;
}

void requestMetrics() {
  if (!g_bms.connected || !g_pChr) return;

  static uint8_t reqCycle = 0;
  if (reqCycle % 3 == 0) {
    g_pChr->writeValue(CMD_CELLS, sizeof(CMD_CELLS), false);       // Frame 0x02: Cell Voltages
  } else if (reqCycle % 3 == 1) {
    g_pChr->writeValue(CMD_STATUS, sizeof(CMD_STATUS), false);     // Frame 0x03: Temperatures & Protection
  } else {
    g_pChr->writeValue(CMD_METRICS, sizeof(CMD_METRICS), false);   // Frame 0x04: Gauges, Current & Cycles
  }
  reqCycle++;
}

// =============================================================================
// 9. WEB SERVER ROUTE HANDLERS
// =============================================================================
void handleMetricsApi() {
  char json[1024];
  snprintf(json, sizeof(json),
    "{"
    "\"ble_connected\":%s,"
    "\"pack_voltage\":%.2f,"
    "\"current\":%.1f,"
    "\"soc\":%u,"
    "\"soh\":%u,"
    "\"remaining_ah\":%.1f,"
    "\"total_ah\":%.1f,"
    "\"cycles\":%u,"
    "\"cell_temp\":%.1f,"
    "\"cell_temp_min\":%.1f,"
    "\"cell_temp_max\":%.1f,"
    "\"ambient_temp\":%.1f,"
    "\"mos_temp\":%.1f,"
    "\"cur_charge_interval\":%u,"
    "\"max_charge_interval\":%u,"
    "\"rem_charge_min\":%u,"
    "\"rem_discharge_min\":%u,"
    "\"cells\":[%u,%u,%u,%u],"
    "\"min_cell\":%u,"
    "\"max_cell\":%u,"
    "\"delta_mv\":%u,"
    "\"charging_state\":%s,"
    "\"discharging_state\":%s,"
    "\"charger_relay\":%s,"
    "\"charger_tripped\":%s,"
    "\"inverter_relay\":%s,"
    "\"inverter_tripped\":%s,"
    "\"auto_mode\":%s,"
    "\"cfg\":{"
      "\"chg_cut_cell\":%u,"
      "\"chg_res_cell\":%u,"
      "\"chg_cut_pack\":%.2f,"
      "\"chg_res_pack\":%.2f,"
      "\"inv_cut_cell\":%u,"
      "\"inv_res_cell\":%u,"
      "\"inv_cut_pack\":%.2f,"
      "\"inv_res_pack\":%.2f"
    "}"
    "}",
    g_bms.connected ? "true" : "false",
    g_bms.packVoltage,
    g_bms.current,
    g_bms.soc,
    g_bms.soh,
    g_bms.remainingCapacityAh,
    g_bms.totalCapacityAh,
    (unsigned int)g_bms.cycleCount,
    g_bms.cellTemp,
    g_bms.cellTempMin,
    g_bms.cellTempMax,
    g_bms.ambientTemp,
    g_bms.mosTemp,
    g_bms.curChargeIntervalH,
    g_bms.maxChargeIntervalH,
    g_bms.remChargeMin,
    g_bms.remDischargeMin,
    g_bms.cellVoltages[0],
    g_bms.cellVoltages[1],
    g_bms.cellVoltages[2],
    g_bms.cellVoltages[3],
    g_bms.minCellMv,
    g_bms.maxCellMv,
    g_bms.diffCellMv,
    g_bms.chargingState ? "true" : "false",
    g_bms.dischargingState ? "true" : "false",
    g_chargerRelay ? "true" : "false",
    g_chargerTripped ? "true" : "false",
    g_inverterRelay ? "true" : "false",
    g_inverterTripped ? "true" : "false",
    g_autoMode ? "true" : "false",
    g_cfg.cellChgCutoffMv,
    g_cfg.cellChgResumeMv,
    g_cfg.packChgCutoffV,
    g_cfg.packChgResumeV,
    g_cfg.cellInvCutoffMv,
    g_cfg.cellInvResumeMv,
    g_cfg.packInvCutoffV,
    g_cfg.packInvResumeV
  );

  g_server.sendHeader("Access-Control-Allow-Origin", "*");
  g_server.send(200, "application/json", json);
}

void handleThresholdsApi() {
  if (g_server.method() == HTTP_POST) {
    if (g_server.hasArg("chg_cut_cell")) g_cfg.cellChgCutoffMv = g_server.arg("chg_cut_cell").toInt();
    if (g_server.hasArg("chg_res_cell")) g_cfg.cellChgResumeMv = g_server.arg("chg_res_cell").toInt();
    if (g_server.hasArg("chg_cut_pack")) g_cfg.packChgCutoffV  = g_server.arg("chg_cut_pack").toFloat();
    if (g_server.hasArg("chg_res_pack")) g_cfg.packChgResumeV  = g_server.arg("chg_res_pack").toFloat();

    if (g_server.hasArg("inv_cut_cell")) g_cfg.cellInvCutoffMv = g_server.arg("inv_cut_cell").toInt();
    if (g_server.hasArg("inv_res_cell")) g_cfg.cellInvResumeMv = g_server.arg("inv_res_cell").toInt();
    if (g_server.hasArg("inv_cut_pack")) g_cfg.packInvCutoffV  = g_server.arg("inv_cut_pack").toFloat();
    if (g_server.hasArg("inv_res_pack")) g_cfg.packInvResumeV  = g_server.arg("inv_res_pack").toFloat();

    // LiFePO4 4S Safety boundaries
    g_cfg.cellChgCutoffMv = constrain(g_cfg.cellChgCutoffMv, 3400, 3700);
    g_cfg.cellChgResumeMv = constrain(g_cfg.cellChgResumeMv, 3200, g_cfg.cellChgCutoffMv - 50);

    if (g_cfg.packChgCutoffV < 13.60f) g_cfg.packChgCutoffV = 13.60f;
    if (g_cfg.packChgCutoffV > 14.80f) g_cfg.packChgCutoffV = 14.80f;
    if (g_cfg.packChgResumeV < 13.00f) g_cfg.packChgResumeV = 13.00f;
    if (g_cfg.packChgResumeV > g_cfg.packChgCutoffV - 0.20f) g_cfg.packChgResumeV = g_cfg.packChgCutoffV - 0.20f;

    g_cfg.cellInvCutoffMv = constrain(g_cfg.cellInvCutoffMv, 2400, 3100);
    g_cfg.cellInvResumeMv = constrain(g_cfg.cellInvResumeMv, g_cfg.cellInvCutoffMv + 50, 3400);

    if (g_cfg.packInvCutoffV < 10.00f) g_cfg.packInvCutoffV = 10.00f;
    if (g_cfg.packInvCutoffV > 12.40f) g_cfg.packInvCutoffV = 12.40f;
    if (g_cfg.packInvResumeV < g_cfg.packInvCutoffV + 0.30f) g_cfg.packInvResumeV = g_cfg.packInvCutoffV + 0.30f;
    if (g_cfg.packInvResumeV > 13.40f) g_cfg.packInvResumeV = 13.40f;

    saveProtectionConfig();
  }

  char json[320];
  snprintf(json, sizeof(json),
    "{"
    "\"status\":\"ok\","
    "\"chg_cut_cell\":%u,"
    "\"chg_res_cell\":%u,"
    "\"chg_cut_pack\":%.2f,"
    "\"chg_res_pack\":%.2f,"
    "\"inv_cut_cell\":%u,"
    "\"inv_res_cell\":%u,"
    "\"inv_cut_pack\":%.2f,"
    "\"inv_res_pack\":%.2f"
    "}",
    g_cfg.cellChgCutoffMv,
    g_cfg.cellChgResumeMv,
    g_cfg.packChgCutoffV,
    g_cfg.packChgResumeV,
    g_cfg.cellInvCutoffMv,
    g_cfg.cellInvResumeMv,
    g_cfg.packInvCutoffV,
    g_cfg.packInvResumeV
  );
  g_server.sendHeader("Access-Control-Allow-Origin", "*");
  g_server.send(200, "application/json", json);
}

void handleRelayApi() {
  String target = g_server.arg("target");
  String action = g_server.arg("action");

  Serial.printf("[WebAPI] Relay control: target='%s', action='%s'\n", target.c_str(), action.c_str());

  if (target == "charger") {
    bool next = (action == "toggle") ? !g_chargerRelay : (action == "on" || action == "1");
    setChargerRelay(next, true);
    g_chargerTripped = false; // Reset trip flag on manual action
  } else if (target == "inverter") {
    bool next = (action == "toggle") ? !g_inverterRelay : (action == "on" || action == "1");
    setInverterRelay(next, true);
    g_inverterTripped = false; // Reset trip flag on manual action
  } else if (target == "auto") {
    bool next = (action == "toggle") ? !g_autoMode : (action == "on" || action == "1");
    g_autoMode = next;
    if (g_autoMode) {
      g_chargerTripped = false;
      g_inverterTripped = false;
    }
    Serial.printf("[Auto] Auto cutoff mode set to: %s\n", g_autoMode ? "ENABLED" : "DISABLED");
  }

  g_server.sendHeader("Access-Control-Allow-Origin", "*");
  g_server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleWifiConfigApi() {
  String ssid = g_server.arg("ssid");
  String pass = g_server.arg("pass");

  if (ssid.length() > 0) {
    g_prefs.begin("bms_wifi", false);
    g_prefs.putString("ssid", ssid);
    g_prefs.putString("pass", pass);
    g_prefs.end();

    Serial.printf("[Wi-Fi] Saved credentials for '%s'. Reconnecting...\n", ssid.c_str());
    g_server.sendHeader("Access-Control-Allow-Origin", "*");
    g_server.send(200, "application/json", "{\"status\":\"saved\"}");

    delay(500);
    WiFi.disconnect();
    WiFi.begin(ssid.c_str(), pass.c_str());
  } else {
    g_server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"SSID required\"}");
  }
}

// =============================================================================
// 10. SETUP & INITIALIZATION
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=======================================================");
  Serial.println("  ESP32 Smart BMS & Inverter Controller Starting...    ");
  Serial.println("  Pack: LiFePO4 12.8V 4S100A (Power4S100A)             ");
  Serial.println("  Dashboard: Local Web Server & mDNS (http://bms.local)");
  Serial.println("=======================================================\n");

  // 1. Hardware Pins
  pinMode(BUTTON_RESET_PIN, INPUT_PULLUP);
  pinMode(RELAY_CHARGER_PIN, OUTPUT);
  pinMode(RELAY_INVERTER_PIN, OUTPUT);
  writeHardwareRelay(RELAY_CHARGER_PIN, g_chargerRelay);
  writeHardwareRelay(RELAY_INVERTER_PIN, g_inverterRelay);
  Serial.println("[Relay] Initialized Charger (GPIO 18) and Inverter (GPIO 19)");

  // 2. Wi-Fi Configuration
  g_prefs.begin("bms_wifi", false);
  String savedSsid = g_prefs.getString("ssid", DEFAULT_STA_SSID);
  String savedPass = g_prefs.getString("pass", DEFAULT_STA_PASS);
  g_prefs.end();

  if (savedSsid.length() == 0 && strlen(DEFAULT_STA_SSID) > 0) {
    savedSsid = DEFAULT_STA_SSID;
    savedPass = DEFAULT_STA_PASS;
  }

  WiFi.mode(WIFI_AP_STA);

  // SoftAP broadcast
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println("-------------------------------------------------------");
  Serial.printf("[Wi-Fi AP] SoftAP SSID: %s | Pass: %s\n", AP_SSID, AP_PASS);
  Serial.printf("[Wi-Fi AP] Direct Web URL: http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("-------------------------------------------------------");

  // Connect to Station if SSID is stored
  if (savedSsid.length() > 0) {
    Serial.printf("[Wi-Fi STA] Connecting to '%s'...\n", savedSsid.c_str());
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    uint32_t startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 8000) {
      delay(300);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      g_staConnected = true;
      Serial.println("\n[Wi-Fi STA] Connected successfully!");
      Serial.printf("[Wi-Fi STA] Local IP: http://%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\n[Wi-Fi STA] Connection timed out. Continuing with SoftAP.");
    }
  }

  // Start mDNS (http://bms.local)
  if (MDNS.begin(MDNS_HOST)) {
    Serial.printf("[mDNS] http://%s.local\n", MDNS_HOST);
    MDNS.addService("http", "tcp", WEB_PORT);
  }

  // 3. Load Protection Thresholds from NVS
  loadProtectionConfig();

  // 4. Start Web Server
  g_server.on("/", HTTP_GET, []() {
    g_server.send_P(200, "text/html", PAGE_INDEX);
  });
  g_server.on("/api/metrics", HTTP_GET, handleMetricsApi);
  g_server.on("/api/thresholds", HTTP_POST, handleThresholdsApi);
  g_server.on("/api/thresholds", HTTP_GET, handleThresholdsApi);
  g_server.on("/api/relay", HTTP_POST, handleRelayApi);
  g_server.on("/api/relay", HTTP_GET, handleRelayApi);
  g_server.on("/api/wifi", HTTP_POST, handleWifiConfigApi);

  // Web Browser OTA Handler
  g_server.on("/update", HTTP_POST, []() {
    g_server.sendHeader("Connection", "close");
    g_server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = g_server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[Web OTA] Firmware Update Start: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[Web OTA] Firmware Update Success: %u bytes. Rebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  g_server.onNotFound([]() {
    g_server.send(404, "text/plain", "404 Not Found");
  });
  g_server.begin();
  Serial.printf("[Web Server] Running on port %d\n", WEB_PORT);

  // 5. Initialize ArduinoOTA for direct wireless flashing from Arduino IDE
  ArduinoOTA.setHostname(MDNS_HOST);
  ArduinoOTA.onStart([]() {
    Serial.println("\n[ArduinoOTA] Wireless firmware update started from Arduino IDE...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[ArduinoOTA] Firmware update complete! Rebooting...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[ArduinoOTA] Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[ArduinoOTA] Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("[ArduinoOTA] Network port ready (bms.local)");

  // 6. Start NimBLE Client
  Serial.println("[BLE] Initializing NimBLE Device...");
  NimBLEDevice::init("ESP32_BMS_Host");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new BmsScanCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);

  if (strlen(BMS_MAC_ADDRESS) > 0) {
    Serial.printf("[BLE] Target MAC: %s configured. Bypassing scan & connecting directly...\n\n", BMS_MAC_ADDRESS);
    g_doConnect = true; // Connect instantly on boot!
  } else {
    Serial.println("[BLE] Scanner started. Scanning for Power4S100A BMS...\n");
    pScan->start(4, false);
  }
}

// =============================================================================
// 11. MAIN LOOP
// =============================================================================
void loop() {
  // 1. Handle Web Server Requests
  g_server.handleClient();

  // 2. Handle ArduinoOTA Requests (from Arduino IDE)
  ArduinoOTA.handle();

  // 3. BLE Connection & Scanning
  if (g_doConnect) {
    g_doConnect = false;
    if (connectToBms()) {
      Serial.println("[BLE] BMS connection active & subscribed to notifications!");
    } else {
      Serial.println("[BLE] Direct connect failed, starting background scan...");
      NimBLEDevice::getScan()->start(4, false);
    }
  }

  if (g_bms.connected) {
    if (millis() - g_lastPollMs >= BMS_POLL_INTERVAL_MS) {
      g_lastPollMs = millis();
      requestMetrics();
    }
  } else {
    if (millis() - g_lastScanMs >= 4000 && !NimBLEDevice::getScan()->isScanning()) {
      g_lastScanMs = millis();
      if (strlen(BMS_MAC_ADDRESS) > 0) {
        g_doConnect = true; // Retry direct connection
      } else {
        NimBLEDevice::getScan()->start(4, false);
      }
    }
  }

  // 3. Evaluate Battery Cutoff Logic
  evaluateAutoProtection();

  // 4. BOOT button Wi-Fi reset (hold > 5s)
  static uint32_t pressStart = 0;
  static bool pressed = false;
  if (digitalRead(BUTTON_RESET_PIN) == LOW) {
    if (!pressed) {
      pressed = true;
      pressStart = millis();
    } else if (millis() - pressStart > 5000) {
      Serial.println("\n[Button] BOOT button held >5s! Clearing saved Wi-Fi and rebooting...");
      g_prefs.begin("bms_wifi", false);
      g_prefs.clear();
      g_prefs.end();
      delay(500);
      ESP.restart();
    }
  } else {
    pressed = false;
  }

  delay(5);
}
