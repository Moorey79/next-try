// --- START OF SKETCH - User Base + NEW Heater Logic ONLY ---

// Include standard libraries
#include <SPI.h>
#include <EEPROM.h>
#include <math.h> // For isnan()

// Include device-specific libraries
// #include <max6675.h> // REMOVED

// Include GUIslice library core
#include "GUIslice.h"
#include "GUIslice_drv.h"

// Include Application-Specific Libraries (Using ORIGINAL Mgr logic assumptions)
#include "Heater.h"
#include "Pump.h"
#include "Mgr.h" // Assumes ORIGINAL Mgr library files/behavior
#include "RuntimeManager.h"
#include "Temp.h"

// Include the correct GUIslice Builder generated header
#include "gui_GSLC.h" // Assumes this declares m_gui and extern element pointers


// --- Pin Definitions ----- (From User Base)
const int heaterControlPin = 46;
const int pumpControlPin = 44;
const int thermocoupleDO = 50;       // MISO
const int thermocoupleCS = 48;       // CS
const int thermocoupleCLK = 52;      // SCK

// --- Setpoint Ranges --- (From User Base)
#define PUMP_START_MIN 40
#define PUMP_START_MAX 80
#define HEATER_STOP_MIN 70
#define HEATER_STOP_MAX 110
#define PUMP_STOP_MIN 30
#define PUMP_STOP_MAX 60
// --- Safety Limits --- (From User Base)
#define PUMP_MANUAL_OFF_MAX_TEMP 95
// --- Safety Override Offsets --- ADDED
#define HEATER_SAFETY_OFFSET 5.0 // Degrees C above Heater Stop SP to trigger safety OFF
#define PUMP_SAFETY_OFFSET   5.0 // Degrees C above Pump Start SP to trigger safety ON
// -----------------------------
// --- Heater Latching Delay --- ADDED
#define HEATER_LATCH_DELAY_MS 60000UL // 60 seconds in milliseconds (Change to 30000UL for 30s if needed)

// --- Hardware Object Creation --- (From User Base)
Heater heater(heaterControlPin);
Pump pump(pumpControlPin);
Temp thermocouple(thermocoupleCLK, thermocoupleCS, thermocoupleDO);

// --- Hysteresis ----- (From User Base)
const float pumpStartHysteresis = 2.0;

// --- State Tracking Variables -----
bool processStarted = false;
static bool pumpHasBeenOnOnce = false;
float currentTemperatureFloat = NAN;
bool pumpLastAutoState = false;
bool heaterLastAutoState = false;
// --- ADDED State Variables for Heater Latch/Delay ---
bool heaterHasReachedTarget = false;
unsigned long heaterAboveThresholdStartTime = 0;
// --------------------------------------------------

// --- Timer Variables ----- (From User Base)
unsigned long averagePredictedRuntime = 0;
unsigned long countdownStartTime = 0;
bool countdownRunning = false;

// --- Element References --- *** DEFINITIONS MUST BE PRESENT ***
// Definitions for the global pointers declared extern in gui_GSLC.h
gslc_tsElemRef* HeaterSTOPslide   = NULL;
gslc_tsElemRef* HeaterSTOPval     = NULL;
gslc_tsElemRef* PumpSTARTslide    = NULL;
gslc_tsElemRef* PumpSTARTval      = NULL;
gslc_tsElemRef* PumpSTOPslide     = NULL;
gslc_tsElemRef* PumpSTOPval       = NULL;
gslc_tsElemRef* m_heattog         = NULL;
gslc_tsElemRef* m_pumptog         = NULL;
gslc_tsElemRef* m_pElemOutTime    = NULL;
gslc_tsElemRef* m_pElemOutTxt20   = NULL;
gslc_tsElemRef* m_pElem_ttg       = NULL;
gslc_tsElemRef* m_pElemBtn1       = NULL; // For E_StartBtn2 ?
gslc_tsElemRef* m_pElemBtn1_4     = NULL; // For E_ELEM_BTN4 (SetPoints)?
gslc_tsElemRef* spBACKbtn         = NULL; // For E_StartBtn (Back on SetPoints)?
gslc_tsElemRef* runBACKbtn        = NULL; // For E_ELEM_BTN7 (Back on Running)?
gslc_tsElemRef* runSTOPbtn        = NULL; // For E_ELEM_BTN8 (Stop on Running)?
gslc_tsElemRef* m_HeatStat6       = NULL;
gslc_tsElemRef* m_PumpStat6       = NULL;
gslc_tsElemRef* m_pElemProgress2  = NULL;
gslc_tsElemRef* m_pElemTextbox2_3_6 = NULL;
gslc_tsElemRef* m_pElemTextbox2_4_7 = NULL;
gslc_tsElemRef* m_pElemTextbox2_5   = NULL;
gslc_tsElemRef* m_pTextSlider2    = NULL; // Check ID (Heat_Seek?)
// Add any others defined in your gui_GSLC.h extern declarations
// --------------------------------------------------------------

// --- Global Variables --- (From User Base)
bool enableDebugSerial = true;

// --- Function Declarations (Prototypes) ---
void InitGUI();
void updateTemperature();
void handleHeaterControl(); // <<< REPLACED WITH NEW LATCH/DELAY LOGIC
void handlePumpControl();   // <<< KEEPS ORIGINAL SIMPLE LOGIC
void updateTimerDisplay();
void sendStatusToESP();
void updateTemperatureDisplay(float temperature);
void updateSetpointDisplays();
bool CbBtnCommon(void* pvGui, void* pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY); // Start/Stop/Heattog modified
bool CbSlidePos(void* pvGui, void* pvElemRef, int16_t nPos); // Original version
bool CbResetRunsBtn(void* pvGui, void* pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY);
static int16_t DebugOut(char ch);

// --- Debug Output Function --- (From User Base)
static int16_t DebugOut(char ch) {
    if (enableDebugSerial) { if (ch == (char)'\n') Serial.println(""); else Serial.write(ch); }
    return 0;
}

// --- Combined Button Callback --- <<< Start/Stop/Heattog MODIFIED >>> ---
bool CbBtnCommon(void* pvGui, void* pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY) {
  gslc_tsGui* pGui = (gslc_tsGui*)pvGui;
  gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)pvElemRef;
  gslc_tsElem* pElem = NULL;
  int16_t resolvedId = -99;

  // Safe element lookup & Debug Print
  if (pElemRef != NULL) { pElem = gslc_GetElemFromRef(pGui, pElemRef); if (pElem != NULL) resolvedId = pElem->nId; else resolvedId = -1; } else { resolvedId = -2; }
  if (eTouch == GSLC_TOUCH_UP_IN && enableDebugSerial) { Serial.print("CbBtnCommon: Touch UP. Resolved ID = "); Serial.println(resolvedId); }

  if (!pElem) { return false; }
  int16_t nElemId = pElem->nId;

  if (eTouch == GSLC_TOUCH_UP_IN) {
    switch (nElemId) {
      case E_StartBtn2: // CHECK ID
        if (enableDebugSerial) Serial.println(" -> Start Process Action");

        // --- <<< ADD THIS EEPROM SAVE LOGIC BLOCK >>> ---
        bool settingsChanged = false;
        // Compare current RAM values (potentially changed by sliders) with saved EEPROM values
        // Assumes New Mgr methods getEeprom...SP() exist in your Mgr.h/.cpp
        if (mgr.PumpStSP() != mgr.getEepromPumpStSP()) { settingsChanged = true; }
        if (mgr.PumpOffSP() != mgr.getEepromPumpOffSP()) { settingsChanged = true; }
        if (mgr.HeatOffSP() != mgr.getEepromHeatOffSP()) { settingsChanged = true; }

        if (settingsChanged) {
            if (enableDebugSerial) Serial.println("   Settings changed, saving to EEPROM...");
            mgr.saveSettingsToEEPROM(); // Call the new save function
            if (enableDebugSerial) Serial.println("   EEPROM Save Complete.");
        } else {
            if (enableDebugSerial) Serial.println("   Settings unchanged, no EEPROM write needed.");
        }
        // --- <<< END OF EEPROM SAVE LOGIC BLOCK >>> ---

        // Resets and original Start logic follows...
        heaterHasReachedTarget = false; // Reset heater state
        heaterAboveThresholdStartTime = 0;
        updateTemperature();
        processStarted = true; runtimeMgr.start(); /* etc... */
        // ... rest of original E_StartBtn2 case ...
        break; // End of case E_StartBtn2

      case E_StartBtn: case E_ELEM_BTN7: // Back Buttons (Original Logic)
        if (enableDebugSerial) Serial.println(" -> Go Back to Main Page Action");
        gslc_SetPageCur(pGui, E_PG_MAIN);
        break;

      case E_ELEM_BTN8: // Stop Button - ADDED Resets for new state vars
        if (enableDebugSerial) Serial.println(" -> MANUAL STOP Button Action");
        if (processStarted) {
            unsigned long finalDuration = runtimeMgr.getCurrentRunTime();
            processStarted = false; runtimeMgr.stop(); countdownRunning = false;
            pumpLastAutoState = false; heaterLastAutoState = false;
            // --- ADDED Resets ---
            heaterHasReachedTarget = false;
            heaterAboveThresholdStartTime = 0;
            // --------------------
            if (finalDuration > 0) { runtimeMgr.saveRunTime(finalDuration); }
            heater.turnOff(); pump.turnOff();
            if (m_heattog != NULL) gslc_ElemXTogglebtnSetState(pGui, m_heattog, false);
            if (m_pumptog != NULL) gslc_ElemXTogglebtnSetState(pGui, m_pumptog, false);
        } else { /* Print not running */ }
        gslc_SetPageCur(pGui, E_PG_MAIN);
        break;

      case E_ELEM_BTN4: // Setpoints button (Original Logic)
        if (enableDebugSerial) Serial.println(" -> Go to Setpoints Page Action");
        gslc_SetPageCur(pGui, SetPoints); // Make sure SetPoints enum exists
        break;

case E_Pumptog: // Pump toggle button - USER ACTION ONLY
        if (enableDebugSerial) { Serial.print(" -> Pump Toggle Case Reached. processStarted="); Serial.println(processStarted); }
        // Only handle if process is running and element pointer is valid
        if (m_pumptog != NULL && processStarted) {
            // Find out what state the user wants based on the toggle press
            bool desiredState = gslc_ElemXTogglebtnGetState(pGui, m_pumptog);

            if (desiredState) {
                // --- User wants to turn Pump ON ---
                // No restrictions on turning ON manually
                pump.turnOn();
                if (enableDebugSerial) { Serial.println(" -> Pump Toggle: USER sets ON (HW)"); }
                // Sync toggle state immediately
                gslc_ElemXTogglebtnSetState(pGui, m_pumptog, true);

            } else {
                // --- User wants to turn Pump OFF ---
                // Check the NEW restriction rule: Must be < (lowest SP + 10)
                int pumpStartSP = mgr.PumpStSP(); // Uses original Mgr read
                int pumpStopSP = mgr.PumpOffSP();   // Uses original Mgr read
                int lowerSP = min(pumpStartSP, pumpStopSP); // Find the lower setpoint
                // Calculate the temperature threshold (lower SP + 10)
                float thresholdTemp = (float)lowerSP + 10.0;

                // Block turning OFF if current temperature is AT or ABOVE the threshold
                if (!isnan(currentTemperatureFloat) && currentTemperatureFloat >= thresholdTemp) {
                    // BLOCK THE ACTION
                    if (enableDebugSerial) {
                        Serial.print(" -> Pump Toggle: Manual OFF blocked (Temp ");
                        Serial.print(currentTemperatureFloat); Serial.print(" >= Threshold ");
                        Serial.print(thresholdTemp, 1); // Print threshold with 1 decimal
                        Serial.print(" [Lowest SP:"); Serial.print(lowerSP); Serial.println("])");
                    }
                    // Force the GUI toggle back to the ON state visually
                    gslc_ElemXTogglebtnSetState(pGui, m_pumptog, true);
                } else {
                    // ALLOW turning OFF (because Temp < threshold OR temp is NAN)
                    pump.turnOff();
                    if (enableDebugSerial) { Serial.println(" -> Pump Toggle: USER sets OFF (HW)"); }
                    // Sync toggle state immediately
                    gslc_ElemXTogglebtnSetState(pGui, m_pumptog, false);
                }
            }
        } else { // Conditions for toggling not met
             if (enableDebugSerial) {
                  if (!processStarted) Serial.println(" -> Pump Toggle: Ignored (process not started)");
                  else if (m_pumptog == NULL) Serial.println(" -> Pump Toggle: Ignored (m_pumptog is NULL)");
             }
             // Ensure toggle shows OFF if process not running
             if (m_pumptog != NULL) gslc_ElemXTogglebtnSetState(pGui, m_pumptog, false);
        }
        break; // End of case E_Pumptog
      // --- MODIFIED E_Heattog Case for Immediate Manual Latch ---
      case E_Heattog: // Heater toggle button - CHECK ID
        if (enableDebugSerial) { Serial.print(" -> Heater Toggle Case Reached. processStarted="); Serial.print(processStarted); Serial.print(" | HLatched="); Serial.println(heaterHasReachedTarget); }
        if (m_heattog != NULL && processStarted) {
              if (heaterHasReachedTarget) { // Check latch FIRST
                  if (enableDebugSerial) { Serial.println(" -> Heater Toggle: Ignored (Heater already LATCHED OFF)"); }
                  gslc_ElemXTogglebtnSetState(pGui, m_heattog, false); // Ensure toggle stays OFF
              } else {
                  // Heater not latched, process toggle
                  bool desiredState = gslc_ElemXTogglebtnGetState(pGui, m_heattog);
                  if (desiredState) { // User wants ON
                      heater.turnOn();
                      if (enableDebugSerial) { Serial.println(" -> Heater Toggle: USER sets ON (HW - Pre-Latch)"); }
                      gslc_ElemXTogglebtnSetState(pGui, m_heattog, true); // Sync toggle immediately
                  } else { // User wants OFF -> LATCH OFF
                      if (enableDebugSerial) { Serial.println(" -> Heater Toggle: USER sets OFF (HW) - LATCHING HEATER OFF NOW!"); }
                      heater.turnOff();
                      heaterHasReachedTarget = true;
                      heaterAboveThresholdStartTime = 0;
                      heaterLastAutoState = false;
                      gslc_ElemXTogglebtnSetState(pGui, m_heattog, false); // Sync toggle immediately
                  }
             }
        } else { /* Print ignored, ensure toggle OFF */ if (m_heattog != NULL) gslc_ElemXTogglebtnSetState(pGui, m_heattog, false); }
        break;
      // --- END MODIFIED E_Heattog Case ---

      default:
         /* Print unknown/ignored ID */
        if (enableDebugSerial && resolvedId > 0) { Serial.print(" -> No specific action defined for ID: "); Serial.println(nElemId); }
        else if (enableDebugSerial && resolvedId <= 0) { Serial.print(" -> Ignoring action for Resolved ID: "); Serial.println(resolvedId); }
        break;
    } // end switch
  } // end if GSLC_TOUCH_UP_IN
  return true;
} // end CbBtnCommon


// --- Callback for Hidden Reset Button --- (Original Logic)
bool CbResetRunsBtn(void* pvGui, void* pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY) {
    if (eTouch == GSLC_TOUCH_UP_IN) { if (enableDebugSerial) Serial.println("Reset Runs Button Pressed!"); runtimeMgr.resetRunTimes(); }
    return true;
}

// Updates only Mgr RAM value.
bool CbSlidePos(void* pvGui, void* pvElemRef, int16_t nPos) {
gslc_tsGui* pGui = (gslc_tsGui*)pvGui;
gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)pvElemRef;
gslc_tsElem* pElem = gslc_GetElemFromRef(pGui, pElemRef);
if (!pElem) { if (enableDebugSerial) Serial.println("CbSlidePos Error: pElem is NULL!"); return false; }
int16_t nElemId = pElem->nId;
bool changed = false; gslc_tsElemRef* targetValueDisplay = NULL;
int valueToSet = nPos;

  // Clamp value
  int minVal = nPos, maxVal = nPos;
  switch(nElemId) { // CHECK IDs HERE match your GUI definition
      case E_ELEM_SEEKBAR7: minVal=PUMP_STOP_MIN; maxVal=PUMP_STOP_MAX; break; // Assumed Pump Stop Slider
      case E_Heat_Seek:     minVal=HEATER_STOP_MIN; maxVal=HEATER_STOP_MAX; break; // Assumed Heater Stop Slider
      case E_Pump_Seek:     minVal=PUMP_START_MIN; maxVal=PUMP_START_MAX; break; // Assumed Pump Start Slider
      default: if (enableDebugSerial) Serial.print("CbSlidePos: Unknown Slider ID: "); Serial.println(nElemId); return true;
  }
  valueToSet = constrain(nPos, minVal, maxVal);
  if (enableDebugSerial && valueToSet != nPos) { Serial.print("  > Slider Pos Clamped: "); Serial.println(valueToSet); }

  // Update Mgr RAM only (Using NEW Mgr Setters which update RAM)
  switch (nElemId) {
    case E_ELEM_SEEKBAR7: // Pump Stop Slider
      if (mgr.PumpOffSP() != valueToSet) { mgr.PumpOffSP(valueToSet); targetValueDisplay = PumpSTOPval; changed = true; }
      break;
    case E_Heat_Seek: // Heater Stop Slider
      if (mgr.HeatOffSP() != valueToSet) { mgr.HeatOffSP(valueToSet); targetValueDisplay = HeaterSTOPval; changed = true; }
      break;
    case E_Pump_Seek: // Pump Start Slider
      if (mgr.PumpStSP() != valueToSet) { mgr.PumpStSP(valueToSet); targetValueDisplay = PumpSTARTval; changed = true; }
      break;
     // No default needed
  }

  // Update GUI Text Display associated with the slider
  if (changed && targetValueDisplay != NULL) {
      char Txt[10]; snprintf(Txt, 10, "%d C", valueToSet);
      gslc_ElemSetTxtStr(pGui, targetValueDisplay, Txt);
      if (enableDebugSerial) { Serial.print("  >> Slider ID: "); Serial.print(nElemId); Serial.print(" Set RAM & GUI to: "); Serial.println(valueToSet); }
  } else if (changed && targetValueDisplay == NULL) {
      // Log if text pointer is missing but value changed
      if (enableDebugSerial) { Serial.print("  >> Slider ID: "); Serial.print(nElemId); Serial.print(" Set RAM to: "); Serial.println(valueToSet); Serial.println("  WARN: Target display text pointer is NULL!");}
  }
  return true;
} // end CbSlidePos


// --- Arduino Setup --- (Original Logic - except InitGUIslice_gen fix)
void setup() {
  Serial.begin(19200); while (!Serial && millis() < 2000); gslc_InitDebug(&DebugOut);
  if (enableDebugSerial) Serial.println("\nSetup Starting");

  SPI.begin(); EEPROM.begin();
  mgr.begin();
  thermocouple.begin();

  // Original Setpoint check/load using original Mgr (EEPROM reads/writes)
  if (enableDebugSerial) Serial.println("Checking Setpoints (using original Mgr)...");
  int ps = mgr.PumpStSP(); int po = mgr.PumpOffSP(); int hs = mgr.HeatOffSP(); bool defaults = false;
  if (ps > PUMP_START_MAX || ps < PUMP_START_MIN || ps == 255) { mgr.PumpStSP(PUMP_START_MIN + 5); defaults = true; }
  if (po > PUMP_STOP_MAX || po < PUMP_STOP_MIN || po == 255)   { mgr.PumpOffSP(PUMP_STOP_MIN + 5); defaults = true; }
  int hs_check_max = (HEATER_STOP_MAX > 255) ? 255 : HEATER_STOP_MAX;
  if (hs > hs_check_max || hs < HEATER_STOP_MIN || hs == 255) { mgr.HeatOffSP(HEATER_STOP_MIN + 10); defaults = true; }
  if (defaults && enableDebugSerial) Serial.println("Loaded default setpoints (writing to EEPROM now).");
  if (enableDebugSerial) { Serial.print("Initial Setpoints: PS="); Serial.print(mgr.PumpStSP()); Serial.print(" PO="); Serial.print(mgr.PumpOffSP()); Serial.print(" HS="); Serial.println(mgr.HeatOffSP()); }

  // Initialize GUI
  InitGUIslice_gen(); InitGUI();

  // Update displays AFTER GUI is initialized (uses original Mgr reads)
  updateSetpointDisplays(); updateTemperature(); updateTemperatureDisplay(currentTemperatureFloat);

  // Initialize states (Original logic)
  processStarted = false;
  heaterHasReachedTarget = false; heaterAboveThresholdStartTime = 0; // Init new vars
  if (!isnan(currentTemperatureFloat)) { /* Set initial LastAutoStates */ heaterLastAutoState = (currentTemperatureFloat < mgr.HeatOffSP()); pumpLastAutoState = (currentTemperatureFloat >= (mgr.PumpStSP() + pumpStartHysteresis));}
  else { heaterLastAutoState = false; pumpLastAutoState = false; }
  heater.turnOff(); pump.turnOff();
  if (m_heattog) gslc_ElemXTogglebtnSetState(&m_gui, m_heattog, false);
  if (m_pumptog) gslc_ElemXTogglebtnSetState(&m_gui, m_pumptog, false);

  if (enableDebugSerial) Serial.println("Setup Complete. System Idle.");
} // End setup()

// --- Arduino Loop --- (Calls NEW handleHeaterControl)
void loop() {
  // if (m_pElemOutTxt20 == NULL && enableDebugSerial) { /* Error */ }
  gslc_Update(&m_gui);

  unsigned long currentMillis = millis();

  // Temp Reading
  static unsigned long lastTempReadTime = 0;
  const unsigned long tempReadInterval = 500;
  if (currentMillis - lastTempReadTime >= tempReadInterval) { lastTempReadTime = currentMillis; updateTemperature(); }

  // Control Logic
  if (processStarted) {
    if (!isnan(currentTemperatureFloat)) {
        handleHeaterControl(); // <<< Calls NEW version below
        handlePumpControl();   // <<< Calls ORIGINAL version below
    } else { /* Handle NAN, Reset heater timer */ if (heaterAboveThresholdStartTime != 0) heaterAboveThresholdStartTime = 0; }
    updateTimerDisplay();

    // Auto-Stop Condition (Only if temp is valid)
  int pumpStopTemp = mgr.PumpOffSP(); // Uses original Mgr read
  if (pumpHasBeenOnOnce && !isnan(currentTemperatureFloat) && currentTemperatureFloat <= pumpStopTemp) {
      if (enableDebugSerial) { Serial.print("Auto Stop Triggered: Temp <= PumpStopSP ("); Serial.print(pumpStopTemp); Serial.println(")"); }

      // <<< --- ADD DEBUG PRINTS FOR AUTO-STOP --- >>>
      if (enableDebugSerial) {
          Serial.print("AutoStop: runSTOPbtn pointer address = 0x"); Serial.println((unsigned long)runSTOPbtn, HEX);
          if (runSTOPbtn != NULL) {
               // Try to get the element struct using the pointer
               gslc_tsElem* pStopElem = gslc_GetElemFromRef(&m_gui, runSTOPbtn); // Use m_gui here as we are in loop() context
               if (pStopElem != NULL) {
                   Serial.print("AutoStop: Element pointed to by runSTOPbtn has ID = "); Serial.println(pStopElem->nId);
               } else {
                   Serial.println("AutoStop: ERROR - gslc_GetElemFromRef failed for runSTOPbtn!");
               }
          } else {
              Serial.println("AutoStop: ERROR - runSTOPbtn pointer IS NULL!");
          }
      }
      // <<< --- END DEBUG PRINTS --- >>>

      // Simulate pressing the dedicated Stop Button to perform cleanup
      if (runSTOPbtn != NULL) {
           // Call CbBtnCommon as if the stop button was pressed
           CbBtnCommon(&m_gui, runSTOPbtn, GSLC_TOUCH_UP_IN, 0, 0); // Use m_gui here too? Or get pGui context? Let's try m_gui.
      } else {
           // Fallback if element ref is missing (less clean)
           Serial.println("ERROR: runSTOPbtn element ref is NULL for Auto Stop!");
           // Manual stop logic here as a fallback:
           // processStarted = false; runtimeMgr.stop(); ... heater.turnOff(); pump.turnOff(); ... gslc_SetPageCur(&m_gui, E_PG_MAIN);
      }
  }
  } else { // If process not started (Original logic + heater timer reset)
      if (heater.isOn()) heater.turnOff(); if (pump.isOn()) pump.turnOff();
      if (m_heattog != NULL && gslc_ElemXTogglebtnGetState(&m_gui, m_heattog)) gslc_ElemXTogglebtnSetState(&m_gui, m_heattog, false);
      if (m_pumptog != NULL && gslc_ElemXTogglebtnGetState(&m_gui, m_pumptog)) gslc_ElemXTogglebtnSetState(&m_gui, m_pumptog, false);
      // Reset heater timer if stopped externally
      if (heaterAboveThresholdStartTime != 0) heaterAboveThresholdStartTime = 0;
  }

 // --- ADD SAFETY OVERRIDE CHECK ---
  // Run safety checks regardless of processStarted, but only if temp is valid
  if (!isnan(currentTemperatureFloat)) {
      checkSafetyOverrides();
  }
  // ----------------------------------

  // Send status to ESP
  static unsigned long lastEspUpdateTime = 0;
  const unsigned long espUpdateInterval = 10000;
  if (currentMillis - lastEspUpdateTime >= espUpdateInterval) { lastEspUpdateTime = currentMillis; sendStatusToESP(); }
} // End loop()


// --- Other Function Definitions ---

void InitGUI() { gslc_SetBkgndColor(&m_gui, GSLC_COL_BLACK); }

// Uses original Mgr reads
void updateSetpointDisplays() {
  // if (enableDebugSerial) Serial.println("Updating Setpoint Displays");
  char Txt[10]; int ps_actual = mgr.PumpStSP(); int po_actual = mgr.PumpOffSP(); int hs_actual = mgr.HeatOffSP();
  // if (enableDebugSerial) { /* Print values */ }
  if (PumpSTARTval && PumpSTARTslide) { snprintf(Txt, 10, "%d C", ps_actual); gslc_ElemSetTxtStr(&m_gui, PumpSTARTval, Txt); gslc_ElemXSeekbarSetPos(&m_gui, PumpSTARTslide, ps_actual); }
  if (PumpSTOPval && PumpSTOPslide) { snprintf(Txt, 10, "%d C", po_actual); gslc_ElemSetTxtStr(&m_gui, PumpSTOPval, Txt); gslc_ElemXSeekbarSetPos(&m_gui, PumpSTOPslide, po_actual); }
  if (HeaterSTOPval && HeaterSTOPslide) { /* Use hs_display clamp if needed */ snprintf(Txt, 10, "%d C", hs_actual); gslc_ElemSetTxtStr(&m_gui, HeaterSTOPval, Txt); gslc_ElemXSeekbarSetPos(&m_gui, HeaterSTOPslide, hs_actual); }
}

// Uses original dtostrf version
void updateTemperatureDisplay(float temperature) {
  // if (enableDebugSerial) { /* Print temp */ }
  if (m_pElemOutTxt20 == NULL) { /* Print Error */ return; }
  char Txt[12]; // Use 12 for safety with " C"
  if (isnan(temperature)) { snprintf(Txt, 12, "Err"); }
  else { dtostrf(temperature, 4, 1, Txt); if (strlen(Txt) <= (12 - 4)) { strcat(Txt, " C"); } }
  // if (enableDebugSerial) { /* Print update attempt */ }
  gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt20, Txt);
}


void updateTimerDisplay() {
  if (m_pElemOutTime) {
      if (runtimeMgr.isTimerRunning()) { gslc_ElemSetTxtStr(&m_gui, m_pElemOutTime, runtimeMgr.formatTime(runtimeMgr.getCurrentRunTime())); }
      else { gslc_ElemSetTxtStr(&m_gui, m_pElemOutTime, "00:00:00"); }
  }
  if (m_pElem_ttg) { /* Handle TTG update */ }
}

void updateTemperature() {
  currentTemperatureFloat = thermocouple.readCelsius();
  updateTemperatureDisplay(currentTemperatureFloat);
}


// --- Heater Control - NEW VERSION with Latching, Delay, LastAutoState Check ---
// Uses original Mgr reads for setpoint
// --- Heater Control - LESS VERBOSE DEBUGGING ---
void handleHeaterControl() {
  // --- Basic Checks ---
  if (!processStarted || isnan(currentTemperatureFloat)) {
    if (heaterAboveThresholdStartTime != 0) {
        heaterAboveThresholdStartTime = 0;
        if (enableDebugSerial) Serial.println("[H] Timer reset due to invalid state/process stop."); // CHANGE: Event print
    }
    return;
  }

  // --- Step 1: Check if already latched OFF ---
  if (heaterHasReachedTarget) {
    if (heater.isOn()) {
        heater.turnOff();
        if (m_heattog) gslc_ElemXTogglebtnSetState(&m_gui, m_heattog, false);
        if (enableDebugSerial) Serial.println("[H] Latched OFF state enforced."); // CHANGE: Event print
    }
    return;
  }

  // --- Step 2: Target NOT latched yet - Main Latching/Delay Logic ---
  int heaterStopTemp = mgr.HeatOffSP();

  // Check Latching Condition (Temp >= SP for Delay)
  if (currentTemperatureFloat >= heaterStopTemp) {
    if (heaterAboveThresholdStartTime == 0) { // Start timer
      heaterAboveThresholdStartTime = millis();
      if (enableDebugSerial) { Serial.print("[H] Temp >= SP. Start Timer @ "); Serial.println(heaterAboveThresholdStartTime); } // CHANGE: Event print
    } else { // Timer running, check expiry
      unsigned long elapsedTime = millis() - heaterAboveThresholdStartTime;
      if (elapsedTime >= HEATER_LATCH_DELAY_MS) { // LATCH OFF due to timer
        if (enableDebugSerial) { Serial.print("[H] Timer Expired ("); Serial.print(elapsedTime); Serial.println("ms). LATCHING OFF."); } // CHANGE: Event print
        heater.turnOff(); heaterHasReachedTarget = true; heaterLastAutoState = false; heaterAboveThresholdStartTime = 0;
        if (m_heattog) gslc_ElemXTogglebtnSetState(&m_gui, m_heattog, false);
        return; // Job done
      }
    }
  } else { // Temp < heaterStopTemp -> Reset timer if it was running
    if (heaterAboveThresholdStartTime != 0) {
      heaterAboveThresholdStartTime = 0;
      if (enableDebugSerial) Serial.println("[H] Temp < SP. Reset Timer."); // CHANGE: Event print
    }
  }

  // --- Step 3: Ensure Heater is ON if not latched (Respect Manual OFF via LastAutoState) ---
  bool autoDesiredState = true; // Auto logic wants heater ON until latching occurs

  // --- REMOVED the constant status print block ---

  // Compare desired auto state (ON) with the last known auto state.
  if (autoDesiredState != heaterLastAutoState) {
     if (enableDebugSerial) { Serial.println("  >> [H] Auto Decision CHANGED! Applying ON state."); } // CHANGE: Event print
     heater.turnOn();
     heaterLastAutoState = autoDesiredState; // Update last state
  }
  // else { /* No print if decision unchanged */ }

  // Sync GUI toggle only if it differs from hardware state
  if (m_heattog) {
    if (gslc_ElemXTogglebtnGetState(&m_gui, m_heattog) != heater.isOn()) {
      gslc_ElemXTogglebtnSetState(&m_gui, m_heattog, heater.isOn());
      if (enableDebugSerial) { Serial.print("   >> [H] Syncing GUI Toggle to: "); Serial.println(heater.isOn() ? "ON" : "OFF"); } // CHANGE: Event print
    }
  }
} // End handleHeaterControl


// --- Pump Control - LESS VERBOSE DEBUGGING ---
void handlePumpControl() {
   if (!processStarted) return;
   if (isnan(currentTemperatureFloat)) { /* Handle NAN */ return; }

   int startSP = mgr.PumpStSP();
   int stopSP = mgr.PumpOffSP();
   bool autoWantsOnCond = (currentTemperatureFloat >= (startSP + pumpStartHysteresis));
   bool autoWantsOffCond = (currentTemperatureFloat <= stopSP);
   bool autoDesiredState;

   if (!pumpLastAutoState && autoWantsOnCond) { autoDesiredState = true; }
   else if (pumpLastAutoState && autoWantsOffCond) { autoDesiredState = false; }
   else { autoDesiredState = pumpLastAutoState; }

   // --- REMOVED the constant status print block ---

   // Only take action if Auto decision changed from last command
   if (autoDesiredState != pumpLastAutoState) {
       if (enableDebugSerial) { Serial.println("  >> [P] Auto Decision CHANGED! Applying state."); } // CHANGE: Event print
       if (autoDesiredState) { pump.turnOn(); pumpHasBeenOnOnce = true; }
       else { pump.turnOff(); }
       pumpLastAutoState = autoDesiredState; // Update last commanded state
   }
   // else { /* No print if decision unchanged */ }

   // Sync GUI toggle only if it differs from hardware state
   if (m_pumptog) {
       if (gslc_ElemXTogglebtnGetState(&m_gui, m_pumptog) != pump.isOn()) {
           gslc_ElemXTogglebtnSetState(&m_gui, m_pumptog, pump.isOn());
           if(enableDebugSerial) { Serial.print("   >> [P] Syncing GUI Toggle to: "); Serial.println(pump.isOn() ? "ON":"OFF");} // CHANGE: Event print
       }
   }
} // End handlePumpControl

// --- Send status to ESP --- (Original Placeholder, added Latch status)
void sendStatusToESP() {
   if (enableDebugSerial) { static unsigned long lastPrt = 0; if(millis() - lastPrt > 10000) { lastPrt = millis(); Serial.print("Status: T="); Serial.print(currentTemperatureFloat, 1); Serial.print(" H:"); Serial.print(heater.isOn()); Serial.print(" P:"); Serial.print(pump.isOn()); Serial.print(" Run:"); Serial.print(processStarted); Serial.print(" HLatched:"); Serial.println(heaterHasReachedTarget); } }
   // Add communication implementation here...
}
// --- Safety Override Checks --- (Using Offsets)
void checkSafetyOverrides() {
    // Read current setpoints (using original Mgr methods)
    int heaterStopSP = mgr.HeatOffSP();
    int pumpStartSP = mgr.PumpStSP();

    // --- Heater Safety OFF ---
    // Check if temperature exceeds the stop setpoint PLUS the safety offset
    if (currentTemperatureFloat > ((float)heaterStopSP + HEATER_SAFETY_OFFSET)) { // <<< MODIFIED Condition
        if (heater.isOn()) { // And the heater is currently ON
            // Force heater OFF immediately and latch it
            if (enableDebugSerial) {
                Serial.print("!!! SAFETY OVERRIDE: Temp ("); Serial.print(currentTemperatureFloat);
                Serial.print(") > Heater SP+Offset ("); Serial.print((float)heaterStopSP + HEATER_SAFETY_OFFSET, 1);
                Serial.println("). Forcing Heater OFF & Latching!");
            }
            heater.turnOff();
            heaterHasReachedTarget = true;
            heaterAboveThresholdStartTime = 0;
            heaterLastAutoState = false;
            if (m_heattog) { gslc_ElemXTogglebtnSetState(&m_gui, m_heattog, false); }
        }
    }

    // --- Pump Safety ON ---
    // Check if temperature exceeds the start setpoint PLUS the safety offset
    if (currentTemperatureFloat > ((float)pumpStartSP + PUMP_SAFETY_OFFSET)) { // <<< MODIFIED Condition
        if (!pump.isOn()) { // And the pump is currently OFF
            // Force pump ON immediately
            if (enableDebugSerial) {
                Serial.print("!!! SAFETY OVERRIDE: Temp ("); Serial.print(currentTemperatureFloat);
                Serial.print(") > Pump Start SP+Offset ("); Serial.print((float)pumpStartSP + PUMP_SAFETY_OFFSET, 1);
                Serial.println("). Forcing Pump ON!");
            }
            pump.turnOn();
            pumpHasBeenOnOnce = true;
            if (m_pumptog) { gslc_ElemXTogglebtnSetState(&m_gui, m_pumptog, true); }
        }
    }
}
// --- End Safety Override Checks ---
// --- END OF SKETCH ---