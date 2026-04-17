
/* ==================== DRS ==================== */

enum DRSState {
  DRS_Disabled,
  DRS_Armed,
  DRS_Active
};

static DRSState drsState = DRS_Disabled;

static float minSpeed = 10;
static int minThrottle = 10;
static int minRPM = 3000;

static unsigned long lastActivation = 0;
static bool lastAllowed = false;

// Init

void drsInit() {
  drsState = DRS_Disabled;
  lastActivation = 0;
}

// Hauptlogik

void drsUpdate(speed, throttle, safety.breakePressed(),rpm,systemOK) {
  
  // Bedingungen
  bool allowed = speed > mindspeed && throttle > minThrottle && rpm > minRPM && !braking && systemOK;

  lastAllowed = allowed;

  // State
  switch (DRSState) {
    case DRS_Disabled:
      if (allowed) {
        drsState = DRS_Armed;
      }
    break;
    case DRS_Armed:
      if(!allowed) {
        drsState = DRS_Disabled;
      }
      break;
  }

  // Input
  void drsButtonPressed() {
    if(drsState == DRS_Armed) {
      drsState = DRS_Active;
      lastActivation = millis();
    }
  }

  // Getter für Loop
  DRSState getDRSState() {
    return DRSState;
  }

  bool isDRSActive() {
    return drsState == DRS_Active;
  }

  bool isDRSAllowed() {
    return lastAllowed;
  }
}
