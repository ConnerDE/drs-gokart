
/* ==================== DRS ==================== */

enum DRSState {
  DRS_Disabled,
  DRS_Armed,
  DRS_Active
};

static DRSState drsState = DRS_Disabled;

static float minSpeed    = 10.0;
static int   minThrottle = 10;
static int   minRPM      = 3000;

static unsigned long lastActivation = 0;
static bool lastAllowed = false;

// Init 

void drsInit() {
  drsState = DRS_Disabled;
  lastActivation = 0;
}

// Hauptlogik

void drsUpdate(float speed, int throttle, bool braking, int rpm, bool systemOK, float oilTemp, float voltage) {

  bool allowed = drsAllowed(speed, throttle, rpm, braking, systemOK, oilTemp, voltage);
  lastAllowed = allowed;

  switch (drsState) {
    case DRS_Disabled:
      if (allowed) drsState = DRS_Armed;
      break;
    case DRS_Armed:
      if (!allowed) drsState = DRS_Disabled;
      break;
    case DRS_Active:
      if (!allowed) drsState = DRS_Disabled;
      break;
  }
}

// Button-Handler 

void drsButtonPressed() {
  if (drsState == DRS_Armed) {
    drsState = DRS_Active;
    lastActivation = millis();
  }
}

// Getter 

DRSState getDRSState() { return drsState; }
bool isDRSActive()     { return drsState == DRS_Active; }
bool isDRSArmed()      { return drsState == DRS_Armed; }
bool isDRSAllowed()    { return lastAllowed; }

