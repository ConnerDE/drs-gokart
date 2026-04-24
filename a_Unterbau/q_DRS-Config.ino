
/* ==================== DRS CONFIG ==================== */

// Parameter 

static float DRS_MIN_SPEED    = 25.0;   // km/h
static int   DRS_MIN_THROTTLE = 20;     // % Gaspedalstellung
static int   DRS_MIN_RPM      = 1800;   // min⁻¹
static float DRS_MAX_OIL_TEMP = 110.0;  // °C
static float DRS_MIN_VOLTAGE  = 11.5;   // V
static float DRS_MAX_BRAKE = 0.0; // 0.0 ist kein Bremsdruck

// Show Mode

static bool drsShowMode = false;

void setDRSShowMode(bool val) { 
  drsShowMode = val; 
}

bool getDRSShowMode() { 
  return drsShowMode; 
}

// Bedingungslogik

bool drsAllowed(float speed, int throttle, int rpm, bool braking, bool systemOK, float oilTemp, float voltage) {

  if (drsShowMode) return true;

  if (speed    < DRS_MIN_SPEED)    return false;
  if (throttle < DRS_MIN_THROTTLE) return false;
  if (rpm      < DRS_MIN_RPM)      return false;
  if (braking)                     return false;
  if (!systemOK)                   return false;
  if (oilTemp  > DRS_MAX_OIL_TEMP) return false;
  if (voltage  < DRS_MIN_VOLTAGE)  return false;

  return true;
}