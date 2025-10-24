/* sampleADC() -> returns milliamps; setGate(false) disables MOSFET */
const int TRIP_MA = 35000;      // trip threshold (mA)
const int DEBOUNCE_MS = 5;      // transient tolerance
volatile bool latched = false;

void powerLoop(void) {
  int i_ma = sampleADC();        // fast ADC read of shunt amp
  static int tcount = 0;
  if (latched) return;           // remain safe until manual reset
  if (i_ma > TRIP_MA) {
    if (++tcount >= DEBOUNCE_MS) {
      setGate(false);            // hardware disconnect
      latched = true;            // require reset procedure
      logEvent("OVERCURRENT", i_ma);
    }
  } else {
    tcount = 0;
  }
}