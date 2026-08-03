// --------------------------------------------------
//
// ADSR Bezier example
// by mo_thunderz (mo-thunderz @github)
//
// Uncomment to use the float hot path instead of fixed-point (default):
// #define ADSR_BEZIER_USE_FLOAT 1
//
// Periodically triggers the ADSR and prints the envelope level.
// For other boards, adjust DACSIZE and the output line below.
//
// Parameters (setAttack/setDecay/setRelease use milliseconds):
//   adsr_attack_ms, adsr_decay_ms, adsr_release_ms
//   adsr_sustain: level 0 .. DACSIZE
//
// --------------------------------------------------

#include <ADSR_Bezier.h>

#define DACSIZE 4096
static const uint16_t MaxValue = 4095;

unsigned long adsr_attack_ms = 1000;
unsigned long adsr_decay_ms = 1000;
int adsr_sustain = 2500;
unsigned long adsr_release_ms = 1000;
unsigned long trigger_duration_ms = 3000;
unsigned long space_between_triggers_ms = 3000;

bool trigger_on = false;
unsigned long t_0 = 0;

// vertical_resolution, legacy alphas, bezier flag, A/D/R curve types
adsr adsr_class(MaxValue, 0.9995f, 0.9995f, true, 1, 2, 1);

void setup() {
  Serial.begin(115200);
  delay(100);

  adsrBezierInitTables(MaxValue, ARRAY_SIZE, _curve_tables);

  adsr_class.setAttack(adsr_attack_ms);
  adsr_class.setDecay(adsr_decay_ms);
  adsr_class.setSustain(adsr_sustain);
  adsr_class.setRelease(adsr_release_ms);
  adsr_class.setResetAttack(true);
}

void loop() {
  unsigned long now = millis();

  Serial.print("0 ");
  Serial.print(MaxValue);
  Serial.print(" ");
  Serial.println(adsr_class.getWave());

  if (trigger_on) {
    if (now >= t_0 + trigger_duration_ms) {
      adsr_class.noteOff();
      t_0 = now;
      trigger_on = false;
    }
  } else {
    if (now >= t_0 + space_between_triggers_ms) {
      adsr_class.noteOn();
      t_0 = now;
      trigger_on = true;
    }
  }

  delay(10);
}
