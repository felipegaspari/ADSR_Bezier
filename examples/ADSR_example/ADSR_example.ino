// ADSR_Bezier minimal example — single envelope, DCO-style wiring.
// EnvVCA-like: resolution 4095, curves 1/2/1, micros timebase (library default).

#define ARRAY_SIZE 512
#ifndef ADSR_BEZIER_USE_FLOAT
#define ADSR_BEZIER_USE_FLOAT 0         // same as DCO/adsr.h
#endif

#include <ADSR_Bezier.h>

static const int kTableMaxVal = 4000;       // DCO ADSR_1_CC for adsrBezierInitTables
static const int kEnvResolution = 4095;     // DCO ADSR_CV_CC (EnvVCA / EnvVCF scale)

unsigned long adsr_attack_ms = 1000;
unsigned long adsr_decay_ms = 1000;
int adsr_sustain = 2500;                    // 0 .. kEnvResolution
unsigned long adsr_release_ms = 1000;
unsigned long trigger_duration_ms = 3000;
unsigned long space_between_triggers_ms = 3000;

bool trigger_on = false;
unsigned long t_0 = 0;

static adsr env(kEnvResolution, 0.9995f, 0.9995f, false, 1, 2, 1);

static void initCurveTypes() {
  env.adsrCurveAttack(1);
  env.adsrCurveDecay(2);
  env.adsrCurveRelease(1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  adsrBezierInitTables((float)kTableMaxVal, ARRAY_SIZE, _curve_tables);
  initCurveTypes();

  env.setAttack(adsr_attack_ms);
  env.setDecay(adsr_decay_ms);
  env.setSustain(adsr_sustain);
  env.setRelease(adsr_release_ms);
  env.setResetAttack(true);
}

void loop() {
  unsigned long now = millis();

  Serial.print("level=");
  Serial.print(env.getWave());
  Serial.print(" / ");
  Serial.println(kEnvResolution);

  if (trigger_on) {
    if (now >= t_0 + trigger_duration_ms) {
      env.noteOff();
      t_0 = now;
      trigger_on = false;
    }
  } else {
    if (now >= t_0 + space_between_triggers_ms) {
      env.noteOn();
      t_0 = now;
      trigger_on = true;
    }
  }

  delay(10);
}
