// ADSR_Bezier on-device self-test and getWave() benchmark.
// Models DCO load: N envelope instances per update (~10 kHz budget check).
// Self-tests and speed bench run from loop() (one step per call) so USB Serial stays responsive.

#define ARRAY_SIZE 512                    // match DCO
#ifndef ADSR_BEZIER_USE_FLOAT
#define ADSR_BEZIER_USE_FLOAT 1        // same as DCO/adsr.h; set to 1 to compare float backend
#endif
// Do not override ADSR_BEZIER_USE_MICROS — library default 1 matches DCO production path

#ifndef ADSR_BENCHMARK_SELFTEST
#define ADSR_BENCHMARK_SELFTEST 1
#endif
#ifndef ADSR_BENCHMARK_SPEED
#define ADSR_BENCHMARK_SPEED 1
#endif
#ifndef ADSR_BENCHMARK_FINGERPRINT
#define ADSR_BENCHMARK_FINGERPRINT 0
#endif
#ifndef ADSR_BENCHMARK_INSTANCES
#define ADSR_BENCHMARK_INSTANCES 1      // 1 = single env; 3 = DCO EnvDCO + EnvVCA + EnvVCF
#endif
#ifndef ADSR_BENCHMARK_ITERATIONS
#define ADSR_BENCHMARK_ITERATIONS 50000UL // use 50000UL for stable timing stats
#endif
#ifndef ADSR_BENCHMARK_SETTER_ITERATIONS
#define ADSR_BENCHMARK_SETTER_ITERATIONS 50000UL
#endif
#ifndef ADSR_BENCHMARK_TIME_MS_COUNT
#define ADSR_BENCHMARK_TIME_MS_COUNT 6
#endif
#ifndef ADSR_BENCHMARK_SUSTAIN_COUNT
#define ADSR_BENCHMARK_SUSTAIN_COUNT 4
#endif
#ifndef ADSR_BENCHMARK_CURVE_COUNT
#define ADSR_BENCHMARK_CURVE_COUNT 8
#endif
#ifndef ADSR_BENCHMARK_WAIT_SERIAL
#define ADSR_BENCHMARK_WAIT_SERIAL 1       // block until Serial Monitor connects
#endif
#ifndef ADSR_BENCHMARK_SERIAL_TIMEOUT_MS
#define ADSR_BENCHMARK_SERIAL_TIMEOUT_MS 0 // 0 = wait forever
#endif

#if ADSR_BENCHMARK_INSTANCES < 1
#error ADSR_BENCHMARK_INSTANCES must be >= 1
#endif

#include <ADSR_Bezier.h>

static const int kTableMaxVal = 4000;     // DCO ADSR_1_CC for adsrBezierInitTables
static const int kEnvDcoResolution = 4000;
static const int kEnvCvResolution = 4095; // DCO ADSR_CV_CC for EnvVCA/EnvVCF
static const int kSustainHalfDco = kEnvDcoResolution / 2;
static const int kSustainHalfCv = kEnvCvResolution / 2;

static int g_fail_count = 0;

// Single reusable test envelope (EnvDCO-like); all self-tests share this instance.
static adsr g_testEnv(kEnvDcoResolution, 0.999f, 0.997f, false, 0, 0, 0);

// Mirror DCO/adsr.h static envelope instances (speed benchmark pool)
static adsr envDco(kEnvDcoResolution, 0.999f, 0.997f, false, 7, 7, 7);
static adsr envVca(kEnvCvResolution, 0.9995f, 0.9995f, false, 1, 2, 1);
static adsr envVcf(kEnvCvResolution, 0.997f, 0.997f, false, 4, 6, 1);

static adsr *g_envs[ADSR_BENCHMARK_INSTANCES];

static const unsigned long kSweepTimeMs[ADSR_BENCHMARK_TIME_MS_COUNT] = {1, 10, 50, 100, 500, 2000};
static const int kSweepSustain[ADSR_BENCHMARK_SUSTAIN_COUNT] = {0, 500, 2000, 4000};

#if ADSR_BENCHMARK_SPEED
enum SpeedStep : uint8_t {
  SPEED_HEADER,
  SPEED_SET_ATTACK_HDR,
  SPEED_SET_ATTACK_CELL,
  SPEED_SET_DECAY_HDR,
  SPEED_SET_DECAY_CELL,
  SPEED_SET_SUSTAIN_HDR,
  SPEED_SET_SUSTAIN_CELL,
  SPEED_SET_RELEASE_HDR,
  SPEED_SET_RELEASE_CELL,
  SPEED_GW_ATTACK_HDR,
  SPEED_GW_ATTACK_CELL,
  SPEED_GW_DECAY_HDR,
  SPEED_GW_DECAY_CELL,
  SPEED_GW_SUSTAIN_HDR,
  SPEED_GW_SUSTAIN_CELL,
  SPEED_GW_RELEASE_HDR,
  SPEED_GW_RELEASE_CELL,
  SPEED_SUMMARY,
  SPEED_FINISHED
};

struct SweepStats {
  uint32_t minCycles;
  uint32_t maxCycles;
  uint8_t minCurve;
  uint8_t minValue;
  uint8_t maxCurve;
  uint8_t maxValue;
  bool valid;
};

static SpeedStep g_speedStep = SPEED_HEADER;
static uint8_t g_speedCurveIdx = 0;
static uint8_t g_speedValueIdx = 0;
static uint32_t g_speedRow[ADSR_BENCHMARK_TIME_MS_COUNT];
static uint32_t g_getWaveRow[ADSR_BENCHMARK_CURVE_COUNT];

static SweepStats g_statSetAttack;
static SweepStats g_statSetDecay;
static SweepStats g_statSetSustain;
static SweepStats g_statSetRelease;
static SweepStats g_statGwAttack;
static SweepStats g_statGwDecay;
static SweepStats g_statGwSustain;
static SweepStats g_statGwRelease;
static SweepStats g_statGwGlobal;
#endif

enum BenchPhase : uint8_t {
#if ADSR_BENCHMARK_SELFTEST
  PHASE_SELFTEST_HEADER,
  PHASE_ST_IDLE,
  PHASE_ST_ZERO_ATTACK,
  PHASE_ST_ATTACK_RISE,
  PHASE_ST_DECAY,
  PHASE_ST_SUSTAIN,
  PHASE_ST_RELEASE,
  PHASE_ST_CLAMP,
  PHASE_ST_LEGATO,
  PHASE_SELFTEST_SUMMARY,
#endif
#if ADSR_BENCHMARK_SPEED
  PHASE_SPEED,
#endif
#if ADSR_BENCHMARK_FINGERPRINT
  PHASE_FINGERPRINT,
#endif
  PHASE_DONE
};

static BenchPhase g_phase = PHASE_DONE;

static void reportPass(const char *name) {
  Serial.print("  PASS  ");
  Serial.println(name);
}

static void reportFail(const char *name, const char *detail) {
  g_fail_count++;
  Serial.print("  FAIL  ");
  Serial.print(name);
  Serial.print(" — ");
  Serial.println(detail);
}

static void reportFailValue(const char *name, const char *detail, int v) {
  reportFail(name, detail);
  Serial.print("    measured=");
  Serial.println(v);
}

static bool nearSustain(int v, int target) {
  const int tol = 2;
  return v >= target - tol && v <= target + tol;
}

#ifndef ADSR_BENCHMARK_QUIET
static void selfTestFailMeasured(const char *detail, int v) {
  Serial.println("FAIL");
  g_fail_count++;
  Serial.print("    measured=");
  Serial.print(v);
  Serial.print(" (");
  Serial.print(detail);
  Serial.println(")");
}

static void selfTestFailSustainHold(int v1, int v2) {
  Serial.println("FAIL");
  g_fail_count++;
  Serial.print("    v1=");
  Serial.print(v1);
  Serial.print(" v2=");
  Serial.println(v2);
}
#endif

#ifndef ADSR_BENCHMARK_QUIET
static void selfTestLine(const char *name) {
  Serial.print("Self-test ");
  Serial.print(name);
  Serial.print(" ... ");
}
#endif

static void initCurveTypes(adsr &e, uint8_t atk, uint8_t dec, uint8_t rel) {
  e.adsrCurveAttack(atk);
  e.adsrCurveDecay(dec);
  e.adsrCurveRelease(rel);
}

static void setEnvParams(adsr &e, unsigned long attackMs, unsigned long decayMs, int sustain, unsigned long releaseMs) {
  e.setAttack(attackMs);
  e.setDecay(decayMs);
  e.setSustain(sustain);
  e.setRelease(releaseMs);
  e.setResetAttack(true);
}

static int defaultSustainFor(const adsr &e) {
  if (&e == &envDco) {
    return kSustainHalfDco;
  }
  return kSustainHalfCv;
}

static void pollEnvelopeMs(adsr &e, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    e.getWave();
    delay(1);
  }
}

static void pollEnvelopeMs(unsigned long ms) {
  pollEnvelopeMs(g_testEnv, ms);
}

static void pollUntilIdleMs(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (g_testEnv.getWave() == 0) {
      return;
    }
    g_testEnv.noteOff();
    delay(5);
    g_testEnv.getWave();
  }
}

static void prepareTestEnv() {
  g_testEnv.setRelease(50);
  g_testEnv.setResetAttack(true);
  g_testEnv.noteOff();
  pollUntilIdleMs(150);
}

static void releaseTestEnv() {
  g_testEnv.setRelease(50);
  g_testEnv.setResetAttack(true);
  g_testEnv.noteOff();
  pollEnvelopeMs(80);
  if (g_testEnv.getWave() != 0) {
    g_testEnv.noteOff();
    pollEnvelopeMs(50);
  }
}

static void initBenchmarkInstances() {
  static adsr *const kPool[] = { &envDco, &envVca, &envVcf };
  const uint8_t kPoolSize = 3;

  for (uint8_t i = 0; i < ADSR_BENCHMARK_INSTANCES; i++) {
    g_envs[i] = kPool[i % kPoolSize];
  }

  initCurveTypes(envDco, 7, 7, 7);
  initCurveTypes(envVca, 1, 2, 1);
  initCurveTypes(envVcf, 4, 6, 1);

  setEnvParams(envDco, 100, 100, kSustainHalfDco, 100);
  setEnvParams(envVca, 100, 100, kSustainHalfCv, 100);
  setEnvParams(envVcf, 100, 100, kSustainHalfCv, 100);
}

static inline void runGetWaveBundle() {
  for (uint8_t i = 0; i < ADSR_BENCHMARK_INSTANCES; i++) {
    volatile int v = g_envs[i]->getWave();
    (void)v;
  }
}

static void printBanner() {
  Serial.println();
  Serial.println("=== ADSR_Bezier benchmark ===");
  Serial.print("ADSR_BEZIER_USE_FLOAT=");
  Serial.println(ADSR_BEZIER_USE_FLOAT);
  Serial.print("ADSR_BEZIER_USE_MICROS=");
  Serial.println(ADSR_BEZIER_USE_MICROS);
  Serial.print("ARRAY_SIZE=");
  Serial.println(ARRAY_SIZE);
  Serial.print("F_CPU=");
  Serial.println(F_CPU);
  Serial.print("ADSR_BENCHMARK_SELFTEST=");
  Serial.println(ADSR_BENCHMARK_SELFTEST);
  Serial.print("ADSR_BENCHMARK_SPEED=");
  Serial.println(ADSR_BENCHMARK_SPEED);
  Serial.print("ADSR_BENCHMARK_FINGERPRINT=");
  Serial.println(ADSR_BENCHMARK_FINGERPRINT);
  Serial.print("ADSR_BENCHMARK_INSTANCES=");
  Serial.println(ADSR_BENCHMARK_INSTANCES);
  Serial.print("ADSR_BENCHMARK_ITERATIONS=");
  Serial.println(ADSR_BENCHMARK_ITERATIONS);
  Serial.print("ADSR_BENCHMARK_SETTER_ITERATIONS=");
  Serial.println(ADSR_BENCHMARK_SETTER_ITERATIONS);
#if defined(ARDUINO_ARCH_RP2040)
  Serial.println("Timer: rp2040.getCycleCount()");
#else
  Serial.println("Timer: micros() fallback");
#endif
  Serial.println();
}

#if defined(ARDUINO_ARCH_RP2040)
static uint32_t benchGetWaveCycles(uint32_t iterations) {
  uint32_t t0 = rp2040.getCycleCount();
  noInterrupts();
  for (uint32_t i = 0; i < iterations; i++) {
    for (uint8_t j = 0; j < ADSR_BENCHMARK_INSTANCES; j++) {
      volatile int v = g_envs[j]->getWave();
      (void)v;
    }
  }
  interrupts();
  return rp2040.getCycleCount() - t0;
}

static uint32_t benchSetAttackCycles(adsr &e, uint32_t iterations, unsigned long attackMs) {
  uint32_t t0 = rp2040.getCycleCount();
  noInterrupts();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setAttack(attackMs + (i & 1UL));
  }
  interrupts();
  return rp2040.getCycleCount() - t0;
}

static uint32_t benchSetDecayCycles(adsr &e, uint32_t iterations, unsigned long decayMs) {
  uint32_t t0 = rp2040.getCycleCount();
  noInterrupts();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setDecay(decayMs + (i & 1UL));
  }
  interrupts();
  return rp2040.getCycleCount() - t0;
}

static uint32_t benchSetReleaseCycles(adsr &e, uint32_t iterations, unsigned long releaseMs) {
  uint32_t t0 = rp2040.getCycleCount();
  noInterrupts();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setRelease(releaseMs + (i & 1UL));
  }
  interrupts();
  return rp2040.getCycleCount() - t0;
}

static uint32_t benchSetSustainCycles(adsr &e, uint32_t iterations, int sustain) {
  uint32_t t0 = rp2040.getCycleCount();
  noInterrupts();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setSustain(sustain + (int)(i & 1UL));
  }
  interrupts();
  return rp2040.getCycleCount() - t0;
}
#else
static uint32_t benchGetWaveCycles(uint32_t iterations) {
  uint32_t t0 = micros();
  for (uint32_t i = 0; i < iterations; i++) {
    runGetWaveBundle();
  }
  return (micros() - t0) * (F_CPU / 1000000UL);
}

static uint32_t benchSetAttackCycles(adsr &e, uint32_t iterations, unsigned long attackMs) {
  uint32_t t0 = micros();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setAttack(attackMs + (i & 1UL));
  }
  return (micros() - t0) * (F_CPU / 1000000UL);
}

static uint32_t benchSetDecayCycles(adsr &e, uint32_t iterations, unsigned long decayMs) {
  uint32_t t0 = micros();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setDecay(decayMs + (i & 1UL));
  }
  return (micros() - t0) * (F_CPU / 1000000UL);
}

static uint32_t benchSetReleaseCycles(adsr &e, uint32_t iterations, unsigned long releaseMs) {
  uint32_t t0 = micros();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setRelease(releaseMs + (i & 1UL));
  }
  return (micros() - t0) * (F_CPU / 1000000UL);
}

static uint32_t benchSetSustainCycles(adsr &e, uint32_t iterations, int sustain) {
  uint32_t t0 = micros();
  for (uint32_t i = 0; i < iterations; i++) {
    e.setSustain(sustain + (int)(i & 1UL));
  }
  return (micros() - t0) * (F_CPU / 1000000UL);
}
#endif

static uint32_t cyclesPerGetWave(uint32_t totalCycles, uint32_t iterations) {
  const uint32_t totalCalls = iterations * (uint32_t)ADSR_BENCHMARK_INSTANCES;
  return totalCycles / totalCalls;
}

static void printCyclesPerGetWave(const char *label, uint32_t totalCycles, uint32_t iterations) {
  const uint32_t cyclesPerCall = cyclesPerGetWave(totalCycles, iterations);
  Serial.print("  ");
  Serial.print(label);
  Serial.print(": ");
  Serial.print(cyclesPerCall);
  Serial.print(" cycles/getWave (");
  Serial.print((cyclesPerCall * 1000000UL + F_CPU / 2) / F_CPU);
  Serial.println(" us approx)");
}

static void pinPhaseAttack(adsr &e, uint8_t curve) {
  const int sustain = defaultSustainFor(e);
  e.adsrCurveAttack(curve);
  e.noteOff();
  setEnvParams(e, 5000, 5000, sustain, 5000);
  e.noteOn();
  pollEnvelopeMs(e, 50);
}

static void pinPhaseDecay(adsr &e, uint8_t curve) {
  const int sustain = defaultSustainFor(e);
  e.adsrCurveDecay(curve);
  e.noteOff();
  setEnvParams(e, 10, 5000, sustain, 5000);
  e.noteOn();
  pollEnvelopeMs(e, 25);
}

static void pinPhaseSustain(adsr &e, uint8_t curve) {
  const int sustain = defaultSustainFor(e);
  e.adsrCurveDecay(curve);
  e.noteOff();
  setEnvParams(e, 10, 10, sustain, 5000);
  e.noteOn();
  pollEnvelopeMs(e, 40);
}

static void pinPhaseRelease(adsr &e, uint8_t curve) {
  const int sustain = defaultSustainFor(e);
  e.adsrCurveRelease(curve);
  e.noteOff();
  setEnvParams(e, 10, 10, sustain, 5000);
  e.noteOn();
  pollEnvelopeMs(e, 40);
  e.noteOff();
  pollEnvelopeMs(e, 50);
}

#if ADSR_BENCHMARK_SPEED
static void sweepStatsReset(SweepStats &s) {
  s.minCycles = UINT32_MAX;
  s.maxCycles = 0;
  s.valid = false;
}

static void sweepStatsUpdate(SweepStats &s, uint32_t cycles, uint8_t curve, uint8_t value) {
  if (!s.valid || cycles < s.minCycles) {
    s.minCycles = cycles;
    s.minCurve = curve;
    s.minValue = value;
  }
  if (!s.valid || cycles > s.maxCycles) {
    s.maxCycles = cycles;
    s.maxCurve = curve;
    s.maxValue = value;
  }
  s.valid = true;
}

static void printSetterTimeHeader() {
  Serial.print("values ms:");
  for (uint8_t i = 0; i < ADSR_BENCHMARK_TIME_MS_COUNT; i++) {
    Serial.print(" ");
    Serial.print(kSweepTimeMs[i]);
  }
  Serial.println();
}

static void printSetterRow(uint8_t curve, const uint32_t *row, uint8_t count) {
  Serial.print("c");
  Serial.print(curve);
  Serial.print(":");
  for (uint8_t i = 0; i < count; i++) {
    if (row[i] < 10) {
      Serial.print("    ");
    } else if (row[i] < 100) {
      Serial.print("   ");
    } else if (row[i] < 1000) {
      Serial.print("  ");
    } else {
      Serial.print(" ");
    }
    Serial.print(row[i]);
  }
  Serial.println();
}

static void printSetterMinMax(const char *name, const SweepStats &s, bool timeIsMs) {
  Serial.print("MIN ");
  Serial.print(s.minCycles);
  Serial.print(" cycles (");
  Serial.print(name);
  Serial.print(" curve=");
  Serial.print(s.minCurve);
  if (timeIsMs) {
    Serial.print(", ");
    Serial.print(kSweepTimeMs[s.minValue]);
    Serial.print("ms");
  } else {
    Serial.print(", level=");
    Serial.print(kSweepSustain[s.minValue]);
  }
  Serial.println(")");
  Serial.print("MAX ");
  Serial.print(s.maxCycles);
  Serial.print(" cycles (");
  Serial.print(name);
  Serial.print(" curve=");
  Serial.print(s.maxCurve);
  if (timeIsMs) {
    Serial.print(", ");
    Serial.print(kSweepTimeMs[s.maxValue]);
    Serial.print("ms");
  } else {
    Serial.print(", level=");
    Serial.print(kSweepSustain[s.maxValue]);
  }
  Serial.println();
}

static void printGetWaveCurveRow(const uint32_t *row) {
  for (uint8_t c = 0; c < ADSR_BENCHMARK_CURVE_COUNT; c++) {
    Serial.print("c");
    Serial.print(c);
    Serial.print(":");
    Serial.print(row[c]);
    if (c + 1 < ADSR_BENCHMARK_CURVE_COUNT) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

static void printGetWavePhaseMinMax(const char *phaseName, const SweepStats &s) {
  Serial.print("MIN ");
  Serial.print(s.minCycles);
  Serial.print("  MAX ");
  Serial.print(s.maxCycles);
  Serial.print(" cycles/getWave (");
  Serial.print(phaseName);
  Serial.print(" phase, curve ");
  Serial.print(s.minCurve);
  Serial.print("-");
  Serial.print(s.maxCurve);
  Serial.println(")");
}

static const char *kGwPhaseNames[] = {"attack", "decay", "sustain", "release"};

static void runSpeedStep() {
  switch (g_speedStep) {
    case SPEED_HEADER:
      Serial.println("--- Speed benchmark ---");
      Serial.print("Setter iterations: ");
      Serial.println(ADSR_BENCHMARK_SETTER_ITERATIONS);
      Serial.print("getWave iterations x");
      Serial.print(ADSR_BENCHMARK_INSTANCES);
      Serial.print(" instances: ");
      Serial.println(ADSR_BENCHMARK_ITERATIONS);
      g_speedStep = SPEED_SET_ATTACK_HDR;
      break;

    case SPEED_SET_ATTACK_HDR:
      sweepStatsReset(g_statSetAttack);
      g_speedCurveIdx = 0;
      g_speedValueIdx = 0;
      Serial.println();
      Serial.println("--- Setter sweep: setAttack ---");
      Serial.print("curves 0-");
      Serial.println(ADSR_BENCHMARK_CURVE_COUNT - 1);
      printSetterTimeHeader();
      g_speedStep = SPEED_SET_ATTACK_CELL;
      break;

    case SPEED_SET_ATTACK_CELL: {
      envDco.adsrCurveAttack(g_speedCurveIdx);
      const uint32_t total = benchSetAttackCycles(envDco, ADSR_BENCHMARK_SETTER_ITERATIONS,
                                                  kSweepTimeMs[g_speedValueIdx]);
      const uint32_t perCall = total / (uint32_t)ADSR_BENCHMARK_SETTER_ITERATIONS;
      g_speedRow[g_speedValueIdx] = perCall;
      sweepStatsUpdate(g_statSetAttack, perCall, g_speedCurveIdx, g_speedValueIdx);
      g_speedValueIdx++;
      if (g_speedValueIdx >= ADSR_BENCHMARK_TIME_MS_COUNT) {
        printSetterRow(g_speedCurveIdx, g_speedRow, ADSR_BENCHMARK_TIME_MS_COUNT);
        g_speedValueIdx = 0;
        g_speedCurveIdx++;
        if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
          printSetterMinMax("setAttack", g_statSetAttack, true);
          g_speedStep = SPEED_SET_DECAY_HDR;
        }
      }
      break;
    }

    case SPEED_SET_DECAY_HDR:
      sweepStatsReset(g_statSetDecay);
      g_speedCurveIdx = 0;
      g_speedValueIdx = 0;
      Serial.println();
      Serial.println("--- Setter sweep: setDecay ---");
      Serial.print("curves 0-");
      Serial.println(ADSR_BENCHMARK_CURVE_COUNT - 1);
      printSetterTimeHeader();
      g_speedStep = SPEED_SET_DECAY_CELL;
      break;

    case SPEED_SET_DECAY_CELL: {
      envDco.adsrCurveDecay(g_speedCurveIdx);
      const uint32_t total = benchSetDecayCycles(envDco, ADSR_BENCHMARK_SETTER_ITERATIONS,
                                                  kSweepTimeMs[g_speedValueIdx]);
      const uint32_t perCall = total / (uint32_t)ADSR_BENCHMARK_SETTER_ITERATIONS;
      g_speedRow[g_speedValueIdx] = perCall;
      sweepStatsUpdate(g_statSetDecay, perCall, g_speedCurveIdx, g_speedValueIdx);
      g_speedValueIdx++;
      if (g_speedValueIdx >= ADSR_BENCHMARK_TIME_MS_COUNT) {
        printSetterRow(g_speedCurveIdx, g_speedRow, ADSR_BENCHMARK_TIME_MS_COUNT);
        g_speedValueIdx = 0;
        g_speedCurveIdx++;
        if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
          printSetterMinMax("setDecay", g_statSetDecay, true);
          g_speedStep = SPEED_SET_SUSTAIN_HDR;
        }
      }
      break;
    }

    case SPEED_SET_SUSTAIN_HDR:
      sweepStatsReset(g_statSetSustain);
      g_speedValueIdx = 0;
      Serial.println();
      Serial.println("--- Setter sweep: setSustain ---");
      Serial.print("values:");
      for (uint8_t i = 0; i < ADSR_BENCHMARK_SUSTAIN_COUNT; i++) {
        Serial.print(" ");
        Serial.print(kSweepSustain[i]);
      }
      Serial.println();
      g_speedStep = SPEED_SET_SUSTAIN_CELL;
      break;

    case SPEED_SET_SUSTAIN_CELL: {
      const uint32_t total = benchSetSustainCycles(envDco, ADSR_BENCHMARK_SETTER_ITERATIONS,
                                                     kSweepSustain[g_speedValueIdx]);
      const uint32_t perCall = total / (uint32_t)ADSR_BENCHMARK_SETTER_ITERATIONS;
      g_speedRow[g_speedValueIdx] = perCall;
      sweepStatsUpdate(g_statSetSustain, perCall, 0, g_speedValueIdx);
      g_speedValueIdx++;
      if (g_speedValueIdx >= ADSR_BENCHMARK_SUSTAIN_COUNT) {
        Serial.print("sustain:");
        for (uint8_t i = 0; i < ADSR_BENCHMARK_SUSTAIN_COUNT; i++) {
          Serial.print(" ");
          Serial.print(g_speedRow[i]);
        }
        Serial.println();
        printSetterMinMax("setSustain", g_statSetSustain, false);
        g_speedStep = SPEED_SET_RELEASE_HDR;
      }
      break;
    }

    case SPEED_SET_RELEASE_HDR:
      sweepStatsReset(g_statSetRelease);
      g_speedCurveIdx = 0;
      g_speedValueIdx = 0;
      Serial.println();
      Serial.println("--- Setter sweep: setRelease ---");
      Serial.print("curves 0-");
      Serial.println(ADSR_BENCHMARK_CURVE_COUNT - 1);
      printSetterTimeHeader();
      g_speedStep = SPEED_SET_RELEASE_CELL;
      break;

    case SPEED_SET_RELEASE_CELL: {
      envDco.adsrCurveRelease(g_speedCurveIdx);
      const uint32_t total = benchSetReleaseCycles(envDco, ADSR_BENCHMARK_SETTER_ITERATIONS,
                                                    kSweepTimeMs[g_speedValueIdx]);
      const uint32_t perCall = total / (uint32_t)ADSR_BENCHMARK_SETTER_ITERATIONS;
      g_speedRow[g_speedValueIdx] = perCall;
      sweepStatsUpdate(g_statSetRelease, perCall, g_speedCurveIdx, g_speedValueIdx);
      g_speedValueIdx++;
      if (g_speedValueIdx >= ADSR_BENCHMARK_TIME_MS_COUNT) {
        printSetterRow(g_speedCurveIdx, g_speedRow, ADSR_BENCHMARK_TIME_MS_COUNT);
        g_speedValueIdx = 0;
        g_speedCurveIdx++;
        if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
          printSetterMinMax("setRelease", g_statSetRelease, true);
          g_speedStep = SPEED_GW_ATTACK_HDR;
        }
      }
      break;
    }

    case SPEED_GW_ATTACK_HDR:
      sweepStatsReset(g_statGwAttack);
      sweepStatsReset(g_statGwGlobal);
      g_speedCurveIdx = 0;
      Serial.println();
      Serial.println("--- getWave sweep: attack phase ---");
      Serial.print("curves 0-");
      Serial.print(ADSR_BENCHMARK_CURVE_COUNT - 1);
      Serial.print(", instances=");
      Serial.print(ADSR_BENCHMARK_INSTANCES);
      Serial.print(", iter=");
      Serial.println(ADSR_BENCHMARK_ITERATIONS);
      g_speedStep = SPEED_GW_ATTACK_CELL;
      break;

    case SPEED_GW_ATTACK_CELL: {
      for (uint8_t i = 0; i < ADSR_BENCHMARK_INSTANCES; i++) {
        pinPhaseAttack(*g_envs[i], g_speedCurveIdx);
      }
      const uint32_t cycles = benchGetWaveCycles(ADSR_BENCHMARK_ITERATIONS);
      const uint32_t cpc = cyclesPerGetWave(cycles, ADSR_BENCHMARK_ITERATIONS);
      g_getWaveRow[g_speedCurveIdx] = cpc;
      sweepStatsUpdate(g_statGwAttack, cpc, g_speedCurveIdx, 0);
      sweepStatsUpdate(g_statGwGlobal, cpc, g_speedCurveIdx, 0);
      g_speedCurveIdx++;
      if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
        printGetWaveCurveRow(g_getWaveRow);
        printGetWavePhaseMinMax("attack", g_statGwAttack);
        g_speedStep = SPEED_GW_DECAY_HDR;
      }
      break;
    }

    case SPEED_GW_DECAY_HDR:
      sweepStatsReset(g_statGwDecay);
      g_speedCurveIdx = 0;
      Serial.println();
      Serial.println("--- getWave sweep: decay phase ---");
      Serial.print("curves 0-");
      Serial.print(ADSR_BENCHMARK_CURVE_COUNT - 1);
      Serial.print(", instances=");
      Serial.print(ADSR_BENCHMARK_INSTANCES);
      Serial.print(", iter=");
      Serial.println(ADSR_BENCHMARK_ITERATIONS);
      g_speedStep = SPEED_GW_DECAY_CELL;
      break;

    case SPEED_GW_DECAY_CELL: {
      for (uint8_t i = 0; i < ADSR_BENCHMARK_INSTANCES; i++) {
        pinPhaseDecay(*g_envs[i], g_speedCurveIdx);
      }
      const uint32_t cycles = benchGetWaveCycles(ADSR_BENCHMARK_ITERATIONS);
      const uint32_t cpc = cyclesPerGetWave(cycles, ADSR_BENCHMARK_ITERATIONS);
      g_getWaveRow[g_speedCurveIdx] = cpc;
      sweepStatsUpdate(g_statGwDecay, cpc, g_speedCurveIdx, 0);
      sweepStatsUpdate(g_statGwGlobal, cpc, g_speedCurveIdx, 1);
      g_speedCurveIdx++;
      if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
        printGetWaveCurveRow(g_getWaveRow);
        printGetWavePhaseMinMax("decay", g_statGwDecay);
        g_speedStep = SPEED_GW_SUSTAIN_HDR;
      }
      break;
    }

    case SPEED_GW_SUSTAIN_HDR:
      sweepStatsReset(g_statGwSustain);
      g_speedCurveIdx = 0;
      Serial.println();
      Serial.println("--- getWave sweep: sustain phase ---");
      Serial.print("curves 0-");
      Serial.print(ADSR_BENCHMARK_CURVE_COUNT - 1);
      Serial.print(", instances=");
      Serial.print(ADSR_BENCHMARK_INSTANCES);
      Serial.print(", iter=");
      Serial.println(ADSR_BENCHMARK_ITERATIONS);
      g_speedStep = SPEED_GW_SUSTAIN_CELL;
      break;

    case SPEED_GW_SUSTAIN_CELL: {
      for (uint8_t i = 0; i < ADSR_BENCHMARK_INSTANCES; i++) {
        pinPhaseSustain(*g_envs[i], g_speedCurveIdx);
      }
      const uint32_t cycles = benchGetWaveCycles(ADSR_BENCHMARK_ITERATIONS);
      const uint32_t cpc = cyclesPerGetWave(cycles, ADSR_BENCHMARK_ITERATIONS);
      g_getWaveRow[g_speedCurveIdx] = cpc;
      sweepStatsUpdate(g_statGwSustain, cpc, g_speedCurveIdx, 0);
      sweepStatsUpdate(g_statGwGlobal, cpc, g_speedCurveIdx, 2);
      g_speedCurveIdx++;
      if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
        printGetWaveCurveRow(g_getWaveRow);
        printGetWavePhaseMinMax("sustain", g_statGwSustain);
        g_speedStep = SPEED_GW_RELEASE_HDR;
      }
      break;
    }

    case SPEED_GW_RELEASE_HDR:
      sweepStatsReset(g_statGwRelease);
      g_speedCurveIdx = 0;
      Serial.println();
      Serial.println("--- getWave sweep: release phase ---");
      Serial.print("curves 0-");
      Serial.print(ADSR_BENCHMARK_CURVE_COUNT - 1);
      Serial.print(", instances=");
      Serial.print(ADSR_BENCHMARK_INSTANCES);
      Serial.print(", iter=");
      Serial.println(ADSR_BENCHMARK_ITERATIONS);
      g_speedStep = SPEED_GW_RELEASE_CELL;
      break;

    case SPEED_GW_RELEASE_CELL: {
      for (uint8_t i = 0; i < ADSR_BENCHMARK_INSTANCES; i++) {
        pinPhaseRelease(*g_envs[i], g_speedCurveIdx);
      }
      const uint32_t cycles = benchGetWaveCycles(ADSR_BENCHMARK_ITERATIONS);
      const uint32_t cpc = cyclesPerGetWave(cycles, ADSR_BENCHMARK_ITERATIONS);
      g_getWaveRow[g_speedCurveIdx] = cpc;
      sweepStatsUpdate(g_statGwRelease, cpc, g_speedCurveIdx, 0);
      sweepStatsUpdate(g_statGwGlobal, cpc, g_speedCurveIdx, 3);
      g_speedCurveIdx++;
      if (g_speedCurveIdx >= ADSR_BENCHMARK_CURVE_COUNT) {
        printGetWaveCurveRow(g_getWaveRow);
        printGetWavePhaseMinMax("release", g_statGwRelease);
        g_speedStep = SPEED_SUMMARY;
      }
      break;
    }

    case SPEED_SUMMARY: {
      Serial.println();
      Serial.println("=== Speed summary ===");
      if (g_statSetAttack.valid) {
        Serial.print("setAttack   min=");
        Serial.print(g_statSetAttack.minCycles);
        Serial.print(" max=");
        Serial.println(g_statSetAttack.maxCycles);
      }
      if (g_statSetDecay.valid) {
        Serial.print("setDecay    min=");
        Serial.print(g_statSetDecay.minCycles);
        Serial.print(" max=");
        Serial.println(g_statSetDecay.maxCycles);
      }
      if (g_statSetSustain.valid) {
        Serial.print("setSustain  min=");
        Serial.print(g_statSetSustain.minCycles);
        Serial.print(" max=");
        Serial.println(g_statSetSustain.maxCycles);
      }
      if (g_statSetRelease.valid) {
        Serial.print("setRelease  min=");
        Serial.print(g_statSetRelease.minCycles);
        Serial.print(" max=");
        Serial.println(g_statSetRelease.maxCycles);
      }
      if (g_statGwGlobal.valid) {
        Serial.print("getWave     min=");
        Serial.print(g_statGwGlobal.minCycles);
        Serial.print(" max=");
        Serial.print(g_statGwGlobal.maxCycles);
        Serial.print(" cycles/getWave (phase=");
        Serial.print(kGwPhaseNames[g_statGwGlobal.minValue]);
        Serial.print(" curve=");
        Serial.print(g_statGwGlobal.minCurve);
        Serial.println(")");
        const uint32_t minCpc = g_statGwGlobal.minCycles;
        const uint64_t callsPerSec = 10000ULL * (uint64_t)ADSR_BENCHMARK_INSTANCES;
        const uint64_t cyclesPerSec = (uint64_t)minCpc * callsPerSec;
        const uint32_t cpuPct = (uint32_t)((cyclesPerSec * 100ULL) / (uint64_t)F_CPU);
        Serial.print("DCO budget: ");
        Serial.print(minCpc);
        Serial.print(" cycles/call x ");
        Serial.print(ADSR_BENCHMARK_INSTANCES);
        Serial.print(" instances @ 10kHz => ~");
        Serial.print(cpuPct);
        Serial.println("% CPU");
      }
      Serial.println();
      g_speedStep = SPEED_FINISHED;
      break;
    }

    case SPEED_FINISHED:
      break;

    default:
      g_speedStep = SPEED_FINISHED;
      break;
  }
}
#endif

#if ADSR_BENCHMARK_FINGERPRINT
static void runFingerprint() {
  Serial.println("--- Envelope fingerprint ---");
  initCurveTypes(g_testEnv, 7, 2, 1);
  setEnvParams(g_testEnv, 500, 500, kSustainHalfDco, 500);
  g_testEnv.noteOn();

  uint32_t checksum = 2166136261UL;
  for (int i = 0; i < 4000; i++) {
    delay(1);
    int v = g_testEnv.getWave();
    checksum ^= (uint32_t)v;
    checksum *= 16777619UL;
  }
  releaseTestEnv();

  Serial.print("fingerprint=0x");
  Serial.println(checksum, HEX);
  Serial.println();
}
#endif

static BenchPhase initialPhase() {
#if ADSR_BENCHMARK_SELFTEST
  return PHASE_SELFTEST_HEADER;
#elif ADSR_BENCHMARK_SPEED
  return PHASE_SPEED;
#elif ADSR_BENCHMARK_FINGERPRINT
  return PHASE_FINGERPRINT;
#else
  return PHASE_DONE;
#endif
}

static void waitForSerialConnection() {
#if ADSR_BENCHMARK_WAIT_SERIAL
  unsigned long t0 = millis();
  while (!Serial) {
    delay(10);
#if ADSR_BENCHMARK_SERIAL_TIMEOUT_MS > 0
    if (millis() - t0 >= (unsigned long)ADSR_BENCHMARK_SERIAL_TIMEOUT_MS) {
      break;
    }
#endif
  }
  delay(100);
#endif
}

void setup() {
  Serial.begin(115200);
  waitForSerialConnection();
  Serial.println("Serial connected, starting...");

  Serial.println("Init: building curve tables...");
  adsrBezierInitTables((float)kTableMaxVal, ARRAY_SIZE, _curve_tables);
  Serial.println("Init: curve tables done.");

  Serial.println("Init: creating envelope instances...");
  initBenchmarkInstances();
  Serial.println("Init: instances ready.");

  printBanner();
  g_phase = initialPhase();
}

void loop() {
  switch (g_phase) {
#if ADSR_BENCHMARK_SELFTEST
    case PHASE_SELFTEST_HEADER:
      Serial.println("--- Self-test ---");
      g_phase = PHASE_ST_IDLE;
      break;

    case PHASE_ST_IDLE:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("idle");
#endif
      if (g_testEnv.getWave() == 0) {
#ifndef ADSR_BENCHMARK_QUIET
        Serial.println("PASS");
#else
        reportPass("idle");
#endif
      } else {
#ifndef ADSR_BENCHMARK_QUIET
        Serial.println("FAIL");
        g_fail_count++;
#else
        reportFail("idle", "expected 0");
#endif
      }
      g_phase = PHASE_ST_ZERO_ATTACK;
      break;

    case PHASE_ST_ZERO_ATTACK:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("zero_attack");
#endif
      prepareTestEnv();
      g_testEnv.setAttack(0);
      g_testEnv.setDecay(0);
      g_testEnv.setSustain(kSustainHalfDco);
      g_testEnv.setRelease(100);
      g_testEnv.noteOff();
      g_testEnv.noteOn();
      {
        int v = g_testEnv.getWave();
        if (v == kEnvDcoResolution || v == kSustainHalfDco) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("zero_attack");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          selfTestFailMeasured("unexpected level", v);
#else
          reportFailValue("zero_attack", "unexpected level", v);
#endif
        }
      }
      releaseTestEnv();
      g_phase = PHASE_ST_ATTACK_RISE;
      break;

    case PHASE_ST_ATTACK_RISE:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("attack_rise");
#endif
      prepareTestEnv();
      g_testEnv.setAttack(30);
      g_testEnv.setDecay(0);
      g_testEnv.setSustain(kEnvDcoResolution);
      g_testEnv.setRelease(100);
      g_testEnv.noteOff();
      g_testEnv.noteOn();
      {
        int v0 = g_testEnv.getWave();
        pollEnvelopeMs(25);
        int v1 = g_testEnv.getWave();
        if (v1 > v0 && v1 > 0) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("attack_rise");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          selfTestFailMeasured("level did not rise", v1);
#else
          reportFailValue("attack_rise", "level did not rise", v1);
#endif
        }
      }
      releaseTestEnv();
      g_phase = PHASE_ST_DECAY;
      break;

    case PHASE_ST_DECAY:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("decay_to_sustain");
#endif
      prepareTestEnv();
      {
        const unsigned long attackMs = 10;
        const unsigned long decayMs = 200;
        const int sustain = kSustainHalfDco;
        g_testEnv.setAttack(attackMs);
        g_testEnv.setDecay(decayMs);
        g_testEnv.setSustain(sustain);
        g_testEnv.setRelease(100);
        g_testEnv.noteOff();
        g_testEnv.noteOn();
        pollEnvelopeMs(attackMs + decayMs + 50);
        int v = g_testEnv.getWave();
        if (nearSustain(v, sustain)) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("decay_to_sustain");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          selfTestFailMeasured("level mismatch", v);
#else
          reportFailValue("decay_to_sustain", "level mismatch", v);
#endif
        }
      }
      releaseTestEnv();
      g_phase = PHASE_ST_SUSTAIN;
      break;

    case PHASE_ST_SUSTAIN:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("sustain_hold");
#endif
      prepareTestEnv();
      {
        const unsigned long attackMs = 10;
        const unsigned long decayMs = 10;
        const int sustain = kSustainHalfDco;
        g_testEnv.setAttack(attackMs);
        g_testEnv.setDecay(decayMs);
        g_testEnv.setSustain(sustain);
        g_testEnv.setRelease(100);
        g_testEnv.noteOff();
        g_testEnv.noteOn();
        pollEnvelopeMs(attackMs + decayMs + 50);
        int v1 = g_testEnv.getWave();
        pollEnvelopeMs(10);
        int v2 = g_testEnv.getWave();
        if (nearSustain(v1, sustain) && nearSustain(v2, sustain)) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("sustain_hold");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          selfTestFailSustainHold(v1, v2);
#else
          reportFailValue("sustain_hold", "sustain not stable", v2);
#endif
        }
      }
      releaseTestEnv();
      g_phase = PHASE_ST_RELEASE;
      break;

    case PHASE_ST_RELEASE:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("release_to_zero");
#endif
      prepareTestEnv();
      {
        const unsigned long attackMs = 10;
        const unsigned long decayMs = 10;
        const unsigned long releaseMs = 200;
        g_testEnv.setAttack(attackMs);
        g_testEnv.setDecay(decayMs);
        g_testEnv.setSustain(kEnvDcoResolution);
        g_testEnv.setRelease(releaseMs);
        g_testEnv.noteOff();
        g_testEnv.noteOn();
        pollEnvelopeMs(attackMs + decayMs + 20);
        g_testEnv.noteOff();
        pollEnvelopeMs(releaseMs + 50);
        int end = g_testEnv.getWave();
        if (end == 0) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("release_to_zero");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          selfTestFailMeasured("expected 0", end);
#else
          reportFailValue("release_to_zero", "expected 0", end);
#endif
        }
      }
      g_phase = PHASE_ST_CLAMP;
      break;

    case PHASE_ST_CLAMP:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("output_clamp");
#endif
      prepareTestEnv();
      g_testEnv.setAttack(100);
      g_testEnv.setDecay(100);
      g_testEnv.setSustain(kSustainHalfDco);
      g_testEnv.setRelease(100);
      g_testEnv.noteOff();
      g_testEnv.noteOn();
      {
        bool ok = true;
        for (int i = 0; i < 3; i++) {
          int v = g_testEnv.getWave();
          if (v < 0 || v > kEnvDcoResolution) {
            ok = false;
            break;
          }
          pollEnvelopeMs(5);
        }
        if (ok) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("output_clamp");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("FAIL");
          g_fail_count++;
#else
          reportFail("output_clamp", "out of range");
#endif
        }
      }
      releaseTestEnv();
      g_phase = PHASE_ST_LEGATO;
      break;

    case PHASE_ST_LEGATO:
#ifndef ADSR_BENCHMARK_QUIET
      selfTestLine("legato");
#endif
      prepareTestEnv();
      g_testEnv.setResetAttack(false);
      g_testEnv.setAttack(100);
      g_testEnv.setDecay(10);
      g_testEnv.setSustain(0);
      g_testEnv.setRelease(500);
      g_testEnv.noteOn();
      pollEnvelopeMs(80);
      g_testEnv.noteOff();
      pollEnvelopeMs(30);
      {
        int before = g_testEnv.getWave();
        g_testEnv.noteOn();
        int after = g_testEnv.getWave();
        g_testEnv.setResetAttack(true);
        if (after >= before && before > 0) {
#ifndef ADSR_BENCHMARK_QUIET
          Serial.println("PASS");
#else
          reportPass("legato");
#endif
        } else {
#ifndef ADSR_BENCHMARK_QUIET
          selfTestFailMeasured("retrigger did not continue", after);
#else
          reportFailValue("legato", "retrigger did not continue from level", after);
#endif
        }
      }
      releaseTestEnv();
      g_phase = PHASE_SELFTEST_SUMMARY;
      break;

    case PHASE_SELFTEST_SUMMARY:
      Serial.print("Self-test summary: ");
      if (g_fail_count == 0) {
        Serial.println("ALL PASS");
      } else {
        Serial.print(g_fail_count);
        Serial.println(" FAILURES");
      }
      Serial.println();
#if ADSR_BENCHMARK_SPEED
      g_phase = PHASE_SPEED;
#elif ADSR_BENCHMARK_FINGERPRINT
      g_phase = PHASE_FINGERPRINT;
#else
      g_phase = PHASE_DONE;
#endif
      break;
#endif

#if ADSR_BENCHMARK_SPEED
    case PHASE_SPEED:
      if (g_speedStep == SPEED_FINISHED) {
#if ADSR_BENCHMARK_FINGERPRINT
        g_phase = PHASE_FINGERPRINT;
#else
        g_phase = PHASE_DONE;
#endif
      } else {
        runSpeedStep();
      }
      break;
#endif

#if ADSR_BENCHMARK_FINGERPRINT
    case PHASE_FINGERPRINT:
      runFingerprint();
      g_phase = PHASE_DONE;
      break;
#endif

    case PHASE_DONE: {
      static bool donePrinted = false;
      if (!donePrinted) {
        Serial.println("Done. Reset board to re-run.");
        donePrinted = true;
      }
      delay(1000);
      break;
    }

    default:
      g_phase = PHASE_DONE;
      break;
  }

  if (g_phase != PHASE_DONE) {
    delay(10);
  }
}
