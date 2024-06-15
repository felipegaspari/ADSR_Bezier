// --------------------------------------------------
//
// ADSR - based on lookup table - Arduino Due
// by mo_thunderz (mo-thunderz @github)
//
// this example periodically triggers the adsr and has been written
// for the Arduino Due. For use on other boards just reduce the
// DACSIZE, remove the analogWriteResolution(12) statement and write
// to a different port in this statement:
// analogWrite(DAC0, adsr_class.getWave(t));
//
// Parameters:
// *) trigger_duration = duration of the adsr trigger (in µs)
// *) space_between_trigger = duration between triggers (in µs)
// *) adsr_attack = attack time (in µs)
// *) adsr_decay = decay time (in µs)
// *) adsr_sustain = level for sustain (0 to DACSIZE - 1)
// *) adsr_release = release time (in µs)
//
// --------------------------------------------------

#include <adsr_fela_bezier.h>                                   // import class

#define DACSIZE 4096                                // vertical resolution of the DACs
uint16_t MaxValue = 4095;

// variables
unsigned long   adsr_attack = 1000000;               // time in µs
unsigned long   adsr_decay = 1000000;                // time in µs
int             adsr_sustain = 2500;                // sustain level -> from 0 to DACSIZE-1
unsigned long   adsr_release = 1000000;              // time in µs
unsigned long   trigger_duration = 3000000;          // time in µs
unsigned long   space_between_triggers = 3000000;    // time in µs

// internal variables
bool trigger_on = false;                            // simple bool to switch trigger on and off
unsigned long   t = 0;                              // timestamp: current time
unsigned long   t_0 = 0;                            // timestamp: last trigger on/off event

// internal classes
//  adsr class: maxValue, attack curve(old method), decay release curve(old method), bool if true use old method else use bezier curves, attack curve type, decay curve type)
// Attack curve types: 0: Standard soft, 1: Softer start, 2:very steep, 3: concave, 4: fast start, dead middle, aggresive rise
// Decay/release curve types: 0: Standard soft, 1: Softer start, 2:very steep, 3: convex, 4: fast start, dead middle, aggresive dive
adsr adsr_class(MaxValue, 0.9995f, 0.9995f, false,1,2);                // ADSR class initialization

void setup() {
  Serial.begin(2000000);
  delay(100);

  adsr_class.setAttack(adsr_attack);                // initialize attack
  adsr_class.setDecay(adsr_decay);                  // initialize decay
  adsr_class.setSustain(adsr_sustain);              // initialize sustain
  adsr_class.setRelease(adsr_release);              // initialize release
}

void loop() {
  t = micros();                                     // take timestamp

										  // Use the arduino IDE Serial Plotter to take a look at the resulting envelope
  Serial.print("0 ");
  Serial.print(MaxValue + (String) " ");
  Serial.println(adsr_class.getWave(t));  // update ADSR and write to DAC

  if (trigger_on) {                                 
    if (t > t_0 + trigger_duration) {               // check if trigger can be switched off
      adsr_class.noteOff(t);                        // inform ADSR class that note/trigger is switched off
                    // switch off LED
      t_0 = t;                                      // reset timestamp
      trigger_on = false;                           // set trigger_on to false
    }
  }
  else {
    if (t > t_0 + space_between_triggers) {            // check if trigger can be switched on
      adsr_class.noteOn(t);                         // inform ADSR class that note/trigger is switched on
           // switch on LED
      t_0 = t;                                      // reset timestamp
      trigger_on = true;                            // set trigger_on to false
    }
  }

  delay(100);
}

