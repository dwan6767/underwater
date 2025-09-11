/**************************************************************
 This code is a generic one ,to configure either master or slave remove the comments 
 accordingly to choose the mode 
 Author: Dwaipayan Shikari
 **************************************************************/
#define ROLE_MASTER   // in this case it is master code
//#define ROLE_SLAVE

#include <Arduino.h>
#include <avr/pgmspace.h>  // works both on esp8266 and avr architechture , used to data storing on flash memory

/* ---------------- Input Output Pins ---------------- */
#ifdef ROLE_MASTER
  const uint8_t PIN_RX_A = A0;           // master input
  const uint8_t PIN_TX_DIGITAL = D2;          //master output
#else
  const uint8_t PIN_RX_A = A0;              //slave input
  const uint8_t PIN_TX_DIGITAL = 8;        //slave output
#endif

/* ---------------- Threshold ---------------- */
const int ADC_THRESHOLD = 400; // if greater than 400 high otherwise low

/* ---------------- Protocol & timing ----------------
This the main to constants for communication , the master will send or request data to slave by sending 2hz signal , slaves sends back 4hz as an acknowledgement
*/
const float TONE_SLAVE_HZ  = 2.0f;    // slave-control (request/ack)
const float TONE_MASTER_HZ = 4.0f;    // master-control

const unsigned long CONTROL_TONE_DURATION_MS = 3000UL; // control tone durations are 3s 
const float CONTROL_TONE_TOLERANCE_HZ = 0.4f; //added small hysteresis
const int CONTROL_TONE_CONFIRM = 4;

#define LETTER_ON_MS 1200UL   //each letter for 1.2 s
#define GAP_MS 250UL  //gaps are 250ms
#define SETTLE_MS 60UL  //plus settling time
/*     If silent for more than 300 or 800 ms */
const unsigned long SILENCE_TIMEOUT_MS = 300UL; 
const unsigned long WORD_SILENCE_MS   = 800UL;
const int MIN_SAMPLES_TO_VOTE = 4;
const int MAX_BUF = 300;

/* letter half-period table (microseconds) for A..Z */
const uint32_t letterDelay[26] PROGMEM = {
  1000,1923,2846,3769,4692,5615,6538,7461,8384,9307,
  10230,11153,12076,12999,13922,14845,15768,16691,17614,18537,
  19460,20383,21306,22229,23152,25000
}; //all the frequency informations are stored in flash to reduce ram usage

#ifdef ESP8266
  #define YIELD_LOOP() yield()
#else
  #define YIELD_LOOP() do{}while(0)
#endif

/* ---------- Analog activity check ---------- */
inline bool analogActive(int analogVal) {
  return analogVal > ADC_THRESHOLD;
}

/* ---------- Helpers ---------- */
void float_line() {
  pinMode(PIN_TX_DIGITAL, INPUT);
  digitalWrite(PIN_TX_DIGITAL, LOW);
  delay(SETTLE_MS);
}
/*Sending Tone is just Toggling the microcontrollers , but the catch is we should not use delay in that */
void send_tone(float freqHz, unsigned long durationMs) {
  if (freqHz <= 0.0f) return;
  unsigned long halfPeriodUs = (unsigned long)(1000000.0f / (2.0f * freqHz));
  pinMode(PIN_TX_DIGITAL, OUTPUT);
  digitalWrite(PIN_TX_DIGITAL, LOW);
  unsigned long start = millis();
  bool state = false;
  while (millis() - start < durationMs) {
    digitalWrite(PIN_TX_DIGITAL, state ? HIGH : LOW);
    state = !state;
    unsigned long t0 = micros();
    while ((unsigned long)(micros() - t0) < halfPeriodUs) {
      YIELD_LOOP();
    }
  }
  float_line();
}

void SEND(const char *msg) {
  Serial.print(F("[SEND] "));
  Serial.println(msg);
  for (int i = 0; msg[i] != '\0'; ++i) {
    char c = msg[i];
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    if (c == ' ') {
      float_line();
      delay(GAP_MS);
      continue;
    }
    if (c < 'A' || c > 'Z') continue;
    uint32_t halfUs = pgm_read_dword(&letterDelay[c - 'A']);
    if (halfUs < 100) halfUs = 100;
    pinMode(PIN_TX_DIGITAL, OUTPUT);
    digitalWrite(PIN_TX_DIGITAL, LOW);
    unsigned long startMs = millis();
    unsigned long lastToggle = micros();
    bool pinState = false;
    while (millis() - startMs < LETTER_ON_MS) {
      unsigned long now = micros();
      if ((unsigned long)(now - lastToggle) >= halfUs) {
        lastToggle = now;
        pinState = !pinState;
        digitalWrite(PIN_TX_DIGITAL, pinState ? HIGH : LOW);
      }
      YIELD_LOOP();
    }
    float_line();
    delay(GAP_MS);
  }
}

float measure_freq_once(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  int raw0 = analogRead(PIN_RX_A);
  bool prevActive = analogActive(raw0);
  unsigned long lastCross = micros();
  while (millis() - t0 < timeoutMs) {
    int val = analogRead(PIN_RX_A);
    bool nowActive = analogActive(val);
    if (nowActive && !prevActive) {
      unsigned long nowMic = micros();
      unsigned long period = nowMic - lastCross;
      lastCross = nowMic;
      if (period > 100 && period < 2000000UL) {
        float freq = 1000000.0f / (float)period;
        return freq;
      }
    }
    prevActive = nowActive;
    YIELD_LOOP();
  }
  return 0.0f;
}

int detect_control_tone(unsigned long listenMs) {
  int countSlave = 0, countMaster = 0;
  unsigned long tstart = millis();
  while (millis() - tstart < listenMs) {
    float f = measure_freq_once(1200);
    if (f <= 0) continue;
    if (fabs(f - TONE_SLAVE_HZ) <= CONTROL_TONE_TOLERANCE_HZ) {
      countSlave++; countMaster = 0;
    } else if (fabs(f - TONE_MASTER_HZ) <= CONTROL_TONE_TOLERANCE_HZ) {
      countMaster++; countSlave = 0;
    } else {
      countSlave = countMaster = 0;
    }
    if (countSlave >= CONTROL_TONE_CONFIRM) return (int)TONE_SLAVE_HZ;
    if (countMaster >= CONTROL_TONE_CONFIRM) return (int)TONE_MASTER_HZ;
    YIELD_LOOP();
  }
  return 0;
}

char decode_letter_from_freq(float f) {
  float best = 1e9; char bestChar = '?';
  for (int i = 0; i < 26; ++i) {
    uint32_t D = pgm_read_dword(&letterDelay[i]);
    float fRef = 1000000.0f / (2.0f * (float)D);
    float diff = fabs(f - fRef);
    if (diff < best) { best = diff; bestChar = 'A' + i; }
  }
  return bestChar;
}

String RECEIVE_untilStop(int stopToneHz, unsigned long overallTimeoutMs) {
  float freqBuf[MAX_BUF];
  int bufCount = 0;
  unsigned long lastCrossMicros = micros();
  unsigned long lastEdgeMillis = millis();
  int raw0 = analogRead(PIN_RX_A);
  bool prevActive = analogActive(raw0);
  unsigned long tStart = millis();
  String result = "";
  unsigned long lastSpaceMillis = 0;

  while (millis() - tStart < overallTimeoutMs) {
    int val = analogRead(PIN_RX_A);
    bool nowActive = analogActive(val);

    if (nowActive && !prevActive) {
      unsigned long nowMic = micros();
      unsigned long period = nowMic - lastCrossMicros;
      lastCrossMicros = nowMic;
      lastEdgeMillis = millis();

      if (period > 100 && period < 2000000UL) {
        float freq = 1000000.0f / (float)period;

        if (stopToneHz > 0 && fabs(freq - (float)stopToneHz) <= CONTROL_TONE_TOLERANCE_HZ) {
          int confirm = 1;
          for (int k = 0; k < CONTROL_TONE_CONFIRM; ++k) {
            float f2 = measure_freq_once(1600);
            if (f2 > 0 && fabs(f2 - (float)stopToneHz) <= CONTROL_TONE_TOLERANCE_HZ) confirm++;
            else break;
          }
          if (confirm >= CONTROL_TONE_CONFIRM) {
            Serial.println();
            return result;
          }
        }

        if (bufCount < MAX_BUF) freqBuf[bufCount++] = freq;
      }
    }

    prevActive = nowActive;

    if ((millis() - lastEdgeMillis) > SILENCE_TIMEOUT_MS) {
      if (bufCount >= MIN_SAMPLES_TO_VOTE) {
        int counts[26] = {0};
        for (int i = 0; i < bufCount; ++i) {
          char c = decode_letter_from_freq(freqBuf[i]);
          if (c >= 'A' && c <= 'Z') counts[c - 'A']++;
        }
        int maxIdx = -1, maxVal = 0;
        for (int i = 0; i < 26; ++i) {
          if (counts[i] > maxVal) { maxVal = counts[i]; maxIdx = i; }
        }
        if (maxIdx >= 0) {
          char outc = 'A' + maxIdx;
          Serial.print(outc);
          result += outc;
        } else {
          Serial.print(' ');
          result += ' ';
        }
      }
      bufCount = 0;
      lastEdgeMillis = millis();
    }

    unsigned long silence = millis() - lastEdgeMillis;
    if (silence > WORD_SILENCE_MS) {
      if (millis() - lastSpaceMillis > WORD_SILENCE_MS) {
        Serial.print(' ');
        result += ' ';
        lastSpaceMillis = millis();
        lastEdgeMillis = millis();
      }
    }

    YIELD_LOOP();
  }
  return result;
}

/* ---------- High-level helpers ---------- */
void high_send_tone_then_SEND(float controlToneHz, const char* msg) {
  send_tone(controlToneHz, CONTROL_TONE_DURATION_MS);
  delay(140);
  SEND(msg);
  delay(140);
  send_tone(controlToneHz, CONTROL_TONE_DURATION_MS);
}

/* ---------- Role APIs ---------- */
#ifdef ROLE_MASTER
void masterSend(const char* msg) {
  Serial.println(F("[MASTER] Sending..."));
  high_send_tone_then_SEND(TONE_MASTER_HZ, msg);
  Serial.println(F("[MASTER] Waiting for slave ACK (2Hz)..."));
  int who = detect_control_tone(10000);
  if (who == (int)TONE_SLAVE_HZ) Serial.println(F("[MASTER] Slave ACK received"));
  else Serial.println(F("[MASTER] No ACK (timeout)"));
}

void masterRequestData(unsigned long receiveTimeoutMs = 15000) {
  Serial.println(F("[MASTER] Requesting data from slave (2Hz)..."));
  send_tone(TONE_SLAVE_HZ, CONTROL_TONE_DURATION_MS);
  delay(140);
  Serial.println(F("[MASTER] Receiving data from slave:"));
  String s = RECEIVE_untilStop((int)TONE_SLAVE_HZ, receiveTimeoutMs);
  Serial.print(F("[MASTER] Slave sent: "));
  Serial.println(s);
}
#endif

#ifdef ROLE_SLAVE
const char SLAVE_DEFAULT_MSG[] = "HELLO EVERYONE MY NAME IS NANO";

void slaveSendAndAck(const char* msg) {
  Serial.println(F("[SLAVE] Sending reply..."));
  SEND(msg);
  send_tone(TONE_SLAVE_HZ, CONTROL_TONE_DURATION_MS);
  Serial.println(F("[SLAVE] ACK (2Hz) sent"));
}

void slaveListenLoop() {
  int t = detect_control_tone(18000);
  if (t == 0) return;
  if (t == (int)TONE_SLAVE_HZ) {
    Serial.println(F("[SLAVE] Master requested data -> sending"));
    slaveSendAndAck(SLAVE_DEFAULT_MSG);
  } else if (t == (int)TONE_MASTER_HZ) {
    Serial.println(F("[SLAVE] Master will send -> listening"));
    String s = RECEIVE_untilStop((int)TONE_MASTER_HZ, 20000);
    Serial.print(F("[SLAVE] Received from master: "));
    Serial.println(s);
    send_tone(TONE_SLAVE_HZ, CONTROL_TONE_DURATION_MS);
    Serial.println(F("[SLAVE] Sent ACK after receive"));
  }
}
#endif

/* ---------- Setup & Loop ---------- */
void setup() {
  pinMode(14,OUTPUT);
    pinMode(12,OUTPUT);
  Serial.begin(115200);
  float_line();
  pinMode(PIN_RX_A, INPUT);
  delay(50);

  #ifdef ROLE_MASTER
    Serial.println(F("== ROLE: MASTER =="));
    Serial.print(F("RX = A0, TX pin = ")); Serial.println(PIN_TX_DIGITAL);
    Serial.print(F("ADC threshold: > ")); Serial.println(ADC_THRESHOLD);
    Serial.println(F("Serial commands: S:msg  R  T2  T4  DUMP"));
  #else
    Serial.println(F("== ROLE: SLAVE =="));
    Serial.print(F("RX = A0, TX pin = ")); Serial.println(PIN_TX_DIGITAL);
    Serial.print(F("ADC threshold: > ")); Serial.println(ADC_THRESHOLD);
  #endif
}

void loop() {
  #ifdef ROLE_MASTER
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.startsWith("S:")) {
        digitalWrite(14,1);
        masterSend(line.substring(2).c_str());
        digitalWrite(14,0);
      } else if (line == "R") {
        digitalWrite(12,1);
        masterRequestData();
        digitalWrite(12,0);
      } else if (line == "T2") {
        send_tone(TONE_SLAVE_HZ, CONTROL_TONE_DURATION_MS);
      } else if (line == "T4") {
        send_tone(TONE_MASTER_HZ, CONTROL_TONE_DURATION_MS);
      } else if (line == "DUMP") {
        Serial.println(F("Dumping 40 ADC samples:"));
        for (int i = 0; i < 40; ++i) {
          Serial.println(analogRead(PIN_RX_A));
          delay(50);
        }
      }
    }
  #else
    slaveListenLoop();
  #endif
}
