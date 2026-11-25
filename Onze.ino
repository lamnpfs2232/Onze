#include <Wire.h>
#include <DFPongController.h>
#include <Adafruit_APDS9999.h>
#include <Adafruit_LSM6DS3TRC.h>

const int   CONTROLLER_NUMBER = 16;
const char* CONTROLLER_NAME   = "Onze";

const int GYRO_LED_PIN = 27;
const int PROX_LED_PIN = 33;

const unsigned long STEP_MS     = 150;
const unsigned long COOLDOWN_MS = 300;

const int   PROX_DELTA        = 6;
const unsigned long PROX_SAMPLE_MS = 30;
const int   PROX_CONFIRM      = 2;
const int   PROX_AVG          = 3;
const float PROX_BASE_SPEED   = 0.01;

const apds9999_prox_resolution_t PROX_RES         = APDS9999_PROX_RES_11BIT;
const apds9999_prox_meas_rate_t  PROX_RATE        = APDS9999_PROX_RATE_6MS;
const apds9999_led_current_t     PROX_LED_CURRENT = APDS9999_LED_CURRENT_25MA;
const uint8_t                    PROX_LED_PULSES  = 16;

const float JOLT_THRESHOLD = 25.0;
const float JOLT_RESET     = 3.0;

const int SIZE_FOR_LEDS[3] = {100, 150, 200};

const unsigned long SIZE_SETTLE_MS = 1500;

const unsigned long POINT_DEBOUNCE_MS      = 2000;
const unsigned long BUFF_RESET_FALLBACK_MS = 8000;

const char* COLOR_FOR_LEDS[3] = {
  "#39ff14",
  "#ff4500",
  "#ffd700"
};

const char* HIT_SOUND   = "wood";
const char* SCORE_SOUND = "https://github.com/lamnpfs2232/Sound/raw/refs/heads/main/douze%20death%20sound.mp3";
const char* WIN_SOUND   = "https://github.com/lamnpfs2232/Sound/raw/refs/heads/main/onze%20victory.mp3";

DFPongController    controller;
Adafruit_APDS9999   apds;
Adafruit_LSM6DS3TRC imu;

bool hasProx = false;
bool hasImu  = false;

uint16_t lastHandRaw = 0;
int   handRaw      = 0;
float proxBase     = -1;
float proxDiff     = 0;
bool  proxArmed    = true;
int   proxAbove    = 0;
int   proxHist[8];
int   proxHistN    = 0;
unsigned long proxSampleAt = 0;
unsigned long proxLastAt   = 0;
bool  proxLedOn    = false;

float joltValue    = 0;
bool  joltArmed    = true;
unsigned long joltLastAt = 0;
bool  gyroLedOn    = false;

int   stepDirection = NEUTRAL;
unsigned long stepEnd = 0;

bool  buffRound = false;
bool  resetSent = false;
unsigned long buffRoundStartedAt = 0;
unsigned long lastPointAt = 0;

int   sentSize  = -1;
int   sentColor = -1;
bool  sizeDirty = false;
unsigned long ledsChangedAt = 0;

volatile bool pointEnded  = false;
volatile bool matchStart  = false;
volatile bool matchEnded  = false;
volatile bool playStarted = false;

// Called by the DF Pong library whenever the game sends me an event.
// It only raises flags on point ended.
// so that loop() can react to them safely.
void gameEvent(int event, int value) {
  Serial.print("[event ");
  Serial.print(event);
  Serial.print(" value ");
  Serial.print(value);
  Serial.println("]");
  if (event == EVENT_COUNTDOWN && value == 0) playStarted = true;
  if (event == EVENT_HIT)                      playStarted = true;
  if (event == EVENT_SCORED || event == EVENT_CONCEDED) pointEnded = true;
  if (event == EVENT_MATCH_START)                        matchStart = true;
  if (event == EVENT_WON || event == EVENT_LOST)         matchEnded = true;
}

// Counts how many mole eye pairs are lit right now (0, 1 or 2).
// This number decides the paddle buff.
int ledsLit() {
  return (gyroLedOn ? 1 : 0) + (proxLedOn ? 1 : 0);
}

// Sends a paddle size to the game, but only when it is different from
// the last size I sent, so the game is not spammed with the same value.
void sendSize(int size) {
  if (size == sentSize) return;
  controller.setPaddleSize(size);
  sentSize = size;
}

// Sends the paddle color for a buff level (0, 1 or 2) to the game,
// again only when it changed.
void sendColor(int level) {
  if (level == sentColor) return;
  controller.setPlayerColor(COLOR_FOR_LEDS[level]);
  sentColor = level;
}

// Starts a fresh earn round: both mole eyes off, paddle back to normal
// size and default color.
void startRound() {
  buffRound = false;
  gyroLedOn = false;
  proxLedOn = false;
  sizeDirty = false;
  sendSize(SIZE_FOR_LEDS[0]);
  sendColor(0);
}

// Starts one paddle step (UP or DOWN) that lasts STEP_MS.
void startStep(int direction, unsigned long now) {
  stepDirection = direction;
  stepEnd = now + STEP_MS;
}

// Reads the raw proximity value from the APDS9999.
// If a reading fails, it returns the last good value instead.
int readProximityRaw() {
  uint16_t prox;
  if (apds.readProximity(&prox)) {
    lastHandRaw = prox;
  }
  return (int)lastHandRaw;
}

// Right button: returns true once per press.
// It averages the last few proximity samples and compares them to a slow
// baseline. A press counts when the average stays above the baseline for
// PROX_CONFIRM samples in a row, and the value must drop again before the
// next press can count.
bool detectPress(int raw, unsigned long now) {
  if (now - proxSampleAt < PROX_SAMPLE_MS) return false;
  proxSampleAt = now;

  int n = constrain(PROX_AVG, 1, 8);
  for (int i = min(proxHistN, n - 1); i > 0; i--) proxHist[i] = proxHist[i - 1];
  proxHist[0] = raw;
  if (proxHistN < n) proxHistN++;
  float sum = 0;
  for (int i = 0; i < proxHistN; i++) sum += proxHist[i];
  float avg = sum / proxHistN;

  if (proxBase < 0) { proxBase = avg; return false; }

  proxDiff = avg - proxBase;
  bool fired = false;

  if (proxDiff < PROX_DELTA * 0.5f) proxArmed = true;
  proxAbove = (proxDiff >= PROX_DELTA) ? proxAbove + 1 : 0;

  if (proxArmed && proxAbove >= PROX_CONFIRM && now - proxLastAt > COOLDOWN_MS) {
    proxArmed  = false;
    proxLastAt = now;
    fired = true;
  }

  if (avg < proxBase) proxBase = avg;
  else proxBase += PROX_BASE_SPEED * proxDiff;
  return fired;
}

// Left button: returns true once per jolt.
// It takes the total acceleration and subtracts gravity, so a resting
// button reads about 0 and a press shows up as a spike. The value must
// calm down before the next jolt can count.
bool detectJolt(unsigned long now) {
  if (!hasImu) return false;
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  float mag = sqrtf(accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.y * accel.acceleration.y +
                    accel.acceleration.z * accel.acceleration.z);
  joltValue = fabsf(mag - 9.81f);

  if (joltValue < JOLT_RESET) joltArmed = true;
  if (joltArmed && joltValue > JOLT_THRESHOLD && now - joltLastAt > COOLDOWN_MS) {
    joltArmed = false;
    joltLastAt = now;
    return true;
  }
  return false;
}

// Reacts to the start of a match and to the end of each point.
// After an earn round, lit eyes start a buff round (size and color from the
// number lit); no lit eyes start a new earn round. After a buff round,
// everything resets. A second point event right after the first is ignored.
void handleMatchStartAndPointEnd(unsigned long now) {
  if (matchStart) {
    matchStart = false;
    startRound();
    Serial.println("=== MATCH START: paddle normal, new earn round");
  }

  if (pointEnded) {
    pointEnded = false;

    if (now - lastPointAt < POINT_DEBOUNCE_MS) {
      Serial.println("(second point event ignored)");
      return;
    }
    lastPointAt = now;

    if (!buffRound) {

      int n = ledsLit();
      if (n > 0) {
        sendSize(SIZE_FOR_LEDS[n]);
        sizeDirty = false;
        sendColor(n);
        buffRound = true;
        resetSent = false;
        playStarted = false;
        buffRoundStartedAt = now;
        Serial.print("=== EARN ROUND END: ");
        Serial.print(n);
        Serial.print(" pair(s) lit -> BUFF round, paddle ");
        Serial.print(SIZE_FOR_LEDS[n]);
        Serial.println("%");
      } else {
        startRound();
        Serial.println("=== EARN ROUND END: 0 lit -> new earn round");
      }
    } else {

      startRound();
      Serial.println("=== BUFF ROUND END: reset, new earn round");
    }
  }
}

// During the buff round, once the rally has really started, sends the
// normal paddle size so it applies from the next serve.
void handleBuffResetQueue(unsigned long now) {
  if (buffRound && !resetSent &&
      (playStarted || now - buffRoundStartedAt > BUFF_RESET_FALLBACK_MS)) {
    sendSize(SIZE_FOR_LEDS[0]);
    resetSent = true;
    Serial.println("Buff round under way: next serve goes back to normal size");
  }
}

// When the whole match is over, resets everything to the starting state.
void handleMatchEnd() {
  if (matchEnded) {
    matchEnded = false;
    startRound();
    Serial.println("=== MATCH END: paddle back to normal");
  }
}

// Left button: a jolt moves the paddle UP one step.
// In an earn round it also toggles the left mole's eyes.
void handleLeftButton(unsigned long now) {
  if (detectJolt(now)) {
    startStep(UP, now);
    if (!buffRound) {
      gyroLedOn = !gyroLedOn;
      sizeDirty = true;  ledsChangedAt = now;
      Serial.println("JOLT -> UP, LEDs 27 toggled");
    } else {
      Serial.println("JOLT -> UP (buff round: LEDs locked)");
    }
  }
}

// Right button: a press moves the paddle DOWN one step.
// In an earn round it also toggles the right mole's eyes.
void handleRightButton(unsigned long now) {
  if (!hasProx) return;
  handRaw = readProximityRaw();
  if (detectPress(handRaw, now)) {
    startStep(DOWN, now);
    if (!buffRound) {
      proxLedOn = !proxLedOn;
      sizeDirty = true;  ledsChangedAt = now;
      Serial.println("PRESS -> DOWN, LEDs 33 toggled");
    } else {
      Serial.println("PRESS -> DOWN (buff round: LEDs locked)");
    }
  }
}

// Sends the new paddle size once the eyes have stopped changing for
// SIZE_SETTLE_MS, so quick toggles are sent to the game only once.
void handleSizeSend(unsigned long now) {
  if (sizeDirty && !buffRound && now - ledsChangedAt >= SIZE_SETTLE_MS) {
    sendSize(SIZE_FOR_LEDS[ledsLit()]);
    sizeDirty = false;
  }
}

// Shows the two mole eye pairs on the LED pins.
void updateLeds() {
  digitalWrite(GYRO_LED_PIN, gyroLedOn ? HIGH : LOW);
  digitalWrite(PROX_LED_PIN, proxLedOn ? HIGH : LOW);
}

// Sends the paddle command to the game: UP or DOWN while a step is
// running, otherwise STOP. Returns the command that was sent.
int sendMovement(unsigned long now) {
  int command = (now < stepEnd) ? stepDirection : NEUTRAL;
  controller.sendControl(command);
  return command;
}

// Runs once at start-up: powers the I2C port, sets up the LEDs and both
// sensors, then sets the controller number, name, sounds and events
// and starts Bluetooth.
void setup() {
  Serial.begin(115200);

#ifdef PIN_I2C_POWER
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#elif defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif
  delay(50);

  pinMode(GYRO_LED_PIN, OUTPUT);
  pinMode(PROX_LED_PIN, OUTPUT);
  digitalWrite(GYRO_LED_PIN, LOW);
  digitalWrite(PROX_LED_PIN, LOW);

  Wire.begin();

  hasProx = apds.begin();
  if (hasProx) {
    apds.enableProximitySensor(true);
    apds.setProxResolution(PROX_RES);
    apds.setProxMeasRate(PROX_RATE);
    apds.setLEDCurrent(PROX_LED_CURRENT);
    apds.setLEDPulses(PROX_LED_PULSES);
  } else {
    Serial.println("APDS9999 not found - check wiring");
  }

  hasImu = imu.begin_I2C();
  if (!hasImu) {
    Serial.println("LSM6DS3TR-C not found - check wiring");
  } else {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
  }

  controller.setControllerNumber(CONTROLLER_NUMBER);
  controller.setControllerName(CONTROLLER_NAME);
  controller.setHitSound(HIT_SOUND);
  controller.setScoreSound(SCORE_SOUND);
  controller.setWinSound(WIN_SOUND);
  startRound();
  controller.setStatusLED(LED_BUILTIN);
  controller.onGameEvent(gameEvent);
  controller.begin();
}

// Runs over and over: handles game events, reads both buttons, sends the
// paddle size and movement, updates the LEDs and prints the debug line.
void loop() {
  controller.update();
  unsigned long now = millis();

  handleMatchStartAndPointEnd(now);
  handleBuffResetQueue(now);
  handleMatchEnd();
  handleLeftButton(now);
  handleRightButton(now);
  handleSizeSend(now);
  updateLeds();
  int command = sendMovement(now);
  printDebug(now, command);

  delay(5);
}

// Prints the live sensor values and state every 200 ms to the Serial
// Monitor, which I use for tuning the thresholds.
void printDebug(unsigned long now, int command) {
  static unsigned long lastPrint = 0;
  if (now - lastPrint <= 200) return;
  lastPrint = now;
  Serial.print("jolt: ");
  Serial.print(joltValue, 2);
  Serial.print("  | hand raw: ");
  Serial.print(handRaw);
  Serial.print(" (base ");
  Serial.print(proxBase, 1);
  Serial.print(" diff ");
  Serial.print(proxDiff, 1);
  Serial.print(")");
  Serial.print("  | LEDs 27: ");
  Serial.print(gyroLedOn ? "ON" : "off");
  Serial.print("  33: ");
  Serial.print(proxLedOn ? "ON" : "off");
  Serial.print(buffRound ? "  [BUFF ROUND]" : "  [earn]");
  Serial.print("  | cmd: ");
  Serial.println(command == UP ? "UP" : command == DOWN ? "DOWN" : "STOP");
}
