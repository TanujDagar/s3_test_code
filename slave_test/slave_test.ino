/*
 * ESP32-S3 OPTIMIZED SLAVE (Final Version - NO STALL PROTECTION)
 * - Synced Telemetry (50ms)
 * - Optimized Hardware CAN Filtering
 * - CAN Timeout Safety ONLY
 * * UPLOAD TO: FR (0x121), BR (0x122), BL (0x123)
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ================== CHANGE PER MOTOR ==================
#define MY_CAN_ID    0x121        // FR: 0x121 | BR: 0x122 | BL: 0x123
#define MY_FB_ID     0x221        // FR: 0x221 | BR: 0x222 | BL: 0x223
#define MOTOR_NAME   "Front Right"
// ======================================================

// --- PINS ---
#define RGB_LED_PIN     6
#define NUM_PIXELS      2
#define LED_BRIGHTNESS  50

#define CAN_TX_PIN      GPIO_NUM_1
#define CAN_RX_PIN      GPIO_NUM_2

#define ENCODER_A_PIN   4
#define ENCODER_B_PIN   5
#define PWM_PIN         10
#define DIR_PIN         11
#define PCNT_UNIT       PCNT_UNIT_0

// --- TUNING & CONSTANTS ---
#define ENCODER_PPR          400
#define KP                   0.1418
#define KI                   2.683
#define KD                   0.0
#define MOTOR_GAIN           9.670
#define MOTOR_OFFSET         350.574

#define PWM_FREQ             25000
#define PWM_RESOLUTION       8
#define PWM_MAX              255
#define MIN_PWM_OUTPUT       20
#define RPM_SAMPLE_TIME_MS   50  // 20Hz Update Rate
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0
#define INTEGRAL_LIMIT       1000.0
#define MAX_CAN_SPEED        1000
#define MAX_RPM              2500
#define CAN_TIMEOUT_MS       500

Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

typedef struct {
  int16_t speed;
  int8_t  direction;
} __attribute__((packed)) motor_cmd_t;

typedef struct {
  float position;
  float velocity;
} __attribute__((packed)) motor_feedback_t;

// --- GLOBALS ---
float targetRPM = 0.0;
float currentRPM = 0.0;
float filteredRPM = 0.0;
int currentPWM = 0;
volatile long globalEncoderTicks = 0;
int16_t last_pulse_count = 0;

const float TICKS_TO_RAD = (2.0 * PI) / (ENCODER_PPR * 4.0);
const float RPM_TO_RADS = (2.0 * PI) / 60.0;

float errorSum = 0.0;
float lastError = 0.0;
bool firstRPM = true;

unsigned long lastRPMTime = 0;
unsigned long lastCANUpdate = 0;
unsigned long lastDebugTime = 0;

bool canTimeoutDetected = false;
uint32_t canRxCount = 0;
uint32_t canTxCount = 0;

// --- PROTOTYPES ---
bool initCAN();
void processCAN();
void sendTelemetry();
void checkCANTimeout();
void initPCNT();
void measureRPM();
void runPID();
float calculateRPM(int16_t delta_ticks);
void updateLED();

// ================== SETUP ==================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  delay(1000); // Brief wait for USB

  Serial.printf("\n=== %s SLAVE (CMD:0x%03X | FB:0x%03X) ===\n", MOTOR_NAME, MY_CAN_ID, MY_FB_ID);

  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  statusLed.fill(statusLed.Color(255, 255, 0)); // Yellow = Init
  statusLed.show();

  if (!initCAN()) {
    Serial.println("✗ CAN Init FAILED");
    statusLed.fill(statusLed.Color(255, 0, 0));
    statusLed.show();
    while(1) delay(100);
  }
  Serial.println("✓ CAN Initialized (Hardware Filter Active)");

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);

  initPCNT();
  
  lastRPMTime = millis();
  lastCANUpdate = millis();
  lastDebugTime = millis();

  statusLed.fill(statusLed.Color(0, 255, 0)); // Green = Ready
  statusLed.show();
}

// ================== MAIN LOOP ==================
void loop() {
  unsigned long now = millis();

  processCAN();       // Read Incoming Commands
  checkCANTimeout();  // Stop if Master dies

  // --- SYNCED CONTROL LOOP (20Hz / 50ms) ---
  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureRPM();      // 1. Measure Physics
    runPID();          // 2. React (Control Motor)
    sendTelemetry();   // 3. Report Physics
    
    lastRPMTime = now;
  }

  // Debug Print (1Hz)
  if (now - lastDebugTime >= 1000) {
    Serial.printf("[DEBUG] RX:%lu TX:%lu | Tgt:%.1f Cur:%.1f PWM:%d | %s\n",
                  canRxCount, canTxCount, targetRPM, currentRPM, currentPWM,
                  canTimeoutDetected ? "TIMEOUT" : "OK");
    lastDebugTime = now;
  }

  updateLED();
}

// ================== CAN BUS ==================
bool initCAN() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  
  // HARDWARE FILTER: Ignore everyone else's messages
  twai_filter_config_t f = {
    .acceptance_code = (MY_CAN_ID << 21),
    .acceptance_mask = ~(0x7FF << 21),
    .single_filter = true
  };

  if (twai_driver_install(&g, &t, &f) == ESP_OK) {
    if (twai_start() == ESP_OK) return true;
  }
  return false;
}

void processCAN() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    canRxCount++;
    if (msg.identifier == MY_CAN_ID && msg.data_length_code == sizeof(motor_cmd_t)) {
      motor_cmd_t cmd;
      memcpy(&cmd, msg.data, sizeof(cmd));

      float rpm = ((float)cmd.speed * MAX_RPM) / MAX_CAN_SPEED;
      if (cmd.direction == 0) rpm = -rpm;

      targetRPM = rpm;
      lastCANUpdate = millis();
    }
  }
}

void sendTelemetry() {
  motor_feedback_t fb;
  fb.position = globalEncoderTicks * TICKS_TO_RAD;
  fb.velocity = currentRPM * RPM_TO_RADS;

  twai_message_t msg = {};
  msg.identifier = MY_FB_ID;
  msg.data_length_code = sizeof(motor_feedback_t);
  memcpy(msg.data, &fb, sizeof(fb));

  if (twai_transmit(&msg, 0) == ESP_OK) {
    canTxCount++;
  }
}

void checkCANTimeout() {
  if (millis() - lastCANUpdate > CAN_TIMEOUT_MS) {
    targetRPM = 0;
    canTimeoutDetected = true;
  } else {
    canTimeoutDetected = false;
  }
}

// ================== ENCODER ==================
void initPCNT() {
  pcnt_config_t ch0 = {
    .pulse_gpio_num = ENCODER_A_PIN,
    .ctrl_gpio_num = ENCODER_B_PIN,
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_DEC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT,
    .channel = PCNT_CHANNEL_0
  };
  pcnt_unit_config(&ch0);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

void measureRPM() {
  int16_t current_count;
  pcnt_get_counter_value(PCNT_UNIT, &current_count);
  int16_t delta = current_count - last_pulse_count;
  last_pulse_count = current_count;
  globalEncoderTicks += delta;

  float rawRPM = calculateRPM(delta);
  if (firstRPM) {
    filteredRPM = rawRPM;
    firstRPM = false;
  } else {
    filteredRPM = EMA_ALPHA * rawRPM + (1.0 - EMA_ALPHA) * filteredRPM;
  }
  currentRPM = filteredRPM;
}

float calculateRPM(int16_t delta_ticks) {
  const int CPR = ENCODER_PPR * 4;
  return (float)delta_ticks * 60000.0 / (CPR * RPM_SAMPLE_TIME_MS);
}

// ================== PID ==================
void runPID() {
  float error = targetRPM - currentRPM;

  // Deadband Check
  if (fabs(targetRPM) < 0.1 && fabs(currentRPM) < 5.0 && fabs(errorSum) < 1.0) {
    ledcWrite(PWM_PIN, 0);
    currentPWM = 0;
    errorSum = 0;
    return;
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  float p = KP * error;
  
  // Anti-Windup
  bool saturated = (currentPWM >= PWM_MAX && error > 0) || (currentPWM <= -PWM_MAX && error < 0);
  if (!saturated) {
    errorSum += error * (RPM_SAMPLE_TIME_MS / 1000.0);
    errorSum = constrain(errorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }
  
  float i = KI * errorSum;
  float d = KD * (error - lastError);
  
  float ffMag = (fabs(targetRPM) + MOTOR_OFFSET) / MOTOR_GAIN;
  float ff = (targetRPM >= 0) ? ffMag : -ffMag;
  
  float control = ff + p + i + d;

  if (control >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    currentPWM = (int)control;
  } else {
    digitalWrite(DIR_PIN, LOW);
    currentPWM = (int)fabs(control);
  }

  if (currentPWM > 0 && currentPWM < MIN_PWM_OUTPUT) currentPWM = MIN_PWM_OUTPUT;
  currentPWM = constrain(currentPWM, 0, PWM_MAX);
  
  ledcWrite(PWM_PIN, currentPWM);
  lastError = error;
}

// ================== LED ==================
void updateLED() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  if (millis() - lastBlink > 300) {
    blinkState = !blinkState;
    lastBlink = millis();
  }

  uint32_t color = 0;

  if (canTimeoutDetected) {
    color = blinkState ? statusLed.Color(255, 0, 255) : 0; // MAGENTA BLINK = TIMEOUT
  } else if (targetRPM != 0) {
    color = statusLed.Color(0, 0, 255); // BLUE = RUNNING
  } else if (blinkState) {
    color = statusLed.Color(0, 50, 0); // GREEN BLINK = IDLE
  }

  statusLed.fill(color);
  statusLed.show();
}