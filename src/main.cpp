#include <Wire.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <VL53L1X.h>
#include "control.h"

// defines
#define AS5600_ADDRESS                0x36

#define TOF_DISTANCE_SCALAR           2                 // this is a fudge factor (couldn't figure out why readings were half what they should)
#define FUSION_DST_TRUST_FACTOR       0.025f            // 0.0f to 1.0f (how much we trust the ToF)
#define ENCODER_STATIC_CALIBRATION    63.6f             // static calibration to zero out the encoder (0/360 is straight down)
#define STARTUP_FRAMES_TOF            30
#define STARTUP_FRAMES_ENCODER        30

#define TRACK_MIN                     52.0f
#define TRACK_MAX                     322.0f

#define POLL_PERIOD_TOF_MICROSEC      30000
#define POLL_PERIOD_ENCODER_MICROSEC  0

#define STEPPER_STEP_PIN              9
#define STEPPER_DIR_PIN               8

#define LOG_INTERVAL_MS               250

// globals: modules
VL53L1X distanceSensor;
FastAccelStepperEngine stepperEngine  = FastAccelStepperEngine();
FastAccelStepper* stepper             = NULL;
Control* control                      = NULL;

// globals: cart/stepper position fusion (ToF & stepper delta)
float cartPos_fusion                  = 0.0f;
long lastStepperPos                   = 0;

// globals: startup & calibration
unsigned long startupFrames_ToF       = STARTUP_FRAMES_TOF;
unsigned long startupFrames_Encoder   = STARTUP_FRAMES_ENCODER;
float encoderDynamicCalibration       = 0.0f;

// globals: timing
unsigned long lastLoopMicros          = 0;
unsigned long lastLogTime             = 0;
long timeTillPollTOF_microSec         = 0;
long timeTillPollEncoder_microSec     = 0;

// globals: cached values
float sensorAngleCached               = 0;

bool initToF() {
  distanceSensor.setTimeout(500);
  if (!distanceSensor.init()) {
    return false;
  }

  // Optimized for 2cm - 34cm range
  distanceSensor.setDistanceMode(VL53L1X::Short); 
  distanceSensor.setMeasurementTimingBudget(20000); // in micro seconds (20ms for fastest reaction)
  distanceSensor.startContinuous(20);              // in ms

  // Set a smaller Region of Interest (ROI), Minimum size is 4x4. 
  //  The center pixel for a 16x16 array is 199 (same center for any ROI size).
  distanceSensor.setROISize(6, 6);
  distanceSensor.setROICenter(199);
    
  return true;
}

bool initStepper() {
  stepperEngine.init();
  stepper = stepperEngine.stepperConnectToPin(STEPPER_STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(STEPPER_DIR_PIN);
    stepper->setAcceleration(STEPPER_ACCEL_SETUP);
    stepper->setSpeedInHz(STEPPER_SPEED_IN_HZ_SETUP);
  }
  return true;
}

bool initControl() {
  control = new Control(TRACK_MIN, TRACK_MAX, stepper);
  return true;
}

void setup() {
  // basic setup
  Serial.begin(115200);

  // I2C
  Wire.begin();
  Wire.setClock(400000);        // 400 kHz I2C for faster scans

  // tof distance sensor
  if (!initToF()) {
    Serial.println("*** Error: ToF Sensor failed");  
    while (1);
  }

  // stepper
  initStepper();

  // constrol
  initControl();

  Serial.println("initialization complete");
}

long getCartDistanceMM_ToF() {
  long distance = distanceSensor.read(false);
  return distance * TOF_DISTANCE_SCALAR;
}

void updateCartDistanceFusion(long deltaTime_microSec) {
  // predict based on delta from stepper
  long currentStepperPos = stepper->getCurrentPosition();
  long deltaSteps = currentStepperPos - lastStepperPos;
  float deltaMM = (float)deltaSteps / STEPPER_STEPS_PER_MM;

  // Update our estimate purely based on high-speed motor data
  cartPos_fusion += deltaMM; 
  lastStepperPos = currentStepperPos;

  // read ToF (don't read it every time, use cached otherwise)
  timeTillPollTOF_microSec -= deltaTime_microSec;
  if ((timeTillPollTOF_microSec <= 0) || (startupFrames_ToF > 0)) {
    timeTillPollTOF_microSec = POLL_PERIOD_TOF_MICROSEC;
    startupFrames_ToF = (startupFrames_ToF > 0) ? startupFrames_ToF - 1 : startupFrames_ToF;

    // Adjust Trust Factor dynamically based on movement (during startup, trust the ToF)
    float currentTrust = (startupFrames_ToF == 0) ? 1.0f :
                            FUSION_DST_TRUST_FACTOR * ((stepper->getCurrentSpeedInMilliHz() == 0) ?  0.05f : 1.0f);

    // fusion for updated distance estimate (complementary filter w/ adaptive gain)
    float sensorDistance = getCartDistanceMM_ToF(); 
    cartPos_fusion += currentTrust * (sensorDistance - cartPos_fusion);
  }
}

float getCartPosMM() {
  return cartPos_fusion;
}

float getPendulumAngleDegrees(long deltaTime_microSec) {
  // don't read it every time, use cached otherwise (unless during startup frames...in which case we are calibrating)
  timeTillPollEncoder_microSec -= deltaTime_microSec;
  if ((timeTillPollEncoder_microSec > 0) && (startupFrames_Encoder == 0)) {
    return sensorAngleCached;
  }
  timeTillPollEncoder_microSec = POLL_PERIOD_ENCODER_MICROSEC;

  // use the high byte of the angle
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0E); 
  Wire.endTransmission();

  // request 2 bytes of data (high and low byte)
  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();

    // combine bytes to get the 12-bit value (0 - 4095)
    uint16_t rawAngle = (highByte << 8) | lowByte;

    // convert to Degrees
    float degrees = rawAngle * (360.0 / 4096.0);

    // calibration: static
    degrees -= ENCODER_STATIC_CALIBRATION + ((startupFrames_Encoder == 0) ? encoderDynamicCalibration : 0.0f);
    degrees = degrees < 0 ? degrees + 360.0f : degrees;
    degrees = -degrees + 360.0f;

    // calibration: dynamic
    if (startupFrames_Encoder > 0) {
      float offsetToZero = degrees > 180.0f ? 360.0f - degrees : 0.0f - degrees;
      if (abs(offsetToZero) < 3.0f) {
        encoderDynamicCalibration += offsetToZero;
        startupFrames_Encoder--;
        if (startupFrames_Encoder == 0) {
          encoderDynamicCalibration /= STARTUP_FRAMES_ENCODER;
        }
      }
    }

    sensorAngleCached = degrees;
    return degrees;
  }

  // failure, try again as soon as possible
  timeTillPollEncoder_microSec = 0;
  return 0;
}

void moveCart(float cartPosDeltaMM) {
  // estimate cart's stopping distance
  float cartSpeedMilliHz = stepper->getCurrentSpeedInMilliHz();
  float cartStepsPerSec = (float)cartSpeedMilliHz / 1000.0f;
  float cartVelocityMM = cartStepsPerSec / STEPPER_STEPS_PER_MM;
  float maxDecelMM = stepper->getAcceleration() / STEPPER_STEPS_PER_MM;
  float stopDstMM = (cartVelocityMM * cartVelocityMM) / (2.0f * maxDecelMM);  // note squared, always positive

  // guard agaist going past edges
  if ((cartPos_fusion - stopDstMM + cartPosDeltaMM < TRACK_MIN && cartPosDeltaMM < 0) ||
      (cartPos_fusion + stopDstMM + cartPosDeltaMM > TRACK_MAX && cartPosDeltaMM > 0)) {
    stepper->stopMove();
    return;
  }

  // get the current target and add the offset
  long stepOffset = (long)(cartPosDeltaMM * STEPPER_STEPS_PER_MM);
  if (stepOffset == 0) {
    return;
  }
  long newTarget = stepper->getCurrentPosition() + stepOffset;
  
  // update the stepper target
  stepper->moveTo(newTarget);
}

void loop() {
  // calc deltaTime_microSec
  unsigned long currentMicros = micros();
  unsigned long deltaTime_microSec = currentMicros - lastLoopMicros;
  lastLoopMicros = currentMicros;

  // read state (also need to update distance fusion estimate)...
  float angleDeg = getPendulumAngleDegrees(deltaTime_microSec);
  updateCartDistanceFusion(deltaTime_microSec);
  float posMM = getCartPosMM();

  // control logic - returns the cart/stepper target position
  float cartPosDeltaMM = control->calcCartDeltaMM(angleDeg, posMM, deltaTime_microSec);
  moveCart(cartPosDeltaMM);
  
  // log (periodically)
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentTime;
      
    Serial.print("state="); Serial.print(control->getState(), 1); 
    //Serial.print(", pos="); Serial.print(posMM, 0);
    //Serial.print(" mm, ang="); Serial.print(angleDeg, 1); 
    Serial.print(", angVel="); Serial.print(control->getAngVelocity(), 1);
    Serial.print(", energy="); Serial.print(control->getEnergy_total(), 2);
    Serial.print(", dt="); Serial.print(deltaTime_microSec);
    Serial.println();
  }
}