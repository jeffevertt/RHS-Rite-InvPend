#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <FastAccelStepper.h>

// defines
#define STEPPER_WHEEL_DIAMETER                  12.75             // wheel diam (mm)
#define STEPPER_STEPS_PER_REV                   800.0f            // 3200.0f for 1/16 (ON|ON|ON), 800.0f for 1/4 (OFF|ON|OFF)
#define STEPPER_MM_PER_REV                      (STEPPER_WHEEL_DIAMETER * 3.14159f)
#define STEPPER_STEPS_PER_MM                    (STEPPER_STEPS_PER_REV / STEPPER_MM_PER_REV)

#define STEPPER_ACCEL_SETUP                     5000
#define STEPPER_ACCEL_SWINGUP                   20000
#define STEPPER_ACCEL_STABILIZING_SETUP         30000
#define STEPPER_ACCEL_STABILIZING               60000
#define STEPPER_SPEED_IN_HZ_SETUP               1500
#define STEPPER_SPEED_IN_HZ_SWINGUP             5000
#define STEPPER_SPEED_IN_HZ_STABILIZING_SETUP   4000
#define STEPPER_SPEED_IN_HZ_STABILIZING         6000

#define PENDULUM_LENGTH_METERS                  0.305f
#define PENDULUM_LENGTH_MM                      (PENDULUM_LENGTH_METERS * 100.0f)
#define PENDULUM_MOMENT_OF_INERTIA              (PENDULUM_LENGTH_METERS / 9.81f)        // L/g

#define TRACK_GUARDRAIL_DST_MM                  15.0f
#define TRACK_GUARDRAIL_DST_MM_FORCESTOP        5.0f                                    // if using force stop, can be more aggressive
#define TRACK_GUARDRAIL_GIVE_UP_DST_SCALAR      5.0f

#define ANG_VEL_SMOOTHING_FACTOR                0.25f                                    // 0 to 1 (0 is no smoothing)

#define STABILIZE_PID_P                         40.0f
#define STABILIZE_PID_I                         0.1f
#define STABILIZE_PID_D                         1.0f
#define STABILIZE_EXPECTED_LATENCY_ANGLE        0.005f                                  // seconds
#define STABILIZE_CASCADE_CENTER_DST_TO_ANGLE   -0.03f
#define STABILIZE_CASCADE_CENTER_MAX_ANGLE      2.0f
#define STABILIZE_INTEGRAL_DECAY                0.98f
#define STABILIZE_DERIVATIVE_SMOOTHING          0.25f                                   // 0 to 1 (0 is no smoothing)

#define STABILIZE_USE_LQR                       1                                       // If not this, then falls back to cascaded PID controller
#define STABILIZE_LQR_GAIN_POS                  60.0f                                   // position gain (stay near the center)
#define STABILIZE_LQR_GAIN_VEL_LIN              12.0f                                   // linear velocity gain (cart speed damping)
#define STABILIZE_LQR_GAIN_ANGLE               150.0f                                   // angle gain (primary balancing force)
#define STABILIZE_LQR_GAIN_ANG_VEL              15.0f                                   // angVel gain (the 'momentum killer')
#define STABILIZE_LQR_GIVE_UP_ANG_VEL           0.075f                                  // angular velocity (rad/s) at which we start giving up
#define STABILIZE_LQR_GIVE_UP_GAIN              0.5f                                    // scaled by micro-sec deltaTime & signal above max

#define SWINGUP_MINIMUM_ENERGY                  2.05f
#define SWINGUP_TARGET_ENERGY                   (SWINGUP_MINIMUM_ENERGY + 0.15f)
#define SWINGUP_PUMP_KICK_MM                    7.5f
#define SWINGUP_ENERGY_GAIN                     2.0f                                    // higher is faster swingup
#define SWINGUP_RAMP_DOWN_GAIN                  1.0f                                    // slows approach to target energy
#define SWINGUP_OUTPUT_SMOOTHING                0.1f                                    // 0.0f none, 0.9f lots of smoothing

#define SWINGUP_SETUP_DELTA_SMOOTHING           0.15f                                   // 0 to 1 (0 is no smoothing)
#define SWINGUP_SETUP_CATCH_GAIN(ENERGY)        (constrain(((ENERGY)-1.975f)*20.0f, 0.0f, 2.0f))

// macros
#define D2R(deg)                                ((deg) * DEG_TO_RAD)

// states
enum ControlState {
    STATE_IDLE,
    STATE_SWINGUP_SETUP,
    STATE_SWINGUP,
    STATE_STABILIZING_SETUP,
    STATE_STABILIZING
};

// Control class
class Control {
public:
    Control(float trackMin, float trackMax, FastAccelStepper* stepper);
    
    // main entry point - calculate the cart delta position (in mm)
    float calcCartDeltaMM(float curAngle, float curPos, unsigned long deltaTime_microSec);
    
    ControlState getState() const { return _state; }
    void gotoState(ControlState state, float curPos);

    bool isControllingVelocityDirectly() const { return _state == STATE_STABILIZING && STABILIZE_USE_LQR == 1; }

    float trackCenter() const { return (_trackMin + _trackMax) * 0.5f; }
    float trackHalfWidth() const { return _trackMax - trackCenter(); }

    float getEnergy_total() const { return _energyKinetic + _energyPotential; }
    float getEnergy_potential() const { return _energyPotential; }
    float getEnergy_kinetic() const { return _energyKinetic; }
    float getAngVelocity() const { return _angVelSmoothed; }
    float getSwingUpPosTrg() const { return _swingUpDeltaMMSmoothed; }

private:
    ControlState _state;
    float _trackMin, _trackMax;
    float _timeInState = 0;                                     // seconds
    FastAccelStepper* _stepper = NULL;

    // stabilize: PID & cascaded-centering
    float _stabilizePID_lastDstError = 0.0f;
    float _stabilizePID_dstErrorIntegral = 0.0f;
    float _stabilizePID_dstErrorDerivative_smoothed = 0.0f;

    // stabilize: LQR
    float _stabilizeLQR_giveUpPerc = 0.0f;

    // stabilize setup state "the catch state"
    float _stabilizingSetupTrgPosMM = 0.0f;
    float _stabilizingSetupDeltaMM = 0.0f;
    float _stabilizingSetupInitAngVel = 0.0f;

    // swing-up state
    float _swingUpDeltaMMSmoothed = 0.0f;

    // calculations shared by the states
    float _angVel_lastAngle = 0;
    float _angVelSmoothed = 0;
    float _energyPotential = 0;
    float _energyKinetic = 0;
    void updateAngVelandFriends(float curAngle, float dt);

    // state fns
    float calcCartDeltaMM_stabilizing_setup(float curAngle, float curPos, float dt);
    float calcCartDeltaMM_stabilizing_cascadedCenteringPID(float curAngle, float curPos, float dt);
    float calcCartDeltaMM_stabilizing_PID(float curAngle, float curPos, float dt);
    float calcCartDeltaMM_stabilizing_LQR(float curAngle, float curPos, float dt);
    float calcCartDeltaMM_swingUp_Setup(float curAngle, float curPos, float dt);
    float calcCartDeltaMM_swingUp(float curAngle, float curPos, float dt);
    float calcCartDeltaMM_idle(float curAngle, float curPos, float dt);
};

#endif