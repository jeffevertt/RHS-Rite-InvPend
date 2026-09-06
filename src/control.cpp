#include "Control.h"

Control::Control(float trackMin, float trackMax, FastAccelStepper* stepper) {
    _state      = STATE_IDLE;
    _trackMin   = trackMin;
    _trackMax   = trackMax;
    _stepper    = stepper;
}

void Control::gotoState(ControlState state, float curPos)
{
    ControlState previousState = _state;
    if (state == previousState) {
        return;
    }
    _state = state;
    _timeInState = 0;

    // reset the stepper (in case it was in velocity vs position control)
    if ((previousState == STATE_STABILIZING) && (STABILIZE_USE_LQR == 1)) {
        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
    }

    // on enter state
    if (_state == STATE_SWINGUP_SETUP) {
        _stepper->setAcceleration(STEPPER_ACCEL_SETUP);
        _stepper->setSpeedInHz(STEPPER_SPEED_IN_HZ_SETUP);
    }
    else if (_state == STATE_SWINGUP) {
        _swingUpDeltaMMSmoothed = 0.0f;
        _stepper->setAcceleration(STEPPER_ACCEL_SWINGUP);
        _stepper->setSpeedInHz(STEPPER_SPEED_IN_HZ_SWINGUP);
    }
    else if (_state == STATE_STABILIZING_SETUP) {
        _stabilizingSetupTrgPosMM = curPos;
        _stabilizingSetupDeltaMM = 0.0f;
        _stabilizingSetupInitAngVel = _angVelSmoothed;
        _stepper->setAcceleration(STEPPER_ACCEL_STABILIZING_SETUP);
        _stepper->setSpeedInHz(STEPPER_SPEED_IN_HZ_STABILIZING_SETUP);
    }
    else if (_state == STATE_STABILIZING) {
        _stabilizePID_lastDstError = 0.0f;
        _stabilizePID_dstErrorIntegral = 0.0f;
        _stabilizePID_dstErrorDerivative_smoothed = 0.0f;
        _stabilizeLQR_giveUpPerc = 0.0f;
        _stabilizeLQR_dstErrorIntegral = 0.0f;
        _stepper->setAcceleration(STEPPER_ACCEL_STABILIZING);
        _stepper->setSpeedInHz(STEPPER_SPEED_IN_HZ_STABILIZING);
    }
}

float Control::calcCartDeltaMM(float curAngle, float curPos, unsigned long deltaTime_microSec) {
    // setup
    float dt = deltaTime_microSec / 1000000.0f; // seconds
    ControlState _prevState = _state;

    // setup
    updateAngVelandFriends(curAngle, dt);

    // execution
    float cartDeltaMM = 0.0f;
    switch (_state) {
        case STATE_SWINGUP_SETUP:
            cartDeltaMM = calcCartDeltaMM_swingUp_Setup(curAngle, curPos, dt);
            break;
        case STATE_SWINGUP:
            cartDeltaMM = calcCartDeltaMM_swingUp(curAngle, curPos, dt);
            break;
        case STATE_STABILIZING_SETUP:
            cartDeltaMM = calcCartDeltaMM_stabilizing_setup(curAngle, curPos, dt);
            break;
        case STATE_STABILIZING:
            if (STABILIZE_USE_LQR) {
                cartDeltaMM = calcCartDeltaMM_stabilizing_LQR(curAngle, curPos, dt);
            }
            else {
                cartDeltaMM = calcCartDeltaMM_stabilizing_cascadedCenteringPID(curAngle, curPos, dt);
            }
            break;
        default: // IDLE
            cartDeltaMM = calcCartDeltaMM_idle(curAngle, curPos, dt);
            break;
    }
    _timeInState += (_prevState == _state) ? dt : 0.0f;

    return cartDeltaMM;
}

void Control::updateAngVelandFriends(float curAngle, float dt) {
    // raw angular velocity (degrees per second)
    float rawVelocity = (curAngle - _angVel_lastAngle) / dt;
    if (rawVelocity > 180 / dt) rawVelocity -= 360 / dt;    // handle 0/360 wrap
    if (rawVelocity < -180 / dt) rawVelocity += 360 / dt;

    // low-pass Filter & convert to radians
    _angVelSmoothed = ((1.0f - ANG_VEL_SMOOTHING_FACTOR) * rawVelocity) + (ANG_VEL_SMOOTHING_FACTOR * _angVelSmoothed);
    float angVelSmoothedRadians = D2R(_angVelSmoothed);

    // energy estimates
    _energyPotential = 1.0f + cos(radians(curAngle - 180.0f));
    _energyKinetic = 0.5f * PENDULUM_MOMENT_OF_INERTIA * (angVelSmoothedRadians * angVelSmoothedRadians);

    // read for next frame
    _angVel_lastAngle = curAngle;
}

float Control::calcCartDeltaMM_idle(float curAngle, float curPos, float dt) {
    // state transition logic (0 degrees is hanging down, 180 is balanced at the top)
    if (abs(curAngle - 180.0f) < 15.0f) {
        gotoState(STATE_STABILIZING, curPos);
    }
    return 0;
}

float Control::calcCartDeltaMM_swingUp_Setup(float curAngle, float curPos, float dt) {
    // wait a short time & for the energy to dissipate
    if ((_timeInState < 2.0f) || (_energyPotential + _energyPotential > 0.05f)) {
        return 0;
    }

    // recenter
    float toCenter = trackCenter() - curPos;
    if (abs(toCenter > 5.0f)) {
        return toCenter;
    }

    // ready, let's move on
    gotoState(STATE_SWINGUP, curPos);

    return 0;
}

// symmetry-based resonance strategy (track to the negative of the arm-end horiz position)
float Control::calcCartDeltaMM_swingUp(float curAngle, float curPos, float dt) {
    // check to see if we are ready to go into stabilizing
    float energyTotal = getEnergy_total();
    if ((energyTotal > SWINGUP_MINIMUM_ENERGY) && (abs(curAngle - 180.0f) < SWINGUP_MINIMUM_ANGLE_ERROR)) {
        gotoState(STATE_STABILIZING, curPos);
        return 0;
    }

    // predict where the arm-end point will be (time estimate includes system latency + cart travel time)
    float lookAheadTime = 0.045f;
    float predictedAngleRad = radians(curAngle + _angVelSmoothed * lookAheadTime);

    // energy-based damping (for smooth transition to stabilizing)
    float energyError = max(SWINGUP_TARGET_ENERGY - energyTotal, 0.0f);
    float preStabilizeDampFactor = 1.0f - constrain(energyError * SWINGUP_ENERGY_GAIN, 0.0f, 1.0f);

    // energy injection for low energy state
    float lowEnergyPump = 0;
    if (energyTotal < SWINGUP_MINIMUM_ENERGY * 0.5f) {
        float pumpDir = (_angVelSmoothed >= 0) ? -1.0f : 1.0f;
        lowEnergyPump = pumpDir * SWINGUP_PUMP_KICK_MM;
    }

    // calculate the cart's target position
    float targetDeltaFromCenter = -1.0f * (PENDULUM_LENGTH_MM * (1.0f - preStabilizeDampFactor) * sin(predictedAngleRad)) + lowEnergyPump;
    float absoluteTarget = trackCenter() + targetDeltaFromCenter;
    float rawDelta = absoluteTarget - curPos;

    // smoothing & decay (increase decay when we are closer to the required energy)
    _swingUpDeltaMMSmoothed = (1.0f - SWINGUP_OUTPUT_SMOOTHING) * rawDelta + SWINGUP_OUTPUT_SMOOTHING * _swingUpDeltaMMSmoothed;
    float decayFactor = 0.995f - 0.1f * max((energyTotal - SWINGUP_MINIMUM_ENERGY * (0.1f * SWINGUP_RAMP_DOWN_GAIN)) * SWINGUP_RAMP_DOWN_GAIN, 0.0f);
    _swingUpDeltaMMSmoothed *= decayFactor;

    return _swingUpDeltaMMSmoothed;
}

// this state attempts to "catch" the arm by matching the arm end points velocity
float Control::calcCartDeltaMM_stabilizing_setup(float curAngle, float curPos, float dt) {
    // check for stabilization transition & for too much energy loss (fall back)
    if ((abs(curAngle - 180.0f) < 10.0f) && (_energyKinetic < 0.075f)) {
        gotoState(STATE_STABILIZING, curPos);
        return 0;
    }
    if (getEnergy_potential() < SWINGUP_TARGET_ENERGY * 0.4f || 
        abs(curAngle - 180.0f) > 60.0f || 
        abs(curPos - trackCenter()) > trackHalfWidth() * 0.7f) {
        gotoState(STATE_SWINGUP_SETUP, curPos);
        return 0;
    }

    // estimate time till vertical
    float deltaAngleFromVert = 180.0f - curAngle;
    float timeToVertical = 0.025f;
    if (abs(_angVelSmoothed) > 1.0f && (deltaAngleFromVert * _angVelSmoothed > 0)) {
        timeToVertical = abs(deltaAngleFromVert / _angVelSmoothed);
    }

    // predict head to the angle we are trying to match
    float leadTime = constrain(timeToVertical, 0.005f, 0.060f);
    float predAngleRad = radians(curAngle + (_angVelSmoothed * leadTime));

    // arm-end's horiz vel (Vx = L * omega * cos(theta))
    float targetVel = PENDULUM_LENGTH_MM * radians(_angVelSmoothed) * cos(predAngleRad);

    // velocity to delta
    float maxVelMM = (float)STEPPER_SPEED_IN_HZ_STABILIZING / STEPPER_STEPS_PER_MM;
    targetVel = constrain(targetVel, -maxVelMM, maxVelMM);
    float deltaMM = targetVel * SWINGUP_SETUP_CATCH_GAIN(getEnergy_total()) * dt;

    // only move in the direction of the initial ang velocity
    deltaMM = (_stabilizingSetupInitAngVel < 0.0f) ? max(deltaMM, 0.0f) : min(deltaMM, 0.0f);

    // apply to target position
    _stabilizingSetupTrgPosMM += deltaMM;

    // if too close to the edge of the track, drop out of the state
    if (_stabilizingSetupTrgPosMM < _trackMin + TRACK_GUARDRAIL_DST_MM || _stabilizingSetupTrgPosMM > _trackMax - TRACK_GUARDRAIL_DST_MM) {
        gotoState(STATE_SWINGUP_SETUP, curPos);
    }

    // smoothing
    _stabilizingSetupDeltaMM = ((1.0f - SWINGUP_SETUP_DELTA_SMOOTHING) * (_stabilizingSetupTrgPosMM - curPos)) + (SWINGUP_SETUP_DELTA_SMOOTHING * _stabilizingSetupDeltaMM);

    return _stabilizingSetupDeltaMM;
}

float Control::calcCartDeltaMM_stabilizing_cascadedCenteringPID(float curAngle, float curPos, float dt) {
    // state transition logic
    if (abs(curAngle - 180.0f) > 60.0f) {
        gotoState(STATE_SWINGUP_SETUP, curPos);
        return 0;
    }

    // predict the angle a bit ahead in time & use that rather than the curAngle (which is already out of date)
    float predictedAngle = curAngle + (_angVelSmoothed * STABILIZE_EXPECTED_LATENCY_ANGLE);

    // position error (secondary goal is getting to the center of the track) translated into angle error
    float posError = curPos - trackCenter();
    predictedAngle += constrain(posError * STABILIZE_CASCADE_CENTER_DST_TO_ANGLE, -STABILIZE_CASCADE_CENTER_MAX_ANGLE, STABILIZE_CASCADE_CENTER_MAX_ANGLE);

    // standard PID control code
    return calcCartDeltaMM_stabilizing_PID(predictedAngle, curPos, dt);
}

float Control::calcCartDeltaMM_stabilizing_PID(float curAngle, float curPos, float dt) {
    // linearize the input (use distance the cart needs to travel to be under the pendulum end point)
    float dstError = PENDULUM_LENGTH_MM * sin(radians(180.0f - curAngle));

    // derivative calculation (and smoothing)
    float derivative_thisFrame = (dstError - _stabilizePID_lastDstError) / dt;
    _stabilizePID_dstErrorDerivative_smoothed = (_timeInState == 0) ? derivative_thisFrame :
        (((1.0f - STABILIZE_DERIVATIVE_SMOOTHING) * derivative_thisFrame) + (STABILIZE_DERIVATIVE_SMOOTHING * _stabilizePID_dstErrorDerivative_smoothed));

    // update integral error tracker
    _stabilizePID_dstErrorIntegral += dstError * dt;
    _stabilizePID_dstErrorIntegral = constrain(_stabilizePID_dstErrorIntegral, -2.5f, 2.5f);    // constrain integral to prevent "windup"
    _stabilizePID_dstErrorIntegral *= STABILIZE_INTEGRAL_DECAY;

    // PID
    float cartDeltaMM = STABILIZE_PID_P * dstError + 
                        STABILIZE_PID_I * _stabilizePID_dstErrorIntegral + 
                        STABILIZE_PID_D * _stabilizePID_dstErrorDerivative_smoothed;

    // update trackers
    _stabilizePID_lastDstError = dstError;

    return cartDeltaMM;
}

float Control::calcCartDeltaMM_stabilizing_LQR(float curAngle, float curPos, float dt)
{
    // guard against going too close to the edges of the track
    float linVelMMpS = (float)_stepper->getCurrentSpeedInMilliHz() / (1000.0f * STEPPER_STEPS_PER_MM);
    bool inMinGuardRail = (curPos < _trackMin + TRACK_GUARDRAIL_DST_MM_FORCESTOP);
    bool inMaxGuardRail = (curPos > _trackMax - TRACK_GUARDRAIL_DST_MM_FORCESTOP);
    if (((linVelMMpS < 0) && inMinGuardRail) || ((linVelMMpS > 0) && inMaxGuardRail)) {
        // if moving toward an edge and can't stop before the guardrail, ABORT
        _stepper->forceStop();
        gotoState(STATE_SWINGUP_SETUP, curPos); 
        return 0;
    }

    // state transition logic (pendulum has falling beyond recovery)
    if (abs(curAngle - 180.0f) > 60.0f) {
        gotoState(STATE_SWINGUP_SETUP, curPos);
        return 0;
    }

    // state
    float posDeltaM = (curPos - trackCenter()) / 1000.0f; 
    float linVelMpS = linVelMMpS / 1000.0f;
    float angleDeltaRad = radians(curAngle - 180.0f); 
    float angVelRad = radians(_angVelSmoothed);

    // update integral error tracker
    _stabilizeLQR_dstErrorIntegral += angleDeltaRad * dt;
    _stabilizeLQR_dstErrorIntegral = constrain(_stabilizeLQR_dstErrorIntegral, -1.5f, 1.5f);    // constrain integral to prevent "windup"
    _stabilizeLQR_dstErrorIntegral *= STABILIZE_INTEGRAL_DECAY;

    // if ang vel is too high, progressively give it up
    float giveUpNearEdgeFactor = max(max(((_trackMin + TRACK_GUARDRAIL_DST_MM * TRACK_GUARDRAIL_GIVE_UP_DST_SCALAR) - curPos), 
                                         (curPos - (_trackMax - TRACK_GUARDRAIL_DST_MM * TRACK_GUARDRAIL_GIVE_UP_DST_SCALAR))), 0.0f) * 0.1f;
    float giveUpFactor = max((abs(angleDeltaRad) - STABILIZE_LQR_GIVE_UP_ANG_VEL) * STABILIZE_LQR_GIVE_UP_GAIN * giveUpNearEdgeFactor - 0.1f, -0.1f);
    _stabilizeLQR_giveUpPerc = constrain(_stabilizeLQR_giveUpPerc + giveUpFactor, 0.0f, 1.0f);

    // LQR Gain Vector (K) - computed in advance from...
    //  m (pend mass) 22g, L 33cm, M (cart mass) ~500g
    const float Kp = STABILIZE_LQR_GAIN_POS;            // position gain (stay near center)
    const float Kv = STABILIZE_LQR_GAIN_VEL_LIN;        // linear velocity gain (damping)
    const float Kt = STABILIZE_LQR_GAIN_ANGLE;          // angle gain (primary balancing force)
    const float Ko = STABILIZE_LQR_GAIN_ANG_VEL;        // angVel gain (the 'momentum killer')
    const float Ki = STABILIZE_LQR_GAIN_INTEGRAL;       // integral gain (for steady-state error)

    // calc target velocity (feedback)
    float targetVelMpS = -(-Kp * posDeltaM + -Kv * linVelMpS + Kt * angleDeltaRad + Ko * angVelRad + Ki * _stabilizeLQR_dstErrorIntegral);
    float targetVelMMpS = targetVelMpS * 1000.0f * (1.0f - _stabilizeLQR_giveUpPerc);

    // constrain
    float maxVelMM = (float)STEPPER_SPEED_IN_HZ_STABILIZING / STEPPER_STEPS_PER_MM;
    targetVelMMpS = constrain(targetVelMMpS, -maxVelMM, maxVelMM);

    // guardrail
    bool intentViolatesMin = (targetVelMMpS < 0 && (curPos < _trackMin + TRACK_GUARDRAIL_DST_MM_FORCESTOP));
    bool intentViolatesMax = (targetVelMMpS > 0 && (curPos > _trackMax - TRACK_GUARDRAIL_DST_MM_FORCESTOP));
    if (intentViolatesMin || intentViolatesMax) {
        _stepper->forceStop();
        gotoState(STATE_SWINGUP_SETUP, curPos); 
        return 0;
    }

    // set the stepper velocity directly for low latency control
    // use applySpeedAcceleration() to let the library handle the ramp, but the 'goal' speed is updated every frame
    int32_t targetHz = (int32_t)(targetVelMMpS * STEPPER_STEPS_PER_MM); // convert MM/S back to Hz.
    _stepper->setSpeedInHz(abs(targetHz));    
    if (targetHz > 0) _stepper->runForward();
    else if (targetHz < 0) _stepper->runBackward();
    else _stepper->stopMove();

    return 0;
}
