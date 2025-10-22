#pragma once
#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <bezier.h>
#include <cstdlib>
#define BASE_RAMP_STEPS 5000

using namespace std;
using namespace TeensyTimerTool;

class Stepper {
    public:
        Stepper(int stepPin, int dirPin, float maxSpeed) : _stepTimer(TCK_RTC), _speedctrlTimer(TCK_RTC) {
            _max_speed = maxSpeed;
            _beziertable = lookup(maxSpeed);
            _stepPin = stepPin;
            _dirPin = dirPin;
        }

        float currentSpeed() {
            return _current_speed;
        }
        void setTargetSpeed(float targetSpeed) {
            _target_speed = targetSpeed;
        }
        void setPullInSpeed(float pullInSpeed) {
            _pullin_speed = pullInSpeed;
        }
        void setStartingSpeed(float startingSpeed) {
            _starting_speed = startingSpeed;
        }
        void setStepCounter(int stepCount) {
            _stepcount = stepCount;
        }
        
        float bezceleration(int startStep, int rampLength, float startSpeed, float endSpeed) {
            int rampsteps = _stepcount - startStep;
            float t = (float)rampsteps / (float)rampLength;
            if (t > 1.0f) t = 1.0f;

            float table_pos = t * (_beziertable.size() - 1);
            int idx_lower = (int)table_pos;
            int idx_upper = idx_lower + 1;
            if (idx_upper >= (int)_beziertable.size()) idx_upper = _beziertable.size() - 1;

            //interpolated bezceleration values
            float frac = table_pos - idx_lower;
            float scalar = _beziertable[idx_lower] * (1.0f - frac) + _beziertable[idx_upper] * frac;

            float res_speed = startSpeed + (endSpeed - startSpeed) * scalar;
            _current_speed = res_speed;
            return res_speed;
        }

        void _stepISR() {
            digitalWriteFast(_stepPin, HIGH);
            delayMicroseconds(2);
            digitalWriteFast(_stepPin, LOW);
            _stepcount++;
            if (_stepcount >= _steps_to_take) {
                stop();
            }
        }

        void stop() {
            _stepTimer.stop();
            _speedctrlTimer.stop();
            _current_speed = 0;
        }

        void speedControl() {
            float peak_speed = _max_speed;
            int actual_ramp = _ramp_steps;
            if (!_cruise) { 
                peak_speed = _pullin_speed + (_max_speed - _pullin_speed) * _scale_factor;
                actual_ramp = min(_ramp_steps, (float)_steps_to_take/2.0);
            }
            // ACCELERATION
            if (_stepcount < actual_ramp) {
                setStepSpeed(bezceleration(0, actual_ramp, _pullin_speed, peak_speed));
            } 
            // CRUISE
            else if (_stepcount < _steps_to_take - actual_ramp) {
                if (_cruise && _current_speed != _max_speed) {
                    setStepSpeed(_max_speed);
                } else if (!_cruise && _current_speed != peak_speed) {
                    setStepSpeed(peak_speed);
                }
            } 
            // DECELERATION
            else {
                if (!_decelerating) {
                    _decelerating = true;
                    _decel_step = _stepcount;
                }
                int decel_length = _steps_to_take - _decel_step;
                if (decel_length < 1) decel_length = 1;
                setStepSpeed(bezceleration(_decel_step, actual_ramp, peak_speed, _pullin_speed));
            }

            // Optional: progress print
            float percentage = ((float)_stepcount / (float)_steps_to_take) * 100.0;
            Serial.printf("PROGRESS: %.2f%%", percentage);
            for (int i = 0; i < (int)_current_speed / 100; i++) Serial.print("_");
            Serial.print("\n");

            if (_stepcount >= _steps_to_take) stop();
        }

        void setStepSpeed(float speed) {
            if (speed < 1.0) {speed = 1.0;}
            _current_speed = speed;
            float period = 1000000.0/speed;
            _stepTimer.setPeriod(period);
        }

        void moveByAngle(float angle) {
            //bezceleration: returns some speed based on start time, acceleration time, starting speed, and ending speed
            //startStepping() begins the _stepISR() function periodically, each step is counted in _stepcount
            //setStepSpeed() modifies the period on the _stepISR() callback function
            if (angle < 0) {
                digitalWrite(_dirPin, HIGH);
                angle = -angle;
            } else {
                digitalWrite(_dirPin, LOW);
            }
            _steps_to_take = abs((angle * 8)/1.8);

            if (_steps_to_take > 2 * _ramp_steps) {
                _scale_factor = 1.0;
                _cruise = true;
            } else {
                _scale_factor = (float)_steps_to_take / (2.0 * _ramp_steps);
                _cruise = false;
            }

            _decelerating = false;
            _decel_step = 0;
            _stepcount = 0;
            _current_speed = _pullin_speed;

            _stepTimer.begin([this] {this->_stepISR();}, 10000); 
            _speedctrlTimer.begin([this] {this->speedControl();}, 10000);
        }

        bool doneMoving() {
            return _stepcount >= _steps_to_take;
        }

    private:
        int _stepPin, _dirPin;
        float _ramp_steps = BASE_RAMP_STEPS;
        volatile int _stepcount = 0;
        int _start_time = 0;
        int _decel_step = 0;
        int _steps_to_take = 0;
        float _scale_factor = 1.0;
        bool _cruise = false;
        bool _decelerating = false;
        float _max_speed = 0;
        float _target_speed = 0;
        float _current_speed = 0;
        float _starting_speed = 0;
        float _pullin_speed = 0;
        vector<float> _beziertable;
        PeriodicTimer _stepTimer;
        PeriodicTimer _speedctrlTimer;
};