#ifndef ACTUATOR_MODULE
#define ACTUATOR_MODULE

#include "Arduino.h"
#include "definitions.h"

struct Actuator
{
  public:
    enum State {
      STILL,
      PUSH,
      PULL
    };

  private:

    PIN_TYPE motor_push_pin;
    PIN_TYPE motor_pull_pin;

    int max_distance_pwm;

    State current_state;
    bool change_required;

    GlobalState* global_state;

  public:

    CONST_PIN_TYPE ANALOUGE_PIN = 0;

    Actuator (PIN_TYPE motor_push_pin, PIN_TYPE motor_pull_pin,
              int max_distance_pwm, GlobalState* global_state)
    {
        this->current_state = STILL;
        this->motor_push_pin = motor_push_pin;
        this->motor_pull_pin = motor_pull_pin;
        this->max_distance_pwm = max_distance_pwm;
        this->global_state = global_state;
    }

    void setup()
    {
        this->global_state->actuator_at_max_push = false;
        // Serial.println("Setting up actiator");
        this->setState(PULL);
        // printState();
        while (this->current_state != STILL)
        {
            this->motorLoop();
            //Serial.print("Setup loop - current state: ");
            //Serial.println(this->current_state);
        }
        //Serial.println("Setup loop done");
    }

    void printState()
    {
      Serial.print("Actuator current state: ");
      Serial.print(this->current_state);
      Serial.print(" - and PWM value: ");
      Serial.println(analogRead(ANALOUGE_PIN));
    }

    void setState(State state)
    {
      // Serial.print("Current state: ");
      // Serial.println(this->current_state);
      if (state != this->current_state)
      {
        this->current_state = state;
        this->change_required = true;
      }
    }

    void motorLoop()
    {
      // Serial.print("Motor loop - ");
      //printState();
      if (this->current_state == STILL)
      {
        return;
      }

      int sensor_value = analogRead(ANALOUGE_PIN);
      if (sensor_value >= this->max_distance_pwm -30) // Leave a bit of buffer
      {
        this->global_state->actuator_at_max_push = true;
      }
      else
      {
        this->global_state->actuator_at_max_push = false;
      }

      if (this->current_state == PUSH)
      {
          if (sensor_value >= this->max_distance_pwm)
          {
              digitalWrite(this->motor_push_pin, LOW);
              digitalWrite(this->motor_pull_pin, LOW);
              this->current_state = STILL;
              this->change_required = false;
          }

          if (this->change_required)
          {
              digitalWrite(this->motor_push_pin, HIGH);
              digitalWrite(this->motor_pull_pin, LOW);
              this->change_required = false;
          }
      }
      else if (this->current_state == PULL)
      {
          if (sensor_value <= 20) // Doesn't have to be completely retracted
          {
              digitalWrite(this->motor_push_pin, LOW);
              digitalWrite(this->motor_pull_pin, LOW);
              this->current_state = STILL;
              this->change_required = false;
          }

          if (this->change_required)
          {
              digitalWrite(this->motor_push_pin, LOW);
              digitalWrite(this->motor_pull_pin, HIGH);
              this->change_required = false;
          }
      }
    }
};

#endif
