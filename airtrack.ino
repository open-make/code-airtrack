#import <Arduino.h>
#include <Wire.h> // Need by sensor.h
#include <SPI.h>
#include <Pixy.h>

#include "definitions.h"
#include "leds.h"
#include "pins.h"
#include "sensor.h"
#include "actuator.h"

const bool AUTOMATED_REWARD = false;
const bool SINGLE_REWARD = true;
const bool FEEDBACK_AUTOMATED_REWARD = false;

GlobalState global_state;
Sensor sensor = Sensor(Pins.Sensor);
Actuator actuator = Actuator(Pins.ActuatorPush, Pins.ActuatorPull,
                             global_state.actuator_max_pwm_distance,
                             &global_state);
Pixy pixy;

// the setup routine runs once when you press reset:
void setup()
{
    Serial.begin(115200);
    sensor.setup();
    setupLeds();
    setupPins();
    setupLanes();
    pixy.init();

    for (int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
    {
        global_state.motor_duration_entries[i].activated = false;
    }

    global_state.is_automated_reward = AUTOMATED_REWARD;
    global_state.trial_number = 0;
    global_state.was_inside_lane = false;
    // Initially report non existing lane
    global_state.last_reported_lane = global_state.NUM_OF_LANES + 1;
    // Assign any random initial value
    global_state.last_reported_light_status = 250;

    global_state.actuator_duration_activated = false;
    actuator.setup();

    // We are seeding the random so it would give us reproducible results
    // randomSeed(0);// call randomSeed(analogRead(A3)) for random order on each run
    randomSeed(analogRead(A3));
    global_state.was_inside_lane = false;

    // Assign last lane id to soeme random value so that it'd be possible to
    // choose lane 0 as the first lane
    global_state.reward_lane_id = -1;
    createShuffledChoice();
    makeNewRewardLane();
}


// the loop routine runs over and over again forever:
void loop()
{
    SubjectLocation subject_location = getSubjectLocation();
    if (subject_location.block_detected)
    {
        global_state.last_subject_location = subject_location;
    }
    else
    {
        subject_location = global_state.last_subject_location;
    }

    bool is_within_reward_lane_angle = isWithinRewardLaneAngle(subject_location);
    bool is_inside_lane = isInsideLane(subject_location);

    bool motor_pushed = false;

    SensorTouched touched_sensor = sensor.readInput();

    // Serial.print("We are within reward lane");
    if (is_inside_lane)
    {
        if (!global_state.was_inside_lane)
        {
            writeStats(Stats.ENTERED_LANE(global_state.current_lane));
        }
        global_state.was_inside_lane = true;

        // Serial.print( " and inside lane ");
        if (is_within_reward_lane_angle)
        {
            if (shouldTriggerMotor(subject_location))
            {
                // Serial.print( " and we should fire motor ");
                //Serial.println("");
                actuator.setState(Actuator::PUSH);
                if (global_state.last_reported_actuator_status != Actuator::PUSH)
                {
                    writeStats(Stats.MOTOR_PUSHED());
                    global_state.last_reported_actuator_status = Actuator::PUSH;
                    global_state.sensor_was_touched = false;
                }
                motor_pushed = true;

                if (global_state.actuator_at_max_push)
                {
                    if (!global_state.reported_motor_max_distance)
                    {
                        writeStats(Stats.MOTOR_MAX_RANGE());
                        global_state.reported_motor_max_distance = true;
                    }

                    // Allow a bit of buffer time for sensor vibration to
                    // rest before reading
                    long int time_now = millis();
                    if (global_state.max_push_current_duration <=
                        time_now - global_state.MAX_PUSH_WAIT)
                    {
                        if (!global_state.reported_motor_max_wait)
                        {
                            Serial.print("max_push_current_duration: ");
                            Serial.print(global_state.max_push_current_duration);
                            Serial.print(" - MAX_PUSH_WAIT: ");
                            Serial.print(global_state.MAX_PUSH_WAIT);
                            Serial.print("- time_now: ");
                            Serial.println(time_now);
                            writeStats(Stats.MOTOR_WAIT_DONE());
                            global_state.reported_motor_max_wait = true;
                        }

                        // Call isCorrectSensor() anyway so it'd call
                        // writeStats() on the touched sensor
                        bool is_correct_sensor = isCorrectSensor(touched_sensor);
                        if (touched_sensor.change_happened &&
                            !global_state.sensor_was_touched)
                        {
                            global_state.sensor_was_touched = true;
                            if (is_correct_sensor)
                                global_state.miss_or_wrong_touch_count = 0;
                            else
                                global_state.miss_or_wrong_touch_count += 1;
                        }

                        if (global_state.is_automated_reward ||
                            touched_sensor.change_happened)
                        {
                            checkGiveReward(global_state.is_automated_reward ||
                                            is_correct_sensor);
                        }
                    }
                }
            }
        }
        else
        {
            // Count that it's a bad trial
            // if (touched_sensor.change_happened)
            // {
            //    checkGiveReward(false);
            // }
        }
        //Serial.println("");
    }
    else // Outside a lane
    {
        if (global_state.was_inside_lane)
        {
            writeStats(Stats.EXITED_LANE(global_state.current_lane));
            // Turn off peizo if it was on
            if (global_state.peizo_motor_entry != NULL)
            {
                do
                {
                    digitalWrite(global_state.peizo_motor_entry->motor_id, LOW);
                }
                while (digitalRead(global_state.peizo_motor_entry->motor_id));
                global_state.peizo_motor_entry->activated = false;
                global_state.peizo_motor_entry = NULL;
            }
        }

        global_state.was_inside_lane = false;
        global_state.reported_motor_max_distance = false;
    }

    if (subject_location.block_detected && !motor_pushed)
    {
        actuator.setState(Actuator::PULL);
        if (global_state.last_reported_actuator_status != Actuator::PULL)
        {
            writeStats(Stats.MOTOR_PULLED());
            global_state.last_reported_actuator_status = Actuator::PULL;
        }
    }

    global_state.was_inside_lane = is_inside_lane;
    turnOffMotor();
    actuator.motorLoop();
}

bool isInsideLane(SubjectLocation subject_location)
{
    int flexible_range = 10;
    if (global_state.was_inside_lane)
    {
        if (subject_location.y >= Distances.y_threshold_max &&
            subject_location.y - Distances.y_threshold_max <= flexible_range)
        {
            subject_location.y = Distances.y_threshold_max - 1;
        }
    }
    else
    {
        if (subject_location.y <= Distances.y_threshold_max &&
            Distances.y_threshold_max - subject_location.y <= flexible_range)
        {
            subject_location.y = Distances.y_threshold_max + 1;
        }
    }

    return Distances.x_threshold_min < subject_location.x &&
           subject_location.x        < Distances.x_threshold_max &&
           Distances.y_threshold_min < subject_location.y &&
           subject_location.y        < Distances.y_threshold_max;
}

bool shouldTriggerMotor(SubjectLocation subject_location)
{
    bool is_within_distance = Distances.y_threshold_min < subject_location.y &&
                              subject_location.y < Distances.y_motor_threshold;

    if (!is_within_distance)
    {
        // Serial.println("Not withing distance");
        global_state.actuator_duration_activated = false;
        if (global_state.reward_given && !global_state.chose_new_lane)
        {
            makeNewRewardLane();
            global_state.chose_new_lane = true;
        }
        return false;
    }
    else
    {
        long int time_now = millis();
        if (!global_state.actuator_duration_activated)
        {
            global_state.chose_new_lane = false;
            // Only start counting when we are at max pwm
            setActuatorTimeout(global_state.MAX_PUSH_TIMEOUT);

            return true;
        }
        // We are still counting, retunr true until we time out
        else if (global_state.max_push_current_duration +
                 global_state.motor_timeout_duration >= time_now)
        {
            return true;
        }
        else
        {
            if (isWithinRewardLaneAngle(subject_location) &&
                !global_state.chose_new_lane)
            {
                if (!global_state.reward_given ||
                    (AUTOMATED_REWARD && !global_state.sensor_was_touched))
                {
                    writeStats(Stats.MISS_DECISION());
                    global_state.miss_or_wrong_touch_count += 1;
                }
                makeNewRewardLane();
                global_state.chose_new_lane = true;
            }

            return false;
        }
    }
}

SubjectLocation getSubjectLocation()
{
    uint16_t num_of_blocks = pixy.getBlocks();
    //Serial.print("Pixy array size: ");
    //Serial.println(num_of_blocks);

    SubjectLocation location = SubjectLocation();
    location.block_detected = num_of_blocks == 1;

    if (location.block_detected)
    {
        location.angle = pixy.blocks[0].angle;
        location.x = pixy.blocks[0].x;
        location.y = pixy.blocks[0].y;

        const bool ENABLE_POSITION_PRINT = false;
        if (ENABLE_POSITION_PRINT)
        {
            Serial.print("subject_location: x: ");
            Serial.print(location.x);
            Serial.print(" y: ");
            Serial.print(location.y);
            Serial.print(" angle: ");
            Serial.print(location.angle);
            Serial.print(" NUm. of blocks: ");
            Serial.println(num_of_blocks);
        }
    }

    return location;
}

bool isCorrectSensor(SensorTouched touched_sensor)
{
    Lane lane = global_state.lanes[global_state.current_lane];

    if (touched_sensor.left_sensor == true)
    {
        // Bug here: switch name
        writeStats(Stats.LEFT_SENSOR_TOUCHED());
        digitalWrite(Leds.SensorLeft, HIGH);

        checkForceSensorMode(true);

        if (lane.reward_sensor == sensor.LEFT_ANALOUGE_PIN)
        {
            writeStats(Stats.CORRECT_SENSOR_TOUCHED());
            return true;
        }
        else
        {
            writeStats(Stats.WRONG_SENSOR_TOUCHED());
            return false;
        }
    }
    else
    {
        digitalWrite(Leds.SensorLeft, LOW);
    }

    if (touched_sensor.right_sensor == true)
    {
        // Bug here: switch name
        writeStats(Stats.RIGHT_SENSOR_TOUCHED());
        digitalWrite(Leds.SensorRight, HIGH);

        checkForceSensorMode(false);

        if (lane.reward_sensor == sensor.RIGHT_ANALOUGE_PIN)
        {
            writeStats(Stats.CORRECT_SENSOR_TOUCHED());
            return true;
        }
        else
        {
            writeStats(Stats.WRONG_SENSOR_TOUCHED());
            return false;
        }
    }
    else
    {
        digitalWrite(Leds.SensorRight, LOW);
    }

    return false;
}

void checkForceSensorMode(bool is_left_sensor_touched)
{
    if (SINGLE_REWARD)
        return;

    if (global_state.actuator_at_max_push && !global_state.sensor_was_touched)
    {
        if (is_left_sensor_touched == global_state.last_sensor_touched_left)
        {
            global_state.same_sensor_touch_count++;
        }
        else
        {
            global_state.last_sensor_touched_left = is_left_sensor_touched;
            global_state.same_sensor_touch_count = 1;
        }

        unsigned int touch_count = global_state.same_sensor_touch_count;
        if (global_state.in_force_sensor_mode)
        {
            if (is_left_sensor_touched == global_state.is_force_sensor_left)
            {
                if (touch_count == global_state.FORCE_OTHER_SENSOR)
                {
                    global_state.in_force_sensor_mode = false;
                    // Should we reset the counter or not?
                    global_state.same_sensor_touch_count = 1;
                    writeStats(Stats.FORCE_SENSOR_OFF());
                }
            }

            if (FEEDBACK_AUTOMATED_REWARD)
            {
                if (is_left_sensor_touched == global_state.is_force_sensor_left)
                {
                    if (global_state.is_automated_reward)
                        writeStats(Stats.FEEDBACK_AUTOMATED_OFF());
                    global_state.is_automated_reward = false;
                }
                else if (global_state.miss_or_wrong_touch_count >=
                         global_state.FEEDBACK_AUTOMATED_REWARD_THRESHOLD)
                {
                    if (!global_state.is_automated_reward)
                        writeStats(Stats.FEEDBACK_AUTOMATED_ON());
                    global_state.is_automated_reward = true;
                }
            }
        }
        else if (touch_count == global_state.SAME_SENSOR_MAX_THRESHOLD)
        {
            global_state.in_force_sensor_mode = true;
            global_state.is_force_sensor_left = !is_left_sensor_touched;
            writeStats(Stats.FORCE_SENSOR_ON(!is_left_sensor_touched));
        }
    }
}

void checkGiveReward(bool give_reward)
{
    if (global_state.reward_given)
        return;

    if (give_reward)
    {
        writeStats(Stats.REWARD_GIVEN());
        Lane lane = global_state.lanes[global_state.reward_lane_id];
        if (lane.reward_sensor == sensor.RIGHT_ANALOUGE_PIN)
        {
            writeStats(Stats.SOLENOID_RIGHT_ON());
            turnOnMotor(Pins.SolenoidRight, global_state.SOLENOID_DURATION);
        }
        else if (lane.reward_sensor == sensor.LEFT_ANALOUGE_PIN)
        {
            writeStats(Stats.SOLENOID_LEFT_ON());
            turnOnMotor(Pins.SolenoidLeft, global_state.SOLENOID_DURATION);
        }
        else
        {
            Serial.println("Want to give reward - Unknown Solenoid");
        }

        setActuatorTimeout(global_state.ALLOWED_REWARD_TIMEOUT);
    }
    else
    {
        //Serial.println("No reward");
        writeStats(Stats.REWARD_NOT_GIVEN());
        setActuatorTimeout(global_state.NO_REWARD_TIMEOUT);
        global_state.peizo_motor_entry = turnOnMotor(Pins.PeizoTone,
                                                    global_state.PEIZO_TIMEOUT);
    }

    global_state.reward_given = true;
}

void makeNewRewardLane()
{
    // Assign to a non existing lane initially
    LANE_ID new_lane_id = global_state.NUM_OF_LANES + 1;
    if (global_state.in_force_sensor_mode)
    {
        bool found_lane = false;
        do
        {
            LANE_ID potential_lane_id = random(global_state.NUM_OF_LANES);
            if (potential_lane_id == global_state.reward_lane_id)
                continue;

            Lane lane = global_state.lanes[potential_lane_id];
            bool is_left = lane.reward_sensor == sensor.LEFT_ANALOUGE_PIN;
            if (is_left != global_state.is_force_sensor_left)
                continue;

            new_lane_id = potential_lane_id;
            found_lane = true;
        }
        while (!found_lane);
    }
    else
    {
        LANE_ID* list_ptr = global_state.lane_shuffle_list;
        size_t index = global_state.shuffle_list_index;

        new_lane_id = list_ptr[index];

        // This might happen if the last lane was due to a forced sensor mode
        if (new_lane_id == global_state.reward_lane_id)
        {
            // IF there is enough space then swap
            if (index < global_state.GUARANTEED_RANDOM_BOUND - 1)
            {
                LANE_ID temp = list_ptr[index];
                list_ptr[index] = list_ptr[index + 1];
                list_ptr[index + 1] = temp;
            }
            else
            {
                createShuffledChoice();
                new_lane_id = list_ptr[0];
            }
        }

        global_state.shuffle_list_index++;
    }

    global_state.reward_lane_id = new_lane_id;
    global_state.reward_given = false;

    // We must assign first thenew lane before calling createShuffledChoice()
    if (global_state.shuffle_list_index == global_state.GUARANTEED_RANDOM_BOUND)
    {
        createShuffledChoice();
    }

    writeStats(Stats.NEW_LANE(new_lane_id));
    writeStats(Stats.NEW_TRIAL(global_state.trial_number));
    global_state.trial_number += 1;

    printRewardLane();
}

void createShuffledChoice()
{
    if (global_state.GUARANTEED_RANDOM_BOUND % global_state.NUM_OF_LANES != 0)
    {
        Serial.println("Num of lanes is not divisble by random bound");
    }

    int j_max = global_state.GUARANTEED_RANDOM_BOUND/global_state.NUM_OF_LANES;
    for (int i = 0; i < global_state.NUM_OF_LANES; i++)
    {
        for (int j = 0; j < j_max; j++)
        {
            size_t index = j + (i*j_max);
            global_state.lane_shuffle_list[index] = i;
        }
    }

    int last_lane_value = global_state.reward_lane_id;
    // Serial.print("Starting with last lane value: ");
    // Serial.println(last_lane_value);
    for (int i = 0; i < global_state.GUARANTEED_RANDOM_BOUND; i++)
    {
        int r_index;
        int new_value = global_state.lane_shuffle_list[i];
        int count = 0;
        bool print = true;
        do
        {
            r_index = random(i, global_state.GUARANTEED_RANDOM_BOUND);
            new_value = global_state.lane_shuffle_list[r_index];
            // Don't get stuck if it's the last element. Roll backwards
            if (count > global_state.GUARANTEED_RANDOM_BOUND - 1)
            {
                int decrement = global_state.GUARANTEED_RANDOM_BOUND /
                                global_state.NUM_OF_LANES;
                if (decrement < i)
                    i -= decrement;
                else
                    i = 0;

                if (i)
                    last_lane_value = global_state.lane_shuffle_list[i - 1];
                else
                    last_lane_value = global_state.reward_lane_id;
                // Serial.print("Stepped in error handling - ");
                // Serial.print("Resetting i to: ");
                // Serial.print(i);
                // Serial.print(" and last lane value to: ");
                // Serial.println(last_lane_value);
                count = 0;
            }
            count++;
        }
        while (last_lane_value == new_value);

        int old_value = global_state.lane_shuffle_list[i];
        global_state.lane_shuffle_list[i] = new_value;
        global_state.lane_shuffle_list[r_index] = old_value;

        last_lane_value = new_value;
    }

    global_state.shuffle_list_index = 0;

    //printShuffleList();
}

void printShuffleList()
{
    LANE_ID lane_counter[global_state.NUM_OF_LANES];
    for (int i = 0; i < global_state.NUM_OF_LANES; i++)
        lane_counter[i] = 0;

    Serial.print("[");
    for (int i = 0; i < global_state.GUARANTEED_RANDOM_BOUND; i++)
    {
        Serial.print(global_state.lane_shuffle_list[i] + 1);
        if (i !=global_state.GUARANTEED_RANDOM_BOUND - 1)
            Serial.print(", ");

        lane_counter[global_state.lane_shuffle_list[i]]++;
    }
    Serial.println("]");

    for (int i = 0; i < global_state.NUM_OF_LANES; i++)
    {
        Serial.print("Count for lane ");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.println(lane_counter[i]);
    }
}


void setActuatorTimeout(long int actuator_time_out)
{
    if (global_state.actuator_at_max_push)
    {
        long int time_now = millis();
        global_state.motor_timeout_duration = actuator_time_out;
        global_state.max_push_current_duration = time_now;
        global_state.actuator_duration_activated = true;
    }
}

void setupPins()
{
    // Solenoid
    pinMode(Pins.SolenoidLeft, OUTPUT);
    pinMode(Pins.SolenoidRight, OUTPUT);
    digitalWrite(Pins.SolenoidLeft, LOW);
    digitalWrite(Pins.SolenoidRight, LOW);

    // Actuator
    pinMode(Pins.ActuatorPush, OUTPUT);
    pinMode(Pins.ActuatorPull, OUTPUT);
    digitalWrite(Pins.ActuatorPush, LOW);
    digitalWrite(Pins.ActuatorPush, LOW);

    pinMode(Pins.LaneLight, OUTPUT);
    digitalWrite(Pins.LaneLight, LOW);

    pinMode(Pins.PeizoTone, OUTPUT);
    digitalWrite(Pins.PeizoTone, LOW);
}

void setupLeds()
{
  //pinMode(Leds.Solenoid, OUTPUT);
  //digitalWrite(Leds.Solenoid, LOW);

  // pinMode(Leds.ActuatorPush, OUTPUT);
  // digitalWrite(Leds.ActuatorPush, HIGH);

  // pinMode(Leds.ActuatorPull, OUTPUT);
  // digitalWrite(Leds.ActuatorPull, HIGH);

  // pinMode(Leds.Unused, OUTPUT);
  // digitalWrite(Leds.Unused, LOW);

  pinMode(Leds.SensorLeft, OUTPUT);
  digitalWrite(Leds.SensorLeft, LOW);

  pinMode(Leds.SensorRight, OUTPUT);
  digitalWrite(Leds.SensorRight, LOW);
}

void setupLanes()
{
    Lane lane;

    lane.lane_id = 0;
    if (SINGLE_REWARD)
        lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
    else
        lane.reward_sensor = sensor.RIGHT_ANALOUGE_PIN;
    lane.region_start_angle = 45;
    lane.region_end_angle =  135;
    global_state.lanes[lane.lane_id] = lane;

    lane.lane_id = 1;
    if (SINGLE_REWARD)
        lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
    else
        lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
    lane.region_start_angle = 136;
    lane.region_end_angle =  -136;
    global_state.lanes[lane.lane_id] = lane;

    lane.lane_id = 2;
    if (SINGLE_REWARD)
        lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
    else
        lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
    lane.region_start_angle = -135;
    lane.region_end_angle =  -46;
    global_state.lanes[lane.lane_id] = lane;

    lane.lane_id = 3;
    if (SINGLE_REWARD)
        lane.reward_sensor = sensor.LEFT_ANALOUGE_PIN;
    else
        lane.reward_sensor = sensor.RIGHT_ANALOUGE_PIN;
    lane.region_start_angle = -45;
    lane.region_end_angle =  44;
    global_state.lanes[lane.lane_id] = lane;
}

bool isWithinRewardLaneAngle(SubjectLocation subject_location)
{
    if (!subject_location.block_detected)
    {
        //Serial.println("Current location: No block detected");
        return false;
    }


    for (int j = 0; j < global_state.NUM_OF_LANES; j++)
    {
        // Sometimes the angle jumps forward and backwards between the  current
        // lane and the next lane. So we need to give a bit of extra buffer to
        // the current until we are sure it's in the next lane. To do so, start
        // looping from the current lane;
        int i = (global_state.current_lane + j) % global_state.NUM_OF_LANES;

        int flexible_range = 0;
        if (i == global_state.current_lane)
            flexible_range = 10;

        Lane lane = global_state.lanes[i];

        int start = lane.region_start_angle - flexible_range;
        int end = lane.region_end_angle + flexible_range;
        int angle = subject_location.angle;

        if (end < start)
            end += 360;

        if (angle < start)
            angle += 360;

        if (start <= angle && angle <= end)
        {
            global_state.current_lane = i;

            if (global_state.last_reported_lane != i)
            {
                writeStats(Stats.ENTERED_LANE_RANGE(i));
                global_state.last_reported_lane = i;
            }

            if (lane.lane_id == global_state.reward_lane_id)
            {
                digitalWrite(Pins.LaneLight, LOW);
                if (global_state.last_reported_light_status != LOW)
                {
                    writeStats(Stats.LIGHT_OFF());
                    global_state.last_reported_light_status = LOW;
                }
                return true;
            }
            else
            {
                digitalWrite(Pins.LaneLight, HIGH);
                if (global_state.last_reported_light_status != HIGH)
                {
                    writeStats(Stats.LIGHT_ON());
                    global_state.last_reported_light_status = HIGH;
                }
                return false;
            }
        }
    }

    Serial.print("Unexpected code redirection. Subject location angle: ");
    Serial.println(subject_location.angle);
    return false;
}

void printRewardLane()
{
    // Already printed by the stats
    //Lane reward_lane = global_state.lanes[global_state.reward_lane_id];
    //Serial.print("Next reward lane id: ");
    //Serial.println(reward_lane.lane_id);
}


MotorDurationEntry* turnOnMotor(PIN_TYPE motor_id, long int activation_period)
{
    long int time_now = millis();
    for(int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
    {
        MotorDurationEntry* motor_entry = &global_state.motor_duration_entries[i];
        if (!motor_entry->activated)
        {
            do
            {
                digitalWrite(motor_id, HIGH);
            }
            while (!digitalRead(motor_id));
            motor_entry->activated = true;
            motor_entry->motor_id = motor_id;
            motor_entry->activation_time = time_now;
            motor_entry->timeout_period = activation_period;
            //Serial.print("Setting high on pin: ");
            //Serial.println(motor_id);
            return motor_entry;
        }
    }

    Serial.print("Didn't find a non-empty motor slot to turn on motor pin: ");
    Serial.println(motor_id);
    return NULL;
}

void turnOffMotor()
{
    long int time_now = millis();
    for(int i = 0; i < global_state.MOTOR_DURATION_ENTERIES_SIZE; i++)
    {
        MotorDurationEntry* motor_entry = &global_state.motor_duration_entries[i];
        if (motor_entry->activated)
        {
            if (motor_entry->activation_time + motor_entry->timeout_period < time_now)
            {
                do
                {
                    digitalWrite(motor_entry->motor_id, LOW);
                }
                while (digitalRead(motor_entry->motor_id));
                motor_entry->activated = false;

                if (motor_entry->motor_id == Pins.SolenoidRight)
                    writeStats(Stats.SOLENOID_RIGHT_OFF());
                else if (motor_entry->motor_id == Pins.SolenoidLeft)
                    writeStats(Stats.SOLENOID_LEFT_OFF());
            }
        }
    }
}


void writeStats(StatsMessage stat)
{
    long int time_now = millis();
    Serial.print("s:");
    Serial.print(time_now);
    Serial.print("\\id:");
    Serial.print(stat.event_id);
    Serial.print("\\msg:");
    Serial.print(stat.msg);
    if (stat.parameter != -1)
    {
        Serial.print("\\parameter:");
        Serial.print(stat.parameter);
    }
    Serial.println("");
    Serial.flush();
}
