#ifndef DEFINITIONS_MODULE
#define DEFINITIONS_MODULE

#import <Arduino.h>

// CONST_PIN_TYPE types cannot change their values after first declartion
// typedef static const unsigned int CONST_PIN_TYPE;
#define CONST_PIN_TYPE static const unsigned int // typedef is not working, dedbug later

typedef unsigned int PIN_TYPE;
typedef int16_t ANGLE;
typedef unsigned int LANE_ID;

struct SensorTouched
{
  bool change_happened;
  bool left_sensor;
  bool right_sensor;

  SensorTouched(bool left_touched, bool right_touched)
  {
    this->change_happened = false;
    this->left_sensor = left_touched;
    this->right_sensor = right_touched;
  }
};

struct SubjectLocation
{
    bool block_detected;
    ANGLE angle;
    uint16_t x;
    uint16_t y;
};

struct Lane
{
   LANE_ID lane_id;
   PIN_TYPE reward_sensor;
   ANGLE region_start_angle;
   ANGLE region_end_angle;

};

struct MotorDurationEntry
{
    bool activated;
    PIN_TYPE motor_id;
    long int activation_time;
    long int timeout_period;
};

struct GlobalState
{
   static const LANE_ID NUM_OF_LANES = 4;
   Lane lanes[NUM_OF_LANES];
   // Guaranteed boind value must be divisble by num of lanes
   static const int GUARANTEED_RANDOM_BOUND = NUM_OF_LANES*1;
   LANE_ID lane_shuffle_list[GUARANTEED_RANDOM_BOUND];
   size_t shuffle_list_index;
   bool was_inside_lane;
   LANE_ID current_lane;
   LANE_ID reward_lane_id;
   SubjectLocation last_subject_location;
   bool actuator_at_max_push;
   static const long int MAX_PUSH_TIMEOUT = 3000;
   static const long int MAX_PUSH_WAIT = 100;
   static const long int ALLOWED_REWARD_TIMEOUT = 2500;
   static const long int NO_REWARD_TIMEOUT = 100;
   static const long int PEIZO_TIMEOUT = 1000;
   long int motor_timeout_duration;
   long int max_push_current_duration;
   bool actuator_duration_activated;
   bool chose_new_lane;
   bool reward_given;
   MotorDurationEntry *peizo_motor_entry;

   LANE_ID last_reported_lane; // Initially report non existing lane
   byte last_reported_light_status; // Assign any random initial value
   byte last_reported_actuator_status; // It's type is Actuator::State
   bool reported_motor_max_distance;
   bool reported_motor_max_wait;
   bool sensor_was_touched;

   // Change to fit the number of motors that you have in your system
   static const size_t MOTOR_DURATION_ENTERIES_SIZE = 4;
   MotorDurationEntry motor_duration_entries[MOTOR_DURATION_ENTERIES_SIZE];

   static const int actuator_max_pwm_distance = 870;
   static const long int SOLENOID_DURATION = 120;

   unsigned int trial_number;

   static const long int SAME_SENSOR_MAX_THRESHOLD = 500;
   static const long int FORCE_OTHER_SENSOR = 3;
   static const long int FEEDBACK_AUTOMATED_REWARD_THRESHOLD = 3;
   bool last_sensor_touched_left;
   unsigned int same_sensor_touch_count;
   bool in_force_sensor_mode;
   bool is_force_sensor_left;
   int miss_or_wrong_touch_count;

   bool is_automated_reward;
};

struct DistancesStruct
{
    uint16_t x_threshold_min;
    uint16_t x_threshold_max;
    uint16_t y_threshold_min;
    uint16_t y_threshold_max;
    uint16_t y_motor_threshold;

    DistancesStruct ()
    {
        this->x_threshold_min = 100;
        this->x_threshold_max = 350;
        this->y_threshold_min = 2;
        this->y_threshold_max = 90;
        this->y_motor_threshold = 35;
    }
} Distances;



struct StatsMessage
{
  byte event_id;
  char* msg;
  short parameter;

  StatsMessage(byte event_id, char* msg, short parameter)
  {
      this->event_id = event_id;
      this->msg = msg;
      this->parameter = parameter;
  }

  StatsMessage(byte event_id, char* msg)
  {
      this->event_id = event_id;
      this->msg = msg;
      this->parameter = -1;
    }
};

struct StatsStruct
{
    StatsMessage ENTERED_LANE(short lane) {return StatsMessage(0, "Entered lane\0", lane + 1);}
    StatsMessage EXITED_LANE(short lane) {return StatsMessage(1, "Exited lane\0", lane + 1);}

    StatsMessage MOTOR_PUSHED() {return StatsMessage(10, "Motor is pushed\0");}
    StatsMessage MOTOR_PULLED() {return StatsMessage(11, "Motor is pulled\0");}
    StatsMessage MOTOR_MAX_RANGE() {return StatsMessage(12, "Motor is at max distance\0");}
    // Next one is not reported yet
    StatsMessage MOTOR_MIN_RANGE() {return StatsMessage(13, "Motor is at min distance\0");}
    StatsMessage MOTOR_WAIT_DONE() {return StatsMessage(14, "Motor wait time is is done.\0");}

    StatsMessage ENTERED_LANE_RANGE(short lane) {return StatsMessage(20, "Entered rotation range of lane\0", lane + 1);}

    StatsMessage CORRECT_SENSOR_TOUCHED() {return StatsMessage(30, "Correct sensor touched\0");}
    StatsMessage WRONG_SENSOR_TOUCHED() {return StatsMessage(31, "Wrong sensor touched\0");}

    StatsMessage RIGHT_SENSOR_TOUCHED() {return StatsMessage(35, "R-located sensor touched\0");}
    StatsMessage LEFT_SENSOR_TOUCHED() {return StatsMessage(36, "L-located sensor touched\0");}
    // Next one is not reported yet
    StatsMessage OTHER_SENSOR_TOUCHED(short sensor) {return StatsMessage(37, "Other sensor touched - Sensor nr.\0", sensor);}

    StatsMessage REWARD_GIVEN() {return StatsMessage(40, "Giving reward\0");}
    StatsMessage REWARD_NOT_GIVEN() {return StatsMessage(41, "Not giving reward\0");}
    StatsMessage MISS_DECISION() {return StatsMessage(42, "No Decision was made in time\0");}

    StatsMessage NEW_LANE(short lane) {return StatsMessage(50, "New lane chosen\0", lane + 1);}
    StatsMessage NEW_TRIAL(short trial) {return StatsMessage(55, "Trial number\0", trial);}

    StatsMessage LIGHT_ON() {return StatsMessage(60, "Cue light turned on\0");}
    StatsMessage LIGHT_OFF() {return StatsMessage(61, "Cue light turned off\0");}

    StatsMessage SOLENOID_RIGHT_ON() {return StatsMessage(70, "R-located solenoid turned on\0");}
    StatsMessage SOLENOID_RIGHT_OFF() {return StatsMessage(71, "R-located solenoid turned off\0");}
    StatsMessage SOLENOID_LEFT_ON() {return StatsMessage(72, "L-located solenoid turned on\0");}
    StatsMessage SOLENOID_LEFT_OFF() {return StatsMessage(73, "L-located solenoid turned off\0");}

    StatsMessage FORCE_SENSOR_ON(bool is_left)
    {
        char* msg = is_left ? (char*)"Force sensor mode on (left)\0" :
                              (char*)"Force sensor mode on (right)\0";
        return StatsMessage(80, msg, is_left);
    }
    StatsMessage FORCE_SENSOR_OFF() {return StatsMessage(81, "Force sensor mode is off\0");}
    StatsMessage FEEDBACK_AUTOMATED_ON() {return StatsMessage(82, "Feedback automated on\0");}
    StatsMessage FEEDBACK_AUTOMATED_OFF() {return StatsMessage(83, "Feedback automated off\0");}
} Stats;


#endif
