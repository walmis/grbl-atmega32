#ifdef DEFAULTS_CYCLONE_2_1
  // Description: GRBL settings for Cyclone PCB Factory v2.1
  // http://reprap.org/wiki/Cyclone_PCB_Factory
  #define MICROSTEPS 16 // 16 --> all three jumpers installed
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV 1.25 // 1.25 mm/rev leadscrew
  #define Cyclone_XY_Gear_Ratio 21.0/21.0 // Number of gear teeth (motor/rod)
  #define Cyclone_Z_Gear_Ratio 8.0/15.0 // Number of gear teeth (motor/rod)
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/(Cyclone_XY_Gear_Ratio*8.0))
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/(Cyclone_XY_Gear_Ratio*8.0))
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/(Cyclone_Z_Gear_Ratio*MM_PER_REV))
  #define DEFAULT_X_MAX_RATE 5*60.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5*60.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 2.5*60.0 // mm/min
  #define DEFAULT_X_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 168.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 101.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 50.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((0<<X_AXIS)|(0<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_AUTO_START 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 1 // true
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 1  // true
  #define DEFAULT_HOMING_DIR_MASK ((1<<X_AXIS)|(1<<Y_AXIS)|(0<<Z_AXIS)) // in Cyclone, z axis is left to move upwards, in case Z homing is triggered (there is no Z endstop!)
  #define DEFAULT_HOMING_FEED_RATE 50.0 // mm/min (slower feed rate to "bump" the endstops)
  #define DEFAULT_HOMING_SEEK_RATE 635.0 // mm/min (will saturate to MAX_RATE)
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 0.0 // mm (distance that the axis move after homing)
 
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  
  #define DEFAULT_INVERT_PROBE_PIN 0
  #define DEFAULT_LASER_MODE 0
#endif
