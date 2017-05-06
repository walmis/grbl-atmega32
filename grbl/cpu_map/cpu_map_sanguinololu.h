#ifdef CPU_MAP_ATmega644P_Sanguinololu

// Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

//leave these, since there are bit-packed variables in there 
#define X_STEP_BIT         0  // Uno Digital Pin 2
#define X_STEP_MASK        (1 << X_STEP_BIT)
#define Y_STEP_BIT         1  // Uno Digital Pin 3
#define Y_STEP_MASK        (1 << Y_STEP_BIT) 
#define Z_STEP_BIT         2  // Uno Digital Pin 4
#define Z_STEP_MASK        (1 << Z_STEP_BIT)
#define X_DIRECTION_BIT    3  // Uno Digital Pin 5
#define X_DIRECTION_MASK   (1 << X_DIRECTION_BIT)
#define Y_DIRECTION_BIT    4  // Uno Digital Pin 6
#define Y_DIRECTION_MASK   (1 << Y_DIRECTION_BIT)
#define Z_DIRECTION_BIT    5  // Uno Digital Pin 7
#define Z_DIRECTION_MASK   (1 << Z_DIRECTION_BIT)

#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

#define STEPPERS_DISABLE_DDR    DDRD
#define STEPPERS_DISABLE_PORT   PORTD
#define STEPPERS_DISABLE_BIT    7 // Not realy used
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)


// NOTE: All limit bit pins must be on the same port
#define LIMIT_DDR       DDRC
#define LIMIT_PIN       PINC
#define LIMIT_PORT      PORTC
#define X_LIMIT_BIT     0  // Uno Digital Pin 9
#define Y_LIMIT_BIT     1  // Uno Digital Pin 10
#define Z_LIMIT_BIT     2  // Uno Digital Pin 11
#define LIMIT_INT       PCIE2  // Pin change interrupt enable pin
#define LIMIT_INT_vect  PCINT2_vect 
#define LIMIT_PCMSK     PCMSK2 // Pin change interrupt register
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#define SPINDLE_ENABLE_DDR   DDRD
#define SPINDLE_ENABLE_PORT  PORTD
#define SPINDLE_ENABLE_BIT   5  // Hotend mosfet output

#define SPINDLE_DIRECTION_DDR   DDRD
#define SPINDLE_DIRECTION_PORT  PORTD
#define SPINDLE_DIRECTION_BIT   4  // Heated bed mosfet output

#define COOLANT_FLOOD_DDR   DDRD
#define COOLANT_FLOOD_PORT  PORTD
#define COOLANT_FLOOD_BIT   4  // Uno Analog Pin 3

// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
// #define ENABLE_M7  // Mist coolant disabled by default. Uncomment to enable.
#ifdef ENABLE_M7
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
#endif  

// NOTE: All pinouts pins must be on the same port
#define PINOUT_DDR       DDRA
#define PINOUT_PIN       PINA
#define PINOUT_PORT      PORTA
#define PIN_RESET        0  // A0
#define PIN_FEED_HOLD    1  // A1
#define PIN_CYCLE_START  2  // A2
#define PINOUT_INT       PCIE0  // Pin change interrupt enable pin
#define PINOUT_INT_vect  PCINT0_vect
#define PINOUT_PCMSK     PCMSK0 // Pin change interrupt register
#define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))



// MOD GRBL for Cyclone

#define PROBE_PIN       31
#define XLIM_PIN        18
#define YLIM_PIN        19
#define ZLIM_PIN        20

#define X_STEP_PIN      15
#define X_DIR_PIN       21
#define X_ENABLE_PIN    14

#define Y_STEP_PIN      22
#define Y_DIR_PIN       23
#define Y_ENABLE_PIN    14

#define Z_STEP_PIN      3
#define Z_DIR_PIN       2
#define Z_ENABLE_PIN    26

#define SPINDLE_ENABLE_PIN      12 // Hotend transistor
#define SPINDLE_DIRECTION_PIN   0  // Extruder direction pin
#define COOLANT_FLOOD_PIN       13 // Hotbed transistor


#endif

