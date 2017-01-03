// Christophanous Radiance Modulator, model 114B (CRM 114B)
//  https://github.com/oelgern/christophanousRadianceModulator114B
//
// Description:
//
//  Precision manual PWM is used to create visually appealing colored light
//   patterns. The original application was for a Christmas tree star topper in
//   the form of an Ikea STRALA fiber optic light fixture (discontinued, see
//   https://www.amazon.com/dp/B00B29CVR8?tag=mydegun-20) retrofitted with an
//   ATtiny2313A microcontroller controlling a 3-watt RGB LED. Control of the
//   pattern mode, speed, and intensity is provided with a rotary encoder.
//
//  The hardware-based PWM available with the selected microcontroller did not
//   not have enough channels with the desired resolution, so manual PWM was
//   used instead. Additionally, standard PWM algorithms don't account for the
//   non-linearity of humans' perception of light intensity. As an example, the
//   perceived difference between a 1% and 2% duty cycle is much greater than
//   that between 98% and 99%. Therefore, the algorithm used with this project
//   changes the intensity exponentially to create a uniformly smooth transition
//   throughout its intensity range.
//
//  This project was submitted as an entry to hackaday's 1kB Challenge contest
//   (see https://hackaday.io/contest/18215-the-1kb-challenge) which challenges
//   contestants to submit projects that are limited in their usage of program
//   memory to 1kB. As such, many things were done to reduce the required
//   program space. Some of the more obvious ones include:
//    * Using compiler program space optimization option.
//    * Using the smallest data type possible.
//    * Declaring functions to be static.
//    * Using else-ifs instead of a case statement.
//    * Omitting conditionals around certain code if its unnecessary execution
//       doesn't have an adverse effect.
//    * Omitting conditionals around certain code if the logic dictates it is
//       the only possibility.
//    * Using unsigned variables where possible.
//    * Using do-while loops where possible.
//    * Using descending loop counters and pre-decrementing where possible.
//    * Inverting a variable at calculation time where its inversion would
//       subsequently be used for multiple comparisons.
//
//  Others aren't as obvious:
//    * Connecting similar hardware to the same port (inputs on one port,
//       outputs on another) and to consecutive pins of the microcontroller.
//    * Using unions to allow single byte access of 16-bit variables when
//       possible and beneficial.
//    * Using a larger data type where used by a macro--see note on
//       prand_address.
//    * Eliminating volatile modifier for some variables, where experience would
//       suggest it is needed.
//    * Conspicuously rearranging code into a non-intuitive order that the
//       compiler optimizer can take advantage of.
//    * Implementing a number of modes that are equal to a power of two.
//    * Designing software so that all needed multiply and division operations
//       are of powers of two, which use bit shifts.
//    * Using runtime variable limitation logic in lieu of variable
//       initialization performed at declaration time.
//    * Placing code that references the same variables close to each other to
//       take advantage of compiler optimizer's abilities.
//    * Reading data from the program space to use as psuedo-random values.
//
//  See the comments embedded in the code for more details.
//
// Connections:
//
//   Outputs       +-\/-+        Inputs
//           PA2  1|    |20  VCC
//           PD0  2|    |19  PB7
//           PD1  3|    |18  PB6
//           PA1  4|    |17  PB5
//           PA0  5|    |16  PB4
//   Red LED PD2  6|    |15  PB3
// Green LED PD3  7|    |14  PB2 Switch button
//  Blue LED PD4  8|    |13  PB1 Switch DT
//           PD5  9|    |12  PB0 Switch CLK
//           GND 10|    |11  PD6
//                 +----+
//
// See: https://github.com/oelgern/christophanousRadianceModulator114B/wiki
//  for additional information.
//
// Code and all project-related design files authored by and released to public
//  domain (whatever that means) by Nathan Oelger, 2017-01-02.
//
// Built with avr-gcc 4.9.2 using -Os
//  1000 bytes program space used
//  0 bytes EEPROM used
//
// Possible improvements:
//  Additional/different pattern modes
//  Optimizations:
//   Redefine the interrupt vector table. This should save 20 bytes of program
//    space.
//   Reconnect/redefine LED ports to PD0..2. This saves 8 bytes of program
//    space by reducing the bit shifts.
//   Add pull-up resistor for switch button so code to turn the internal pull-up
//    on won't be necessary. This saves 4 bytes of program space.

// Includes
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Approximate clock speed after manipulation of CLKPR and OSCCAL
#define F_CPU 13000000UL

// Definitions for input hardware
#define SWITCH_DATA_DIRECTION DDRB
#define SWITCH_PORT PORTB
#define SWITCH_STATES PINB
#define SWITCH_CLK PB0
#define SWITCH_DT PB1
#define SWITCH_BUTTON PB2
#define SWITCH_MASK (1 << SWITCH_CLK) | (1 << SWITCH_DT) | (1 << SWITCH_BUTTON)

// Definitions for output hardware
#define LED_DATA_DIRECTION DDRD
#define LED_PORT PORTD
#define LED_RED PD2
#define LED_GREEN PD3
#define LED_BLUE PD4
#define LED_MASK (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE)
#define NUM_LEDS 3

// Function declarations
//  These are only called once. Declaring these static tells the compiler to
//  optimize them as inline. This saves an additional 52 bytes of program space.
static void init_devices(void);
static void timer1_init(void);
static void read_switches(void);
static void create_pattern(void);
static void scale_intensities(void);
static void calculate_LED_times(void);

// Enumeration list of pattern modes
//  Assigned values so as to leave less to chance.
enum Mode
{
  RANDOM_APPROACH = 0,
  RANDOM_FLIP     = 1,
  PRIMARY_GROW    = 2,
  SECONDARY_FLIP  = 3,
  MANUAL_RED      = 4,
  MANUAL_GREEN    = 5,
  MANUAL_BLUE     = 6,
  TWINKLE         = 7,
  NUM_MODES       = 8 // This needs to be the last mode.
};

// The currently selected pattern mode
static uint8_t current_mode;

// Used to keep track of clockwise/counter-clockwise movement of the rotary
//  encoder.
static int8_t knob_adjustment;

// Linear levels of brightness of each of the three colors.
uint8_t levels[NUM_LEDS];

// Exponentially scaled intensity levels of brightness of each of the three
//  colors.
// Union used to allow access to high/low individual bytes which can save
//  program space.
union
{
   uint8_t  bytes[NUM_LEDS * 2];
   uint16_t value[NUM_LEDS];
} intensities;

// States of the light channels for each of the four minor frames of a major
//  frame. See calculate_LED_times() for more information.
uint8_t on_masks[4]; // Used at the beginning of major and minor frames

// States of the light channels after partial periods--used at conclusion of
//  partial periods
volatile uint8_t off_masks[3];

// Times of ends of partial periods
volatile uint16_t off_timers[NUM_LEDS];

// Buffered state of light channels for beggining of the next minor frame.
volatile uint8_t turn_on_mask;

// Buffered state of light channels for end of the next partial period.
volatile uint8_t turn_off_mask;

// The main routine. Everything starts here.
int main(void)
{

  // Initialize all microcontroller hardware.
  init_devices();

  // This will repeat until power is lost.
  for( ; ; )
  {

  // Nothing to do here. Everything called from interrupts.
  }
}

// This function initializes all hardware.
void init_devices(void)
{

  // Disable errant interrupts until set up.
  //cli(); // Bit initialized to disabled, so this isn't needed.

  // Disable the watchdog timer. This isn't needed as watchdog isn't used.
  // Reset the watchdog timer.
  // asm volatile("wdr");

  // Clear WDRF in MCUSR.
  //MCUSR &= ~(1<<WDRF); // Register initialized to zero, so this isn't needed.

  // Write logical one to WDCE and WDE.
  //WDTCSR |= (1<<WDCE) | (1<<WDE); // Not needed.

  // Turn off WDT.
  //WDTCSR = 0x00; // Register initialized to zero, so this isn't needed.

  // Clear the divide-by-eight clock bit so microcontroller runs at full speed.
  //  Per the datasheet, this requires a special procedure.
  //  1. Write CLKPCE high and all other bits low.
  CLKPR = 1 << CLKPCE;

  //  2. Write CLKPCE low and all other bits as desired.
  //  Note: uint8_t typecast here eliminates warning about truncated unsigned
  //   type due to the default behavior of arithmetic being performed as signed
  //   16-bit.
  CLKPR = (uint8_t)(~(1 << CLKPCE)) &

          // Without the following 3 additional (seemingly unecessary) lines, an
          //  additional 2 bytes of program space are used.
          (~(1 << 6)) &
          (~(1 << 5)) &
          (~(1 << 4)) &

          // CLKPS3..0 corresponds to a CLK/1 (no prescaler).
          (~(1 << CLKPS0)) &
          (~(1 << CLKPS1)) &
          (~(1 << CLKPS2)) &
          (~(1 << CLKPS3));

  // Squeeze extra speed out of internal oscillator. According to the datasheet
  //  this, along with the CLK/1 prescaler, should set the internal oscillator
  //  speed to about 13MHz.
  OSCCAL = (1 << CAL0) |
           (1 << CAL1) |
           (1 << CAL2) |
           (1 << CAL3) |
           (1 << CAL4) |
           (1 << CAL5) |
           (1 << CAL6);

  // Configure switches as inputs.
  //SWITCH_DATA_DIRECTION &= ~SWITCH_MASK; // Register initialized to zero, so
                                           //  this isn't needed.

  // Enable pull-up resistor for switch pushbutton.
  SWITCH_PORT = (1 << SWITCH_BUTTON);

  // Configure LED color channels as outputs.
  LED_DATA_DIRECTION = LED_MASK;

  // Turn off color channels.
  //LED_PORT &= (~(1 << LED_RED)) &
              //(~(1 << LED_GREEN)) &
              //(~(1 << LED_BLUE));  // Register initialized to zero, so this
                                     //  isn't needed.

  // Initialize timer 1--the 16-bit timer.
  timer1_init();

  sei(); // Enable interrupts.
  // All peripherals are now initialized.
}

// Initialize 16-bit timer1.
void timer1_init(void)
{

  // Set CS12:0 to 0 to stop the timer while we set it up.
  //TCCR1B &= ~(1<<CS10); // Initialized to zero, so this isn't needed.
  //TCCR1B &= ~(1<<CS11); // Initialized to zero, so this isn't needed.
  //TCCR1B &= ~(1<<CS12); // Initialized to zero, so this isn't needed.

  // Reset the timer count to 0.
  //TCNT1 = 0; // Initialized to zero, so this isn't needed.

  // Set COM1A1:0 and COM1B2:0 to 0 disabling the compare output pin modes.
  //TCCR1A &= ~(1<<COM1A1); // Initialized to zero, so this isn't needed.
  //TCCR1A &= ~(1<<COM1A0); // Initialized to zero, so this isn't needed.
  //TCCR1A &= ~(1<<COM1B1); // Initialized to zero, so this isn't needed.
  //TCCR1A &= ~(1<<COM1B0); // Initialized to zero, so this isn't needed.

  // Set WGM13:0 to 0b0000 enabling Normal mode.
  //TCCR1A &= ~(1<<WGM10); // Initialized to zero, so this isn't needed.
  //TCCR1A &= ~(1<<WGM11); // Initialized to zero, so this isn't needed.
  //TCCR1B &= ~(1<<WGM12); // Initialized to zero, so this isn't needed.
  //TCCR1B &= ~(1<<WGM13); // Initialized to zero, so this isn't needed.

  // Disable input capture options.
  //TCCR1B &= ~(1<<ICNC1); // Initialized to zero, so this isn't needed.
  //TCCR1B &= ~(1<<ICES1); // Initialized to zero, so this isn't needed.

  // Set CS12:0 to 0b001 to start the timer incrementing and to enable the
  //  timer's clock source as the system clock with no prescaler (that is, a
  //  scale divider value of "1").
  TCCR1B = (1<<CS10);
  //TCCR1B &= ~(1<<CS11); // Initialized to zero, so this isn't needed.
  //TCCR1B &= ~(1<<CS12); // Initialized to zero, so this isn't needed.

  // Enable the TIMER1 overflow interrupt.
  TIMSK /*|*/= (1<<TOIE1)/*;*/

  // Enable the timer1's output compare A match interrupt.
  /*TIMSK */|/*=*/ (1 << OCIE1A)/*;*/

  // Enable the timer1's output compare B match interrupt.
  /*TIMSK */|/*=*/ (1 << OCIE1B);

  // Disable the TIMER1 input capture interrupt.
  //TIMSK &= ~(1<<ICIE1); // Initialized to zero, so this isn't needed.
}

// This interrupt routine will automagically be called everytime TIMER1 has
//  overflowed.  See timer1 stuff in init_devices() to see how the timer was
//  setup.
//  13000000 CPU cycle   timer cycle        overflow         200 overflows
//  ------------------ * ----------- * ------------------ ~= -------------
//        second         1 CPU cycle   65536 timer cycles       second
//  or an overflow about every 5.0 milliseconds.
// The contents of this method would typically be minimalistic (set a flag for
//  main method to check). Unfortunately, putting the logic contained below into
//  the main method had significant undesired effects. After spending some time
//  trying to figure it out, I gave up and put it inside this ISR. It probably
//  works because this disallows other interrupts from altering the logic flow
//  contained herein.
ISR(TIMER1_OVF_vect)
{

  // Read the rotary encoder position changes and button depressions, and
  //  change mode (based on button depressions).
  read_switches();

  // Calculate the light pattern based on mode, time, and other variables.
  create_pattern();

  // Exponentially scale the light intensities, accounting for the non-linearity
  //  of humans' perception of light intensity.
  scale_intensities();

  // Calculate when, and how long each of the color channels will be lit.
  calculate_LED_times();
}

// Read the rotary encoder position changes and button depressions, and change
//  mode (based on button depressions).
void read_switches(void)
{

  // The state of the switches remembered from the previous frame.
  static uint8_t last_switches;

  // Copy the SWITCH_STATES to a temporary variable.  Since SWITCH_STATES is
  //  treated as volatile, this ends up saving 4 bytes of program space
  //  (compiler would copy SWITCH_STATES every time it is called out in the
  //  following code).
  uint8_t inverted_switches = ~SWITCH_STATES;

  // Rotary encoder signals
  // CLK DT Knob position
  // 1   1
  // 0   1  Counter-clockwise from detent
  // 0   0  <- Detent position
  // 1   0  Clockwise from detent
  // 1   1

  // Rotary encoder just got finished transitioning.
  // Look at just the encoder's clock signal.
  if ((1 << SWITCH_CLK) &

      // This is the current state of the switches, inverted.
      inverted_switches &

      // Compare to state of switches during previous frame.
      last_switches)
  {

    // The rotary encoder's clock signal has changed.

    // Knob was turned clockwise.
    if (last_switches & (1 << SWITCH_DT))
    {

      // Increase speed/intensity, depending on mode. Used in create_pattern().
      knob_adjustment = 1;
    }

    // Knob was turned counter-clockwise.
    else
    {

      // Decrease speed/intensity, depending on mode. Used in create_pattern().
      knob_adjustment = -1;
    }
  }

  // Examine button signal.
  // Look at just the encoder's clock signal.
  if ((1 << SWITCH_BUTTON) &

      // This is the current state of the switches, inverted.
      inverted_switches &

      // Compare to state of switches during previous frame.
      last_switches)
  {

    // Transition to the next pattern mode.
    current_mode ++;

    // Return to beginning of mode list if beyond the end of the list.
    //  The following only works if the number of modes is a power of two.
    current_mode &= (NUM_MODES - 1);

    // Initialize random color approach mode
    if (current_mode == RANDOM_APPROACH)
    {

      // Nothing needed for initialization of this mode.
    }

    // Initialize random color flip-through mode
    else if (current_mode == RANDOM_FLIP)
    {

      // Nothing needed for initialization of this mode.
    }

    // Initialize primary color grow mode.
    else if (current_mode == PRIMARY_GROW)
    {

      // Space out color levels evenly--256/3 = 85.3.
      levels[0] = 0;
      levels[1] = 85;
      levels[2] = 171;
    }

    // Initialize secondary color flip-through mode
    else if (current_mode == SECONDARY_FLIP)
    {

      // Nothing needed for initialization of this mode.
    }

    // Initialize manual red mode
    else if (current_mode == MANUAL_RED)
    {

      // Nothing needed for initialization of this mode.
    }

    // Initialize manual green mode
    else if (current_mode == MANUAL_GREEN)
    {

      // Nothing needed for initialization of this mode.
    }

    // Initialize manual blue mode
    else if (current_mode == MANUAL_BLUE)
    {

      // Nothing needed for initialization of this mode.
    }

    // Initialize twinkle mode.
    else if (current_mode == TWINKLE)
    {

      // Nothing needed for initialization of this mode.
    }
  }

  // Remember the switches states for comparison during the subsequent frame.
  last_switches = ~inverted_switches;
}

// Calculate the light pattern based on mode, time, and other variables.
void create_pattern(void)
{

  // Subsequent levels of brightness of each of the three colors used by the
  //  random color approach and random color flip-through modes.
  static uint8_t goals[NUM_LEDS];

  // Psuedo-random number generator address. Program space-efficient PRNG uses
  //  program space bytes to choose colors that appear visually to be random.
  // Oddly enough, declaring this as 16-bit (vs. 8-bit) saves 10 bytes of
  //  program space. This is due to it being used as an address and extra
  //  manipulation compiler performs for the 8-bit.
  static uint16_t prand_address;

  // Delay variable (controlled by rotary encoder's knob)
  static uint8_t delay;

  // Frame counter used in conjunction with the delay variable to control the
  //  speed of the color transitions.
  static uint8_t frame_counter;

  // Frame counter used to further slow down speed of certain color transitions.
  static uint8_t slow_counter;

  // The rotary encoder was just turned clockwise.
  if (knob_adjustment == 1)
  {

    // Add a delay to the color transitions (speed them up).
    delay >>= 1;
  }

  // The rotary encoder was just turned counter-clockwise and
  else if ((knob_adjustment == -1) &&

           // the delay isn't at its highest limit.
           (delay < 64))
  {

    // Remove a delay from the color transitions (slow them down).
    delay <<= 1;
  }

  // If the delay is reduced too much.
  //  Also, this reduces program space that might otherwise be used to
  //   initialize this variable during its declaration.
  //  Additionally, having this here instead of a conditional check above saves
  //   additional program space.
  if (delay == 0)
  {

    // Set the delay to the minimum--a zero can't be bit-shifted to the left, so
    //  will always need at least a '1' in delay.
    delay = 1;
  }

  // Increment to the frame counter.
  //  Placing this here next to a conditional that references it saves program
  //   space due to compiler optimizer recognizing that it is already in a
  //   register.
  frame_counter ++;

  // Only do the following if we've waited long enough.
  if ((frame_counter & (delay - 1)) == 0)
  {

    // Increment the additional slowing counter.
    slow_counter ++;

    // Set the level values for each of the color channels
    uint8_t channel = NUM_LEDS;
    do
    {

      // Decrement color channel counter.
      channel --;

      // Random color approach mode
      //  A random color is chosen and each executed frame brings the displayed
      //   color closer to the chosen color. When a color channel's color
      //   reaches the chosen color,the process repeats.
      if (current_mode == RANDOM_APPROACH)
      {

        // If this channel isn't bright enough yet.
        if (levels[channel] < goals[channel])
        {

          // Add a bit of brightness.
          levels[channel] ++;
        }

        // If it is too bright.
        else if (levels[channel] > goals[channel])
        {

          // Lighten it a bit
          levels[channel] --;
        }
      }

      // Random color flip-through mode
      //  A random color is chosen and when the slow counter overflows the
      //   displayed color is set to the chosen color. The process repeats.
      else if (current_mode == RANDOM_FLIP)
      {

        // The slow counter has overflowed.
        if (slow_counter == 0)
        {

          // Set the color for this channel to the new chosen color.
          levels[channel] = goals[channel];
        }
      }

      // The following is performed for both the random color approach mode and
      //  the random color flip-through mode. It will also run during other
      //  modes, but it won't have any appparent effect.
      // The current color channel's level is the same as the last chosen random
      //  level for this color channel.
      if (levels[channel] == goals[channel])
      {

        // Read a byte from program space to get a psuedo-random value.
        // Obviously this is only barely random. For the purposes of getting a
        //  value for a color/brightness that doesn't seem to repeat, this is a
        //  fine source.
        goals[channel] = pgm_read_byte(prand_address);
      }

      // Primary color grow mode
      //  The brightness of all three color channels are set equidistant from
      //   each other (in the initialization) and all brightnesses are
      //   continuously increased. When their values overflow, the brightness is
      //   returned to off before repeating the sequence again.
      else if (current_mode == PRIMARY_GROW)
      {

        // Increment the brightness of this color channel.
        levels[channel] ++;
      }

      // Secondary color flip-through mode
      //  Flip through all primary, secondary, and white and off colors. The
      //   sequence repeats.
      else if (current_mode == SECONDARY_FLIP)
      {
        // There are a number of ways of doing this. The following used the
        //  least number bytes of program space.
        // Bit-shift the slow counter such that only one of the 3 most
        //  significant bits (which of the 3 depends on which of the three color
        //  channels is performed each of the three passes) are examined.
        if (((slow_counter >> (channel + 5))

            // Look at only the least significant bit (the bit important for
            //  this color channel).
            & 0x1))
        {

          // Turn this color channel on (and all the way).
          levels[channel] = 0xFF;
        }
        else
        {

          // Turn this color channel off.
          levels[channel] = 0;
        }
      }

      // Twinkle mode
      //  A mostly white color is displayed, changing such that it appears to be
      //   twinkling. The process repeats.
      else if (current_mode == TWINKLE)
      {

        // Grab a byte from program space, but assert some of the higher bits so
        //  that this is a brighter color.
        levels[channel] = (pgm_read_byte(prand_address) | 0xC8);
      }

      // Increment the psuedo-random number generator's program counter. Note
      //  that this may or may not be needed here, but without a conditional we
      //  save additional program space.
      prand_address ++;

      // Used program space is at/just under 1024. Make sure we don't read
      //  beyond what's used/exists as we'll get constants.
      if (prand_address > 1024)
      {

        // Reset the psuedo-random number generator's program counter to zero.
        prand_address = 0;
      }

    // Check if the iterator is at it's lower limit.
    } while (channel != 0);
  }

  // Manual red mode, manual green mode, and manual blue mode
  //  These modes allow manual control of each of the color channels by rotating
  //   the knob. The modes are cycled starting at red, then green, and lastly
  //   blue. Pushing the mode button while in manual blue mode transitions the
  //   mode to a non-manual mode.
  if ((current_mode == MANUAL_RED) ||
      (current_mode == MANUAL_GREEN) ||
      (current_mode == MANUAL_BLUE))
  {

    // Adjust the brightness level of this color channel when the knob is
    //  rotated.
    // MANUAL_RED is subtracted as it is the first manual mode.
    levels[current_mode - MANUAL_RED] += knob_adjustment;
  }

  // Knob adjustment has been accounted for, so reset it now.
  knob_adjustment = 0;
}

// Exponentially scale the light intensities, accounting for the non-linearity
//  of humans' perception of light intensity.
void scale_intensities(void)
{

  // Use the linear level values to create exponential light intensities for
  //  more perceptually uniform appearing intensities.
  //
  // The end results of the code's calculations:
  // level intensity          level intensity          level intensity
  // ===   ==================*===   ==================*===   ==================
  // 0     0b0000000000000000|70    0b0000000010110000|140   0b0000111000000000
  // 1     0b0000000000000001|71    0b0000000010111000|141   0b0000111010000000
  // 2     0b0000000000000010|72    0b0000000011000000|142   0b0000111100000000
  // 3     0b0000000000000011|73    0b0000000011001000|143   0b0000111110000000
  // 4     0b0000000000000100|74    0b0000000011010000|144   0b0001000000000000
  // 5     0b0000000000000101|75    0b0000000011011000|145   0b0001000100000000
  // 6     0b0000000000000110|76    0b0000000011100000|146   0b0001001000000000
  // 7     0b0000000000000111|77    0b0000000011101000|147   0b0001001100000000
  // 8     0b0000000000001000|78    0b0000000011110000|148   0b0001010000000000
  // 9     0b0000000000001001|79    0b0000000011111000|149   0b0001010100000000
  // 10    0b0000000000001010|80    0b0000000100000000|150   0b0001011000000000
  // 11    0b0000000000001011|81    0b0000000100010000|151   0b0001011100000000
  // 12    0b0000000000001100|82    0b0000000100100000|152   0b0001100000000000
  // 13    0b0000000000001101|83    0b0000000100110000|153   0b0001100100000000
  // 14    0b0000000000001110|84    0b0000000101000000|154   0b0001101000000000
  // 15    0b0000000000001111|85    0b0000000101010000|155   0b0001101100000000
  // 16    0b0000000000010000|86    0b0000000101100000|156   0b0001110000000000
  // 17    0b0000000000010001|87    0b0000000101110000|157   0b0001110100000000
  // 18    0b0000000000010010|88    0b0000000110000000|158   0b0001111000000000
  // 19    0b0000000000010011|89    0b0000000110010000|159   0b0001111100000000
  // 20    0b0000000000010100|90    0b0000000110100000|160   0b0010000000000000
  // 21    0b0000000000010101|91    0b0000000110110000|161   0b0010001000000000
  // 22    0b0000000000010110|92    0b0000000111000000|162   0b0010010000000000
  // 23    0b0000000000010111|93    0b0000000111010000|163   0b0010011000000000
  // 24    0b0000000000011000|94    0b0000000111100000|164   0b0010100000000000
  // 25    0b0000000000011001|95    0b0000000111110000|165   0b0010101000000000
  // 26    0b0000000000011010|96    0b0000001000000000|166   0b0010110000000000
  // 27    0b0000000000011011|97    0b0000001000100000|167   0b0010111000000000
  // 28    0b0000000000011100|98    0b0000001001000000|168   0b0011000000000000
  // 29    0b0000000000011101|99    0b0000001001100000|169   0b0011001000000000
  // 30    0b0000000000011110|100   0b0000001010000000|170   0b0011010000000000
  // 31    0b0000000000011111|101   0b0000001010100000|171   0b0011011000000000
  // 32    0b0000000000100000|102   0b0000001011000000|172   0b0011100000000000
  // 33    0b0000000000100010|103   0b0000001011100000|173   0b0011101000000000
  // 34    0b0000000000100100|104   0b0000001100000000|174   0b0011110000000000
  // 35    0b0000000000100110|105   0b0000001100100000|175   0b0011111000000000
  // 36    0b0000000000101000|106   0b0000001101000000|176   0b0100000000000000
  // 37    0b0000000000101010|107   0b0000001101100000|177   0b0100010000000000
  // 38    0b0000000000101100|108   0b0000001110000000|178   0b0100100000000000
  // 39    0b0000000000101110|109   0b0000001110100000|179   0b0100110000000000
  // 40    0b0000000000110000|110   0b0000001111000000|180   0b0101000000000000
  // 41    0b0000000000110010|111   0b0000001111100000|181   0b0101010000000000
  // 42    0b0000000000110100|112   0b0000010000000000|182   0b0101100000000000
  // 43    0b0000000000110110|113   0b0000010001000000|183   0b0101110000000000
  // 44    0b0000000000111000|114   0b0000010010000000|184   0b0110000000000000
  // 45    0b0000000000111010|115   0b0000010011000000|185   0b0110010000000000
  // 46    0b0000000000111100|116   0b0000010100000000|186   0b0110100000000000
  // 47    0b0000000000111110|117   0b0000010101000000|187   0b0110110000000000
  // 48    0b0000000001000000|118   0b0000010110000000|188   0b0111000000000000
  // 49    0b0000000001000100|119   0b0000010111000000|189   0b0111010000000000
  // 50    0b0000000001001000|120   0b0000011000000000|190   0b0111100000000000
  // 51    0b0000000001001100|121   0b0000011001000000|191   0b0111110000000000
  // 52    0b0000000001010000|122   0b0000011010000000|192   0b1000000000000000
  // 53    0b0000000001010100|123   0b0000011011000000|193   0b1000100000000000
  // 54    0b0000000001011000|124   0b0000011100000000|194   0b1001000000000000
  // 55    0b0000000001011100|125   0b0000011101000000|195   0b1001100000000000
  // 56    0b0000000001100000|126   0b0000011110000000|196   0b1010000000000000
  // 57    0b0000000001100100|127   0b0000011111000000|197   0b1010100000000000
  // 58    0b0000000001101000|128   0b0000100000000000|198   0b1011000000000000
  // 59    0b0000000001101100|129   0b0000100010000000|199   0b1011100000000000
  // 60    0b0000000001110000|130   0b0000100100000000|200   0b1100000000000000
  // 61    0b0000000001110100|131   0b0000100110000000|201   0b1100100000000000
  // 62    0b0000000001111000|132   0b0000101000000000|202   0b1101000000000000
  // 63    0b0000000001111100|133   0b0000101010000000|203   0b1101100000000000
  // 64    0b0000000010000000|134   0b0000101100000000|204   0b1110000000000000
  // 65    0b0000000010001000|135   0b0000101110000000|205   0b1110100000000000
  // 66    0b0000000010010000|136   0b0000110000000000|206   0b1111000000000000
  // 67    0b0000000010011000|137   0b0000110010000000|207   0b1111100000000000
  // 68    0b0000000010100000|138   0b0000110100000000|>207  0b1111111000000000
  // 69    0b0000000010101000|139   0b0000110110000000

  // Channel number is index into loop.
  uint8_t channel = NUM_LEDS;
  do
  {

    // Decrement color channel counter.
    channel --;

    // Main equation doesn't work in this range.
    if (levels[channel] < 16)
    {

      // The table is a one-to-one correlation at this range.
      intensities.value[channel] = levels[channel];
    }

    // Main equation is used for the level range of 16-207.
    else if (levels[channel] < 208)
    {

                                   // 4 least significant bits of level
      intensities.value[channel] = ((levels[channel] & 0b00001111)

                                   // Add 16 to it
                                   | 16)

                                   // Bit shift the result by ...
                                   <<

                                   // This many bits:
                                   // The level divided by 16 (which is 2^4) ...
                                   ((levels[channel] >> 4)

                                   // Subtract 1 since this equation starts at
                                   //  16 (16/16 = 1).
                                   - 1);
    }

    // Main equation is topped out at this range.
    else // if (levels[channel] > 207)
    {

      // Save 4 bytes program space by setting only the high byte.
      intensities.bytes[(channel << 1) + 1] = 0xFE;
    }

  // Check if the iterator is at it's lower limit.
  } while (channel != 0);
}

// Calculate when, and how long each of the color channels will be lit.
//
// Frame/period/ISR action timeline:
//
//          Start of major frame (marked by ISR(TIMER1_OVF_vect)).
//          | Most calculations performed here (in main()).
//          | | Turn on/off (a bit late) appropriate channels (in
//          | | | calculate_LED_times()).
//          | | |         Start of 1st minor frame
//          | | |         | Turn on/off appropriate channels (in
//          | | |         | | ISR(TIMER1_COMPA_vect)).
//          | | |         | |     Turn off red (in ISR(TIMER1_COMPB_vect)).
//          | | |         | |     |    Start of 2nd minor frame.
//          | | |         | |     |    | Turn on/off appropriate channels.
//          | | |         | |     |    | |     Turn off green.
//          | | |         | |     |    | |     |    Start of 3rd minor frame.
//          | | |         | |     |    | |     |    | Turn on/off appropriate
//          | | |         | |     |    | |     |    | | channels.
//          | | |         | |     |    | |     |    | |     Turn off blue.
//          | | |         | |     |    | |     |    | |     |     End of major
//          | | |         | |     |    | |     |    | |     |     | frame.
//          | | |         | |     |    | |     |    | |     |     |
//          +-+-+---------+-+-----+----+-+-----+----+-+-----+-----+
//          <------------><-----------><-----------><------------->
// Frames-> Major/Minor A Minor B      Minor C      Minor D
// ---------*-------------*------------*------------*--------------
// Channels Periods
// ======== ============= ============ ============ ============
// Red      3rd standard  Partial      1st standard 2nd standard
// Green    2nd standard  3rd standard Partial      1st standard
// Blue     1st standard  2nd standard 3rd standard Partial
void calculate_LED_times(void)
{

  // Initialize iterator of loop.
  uint8_t channel = NUM_LEDS;

  // We use a do while loop as it becomes compiled more space-efficiently than a
  //  for or while loop.
  do
  {

    // Decrement iterator (check of value is at conclusion of loop).
    channel --;

    // The timers for the off mask are the timers for the partial period--the
    //  time at which the colors are turned off during their partial period
    //  relative to the start of the major frame. The beginning of each partial
    //  period is the beginning of each minor frame which occurs at 1-3
    //  multiplied by the length of a minor frame.
    off_timers[channel] = ((channel + 1) << 14) |

                          // We add to that the partial period for the color
                          //  channel, which is the 14 least significant bits of
                          //  its intensity.
                          (intensities.value[channel] & 0x3FFF);

    // Reset (turn off) the color channel for it's partial frame.
    on_masks[channel + 1] &= ~(1 << (channel + LED_RED));

    // If the channel is supposed to be on.
    if ((intensities.value[channel] & 0x3FFF) > 0)
    {

      // Turn on this color channel during it's partial frame.
      //  Channels are on port pins 2-4, and start at LED_RED, so add it.
      on_masks[channel + 1] |= (1 << (channel + LED_RED));
    }

    // Moving the following to above the 'if' saves 78 bytes!?!
    //else
    //{
      //on_masks[channel + 1] &= ~(1 << (channel + LED_RED));
    //}

    // In addition to the partial period for each of the color channels, there
    //  are 3 standard periods for each of the color channels. This nested loop
    //  is going to loop through the three standard periods.
    // Initialize iterator of internal loop
    uint8_t standard_period = NUM_LEDS;
    do
    {

      // Decrement iterator (check of value is at conclusion of loop).
      standard_period --;

      // The major frame (which occurs every overflow of the 16-bit timer 1) is
      //  broken into 4 minor frames. Different color channel's standard periods
      //  get mapped to different minor frames (see table above). We can tell
      //  how many (and which) standard periods a particular channel needs to be
      //  illuminated by looking at the 2 most significant bits of the 16-bit
      //  timer.
      // Shift intensity to right by 14 bits and compare that to the iterator.
      //  2 most significant  Illuminated
      //   bits of intensity  standard periods
      //  ==================  ================
      //                0b00  0
      //                0b01  1
      //                0b10  2
      //                0b11  3
      //if ((intensities.value[channel] >> 14) > standard_period)
      // Accessing the single byte saves 16 bytes of program space.
      if ((intensities.bytes[(channel << 1) + 1] >> 6) > standard_period)
      {

        // Turn on the appropriate color channels for their full standard
        //  illuminated periods when appropriate. This index into on_masks
        //  starts at the first standard period for a color channel (1st one
        //  after the partial period) and rolls to the beginning (1st minor
        //  frame).
        on_masks[(standard_period + channel + LED_RED) & 0x3] |=

         //  Turns on the appropriate color channel--channels are on port pins
         //  2-4, so we add 2 to the 0-2 channel).
         (1 << (channel + LED_RED));
      }

      // This color channel shouldn't be on during this standard period.
      else
      {

        //  Turns on the appropriate color channel--channels are on port pins
        //  2-4, so we add 2 to the 0-2 channel).
        on_masks[(standard_period + channel + LED_RED) & 0x3] &=
         (~(1 << (channel + LED_RED)));
      }

    // Check if the iterator is at it's lower limit.
    } while (standard_period != 0);

    // The off mask turns the color channel off at the end of the partial
    //  period. There's one more on_mask (at the beginning of a major frame)
    //  than there are off_masks.
    off_masks[channel] = on_masks[channel + 1] &

     //  Turns off the appropriate color channel--channels are on port pins 2-4,
     //  so we add 2 to the 0-2 channel).
     (~(1 << (channel + LED_RED)));

  // Check if the iterator is at it's lower limit.
  } while (channel != 0);

  // Turn on the appropriate color channels now.
  LED_PORT = on_masks[0];

  // Doing the following two lines of code ahead of time so ISRs timing is more
  //  precise.
  // Setup beginning of the first minor frame.
  turn_on_mask = on_masks[1];

  // Setup mask to turn off the partial period.
  turn_off_mask = off_masks[0];

  // The time of beginning of next minor frame.
  OCR1A = 0x4000;

  // The time of end of next partial period.
  OCR1B = off_timers[0];
}

// Beginning of a minor frame
// We want to get in and do the bare minimum and get out so we can subsequently
//  turn off colors' partial frame as quickly as possible. Note that the
//  turn_on_mask variable is prepopulated to aid in the speed of this ISR.
ISR(TIMER1_COMPA_vect)
{

  // Turn on appropriate colors. This is a minor frame, so up to one partial
  //  period and two full periods.
  LED_PORT = turn_on_mask;
}

// End of a partial period.
ISR(TIMER1_COMPB_vect)
{

  // The very first thing we want to do (and do as quicly as possible--that's
  //  why we prepopulated the turn_off_mask variable) is to turn off the color
  //  channel that has a partial period within this minor frame. The quicker we
  //  can do this, the broader dynamic range of brightnesses we can have (at the
  //  dim end of the dim/bright range).
  LED_PORT = turn_off_mask;

  // Doing the remaining things in this method ahead of time so future ISRs
  //  timing is more precise.
  // Minor is based on 2 most significant bits of timer 1 counter.
  uint8_t minor_frame = TCNT1H >> 6; // This is more efficient with program
                                     //  space.
  //uint8_t minor_frame = TCNT1 >> 14; // This is less efficient with program
                                       //  space.

  // If we're not in the last minor frame.
  if (minor_frame < 3)
  {

    // Beginning time (relative to the beginning of the major frame which equals
    //  0) of the next minor frame is the frame number (next one, so plus 1)
    //  times 2^14.
    //OCR1A = (minor_frame + 1) << 14; // This is less effiicient with program
                                       //  space.
    //OCR1AH = (minor_frame + 1) << 6; // This is less effiicient with program
                                       //  space.
    OCR1AH += 0x40; // This is more efficient with program space.

    // Per the datasheet, need to set the lower byte at the same time.
    OCR1AL = 0;

    // The time of end of next partial period.
    OCR1B = off_timers[minor_frame];

    // Doing the following two lines of code ahead of time so ISRs timing is
    //  more precise.
    // Setup mask to turn off the next partial period.
    turn_off_mask = off_masks[minor_frame];

    // Setup beginning of the next frame.
    turn_on_mask = on_masks[minor_frame + 1];
  }
}
