/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  Copyright (c) 2011 Jens Geisler
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "settings.h"
#include "config.h"
#include "gcode.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "planner.h"
#include "limits.h"
#include "protocol.h"
#include "probe.h"

#include <avr/interrupt.h>

#define MICROSECONDS_PER_ACCELERATION_TICK  (1000000/ACCELERATION_TICKS_PER_SECOND)

void probe_init(void)
{

  sys.pulse_counter[X_AXIS] = 0;
  sys.pulse_counter[Y_AXIS] = 0;
  sys.pulse_counter[Z_AXIS] = 0;

  sys.probe_z_contact_position = 0.0;
  probe_disable();

}

void probe_enable(void)
{
  PROBE_DDR  &= ~(1<<PROBE_BIT);     // set as input
  PROBE_PORT |=  (1<<PROBE_BIT);     // set pull-up
  settings.probe_enabled = 1;
}

void probe_disable(void)
{
  PROBE_DDR  &= ~(1<<PROBE_BIT);    // keep as input
  PROBE_PORT &= ~(1<<PROBE_BIT);    // disable pull-up
  settings.probe_enabled = 0;
}


#define PROBE_DEBOUNCE_COUNT 8
//#define PROBE_DEBOUNCE_WAIT 50

// The work horse of the probe functionality.
// Once called, this will decrease the Z axis until either the probe pin
// goes high (after debouncing) or the z_limit threshold is reached.
// This function does not have any acceleration and assumes that the
// nominal feed rate of probe_rate can be attained right away without
// any acceleration.
// Debouncing is necessary as spurious signals can happen.  
// Set PROBE_DEBOUNCE_WAIT if debouncing without any delay is causing problems.
void probe_z(float z_limit, float probe_rate, float probe_acceleration)
{
  uint32_t steps[3];

  // For simplicity, keep the bit vectors for Z direction in variables
  uint8_t out_bits, out_bits_orig;

  // calculate nominal rate, time delay for the nominal rate (stored in dt)
  // and the actual step delay (stored in step_delay, taking into account
  // the pulse duration)
  uint32_t nominal_rate = ceil( settings.steps_per_mm[Z_AXIS] * probe_rate ) ;
  uint32_t dt = 1000000*60 / nominal_rate ; 
  uint32_t step_delay = dt-settings.pulse_microseconds;  // Step delay after pulse

  uint8_t debounce_count;
  int32_t z_limit_step = lround( z_limit * settings.steps_per_mm[Z_AXIS] );

  // enable only stepper subsystem
  st_wake_up();

  steps[Z_AXIS] = lround(settings.steps_per_mm[Z_AXIS]);

  // go in negative z direction
  out_bits_orig = settings.invert_mask ^ (1<<Z_DIRECTION_BIT);
  out_bits      = out_bits_orig        ^ (1<< Z_STEP_BIT);
  
  for (;;)
  {

    // stop if we've reached the limit
    if ( sys.position[Z_AXIS] <= z_limit_step )
      break;

    #ifdef PROBE_DEBOUNCE_COUNT
    // poor mans debouncing
    for (debounce_count = 0; debounce_count < PROBE_DEBOUNCE_COUNT; debounce_count++)
    {
    #endif

      if (sys.execute & EXEC_RESET) break;
      if (PROBE_PIN & (1 << PROBE_BIT)) 
        break;
      #ifdef PROBE_DEBOUNCE_WAIT
      delay_us(PROBE_DEBOUNCE_WAIT);
      #endif

    #ifdef PROBE_DEBOUNCE_COUNT
    }
    #endif

    // if we've entered a reset or if our probe has hit something, break
    if (sys.execute & EXEC_RESET) break;
    if (debounce_count == PROBE_DEBOUNCE_COUNT) break;

    // Perform step.
    STEPPING_PORT = out_bits;
    delay_us(settings.pulse_microseconds);
    STEPPING_PORT = out_bits_orig;
    delay_us(step_delay);

    sys.position[Z_AXIS]--;

  }

  // Whether we've entered a reset condition or not, 
  // we sill want to just record position, put the
  // stepper subsystem back into idle, and return.

  // record position
  sys.probe_z_contact_position = sys.position[Z_AXIS];

  st_go_idle();

}


// probe z and return to original position
//
void probe_z_and_return(float z_limit, float probe_rate, float probe_acceleration)
{
  uint32_t steps[3];

  // For simplicity, keep the bit vectors for Z direction in variables
  uint8_t out_bits, out_bits_orig;

  // calculate nominal rate, time delay for the nominal rate (stored in dt)
  // and the actual step delay (stored in step_delay, taking into account
  // the pulse duration)
  uint32_t nominal_rate = ceil( settings.steps_per_mm[Z_AXIS] * probe_rate ) ;
  uint32_t dt = 1000000*60 / nominal_rate ; 
  uint32_t step_delay = dt-settings.pulse_microseconds;  // Step delay after pulse

  uint8_t debounce_count;
  int32_t z_limit_step = lround( z_limit * settings.steps_per_mm[Z_AXIS] );

  // enable only stepper subsystem
  st_wake_up();

  steps[Z_AXIS] = lround(settings.steps_per_mm[Z_AXIS]);

  // go in negative z direction
  out_bits_orig = settings.invert_mask ^ (1<<Z_DIRECTION_BIT);
  out_bits      = out_bits_orig        ^ (1<< Z_STEP_BIT);

  int32_t orig_z_position = sys.position[Z_AXIS];
  
  for (;;)
  {


    // stop if we've reached the limit
    if ( sys.position[Z_AXIS] <= z_limit_step )
      break;

    #ifdef PROBE_DEBOUNCE_COUNT
    // poor mans debouncing
    for (debounce_count = 0; debounce_count < PROBE_DEBOUNCE_COUNT; debounce_count++)
    {
    #endif

      if (sys.execute & EXEC_RESET) break;
      if (PROBE_PIN & (1 << PROBE_BIT)) 
        break;
      #ifdef PROBE_DEBOUNCE_WAIT
      delay_us(PROBE_DEBOUNCE_WAIT);
      #endif

    #ifdef PROBE_DEBOUNCE_COUNT
    }
    #endif

    // if we've entered a reset or if our probe has hit something, break
    if (sys.execute & EXEC_RESET) break;
    if (debounce_count == PROBE_DEBOUNCE_COUNT) break;

    // Perform step.
    STEPPING_PORT = out_bits;
    delay_us(settings.pulse_microseconds);
    STEPPING_PORT = out_bits_orig;
    delay_us(step_delay);

    sys.position[Z_AXIS]--;

  }


  // record position
  sys.probe_z_contact_position = sys.position[Z_AXIS];

  // bail out if we're in a reset
  if (sys.execute & EXEC_RESET) 
  {
    st_go_idle();
    return;
  }

  // let it settle down before going back up
  delay_us(step_delay);
  delay_us(step_delay);
  delay_us(step_delay);
  delay_us(step_delay);

  // go in the other direction
  out_bits_orig ^= (1<<Z_DIRECTION_BIT);
  out_bits      = out_bits_orig ^ (1<< Z_STEP_BIT);

  // return to original position
  for (;;)
  {
    if (sys.position[Z_AXIS] == orig_z_position) 
      break;

    // if we've entered a reset, break
    if (sys.execute & EXEC_RESET) break;

    // Perform step.
    STEPPING_PORT = out_bits;
    delay_us(settings.pulse_microseconds);
    STEPPING_PORT = out_bits_orig;
    delay_us(step_delay);

    sys.position[Z_AXIS]++;

  }

  // Whether we've entered a reset condition or not, 
  // we sill want to just record position, put the
  // stepper subsystem back into idle, and return.

  st_go_idle();

}

