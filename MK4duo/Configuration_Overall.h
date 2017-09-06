/**
* MK4duo Firmware for 3D Printer, Laser and CNC
*
* Based on Marlin, Sprinter and grbl
* Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
* Copyright (C) 2013 Alberto Cotronei @MagoKimbra
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */

/********************************
 * Firmware Version V4.3.27 dev *
 ********************************/

#define CONFIGURATION_OVERALL


/***********************
 * Configuration_Basic *
 ***********************/
#define SERIAL_PORT -1
#define BAUDRATE 115200
//#define BLUETOOTH
#define BLUETOOTH_PORT 0
#define BLUETOOTH_BAUD 115200
#define STRING_CONFIG_H_AUTHOR "Remspoor"
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#define KILL_METHOD 0
#define NO_TIMEOUTS 1000
#define ADVANCED_OK
//#define EMERGENCY_PARSER
//#define FASTER_GCODE_PARSER
//#define HOST_KEEPALIVE_FEATURE
#define DEFAULT_KEEPALIVE_INTERVAL 2
#define BUSY_WHILE_HEATING
#define MOTHERBOARD BOARD_ULTRATRONICS
#define MECHANISM MECH_COREXY
//#define MECHANISM MECH_COREXY
//#define MECHANISM MECH_COREYX
//#define MECHANISM MECH_COREXZ
//#define MECHANISM MECH_COREZX
//#define MECHANISM MECH_COREYZ
//#define MECHANISM MECH_COREZY
//#define MECHANISM MECH_DELTA
//#define MECHANISM MECH_MORGAN_SCARA
//#define MECHANISM MECH_MAKERARM_SCARA
//#define MECHANISM MECH_MUVE3D
#define POWER_SUPPLY 0
//#define PS_DEFAULT_OFF
#define DELAY_AFTER_POWER_ON 5
#define EXTRUDERS 1
#define DRIVER_EXTRUDERS 3

/*****************************
 * Configuration_Temperature *
 *****************************/
//#define TEMPERATURE_UNITS_SUPPORT
#define TEMP_SENSOR_0 11
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 11
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100
//#define SHOW_TEMP_ADC_VALUES
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.
#define TEMP_BED_RESIDENCY_TIME 0   // (seconds)
#define TEMP_BED_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_BED_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.
#define TEMP_CHAMBER_RESIDENCY_TIME 0   // (seconds)
#define TEMP_CHAMBER_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_CHAMBER_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.
#define TEMP_COOLER_RESIDENCY_TIME 0    // (seconds)
#define TEMP_COOLER_HYSTERESIS 1        // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_COOLER_WINDOW     1        // (degC) Window around target to start the residency timer x degC early.
#define HEATER_0_MAXTEMP 275 // (degC)
#define HEATER_1_MAXTEMP 275 // (degC)
#define HEATER_2_MAXTEMP 275 // (degC)
#define HEATER_3_MAXTEMP 275 // (degC)
#define BED_MAXTEMP      150 // (degC)
#define CHAMBER_MAXTEMP  100 // (degC)
#define COOLER_MAXTEMP   35  // (degC) 
#define HEATER_0_MINTEMP 5 // (degC)
#define HEATER_1_MINTEMP 5 // (degC)
#define HEATER_2_MINTEMP 5 // (degC)
#define HEATER_3_MINTEMP 5 // (degC)
#define BED_MINTEMP      5 // (degC)
#define CHAMBER_MINTEMP  5 // (degC)
#define COOLER_MINTEMP  10 // (degC) 
#define PREHEAT_1_TEMP_HOTEND 190
#define PREHEAT_1_TEMP_BED     60
#define PREHEAT_1_FAN_SPEED   255   // Insert Value between 0 and 255
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    100
#define PREHEAT_2_FAN_SPEED   255   // Insert Value between 0 and 255
#define PREHEAT_3_TEMP_HOTEND 230
#define PREHEAT_3_TEMP_BED     60
#define PREHEAT_3_FAN_SPEED   255   // Insert Value between 0 and 255
#define AUTOTEMP
#define AUTOTEMP_OLDWEIGHT 0.98
//#define TEMP_STAT_LEDS
#define HEATER_PWM_SPEED 0
#define PIDTEMP true
#define BANG_MAX  255       // Limits current to nozzle while in bang-bang mode; 255 = full current
#define PID_MIN     0       // Limits min current to nozzle while PID is active;   0 = no current
#define PID_MAX   255       // Limits max current to nozzle while PID is active; 255 = full current
//#define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.
//#define PID_DEBUG         // Sends debug data to the serial port.
#define PID_FUNCTIONAL_RANGE 10
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50
#define DEFAULT_Kp {40, 40, 40, 40}     // Kp for H0, H1, H2, H3
#define DEFAULT_Ki {07, 07, 07, 07}     // Ki for H0, H1, H2, H3
#define DEFAULT_Kd {60, 60, 60, 60}     // Kd for H0, H1, H2, H3
#define DEFAULT_Kc {100, 100, 100, 100} // heating power = Kc * (e_speed)
#define PIDTEMPBED true
#define BED_HYSTERESIS        2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS (works only if BED_LIMIT_SWITCHING is enabled)
#define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
#define MIN_BED_POWER   0 // Limits duty cycle to bed;   0 = no current
#define MAX_BED_POWER 255 // Limits duty cycle to bed; 255 = full current
#define DEFAULT_bedKp   10.00
#define DEFAULT_bedKi    0.1
#define DEFAULT_bedKd  300.0
#define PIDTEMPCHAMBER false
#define CHAMBER_HYSTERESIS 2 //only disable heating if T>target+CHAMBER_HYSTERESIS and enable heating if T>target-CHAMBER_HYSTERESIS (works only if CHAMBER_LIMIT_SWITCHING is enabled)
#define CHAMBER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control
#define MIN_CHAMBER_POWER   0 // Limits duty cycle to chamber;   0 = no current
#define MAX_CHAMBER_POWER 255 // Limits duty cycle to chamber; 255 = full current
#define DEFAULT_chamberKp   10.00
#define DEFAULT_chamberKi    0.1
#define DEFAULT_chamberKd  300.0
#define PIDTEMPCOOLER false
//#define FAST_PWM_COOLER
#define COOLER_HYSTERESIS 2 //only disable heating if T<target-COOLER_HYSTERESIS and enable heating if T<target+COOLER_HYSTERESIS (works only if COOLER_LIMIT_SWITCHING is enabled)
#define COOLER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control
#define MIN_COOLER_POWER   0 // Limits duty cycle to cooler;   0 = no current
#define MAX_COOLER_POWER 255 // Limits duty cycle to cooler; 255 = full current
#define DEFAULT_coolerKp 10.00
#define DEFAULT_coolerKi .023
#define DEFAULT_coolerKd 305.4
#define INVERTED_HEATER_PINS false
#define INVERTED_BED_PIN false
#define INVERTED_CHAMBER_PIN false
#define INVERTED_COOLER_PIN false
#define THERMAL_PROTECTION_HOTENDS true
#define THERMAL_PROTECTION_BED true
#define THERMAL_PROTECTION_CHAMBER false
#define THERMAL_PROTECTION_COOLER false
#define THERMAL_PROTECTION_PERIOD    40     // Seconds
#define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius
#define WATCH_TEMP_PERIOD  20               // Seconds
#define WATCH_TEMP_INCREASE 2               // Degrees Celsius
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170                 // Degree Celsius
//#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH + Y_MAX_LENGTH)

/**********************
 * Configuration_Core *
 **********************/
#define _CONFIGURATION_CORE_H_
#define KNOWN_MECH
#define CUSTOM_MACHINE_NAME "VulcanusMAX40"
#define CORE_FACTOR 1
#define ENDSTOPPULLUPS
#if DISABLED(ENDSTOPPULLUPS)
//#define ENDSTOPPULLUP_XMIN
//#define ENDSTOPPULLUP_YMIN
//#define ENDSTOPPULLUP_ZMIN
//#define ENDSTOPPULLUP_Z2MIN
//#define ENDSTOPPULLUP_Z3MIN
//#define ENDSTOPPULLUP_Z4MIN
//#define ENDSTOPPULLUP_XMAX
//#define ENDSTOPPULLUP_YMAX
//#define ENDSTOPPULLUP_ZMAX
//#define ENDSTOPPULLUP_Z2MAX
//#define ENDSTOPPULLUP_Z3MAX
//#define ENDSTOPPULLUP_Z4MAX
//#define ENDSTOPPULLUP_ZPROBE
//#define ENDSTOPPULLUP_EMIN
#endif
#define X_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z3_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z4_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z3_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z4_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z_PROBE_ENDSTOP_LOGIC false   // set to true to invert the logic of the probe.
#define E_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define ENDSTOP_INTERRUPTS_FEATURE
#define Z_ENDSTOP_SERVO_NR -1
#define Z_ENDSTOP_SERVO_ANGLES {90,0} // Z Servo Deploy and Stow angles
#define PROBE_MANUALLY
//#define Z_PROBE_FIX_MOUNTED
//#define BLTOUCH
//#define BLTOUCH_DELAY 375 // (ms) Enable and increase if needed
//#define Z_PROBE_SLED
#define SLED_DOCKING_OFFSET 5
#define X_PROBE_OFFSET_FROM_NOZZLE  0     // X offset: -left  [of the nozzle] +right
#define Y_PROBE_OFFSET_FROM_NOZZLE  0     // Y offset: -front [of the nozzle] +behind
#define Z_PROBE_OFFSET_FROM_NOZZLE -1     // Z offset: -below [of the nozzle] (always negative!)
#define XY_PROBE_SPEED 10000
#define Z_PROBE_SPEED_FAST 120
#define Z_PROBE_SPEED_SLOW 60
#define Z_PROBE_REPETITIONS 1
//#define Z_MIN_PROBE_REPEATABILITY_TEST
#define Z_PROBE_DEPLOY_HEIGHT 15  // Z position for the probe to deploy/stow
#define Z_PROBE_BETWEEN_HEIGHT 5  // Z position for travel between points
#define Z_PROBE_OFFSET_RANGE_MIN -50
#define Z_PROBE_OFFSET_RANGE_MAX  50
//#define PROBING_HEATERS_OFF       // Turn heaters off when probing
//#define PROBING_FANS_OFF          // Turn fans off when probing
#define LCD_BED_LEVELING
#define LCD_Z_STEP 0.025    // Step size while manually probing Z axis.
#define LCD_PROBE_Z_RANGE 4 // Z Range centered on Z_MIN_POS for LCD Z adjustment
#define LEVEL_BED_CORNERS   // Add an option to move between corners
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define E_HOME_DIR -1
#define MIN_Z_HEIGHT_FOR_HOMING   0
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false
//#define DISABLE_INACTIVE_EXTRUDER
#define X_MAX_POS 380
#define X_MIN_POS 0
#define Y_MAX_POS 405
#define Y_MIN_POS 0
#define Z_MAX_POS 300
#define Z_MIN_POS 0
#define E_MIN_POS 0
#define AXIS_RELATIVE_MODES {false, false, false, false}
#define Z_SAFE_HOMING
#define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)
#define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)
#define MESH_BED_LEVELING
//#define AUTO_BED_LEVELING_LINEAR
//#define AUTO_BED_LEVELING_BILINEAR
//#define AUTO_BED_LEVELING_3POINT
//#define DEBUG_LEVELING_FEATURE
#define MESH_INSET 10
#define MESH_MIN_X (X_MIN_POS + MESH_INSET)
#define MESH_MAX_X (X_MAX_POS - (MESH_INSET))
#define MESH_MIN_Y (Y_MIN_POS + MESH_INSET)
#define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))
//#define MESH_G28_REST_ORIGIN
#define GRID_MAX_POINTS_X 3
#define GRID_MAX_POINTS_Y 3
#define LEFT_PROBE_BED_POSITION 20
#define RIGHT_PROBE_BED_POSITION 360
#define FRONT_PROBE_BED_POSITION 20
#define BACK_PROBE_BED_POSITION 360
#define MIN_PROBE_EDGE 10
//#define PROBE_Y_FIRST
//#define ABL_BILINEAR_SUBDIVISION
#define BILINEAR_SUBDIVISIONS 3
#define ABL_PROBE_PT_1_X 15
#define ABL_PROBE_PT_1_Y 180
#define ABL_PROBE_PT_2_X 15
#define ABL_PROBE_PT_2_Y 15
#define ABL_PROBE_PT_3_X 180
#define ABL_PROBE_PT_3_Y 15
//#define Z_PROBE_END_SCRIPT "G1 Z10 F8000\nG1 X10 Y10\nG1 Z0.5"
#define ENABLE_LEVELING_FADE_HEIGHT
//#define BED_CENTER_AT_0_0
//#define MANUAL_X_HOME_POS 0
//#define MANUAL_Y_HOME_POS 0
//#define MANUAL_Z_HOME_POS 0
#define DEFAULT_AXIS_STEPS_PER_UNIT   {160, 160, 5120, 24, 24, 24, 24}
#define DEFAULT_MAX_FEEDRATE          {300, 300, 2, 100, 100, 100, 100}
#define MANUAL_FEEDRATE               {50*60, 50*60, 4*60, 10*60}
#define DEFAULT_MINIMUMFEEDRATE       0.0
#define DEFAULT_MINTRAVELFEEDRATE     0.0
#define MINIMUM_PLANNER_SPEED         0.05                      // (mm/sec)
#define DEFAULT_MAX_ACCELERATION              {3000, 3000, 50, 1000, 1000, 1000, 1000}
#define DEFAULT_RETRACT_ACCELERATION          {10000, 10000, 10000, 10000}
#define DEFAULT_ACCELERATION          3000
#define DEFAULT_TRAVEL_ACCELERATION   3000
#define DEFAULT_XJERK 10.0
#define DEFAULT_YJERK 10.0
#define DEFAULT_ZJERK  0.4
#define DEFAULT_EJERK                   {5.0, 5.0, 5.0, 5.0}
#define HOMING_FEEDRATE_X (50*60)
#define HOMING_FEEDRATE_Y (50*60)
#define HOMING_FEEDRATE_Z (2*60)
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {5, 5, 2}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
#define HOTEND_OFFSET_X {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the X axis
#define HOTEND_OFFSET_Y {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Y axis
#define HOTEND_OFFSET_Z {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Z axis

/*************************
 * Configuration_Feature *
 *************************/
#define FAN_PWM_SPEED 0
#define FAN_KICKSTART_TIME 10000
#define FAN_MIN_PWM 64
//#define INVERTED_FAN_PINS
//#define CONTROLLERFAN
#define CONTROLLERFAN_SECS       60   // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED     255   // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED   0
//#define HOTEND_AUTO_FAN
//#define INVERTED_AUTO_FAN_PINS
#define HOTEND_AUTO_FAN_TEMPERATURE  50
#define HOTEND_AUTO_FAN_SPEED       255  // 255 = full speed
#define HOTEND_AUTO_FAN_MIN_SPEED     0
//#define VOLUMETRIC_DEFAULT_ON
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75
#define SINGLENOZZLE
//#define BARICUDA
//#define EXT_SOLENOID
#define COLOR_MIXING_EXTRUDER
#define MIXING_STEPPERS 3
#define MIXING_VIRTUAL_TOOLS 16
//#define MKR4
//#define INVERTED_RELE_PINS
//#define MKR6
//#define INVERTED_RELE_PINS
//#define MKR12
//#define INVERTED_RELE_PINS
//#define MKSE6
#define MKSE6_SERVO_INDEX    0
#define MKSE6_SERVOPOS_E0  -60
#define MKSE6_SERVOPOS_E1  -30
#define MKSE6_SERVOPOS_E2    0
#define MKSE6_SERVOPOS_E3   30
#define MKSE6_SERVOPOS_E4   60
#define MKSE6_SERVOPOS_E5   90
#define MKSE6_SERVO_DELAY 1000
//#define NPR2
#define COLOR_STEP {0, 10, 20, 30}   // CARTER ANGLE
#define COLOR_SLOWRATE 170           // MICROSECOND delay for carter motor routine (Carter Motor Feedrate: upper value-slow feedrate)  
#define COLOR_HOMERATE 4             // FEEDRATE for carter home
#define MOTOR_ANGLE 1.8              // Nema angle for single step 
#define DRIVER_MICROSTEP 4           // Microstep moltiplicator driver (set jumper MS1-2-3) off-on-off 1/4 microstepping.
#define CARTER_MOLTIPLICATOR 14.22   // CARTER MOLTIPLICATOR (gear ratio 13/31-10/31)
//#define DONDOLO_SINGLE_MOTOR
//#define DONDOLO_DUAL_MOTOR
#define DONDOLO_SERVO_INDEX 0
#define DONDOLO_SERVOPOS_E0 120
#define DONDOLO_SERVOPOS_E1 10
#define DONDOLO_SERVO_DELAY 1000
//#define IDLE_OOZING_PREVENT
#define IDLE_OOZING_MINTEMP           190
#define IDLE_OOZING_FEEDRATE          50    //default feedrate for retracting (mm/s)
#define IDLE_OOZING_SECONDS           5
#define IDLE_OOZING_LENGTH            15    //default retract length (positive mm)
#define IDLE_OOZING_RECOVER_LENGTH    0     //default additional recover length (mm, added to retract length when recovering)
#define IDLE_OOZING_RECOVER_FEEDRATE  50    //default feedrate for recovering from retraction (mm/s)
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS  30
#define EXTRUDER_RUNOUT_SPEED  1500 // mm/m
#define EXTRUDER_RUNOUT_EXTRUDE   5 // mm
#define EASY_LOAD
#define BOWDEN_LENGTH 250       // mm
#define LCD_PURGE_LENGTH 3      // mm
#define LCD_RETRACT_LENGTH 3    // mm
#define LCD_PURGE_FEEDRATE 3    // mm/s
#define LCD_RETRACT_FEEDRATE 10 // mm/s
#define LCD_LOAD_FEEDRATE 8     // mm/s
#define LCD_UNLOAD_FEEDRATE 8   // mm/s
//#define ADVANCE
#define EXTRUDER_ADVANCE_K 0.0
#define D_FILAMENT 1.75
//#define LIN_ADVANCE
#define LIN_ADVANCE_K 75
#define LIN_ADVANCE_E_D_RATIO 0
//#define WORKSPACE_OFFSETS
#define MIN_SOFTWARE_ENDSTOPS
#define MAX_SOFTWARE_ENDSTOPS
#define ENDSTOPS_ONLY_FOR_HOMING
//#define ENABLED_ALL_SIX_ENDSTOP
#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
#define ABORT_ON_ENDSTOP_HIT_INIT true
//#define G38_PROBE_TARGET
#define G38_MINIMUM_MOVE 0.0275
//#define SCAD_MESH_OUTPUT
//#define ENABLE_SERVOS
#define NUM_SERVOS 0
//#define DEACTIVATE_SERVOS_AFTER_MOVE
#define SERVO_DEACTIVATION_DELAY 300
//#define Z_LATE_ENABLE
#define SLOWDOWN
//#define QUICK_HOME
//#define HOME_Y_BEFORE_X
//#define FORCE_HOME_XY_BEFORE_Z
//#define BABYSTEPPING
#define BABYSTEP_XY  
#define BABYSTEP_INVERT_Z false
#define BABYSTEP_MULTIPLICATOR 2
//#define FWRETRACT                     // ONLY PARTIALLY TESTED
#define MIN_RETRACT                 0.1 // A retract/recover of this length or longer can be converted to auto-retract
#define RETRACT_LENGTH                3 // Default retract length (positive mm)
#define RETRACT_LENGTH_SWAP          13 // Default swap retract length (positive mm), for extruder change
#define RETRACT_FEEDRATE             45 // Default feedrate for retracting (mm/s)
#define RETRACT_ZLIFT                 0 // Default retract Z-lift
#define RETRACT_RECOVER_LENGTH        0 // Default additional recover length (mm, added to retract length when recovering)
#define RETRACT_RECOVER_LENGTH_SWAP   0 // Default additional swap recover length (mm, added to retract length when recovering from extruder change)
#define RETRACT_RECOVER_FEEDRATE      8 // Default feedrate for recovering from retraction (mm/s)
#define RETRACT_RECOVER_FEEDRATE_SWAP 8 // Default feedrate for recovering from swap retraction (mm/s)
//#define DUAL_X_CARRIAGE
#define X2_MIN_POS 80     // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_MAX_POS 353    // set maximum to the distance between toolheads when both heads are homed
#define X2_HOME_DIR 1     // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position
#define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_FULL_CONTROL_MODE
#define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder
#define DEFAULT_DUPLICATION_X_OFFSET 100
//#define X_TWO_STEPPER
#define INVERT_X2_VS_X_DIR false
//#define Y_TWO_STEPPER
#define INVERT_Y2_VS_Y_DIR false
//#define Z_TWO_STEPPER
//#define Z_THREE_STEPPER
//#define Z_FOUR_STEPPER
#define INVERT_Z2_VS_Z_DIR false
#define INVERT_Z3_VS_Z_DIR false
#define INVERT_Z4_VS_Z_DIR false
//#define Z_TWO_ENDSTOPS
//#define Z_THREE_ENDSTOPS
//#define Z_FOUR_ENDSTOPS
//#define XY_FREQUENCY_LIMIT  15
//#define SF_ARC_FIX
#define EXTRUDER_ENCODER_CONTROL
#define ENC_ERROR_STEPS     500
#define ENC_MIN_STEPS        10
//#define INVERTED_ENCODER_PINS
//#define FILAMENT_SENSOR
#define FILAMENT_SENSOR_EXTRUDER_NUM  0     //The number of the extruder that has the filament sensor (0,1,2,3)
#define MEASUREMENT_DELAY_CM         14     //measurement delay in cm.  This is the distance from filament sensor to middle of barrel
#define MEASURED_UPPER_LIMIT          2.00  //upper limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT          1.35  //lower limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY        20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially
//#define FILAMENT_LCD_DISPLAY
//#define FILAMENT_RUNOUT_SENSOR
//#define FILAMENT_RUNOUT_DAV_SYSTEM
#define FIL_RUNOUT_PIN_INVERTING true
//#define ENDSTOPPULLUP_FIL_RUNOUT
#define FILAMENT_RUNOUT_DOUBLE_CHECK 0
#define FILAMENT_RUNOUT_SCRIPT "M600"
//#define POWER_CONSUMPTION
#define POWER_VOLTAGE      12.00    //(V) The power supply OUT voltage
#define POWER_SENSITIVITY   0.066   //(V/A) How much increase V for 1A of increase
#define POWER_OFFSET        0.005   //(A) Help to get 0A when no load is connected.
#define POWER_ZERO          2.500   //(V) The /\V coming out from the sensor when no current flow.
#define POWER_ERROR         0.0     //(%) Ammortize measure error.
#define POWER_EFFICIENCY  100.0     //(%) The power efficency of the power supply
//#define POWER_CONSUMPTION_LCD_DISPLAY
//#define FLOWMETER_SENSOR
#define FLOWMETER_MAXFLOW  6.0      // Liters per minute max
#define FLOWMETER_MAXFREQ  55       // frequency of pulses at max flow
//#define MINFLOW_PROTECTION 4      
//#define DOOR_OPEN
#define DOOR_OPEN_LOGIC false
//#define DOOR_OPEN_PULLUP
//#define POWER_CHECK
#define POWER_CHECK_LOGIC false
//#define POWER_CHECK_PULLUP
#define EEPROM_SETTINGS
#define EEPROM_CHITCHAT // Uncomment this to enable EEPROM Serial responses.
//#define EEPROM_SD
//#define DISABLE_M503
#define SDSUPPORT
//#define SDSLOW              // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SDEXTRASLOW         // Use even slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY  // Use CRC checks and retries on the SD communication
#define SD_EXTENDED_DIR     // Show extended directory including file length. Don't use this with Pronterface
//#define SD_DISABLED_DETECT
//#define SD_DETECT_INVERTED
#define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.
#define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display. Its sorted practically after the file system block order.
//#define MENU_ADDAUTOSTART
//#define SD_SETTINGS                     // Uncomment to enable
#define SD_CFG_SECONDS        300         // seconds between update
#define LCD_LANGUAGE en
#define DISPLAY_CHARSET_HD44780 JAPANESE
#define SHOW_BOOTSCREEN
//#define SHOW_CUSTOM_BOOTSCREEN
#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION       // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE      // will be shown during bootup in line 2
#define BOOTSCREEN_TIMEOUT 2000
//#define ULTRA_LCD   // Character based
//#define DOGLCD      // Full graphics display
//#define XYZ_HOLLOW_FRAME
//#define MENU_HOLLOW_FRAME
#define USE_BIG_EDIT_FONT
#define USE_SMALL_INFOFONT
//#define DOGM_SPI_DELAY_US 5
#define ENCODER_PULSES_PER_STEP 2
#define ENCODER_STEPS_PER_MENU_ITEM 3
//#define LCD_SCREEN_ROT_90    // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   // Rotate screen orientation for graphics display by 270 degree clockwise
//#define INVERT_CLICK_BUTTON           // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            // Option for invert back button logic if avaible
//#define REVERSE_ENCODER_DIRECTION
//#define REVERSE_MENU_DIRECTION
#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value
#define DOUBLECLICK_FOR_Z_BABYSTEPPING
#define DOUBLECLICK_MAX_INTERVAL 1250
#define ULTIPANEL_FEEDMULTIPLY
#define SPEAKER
#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 20
#define LCD_FEEDBACK_FREQUENCY_HZ 100
//#define UI_VOLTAGE_LEVEL 0 // 3.3 V
#define UI_VOLTAGE_LEVEL 1   // 5 V
#define LCD_INFO_MENU
#define STATUS_MESSAGE_SCROLLING
#define LCD_DECIMAL_SMALL_XY
#define LCD_TIMEOUT_TO_STATUS 30000
//#define ULTIMAKERCONTROLLER
//#define ULTIPANEL
//#define CARTESIO_UI
//#define RADDS_DISPLAY
//#define PANEL_ONE
//#define MAKRPANEL
#define REPRAPWORLD_GRAPHICAL_LCD
//#define VIKI2
//#define miniVIKI
//#define ELB_FULL_GRAPHIC_CONTROLLER
//#define REPRAP_DISCOUNT_SMART_CONTROLLER
//#define G3D_PANEL
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
//#define MINIPANEL
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
//#define RIGIDBOT_PANEL
//#define BQ_LCD_SMART_CONTROLLER
//#define RA_CONTROL_PANEL
//#define LCD_I2C_SAINSMART_YWROBOT
//#define LCM1602
//#define LCD_I2C_PANELOLU2
//#define LCD_I2C_VIKI
//#define U8GLIB_SSD1306
//#define WANHAO_D6_OLED
//#define SAV_3DGLCD
//#define ANET_KEYPAD_LCD
//#define ANET_FULL_GRAPHICS_LCD
//#define SAV_3DLCD
//#define NEXTION
#define NEXTION_SERIAL 1
#define NEXTION_UPDATE_INTERVAL 3000
//#define NEXTION_GFX
#define NEXTION_FIRMWARE_FILE "mk4duo.tft"
//#define LCD_PROGRESS_BAR
#define PROGRESS_BAR_BAR_TIME 5000
#define PROGRESS_BAR_MSG_TIME 1500
#define PROGRESS_MSG_EXPIRE 0
//#define PROGRESS_MSG_ONCE
//#define LCD_PROGRESS_BAR_TEST
//#define PHOTOGRAPH
//#define CHDK
#define CHDK_DELAY 50   //How long in ms the pin should stay HIGH before going LOW again
//#define RFID_MODULE
#define RFID_SERIAL 1
//#define BLINKM
//#define RGB_LED
//#define RGBW_LED
//#define PCA9632
//#define NEOPIXEL_RGB_LED
//#define NEOPIXEL_RGBW_LED
#define NEOPIXEL_PIXELS 16
//#define NEOPIXEL_STARTUP_TEST
//#define PRINTER_EVENT_LEDS
//#define LASER
//#define CNCROUTER
//#define CASE_LIGHT
#define INVERT_CASE_LIGHT false
#define CASE_LIGHT_DEFAULT_ON false
#define CASE_LIGHT_DEFAULT_BRIGHTNESS 255   
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X
#define DISABLE_INACTIVE_Y
#define DISABLE_INACTIVE_Z
#define DISABLE_INACTIVE_E
#define MINIMUM_STEPPER_PULSE 0
//#define USE_MICROSTEPS
#define MICROSTEP_MODES {16, 16, 16, 16}
#define MOTOR_CURRENT {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
#define DIGIPOT_MOTOR_CURRENT {135, 135, 135, 135, 135}
#define PWM_MOTOR_CURRENT {1200, 1000, 1000}
//#define DIGIPOT_I2C
#define DIGIPOT_I2C_NUM_CHANNELS 8
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
//#define CONFIG_STEPPERS_TOSHIBA
//#define HAVE_TMCDRIVER
//#define HAVE_TMC2130
//#define HAVE_L6470DRIVER
#define PWM_HARDWARE true
#define BLOCK_BUFFER_SIZE 32
#define MAX_CMD_SIZE 96
#define BUFSIZE 8
#define NUM_POSITON_SLOTS 2
#define DEFAULT_MINSEGMENTTIME 20000
#define ARC_SUPPORT
#define MM_PER_ARC_SEGMENT 1    // Length of each arc segment
#define N_ARC_CORRECTION  25    // Number of intertpolated segments between corrections
//#define ARC_P_CIRCLES         // Enable the 'P' parameter to specify complete circles
//#define CNC_WORKSPACE_PLANES  // Allow G2/G3 to operate in XY, ZX, or YZ planes
#define MIN_STEPS_PER_SEGMENT 6
#define M100_FREE_MEMORY_WATCHER
#define M100_FREE_MEMORY_DUMPER
#define M100_FREE_MEMORY_CORRUPTOR
//#define NOZZLE_CLEAN_FEATURE
#define NOZZLE_CLEAN_STROKES 12
#define NOZZLE_CLEAN_TRIANGLES 3
#define NOZZLE_CLEAN_START_POINT { 30, 30, (Z_MIN_POS + 1)}
#define NOZZLE_CLEAN_END_POINT   {100, 60, (Z_MIN_POS + 1)}
#define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5
#define NOZZLE_CLEAN_CIRCLE_FN 10
#define NOZZLE_CLEAN_CIRCLE_MIDDLE NOZZLE_CLEAN_START_POINT
#define NOZZLE_CLEAN_GOBACK
//#define NOZZLE_PARK_FEATURE
#define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
//#define ADVANCED_PAUSE_FEATURE
#define PAUSE_PARK_X_POS 3                  // X position of hotend
#define PAUSE_PARK_Y_POS 3                  // Y position of hotend
#define PAUSE_PARK_Z_ADD 10                 // Z addition of hotend (lift)
#define PAUSE_PARK_XY_FEEDRATE 100          // X and Y axes feedrate in mm/s (also used for delta printers Z axis)
#define PAUSE_PARK_Z_FEEDRATE 5             // Z axis feedrate in mm/s (not used for delta printers)
#define PAUSE_PARK_RETRACT_FEEDRATE 20      // Initial retract feedrate in mm/s
#define PAUSE_PARK_RETRACT_LENGTH 2         // Initial retract in mm
#define PAUSE_PARK_COOLDOWN_TEMP 160        // Temp for cooldown, if this parameter is equal to 0 no cooling.
#define PAUSE_PARK_RETRACT_2_FEEDRATE 20    // Second retract filament feedrate in mm/s - filament retract post cool down
#define PAUSE_PARK_RETRACT_2_LENGTH 20      // Second retract filament length from hotend in mm
#define PAUSE_PARK_UNLOAD_FEEDRATE 100      // Unload filament feedrate in mm/s - filament unloading can be fast
#define PAUSE_PARK_UNLOAD_LENGTH 100        // Unload filament length from hotend in mm
#define PAUSE_PARK_LOAD_FEEDRATE 100        // Load filament feedrate in mm/s - filament loading into the bowden tube can be fast
#define PAUSE_PARK_LOAD_LENGTH 100          // Load filament length over hotend in mm
#define PAUSE_PARK_EXTRUDE_FEEDRATE 5       // Extrude filament feedrate in mm/s - must be slower than load feedrate
#define PAUSE_PARK_EXTRUDE_LENGTH 50        // Extrude filament length in mm after filament is load over the hotend,
#define PAUSE_PARK_NOZZLE_TIMEOUT 45        // Turn off nozzle if user doesn't change filament within this time limit in seconds
#define PAUSE_PARK_PRINTER_OFF 5            // Turn off printer if user doesn't change filament within this time limit in Minutes
#define PAUSE_PARK_NUMBER_OF_ALERT_BEEPS 5  // Number of alert beeps before printer goes quiet
#define PAUSE_PARK_NO_STEPPER_TIMEOUT       // Enable to have stepper motors hold position during filament change
//#define PARK_HEAD_ON_PAUSE                // Go to filament change position on pause, return to print position on resume
#define INCH_MODE_SUPPORT
//#define JSON_OUTPUT
#define PINS_DEBUGGING
#define AUTO_REPORT_TEMPERATURES
#define EXTENDED_CAPABILITIES_REPORT
//#define USE_WATCHDOG
//#define WATCHDOG_RESET_MANUAL
#define START_GCODE
#define START_PRINTING_SCRIPT "G28\nG1 Z10 F8000"
#define STOP_GCODE
#define STOP_PRINTING_SCRIPT "G28\nM107\nM104 T0 S0\nM140 S0\nM84\nM81"
#define CUSTOM_USER_MENUS
#define USER_SCRIPT_DONE "M117 User Script Done"
#define USER_DESC_1 "Home & ABL"
#define USER_GCODE_1 "G28\nG29"
#define USER_DESC_2 "Preheat for PLA"
#define USER_GCODE_2 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
#define USER_DESC_3 "Preheat for ABS"
#define USER_GCODE_3 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)
#define USER_DESC_4 "Heat Bed/Home/Level"
#define USER_GCODE_4 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"
#define USER_DESC_5 "Home & Info"
#define USER_GCODE_5 "G28\nM503"

