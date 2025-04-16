//Default config for GTS. Typical s series hypercore values, GTS battery, Corrected IMU Yaw. 

//Default motor conf
#ifndef MCCONF_FOC_MOTOR_R
#define MCCONF_FOC_MOTOR_R      0.087
#endif
#ifndef MCCONF_FOC_MOTOR_L
#define MCCONF_FOC_MOTOR_L      0.000198
#endif
#ifndef MCCONF_FOC_MOTOR_LD_LQ_DIFF
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF     0.000040
#endif
#ifndef MCCONF_FOC_MOTOR_FLUX_LINKAGE
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE   0.0280
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN
#define MCCONF_FOC_OBSERVER_GAIN    8e5
#endif
#ifndef MCCONF_FOC_CURRENT_KP
#define MCCONF_FOC_CURRENT_KP       0.1980
#endif
#ifndef MCCONF_FOC_CURRENT_KI
#define MCCONF_FOC_CURRENT_KI       87.0
#endif
#ifndef MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY
#define MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY    1
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE        30
#endif
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX        85
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN        -85
#endif
#ifndef MCCONF_SENSOR_MODE
#define MCCONF_SENSOR_MODE          FOC_SENSOR_MODE_HALL
#endif
#ifndef MCCONF_HALL_TAB_0
#define MCCONF_HALL_TAB_0       -1
#endif
#ifndef MCCONF_HALL_TAB_1
#define MCCONF_HALL_TAB_1       -1
#endif
#ifndef MCCONF_HALL_TAB_2
#define MCCONF_HALL_TAB_2       -1
#endif
#ifndef MCCONF_HALL_TAB_3
#define MCCONF_HALL_TAB_3       -1
#endif
#ifndef MCCONF_HALL_TAB_4
#define MCCONF_HALL_TAB_4       -1
#endif
#ifndef MCCONF_HALL_TAB_5
#define MCCONF_HALL_TAB_5       -1
#endif
#ifndef MCCONF_HALL_TAB_6
#define MCCONF_HALL_TAB_6       -1
#endif
#ifndef MCCONF_HALL_TAB_7
#define MCCONF_HALL_TAB_7       -1
#endif
#ifndef MCCONF_FOC_SL_ERPM_START
#define MCCONF_FOC_SL_ERPM_START        1500
#endif
#ifndef MCCONF_FOC_SL_ERPM
#define MCCONF_FOC_SL_ERPM      2000
#endif
#ifndef MCCONF_FOC_HALL_INTERP_ERPM
#define MCCONF_FOC_HALL_INTERP_ERPM     250
#endif
#ifndef MCCONF_M_INVERT_DIRECTION
#define MCCONF_M_INVERT_DIRECTION       0
#endif

#ifndef MCCONF_SI_MOTOR_POLES
#define MCCONF_SI_MOTOR_POLES       30
#endif
#ifndef MCCONF_SI_GEAR_RATIO
#define MCCONF_SI_GEAR_RATIO        1
#endif
#ifndef MCCONF_SI_WHEEL_DIAMETER
#define MCCONF_SI_WHEEL_DIAMETER    292
#endif

#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE        119 //same as lim vin comment. this will trigger when we really shouldn't be triggering an OV fault, but the alternative is risking the fets blowing up. 
#endif
#ifndef MCCONF_SI_BATTERY_CELLS
#define MCCONF_SI_BATTERY_CELLS     27
#endif
#ifndef MCCONF_SI_BATTERY_AH
#define MCCONF_SI_BATTERY_AH        4.5
#endif

//Default app conf
#ifndef APPCONF_IMU_ROT_YAW
#define APPCONF_IMU_ROT_YAW         90
#endif
#ifndef APPCONF_IMU_SAMPLE_RATE
#define APPCONF_IMU_SAMPLE_RATE     832
#endif
