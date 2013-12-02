/* 
	Editor: http://www.visualmicro.com
	        arduino debugger, visual micro +, free forum and wiki
	
	Hardware: Arduino Due (Programming Port), Platform=sam, Package=arduino
*/

#define __SAM3X8E__
#define USB_VID 0x2341
#define USB_PID 0x003e
#define USBCON
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 84000000L
#define printf iprintf
#define __SAM__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

void setConfiguration();
//
//
void newConfig();
void A2interruptFun();
void A3interruptFun();
void read_sensors();
void reset_sensor_fusion();
void compensate_sensor_errors();
void check_reset_calibration_session();
void turn_output_stream_on();
void turn_output_stream_off();
char readChar();
void init_imu_ahrs();
void loop_k();
void Compass_Heading();
void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);
float Vector_Dot_Product(const float v1[3], const float v2[3]);
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
void Vector_Scale(float out[3], const float v[3], float scale);
void Vector_Add(float out[3], const float v1[3], const float v2[3]);
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
void I2C_Init();
void Accel_Init();
void Read_Accel();
void Magn_Init();
void Read_Magn();
void Gyro_Init();
void Read_Gyro();
void processCommand();
void printCommand();
void setCommand();
static void vControlTask(void *pvParameters);
static void vInterfaceAndDebugTask(void *pvParameters);

#include "C:\Program Files (x86)\Arduino\hardware\arduino\sam\variants\arduino_due_x\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\sam\variants\arduino_due_x\variant.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\sam\cores\arduino\arduino.h"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\selfbalancing_robot_arduino_due.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\imu_ahrs.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\imu_compass.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\imu_dcm.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\imu_math.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\imu_sensors.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\serial_commands.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\v_control_task.ino"
#include "Z:\sebnil On My Mac\Google Drive\Arduino\selfbalancing_robot_v2\selfbalancing_robot_arduino_due\v_interface_and_debug_task.ino"
