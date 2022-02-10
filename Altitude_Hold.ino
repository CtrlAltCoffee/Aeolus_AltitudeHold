#include <Servo.h>
#include <MPU6050.h>
#include <BMP280_DEV.h>
#include <RF24.h>
using namespace std;

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

#define YAW     0
#define PITCH   1
#define ROLL    2
#define THRUST  3
#define ALT     3

#define FREQ 250 // Sampling frequency

#define ESC_PIN 4 // ESC connection pins 4 to 7
#define RADIO_CE 8 // Radio CE connection pin
#define RADIO_CSN 9 // Radio CSN connection pin

#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000
#define MOTOR_SPEED_RANGE 1023 // Speed levels in brushless motor

Servo ESC[4]; 
MPU6050 Mpu;
BMP280_DEV Bmp;
RF24 Radio(RADIO_CE, RADIO_CSN);  

unsigned long timer = 0;            // Central system timer in 
float time_step = (1000000/FREQ);   // Sampling period in µm
bool altHld = false;                // Altitude Hold is disabled in the start

const byte address[6] = "00001";    // Pude Address for RF Radio
char command[32] = {0};             // Carrier string for RF Radio

float acc[3]   = {0, 0, 0};         // Acceleration vector (m/s^2)
float rot[3]   = {0, 0, 0};         // Angular velocity vector (rad/s)
float state[3] = {0, 0, 0};			// Yaw, Pitch, Roll and Thrust in Real Time
float bar, alt;                     // Pressure (hPa) and altitude (m)

// PID Controller Coefficients
float Kp[3] = {4.0, 1.3, 1.3};      // Proportional coefficients
float Ki[3] = {0.02, 0.04, 0.04};   // Integral coefficients
float Kd[3] = {0, 18, 18};          // Derivative coefficients 


float limit(float val, float min_val, float max_val) {
    
    if(val > max_val) 
        return max_val;
    else if(val < min_val)
        return min_val;
    else
        return val;
    
}

void setup() {
    
    Serial.begin(115200);
    for(int i = 0; i < 4; i++)
        ESC[i].attach(ESC_PIN+i, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    
    // Initialize MPU  
    Mpu.CalibrateGyro();

    // Initialise BMP
    Bmp.begin();
    Bmp.setTimeStandby(TIME_STANDBY_1000MS);
    Bmp.startNormalConversion();

    // Initialise Radio
    Radio.begin();
    Radio.openReadingPipe(0, address);
    Radio.startListening(); // Set module as receiver
    
}

void getReadings() {

    // Get acceleration readings
    acc[0] = Mpu.getAccelerationX();
    acc[1] = Mpu.getAccelerationY();
    acc[2] = Mpu.getAccelerationZ();

    // Get angular velocity readings
    rot[0] = Mpu.getRotationX();
    rot[1] = Mpu.getRotationY();
    rot[2] = Mpu.getRotationZ();

    // Get pressure and altitude 
    Bmp.getCurrentPressure(bar);
    Bmp.getCurrentAltitude(alt); 

    // Update current drone state
    state[YAW] += 180/PI * rot[0] * time_step;
    state[ROLL] += 180/PI * rot[1] * time_step;
    state[PITCH] += 180/PI * rot[2] * time_step;

}

void logReadings() {
    
    // Print acceleration info
    Serial.print("Acceleration X: ");
	Serial.print(acc[0]);
	Serial.print(", Y: ");
	Serial.print(acc[1]);
	Serial.print(", Z: ");
	Serial.print(acc[2]);
	Serial.println("m/s^2");

    // Print angular velocity info
    Serial.print("Rotation X: ");
    Serial.print(rot[0]);    
    Serial.print(", Y: ");
    Serial.print(rot[1]); 
    Serial.print(", Z: ");
    Serial.print(rot[2]);
    Serial.println("rad/s");

    // Print angular orientation info
    Serial.print("Orientation Yaw: ");
    Serial.print(state[0]);    
    Serial.print(", Pitch: ");
    Serial.print(state[1]); 
    Serial.print(", Roll: ");
    Serial.print(state[2]);
    Serial.println("degrees");

    // Print pressure and altitude info
    Serial.print("Pressure: ");
    Serial.print(bar);    
    Serial.println("hPa");
    Serial.print("Altitude: ");
    Serial.print(alt);
    Serial.println("m");

    Serial.println("");
}

void receiveRadio(int controls[4]) {

	altHld = true;

}

void altitudeHold(int controls[4]) {

    // PID controller system variables: yaw, roll, pitch and altitude   
    const static float set_points[4] = {15, 15, 15, alt};            
    static float error[4];                                          
    static float error_delta[4] = {0, 0, 0, 0};               
    static float error_sum[4]   = {0, 0, 0, 0};                 
    static float error_prev[4]  = {0, 0, 0, 0};                 

    error[YAW] = sgn(state[YAW]) * (abs(state[YAW]) - set_points[YAW]);
    error[ROLL] = sgn(state[ROLL]) * (abs(state[ROLL]) - set_points[ROLL]);
    error[PITCH] = sgn(state[PITCH]) * (abs(state[PITCH]) - set_points[PITCH]);
    error[ALT] = alt - set_points[ALT];

	for(int i = 0; i < 4; i++){
		error_delta[i] = error[i] - error_prev[i];
    	error_sum[i] += error[i];
    	error_prev[i] = error[i];
	}

    // PID = Kp.e + Ki.sum(e) + Kd.de/dt 
    controls[YAW] = (Kp[YAW] * error[YAW]) + (Ki[YAW] * error_sum[YAW]) + (Kd[YAW] * error_delta[YAW]/time_step); 
    controls[ROLL] = (Kp[ROLL] * error[ROLL]) + (Ki[ROLL] * error_sum[ROLL]) + (Kd[ROLL] * error_delta[ROLL]/time_step); 
    controls[PITCH] = (Kp[PITCH] * error[PITCH]) + (Ki[PITCH] * error_sum[PITCH]) + (Kd[PITCH] * error_delta[PITCH]/time_step); 
    controls[THRUST] = (Kp[THRUST] * error[THRUST]) + (Ki[THRUST] * error_sum[THRUST]) + (Kd[THRUST] * error_delta[THRUST]/time_step); 

}

void moveDrone(int controls[4]) {

    // Send motor controls signals to ESCs using Pulse Width Modulation
    ESC[0].write(map(controls[THRUST] + controls[YAW] - controls[PITCH] - controls[ROLL], 0, MOTOR_SPEED_RANGE, 0, 180));
    ESC[1].write(map(controls[THRUST] - controls[YAW] - controls[PITCH] + controls[ROLL], 0, MOTOR_SPEED_RANGE, 0, 180));
    ESC[2].write(map(controls[THRUST] - controls[YAW] + controls[PITCH] - controls[ROLL], 0, MOTOR_SPEED_RANGE, 0, 180));
    ESC[3].write(map(controls[THRUST] + controls[YAW] + controls[PITCH] + controls[ROLL], 0, MOTOR_SPEED_RANGE, 0, 180));

}

void loop() {

    int controls[4]; // Signals array sent to motors through ESC

    timer = micros();

    getReadings(); // Update sensor values
    logReadings(); // Output data to Serial Monitor
    receiveRadio(controls); // Receive controller signals from RF

    if(altHld)
        altitudeHold(controls); // Maintain quadcopter altitude
    
    moveDrone(controls); // Communicate controls with ESC
    
    delay((time_step*1000) - (micros() - timer)); // wait for time step to finish

}
