// 
// Robot Control Program
// Amy Everitt
// 24/09/2021
//
#include <Wire.h> // support library for talking to i2c

// define hardware addresses for easy reference
#define LM_PWMA   6           // Left Motor Speed pin (ENA)
#define LM_AIN2   A0          // Motor-L forward (IN2).
#define LM_AIN1   A1          // Motor-L backward (IN1)
#define RM_PWMB   5           // Right Motor Speed pin (ENB)
#define RM_BIN1   A2          // Motor-R forward (IN3)
#define RM_BIN2   A3          // Motor-R backward (IN4)
#define ULTRA_ECHO   2        // Ultrasound echo pin
#define ULTRA_TRIG   3        // Ultrasound trigger pin

// Define address of i2c devices
#define PCF8574_ADDR  0x20

// defines required for Kris Winer's code - MPU9250
#define MPU9250_ADDRESS 0x68  // Device address
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A

//define some speeds
#define SPD_FAST 70
#define SPD_NORMAL 50
#define SPD_SLOW 35
#define SPD_SUPER_SLOW 20
#define SPD_STOP 0

// define delays for checks in loop
#define CHECK_IMU 20

// Try to filter accelerometer noise
#define ACCEL_NOISE_THRESHOLD 0.2

// define G - 981 cms/s/s
#define G 980.6

// define values to simplify turning off and on serial debug messages
#define SerialDebugMPU9250 true      // set to true to get Serial output for debugging

// define global variables
int g_distance = 0;
int g_speed = SPD_NORMAL;
unsigned long g_current_ms = 0;
unsigned long g_prev_imu_check_ms = 0;

// speed and location variables
float prev_ax=0.0, prev_ay=0.0, prev_az=0.0;
float vx, vy, vz;
float prev_vx=0.0, prev_vy=0.0, prev_vz=0.0;
float dx=0.0, dy=0.0, dz=0.0;

// some variables to check how often we check the IMU
uint32_t num_greater_than_limit=0, max_imu_check=0;

// ############################################################################################################### //
// global constants for Kris Winers code
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0;                      // used to control display output rate
uint32_t count = 0, sumCount = 0;         // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

// globals for Kris Winer's code
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];                                            // holds results of gyro and accelerometer self test
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz;                     // variables to hold latest sensor data values 
float ax_yaw, ay_yaw, az_yaw;                                 // variables to hold acceleration with relation to the robot orientation
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};                        // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};                           // vector to hold integral error for Mahony method
float aRes, gRes, mRes;                                       // scale resolutions per LSB for the sensors
// ############################################################################################################### //

// define internal functions
void forward();
void backward();
void right();
void left();
void reverseToLeft();
void stop();
void MPU9250SelfTest(float * destination);
void calibrateMPU9250(float * dest1, float * dest2);
void initMPU9250();
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);
void getMres();
void getGres();
void getAres();

// Main program
void setup() {
  // Start serial interface for debug messages and send initial welcome message
  Serial.begin(115200);  
  Serial.println("Robot Control Program V7.10");

  // Initialise miilisecond counters
  g_current_ms = millis();
  g_prev_imu_check_ms = millis();

  // Start the wire library
  Wire.begin();

  // Configure left and right motor controllers
  pinMode(LM_PWMA,OUTPUT);                     
  pinMode(LM_AIN2,OUTPUT);      
  pinMode(LM_AIN1,OUTPUT);
  pinMode(RM_PWMB,OUTPUT);       
  pinMode(RM_BIN1,OUTPUT);     
  pinMode(RM_BIN2,OUTPUT); 

  // Start of Kris Winer's MPU9250 initialisation and calibration code
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(1000); 

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
 
    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    Serial.println("MPU9250 bias");
    Serial.println(" x   y   z  ");

    Serial.print((int)(1000*accelBias[0])); 
    Serial.print(" ");
    Serial.print((int)(1000*accelBias[1])); 
    Serial.print(" ");
    Serial.print((int)(1000*accelBias[2]));
    Serial.print(" "); 
    Serial.println("mg");
    
    Serial.print(gyroBias[0], 1); 
    Serial.print(" ");
    Serial.print(gyroBias[1], 1); 
    Serial.print(" ");
    Serial.print(gyroBias[2], 1); 
    Serial.print(" ");
    Serial.println("o/s");   
  
    delay(1000); 
  
    initMPU9250(); 
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  }
  else
  {  
    Serial.println("MPU9250 is not online...");
  }

  // Start motors and then bring to a stop until first distance measured
  analogWrite(LM_PWMA,g_speed);
  analogWrite(RM_PWMB,g_speed);
  stop();    
}

void loop() {
  g_current_ms = millis();

  if ((g_current_ms - g_prev_imu_check_ms) >= CHECK_IMU)
  {
    if ((g_current_ms - g_prev_imu_check_ms) > CHECK_IMU) {num_greater_than_limit++;};
    if ((g_current_ms - g_prev_imu_check_ms) > max_imu_check) {max_imu_check=(g_current_ms - g_prev_imu_check_ms);};
    g_prev_imu_check_ms = millis();

    // If intPin goes high, all data registers have new data
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
      readAccelData(accelCount);  // Read the x/y/z adc values
      getAres();

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
      ay = (float)accelCount[1]*aRes; // - accelBias[1];   
      az = (float)accelCount[2]*aRes; // - accelBias[2];  
   
      readGyroData(gyroCount);  // Read the x/y/z adc values
      getGres();
 
      // Calculate the gyro value into actual degrees per second
      gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
      gy = (float)gyroCount[1]*gRes;  
      gz = (float)gyroCount[2]*gRes;   

      // Don't worry about the magnetometer
//      readMagData(magCount);  // Read the x/y/z adc values
//      getMres();
//      magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//      magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
//      magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
      // Unsure of correction in Stellenbosch so will leave uncorrected
    
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
//      mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
//      my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
//      mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
  
      Now = micros();
      deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;

      sum += deltat; // sum for averaging filter update rate
      sumCount++;

      // Ignore first 10 iterations as the loop does seem to execute before the setup has finished 
      if (sumCount >= 10) 
      {
        // Do a check for noise. If total acceleratino is less than a predefined threshold, then assume robot is not accelerating in any direction.
        if ((abs(ax) + abs(ay) + abs(az)) > ACCEL_NOISE_THRESHOLD)
        { 
          // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
          // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
          // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
          // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
          // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
          // This is ok by aircraft orientation standards!  
          // Pass gyro rate as rad/s

          // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
          // MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
          // Swap y and z due to the orientation of the IMU board
          MadgwickAHRSupdateIMU(gx*PI/180.0f, gz*PI/180.0f, gy*PI/180.0f,ax,az,ay);
          
          // Assume only yaw is of interest (the robot is assumed not to pitch or roll)
          // float roll  = atan2(2.0 * (q[3] * q[2] + q[0] * q[1]) , 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
          // float pitch = asin(2.0 * (q[2] * q[0] - q[3] * q[1]));
          float yaw   = atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1]));
          // Serial.print("Yaw: ");Serial.println(yaw*180.0f/PI); // in degrees not radians

          // update ax and az accelerations to take in to account yaw i.e. the direction the robot is actually heading.
          ax_yaw = (ax * sin((yaw*180.0f/PI))) + (az * cos((yaw*180.0f/PI)));
          az_yaw = (ax * cos((yaw*180.0f/PI))) + (az * sin((yaw*180.0f/PI)));
             
          // calculate the speed as the average of the last 2 accelerations multiplied by the time between the measurements
          vx += ((prev_ax + ax_yaw) / 2.0f) * deltat * G; // speed in cm/s
          // vy += ((prev_ay + ay) / 2.0f) * deltat * G; // speed in cm/s
          vz += ((prev_az + az_yaw) / 2.0f) * deltat * G; // speed in cm/s
    
          // update previous acceleration for next calculation
          prev_ax = ax_yaw;
          // prev_ay = ay;
          prev_az = az_yaw;
    
          // update location given update in speeds, again averaging last two speeds and then muitplying by the time gap
          dx += ((prev_vx + vx) / 2.0f) * deltat; // distance in cms 
          // dy += ((prev_vy + vy) / 2.0f) * deltat; // distance in cms 
          dz += ((prev_vz + vz) / 2.0f) * deltat; // distance in cms 
          // Serial.print(ax,2);Serial.print(",");Serial.print(ay,2);Serial.print(",");Serial.println(az,2);
    
          // update previous acceleration for next calculation
          prev_vx = vx;
          // prev_vy = vy;
          prev_vz = vz;
    
          delt_t = millis() - count;
          if(delt_t > 5000) {
    
            if(SerialDebugMPU9250) {
              // Print acceleration values in milligs!
              //Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
              //Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
              //Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
              //Serial.print("Yaw: "); Serial.print(yaw); Serial.println(" deg ");
    
              Serial.print("Calculated Location (x,y,z) : "); Serial.print(dx,2);Serial.print(",");Serial.print(dy,2);Serial.print(",");Serial.println(dz,2);
              Serial.print("Yaw : "); Serial.println(yaw*180.0f/PI);
              Serial.print("Total Iterations (5s): "); Serial.print(sumCount);Serial.print("Late iterations: ");Serial.print(num_greater_than_limit);Serial.print("Max iteration: ");Serial.println(max_imu_check);
              num_greater_than_limit=0;
              max_imu_check=0;
    
              //Serial.print("DEBUG: (vx, vy, vz) = ");Serial.print(vx,6),Serial.print(",");Serial.print(vy,6);Serial.print(",");Serial.println(vz,6);
              //Serial.print("DEBUG: (dx, dy, dz) = ");Serial.print(dx,6),Serial.print(",");Serial.print(dy,6);Serial.print(",");Serial.println(dz,6);
     
              // Print gyro values in degree/sec
              // Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec "); 
              // Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec "); 
              // Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec"); 
        
              // Print mag values in degree/sec
              // Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
              // Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
              // Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 
     
              // tempCount = readTempData();  // Read the adc values
              // temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
              // Print temperature in degrees Centigrade      
              // Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
            }
            count = millis();
          }
        }
      }
    }
  }
}

//Supplementary functions

//move robot forward
void forward()
{
    analogWrite(LM_PWMA,g_speed);
    analogWrite(RM_PWMB,g_speed);
    digitalWrite(LM_AIN1,LOW);
    digitalWrite(LM_AIN2,HIGH);
    digitalWrite(RM_BIN1,LOW);  
    digitalWrite(RM_BIN2,HIGH); 
}

// reverse robot slowly
void reverse()
{
    stop();
    analogWrite(LM_PWMA,SPD_SLOW);
    analogWrite(RM_PWMB,SPD_SLOW);
    digitalWrite(LM_AIN1,HIGH);
    digitalWrite(LM_AIN2,LOW);
    digitalWrite(RM_BIN1,HIGH); 
    digitalWrite(RM_BIN2,LOW); 
}

void reverseToLeft()
{
    stop();
    analogWrite(LM_PWMA,SPD_SUPER_SLOW);
    analogWrite(RM_PWMB,SPD_SLOW);
    digitalWrite(LM_AIN1,HIGH);
    digitalWrite(LM_AIN2,LOW);
    digitalWrite(RM_BIN1,HIGH); 
    digitalWrite(RM_BIN2,LOW); 
}

// turn robot right 
void right()
{
    analogWrite(LM_PWMA,SPD_SLOW);
    analogWrite(RM_PWMB,SPD_SLOW);
    digitalWrite(LM_AIN1,LOW);
    digitalWrite(LM_AIN2,HIGH);
    digitalWrite(RM_BIN1,HIGH); 
    digitalWrite(RM_BIN2,LOW); 
}

// turn robot left 
void left()
{
    analogWrite(LM_PWMA,g_speed);
    analogWrite(RM_PWMB,g_speed);
    digitalWrite(LM_AIN1,HIGH);
    digitalWrite(LM_AIN2,LOW);
    digitalWrite(RM_BIN1,LOW); 
    digitalWrite(RM_BIN2,HIGH);  
}

// just stop robot
void stop()
{
  analogWrite(LM_PWMA,SPD_STOP);
  analogWrite(RM_PWMB,SPD_STOP);
  digitalWrite(LM_AIN1,LOW);
  digitalWrite(LM_AIN2,LOW);
  digitalWrite(RM_BIN1,LOW); 
  digitalWrite(RM_BIN2,LOW);  
}
