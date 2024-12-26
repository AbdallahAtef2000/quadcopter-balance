#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>


SoftwareSerial bluetooth(0,1);

Servo prop;

volatile uint32_t pulseCounter = 0;
volatile bool Timerflag = false;
volatile bool Timerflag_pid = false;
volatile int counter= 0;
unsigned long filteredRPM = 0;
const int filterSize = 5;
unsigned long rpmBuffer[filterSize] = {0};
int bufferIndex = 0;                   
const int encoderPin = 2;               
const int timerInterval = 200;
char data_in; 


int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY;
 

float Acceleration_angle, Gyro_angle, Total_angle;
float rad_to_deg = 180/3.141592654;
///////////////////////////////////////////////
float PID = 0;
float pwm, error, previous_error;
///////////////////////////////////////////////
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=4;//3.55
double ki=0.0005;//0.003
double kd=2;//2.05
///////////////////////////////////////////////

double throttle = 1200 ; //initial value of throttle to the motors
float desired_angle = 0;


void setup() {
  Serial.begin(250000);
  prop.attach(11); // pin of the motor PWM
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulses, RISING);
  // Timer2 configuration for 200ms intervals
  noInterrupts();
  TCCR2A = 0; // Clear control register A
  TCCR2B = 0; // Clear control register B
  TCNT2 = 0;  // Reset Timer2 counter
  
  // Set compare match value for 200ms
  OCR2A = 249; // (16MHz / (1024 prescaler * 62.5Hz)) - 1
  TCCR2B |= (1 << WGM21); // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 compare interrupt
  interrupts();
  // setup the I2C settings
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  // accelrometer range setting
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  //gyroscope setting
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  bluetooth.begin(9600);

  prop.writeMicroseconds(230);
  delay(10000);

  Serial.println("RawRPM FilteredRPM");
}

void loop() {
  // first setting the desired angle from a simple bluetooth application
  if (bluetooth.available()){
    data_in=bluetooth.read();  //Get next character 

    if(data_in=='D'){
      desired_angle = -10;
    }

    if(data_in=='K'){ //Button Pressed
      desired_angle = 0; 
    }
    
    if(data_in=='M'){ //Button Pressed
      desired_angle = 10; 
    }
  }
  //Serial.println(desired_angle);
  if (Timerflag){
    Timerflag = false; // Reset the flag

    // Calculate RPM
    unsigned long pulses = pulseCounter;
    pulseCounter = 0; // Reset pulse count for next interval
    unsigned long rpm = (pulses * 5 * 60) / 20; // RPM = (pulses/sec * 60) / pulsesPerRev
    
    // Apply moving average filter
    rpmBuffer[bufferIndex] = rpm; // Add new RPM to buffer
    bufferIndex = (bufferIndex + 1) % filterSize; // Update buffer index
    filteredRPM = 0; // Reset filtered RPM
    for (int i = 0; i < filterSize; i++) {
      filteredRPM += rpmBuffer[i];
    }
    filteredRPM /= filterSize; // Calculate average

    // Print the results
    /*Serial.print("Raw RPM: ");
    Serial.print(rpm);
    Serial.print(", Filtered RPM: ");
    Serial.println(filteredRPM);*/

  }
  
  if (Timerflag_pid){
    // get accelerometer data
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true); 

    Acc_rawX=(Wire.read()<<8|Wire.read()); //each value needs two registres
    Acc_rawY=(Wire.read()<<8|Wire.read()); 
    Acc_rawZ=(Wire.read()<<8|Wire.read());
    // pitch angle
    //Acceleration_angle = atan(Acc_rawX/Acc_rawZ)*rad_to_deg; // if rotation is about y axis only
    //Acceleration_angle = atan(Acc_rawX/sqrt(pow(Acc_rawZ,2)+pow(Acc_rawY,2)));
    //roll angle
    //Acceleration_angle = atan(Acc_rawY/Acc_rawZ)*rad_to_deg;
    Acceleration_angle = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    //Serial.println(Acceleration_angle);
    //get gyroscope data
    Wire.beginTransmission(0x68);
    Wire.write(0x43); //Gyro data first adress
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);

    Gyr_rawX=(Wire.read()<<8|Wire.read()); //Once again we shif and sum
    Gyr_rawY=(Wire.read()<<8|Wire.read());
    // roll angle 
    Gyro_angle = Gyr_rawX*0.6/131.0; //where the dt equals 0.6 sec
    //Serial.println(Gyro_angle);
    // pitch angle
    // //Gyro_angle = Gyr_rawY*0.6; // if rotation is about y axis
    // if (Gyr_rawX >= 10){
    Total_angle = 0.8*(Total_angle+ Gyro_angle) + 0.2*Acceleration_angle;
    // }
    // if (Gyr_rawX < 10){
    // Total_angle = 0.05*(Total_angle+ Gyro_angle) + 0.95*Acceleration_angle;  
    // }
    Serial.println(Total_angle);
    // let's start PID 
    error = desired_angle - Total_angle;
    // P term
    pid_p = kp*error;
    // I term
    pid_i = ki*(pid_i+error*0.6);
    // D term
    pid_d = kd*((error - previous_error)/0.6); // where dt equals 0.6 sec
    // PID value
    PID = pid_p + pid_i + pid_d;
    PID = constrain(PID, -1000, 1000); // limit the out of pwm
    previous_error = error;
    pwm = throttle + PID; // the output of pwm in microseconds
    pwm = constrain(pwm, 1215, 1500);
    //Serial.println(pwm);
    prop.writeMicroseconds(pwm);
    Timerflag_pid = false;
  }
  
}

void countPulses() {
  pulseCounter++; // Increment pulse count on emvery rising edge
}

ISR(TIMER2_COMPA_vect) {
  Timerflag = true; // Set the flag every 200ms
  counter++;
  if (counter == 3 ){
    Timerflag_pid = true; //ever 600ms
    counter = 0;
  }
}
