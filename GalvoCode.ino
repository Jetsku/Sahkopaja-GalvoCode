#include "ADC.h"

// Teensy 3.2 pin definitions
#define STEP_PIN 23
#define DIR_PIN 5
#define SENSOR_PIN A8
#define IN1 4
#define IN2 3

volatile bool direction = false;

// initial angle when the program starts
int desiredAngle = 5650;
int currentAngle;

//pid variables
float error, previousError = 0, integral = 0, derivative;
double Kp=0.4, Ki=0.000, Kd=0.8;
double output;

// Define the ADC object for changing the ADC parameters
ADC *adc = new ADC();

/// @brief Function to increment or decrement the desired angle based on the direction signal
void stepPulse(){
  if(direction == true){
      desiredAngle = desiredAngle +1;
  }else{
    desiredAngle = desiredAngle -1;
  }
}

void setup() {
    //Setup ADC parameters
    analogWriteResolution(8);
    analogReadResolution(16);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
    adc->adc0->setAveraging(32);

    // Initialize the serial communication
    Serial.begin(9600);

    // Setup the pins for reading the step and direction signals
    pinMode(STEP_PIN, INPUT);
    pinMode(DIR_PIN, INPUT);

    // Attach the interrupt to the step signal for detecting coming STEP signals and begin reading the direction signal
    attachInterrupt(digitalPinToInterrupt(STEP_PIN), stepPulse, RISING);
    direction = digitalRead(DIR_PIN);
}

void loop() {
    // Read the direction pin in order to determine STEP signal direction
    direction = digitalRead(DIR_PIN);

    // Read the current angle from the sensor and bit shift it to the right by 3 so that 13 bits are left
    currentAngle = analogRead(SENSOR_PIN)>>3;

    // Update the PID controller
    error = desiredAngle - currentAngle;
    integral = integral + error;
    derivative = error - previousError;
    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, -4095, 4095);

    // Output the signal to the motor based on the output of the PID controller
    if(output>0){
        tone(IN2,15000);
        delayMicroseconds((int)(abs(output/250)*1000));
        noTone(IN2);
    }else{
        tone(IN1,15000);
        delayMicroseconds((int)(abs(output/250)*1000));
        noTone(IN1);
    }
    previousError = error;

    //Uncomment to debug
    //Serial.println(currentAngle);
    //Serial.println(output);
}