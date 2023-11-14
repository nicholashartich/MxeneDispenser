/*
  Mxene Dispenser
  Author: Nicholas Hartich
  Year: 2023

  This Arduino sketch is designed to control a system comprising three stepper motors
  for creating an evenly spaced spiral pattern on a dish. The system uses the AccelStepper 
  library for smooth and precise control of stepper motors.

  System Components:
  - Three stepper motors controlled by AccelStepper instances:
    1. Arm Stepper: Moves the extruder arm linearly.
    2. Plunger Stepper: Controls the extrusion mechanism.
    3. Belt Stepper: Rotates the dish.

  Functionality:
  - Homing Process: Initially, the arm and plunger steppers move to a 'zero' position using IR sensors.
  - Start Mechanism: The system starts its main operation when a start button is pressed.
  - Spiral Pattern Generation: The arm stepper moves outward in a spiral pattern, controlled by dynamically
    adjusting its speed based on its current radial position.

  Mathematical Logic for Spiral Movement:
  - The speed of the arm stepper is adjusted to maintain a constant spacing between the spiral arms.
  - As the arm moves outward, the distance it covers per revolution increases due to the increasing
    circumference of the spiral.
  - The speed (in steps/sec) of the arm stepper is calculated as follows:
      newArmSpeed = (2 * PI * currentRadius / spiralSpacing) * stepsPerRevolution / 60
    where currentRadius is the distance of the extruder from the center of the dish,
    spiralSpacing is the desired spacing between spiral arms, and stepsPerRevolution is the
    number of steps the stepper motor takes to complete one full revolution.
  - This speed calculation ensures that the extruder arm moves faster as it goes further from the center,
    keeping the spacing between spiral arms consistent.

  AccelStepper Library Usage:
  - AccelStepper provides an interface for controlling stepper motors with acceleration and speed control.
  - The `moveTo` function is used for setting the target position for the motors.
  - The `setSpeed` function dynamically adjusts the speed of the arm stepper for the spiral pattern.
  - `runSpeedToPosition` is called within the main loop to move the stepper towards the target position set by `moveTo`.

  Notes:
  - The system's initialization phase includes moving the arm and plunger steppers to their home positions.
  - The start button triggers the transition from the initialization phase to the operational phase.
  - During operation, the arm stepper's speed is recalculated every 20th loop iteration to optimize performance.
*/



// Include the AccelStepper Library
#include <AccelStepper.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define pin connections
const int dirPinArm = 3;
const int stepPinArm = 4;

const int dirPinPlunger = 5;
const int stepPinPlunger = 2;

const int dirPinBelt = 6;
const int stepPinBelt = 7;

const int irPinArm = 11;
const int irPinPlunger = 12;


const int startButtonPin = 8;

bool booting = true;  // nano was just powered up

// Constants
const float dishRadius = 100.0;      // Radius of the dish in mm
const int stepsPerRevolution = 200;  // Steps per revolution for the stepper motor
const float mmPerStep = 0.01;        // Movement in mm per step for the extruder
const float spiralSpacing = 2.0;     // Desired spacing between spiral arms in mm

// Define motor interface type
// 1 means a stepper driver (with Step and Direction pins)
#define motorInterfaceType 1

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const unsigned char PROGMEM logo_bmp[] = { 0b00000000, 0b11000000,
                                                  0b00000001, 0b11000000,
                                                  0b00000001, 0b11000000,
                                                  0b00000011, 0b11100000,
                                                  0b11110011, 0b11100000,
                                                  0b11111110, 0b11111000,
                                                  0b01111110, 0b11111111,
                                                  0b00110011, 0b10011111,
                                                  0b00011111, 0b11111100,
                                                  0b00001101, 0b01110000,
                                                  0b00011011, 0b10100000,
                                                  0b00111111, 0b11100000,
                                                  0b00111111, 0b11110000,
                                                  0b01111100, 0b11110000,
                                                  0b01110000, 0b01110000,
                                                  0b00000000, 0b00110000 };


// Creates an instance
AccelStepper armStepper(motorInterfaceType, stepPinArm, dirPinArm);

AccelStepper plungerStepper(motorInterfaceType, stepPinPlunger, dirPinPlunger);

AccelStepper beltStepper(motorInterfaceType, stepPinBelt, dirPinBelt);

void setup() {


  // set the maximum speed, acceleration factor,
  // set initial speed and the target position to reverse in the direction of ZERO
  // (until the ir sensor logic backstops us)
  armStepper.setMaxSpeed(1000);  // 60 RPM in steps per second
  armStepper.setSpeed(1000);     // Constant speed
  armStepper.setAcceleration(100000);
  armStepper.moveTo(-16000);     // reverse to a far away target position

  // Plunger stepper setup
  plungerStepper.setMaxSpeed(1000);  // 600 RPM in steps per second
  plungerStepper.setSpeed(1000);     // Constant speed
  plungerStepper.setAcceleration(100000);
  plungerStepper.moveTo(-16000);     // reverse to a far away target position

  // Belt motor setup
  beltStepper.setMaxSpeed(0);  // 120 RPM in steps per second
  beltStepper.setSpeed(0);



  pinMode(irPinArm, INPUT);
  pinMode(irPinPlunger, INPUT);
  pinMode(startButtonPin, INPUT);

  pinMode(dirPinArm, OUTPUT);
  pinMode(stepPinArm, OUTPUT);

  pinMode(dirPinPlunger, OUTPUT);
  pinMode(stepPinPlunger, OUTPUT);

  pinMode(dirPinBelt, OUTPUT);
  pinMode(stepPinBelt, OUTPUT);

  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }



  // Clear the buffer
  display.clearDisplay();

  Serial.println("setup");
   
  displayInitialStats();
}

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

void drawLogo(void) {
  display.clearDisplay();

  display.drawBitmap(
    0,
    0,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(27, 3);
  display.println(F("Motor Control"));
  display.display();  // Show initial text
}

bool stopped = false;

void displayInitialStats(void) {

  
  drawLogo();


  display.setCursor(0, 20);
  display.println(F("Arm: zeroing"));

  display.setCursor(0, 30);
  display.println(F("Plunger: zeroing"));

  display.setCursor(0, 40);
  display.println(F("Belt: stopped"));
  display.display();  // Show initial text

  display.setCursor(0, 55);
  display.println(F("System Status: Boot"));
  display.display();  // Show initial text
}

void displayHomeStats(int armSpeed, int plSpeed, int beltSpeed, String status) {

 
  drawLogo();


  display.setCursor(0, 20);
  display.println(F("Arm: zero"));

  display.setCursor(0, 30);
  display.println(F("Plunger: zero"));

  display.setCursor(0, 40);
  display.println(F("Belt: stopped"));
  display.display();  // Show initial text

  display.setCursor(0, 55);
  display.println(F("System Status: Ready"));
  display.display();  // Show initial text
} 

void displayRunStats() {

 
  drawLogo();


  display.setCursor(0, 20);
  display.println(F("Arm: run"));

  display.setCursor(0, 30);
  display.println(F("Plunger: run"));

  display.setCursor(0, 40);
  display.println(F("Belt: run"));
  display.display();  // Show initial text

  display.setCursor(0, 55);
  display.println(F("System Status: Run"));
  display.display();  // Show initial text
} 


int counterDisplay = 0;
int counterSpeedUpdates = 0;
bool armAtZero = false;
bool plungerAtZero = false;
String systemStatus = "Boot";
bool running = false;

bool zeroStatsDisplayed = false;

bool armStopped = false;
bool plungerStopped = false;

void loop() {


  counterDisplay++;


  // display motor stats every 100th pass to save processing power
  if (counterDisplay >= 100) {
    // display motor stats only when in normal operation,
    // not when booting up and zeroing in
    if (!booting) {
      int armSpeed = armStepper.speed();
      int plungerSpeed = plungerStepper.speed();
      int beltSpeed = beltStepper.speed();

      // System status
      systemStatus = "Running";

        // Call the function to display running stats

        if (!zeroStatsDisplayed)
          displayHomeStats(armSpeed, plungerSpeed, beltSpeed, systemStatus);

        zeroStatsDisplayed= true;
        
    }

    counterDisplay = 0;
  }

  int irArmValue = digitalRead(irPinArm);
  int irPlungerValue = digitalRead(irPinPlunger);

  armAtZero = (irArmValue == HIGH);
  plungerAtZero = (irPlungerValue == HIGH);

  if (armAtZero) {

    if( armStopped == false) {
    // zero found, stop motor
        armStepper.stop();
        armStepper.runToPosition(); 
         Serial.println("stop arm");
         armStopped = true;

    }
   
  }
  if (plungerAtZero) {
    // zero found, stop motor

    if (plungerStopped == false) {
      plungerStepper.stop();
      plungerStepper.runToPosition(); 
      Serial.println("stop plunger");
      plungerStopped = true;
    }
  }

  //set booting to false when both motors have arrived at zero
  // todo: maybe we dont need this variable
  booting = !(armAtZero && plungerAtZero);


  // Serial.println((String)"irA:"+irArmValue);
  //Serial.println((String)"irP:"+irPlungerValue);

  int startButtonValue = digitalRead(startButtonPin);
  //               Serial.println((String)"b:"+buttonValue);



  if (booting) {
    // while booting, run the arm and plunger motors until ir sensors find zero
    if (!armAtZero) {
      //Serial.println(armStepper.speed());
      armStepper.runSpeedToPosition();
    }

    if (!plungerAtZero) {
      //Serial.println(plungerStepper.speed());
      plungerStepper.runSpeedToPosition();
    }
  }

if (running == false) {
  // start running forward when start button is pressed and all arms are at zero
  running = (startButtonValue == HIGH);// && armAtZero && plungerAtZero;



}

  if (running) {

    

    // reprogram motors when leaving zero position
    // do this only once when we transition to running mode
    if (armAtZero && plungerAtZero) {

      // belt and plunger at constant speed
      // arm at acceleration

      Serial.println("starting");
      // this can move forever, no stopping
      beltStepper.setMaxSpeed(200);
      beltStepper.setSpeed(200);  // Constant speed


      plungerStepper.setMaxSpeed(200);
      plungerStepper.setSpeed(200);  // Constant speed
      // travel a total of 10cms
      //   linear travel per step is 0.01 mm, 5cm = 5000 steps
      // this may be totally wrong, because there may be a ratio in the hydraulic system
      plungerStepper.moveTo(5000);

      armStepper.setMaxSpeed(1000);     // Initial max speed
      armStepper.setSpeed(200);
      armStepper.setAcceleration(100);  // Set some acceleration
      // travel 10cm i.e. 10000 steps
      armStepper.moveTo(10000);
    }


    // leaving the zero position ....
    // todo: not sure if this works like that, we may have to turn off ir sensors for a bit so they dont
    //  set armAtZero and plungerAtZero back to true
    armAtZero = false;
    plungerAtZero = false;


    // now move everything forward

    counterSpeedUpdates++;
    
    float currentRadius = armStepper.currentPosition() * mmPerStep;
    /*
    // update display and speed every 1000 passes (roughly every 7 seconds)
    if (counterSpeedUpdates>1000) {

      counterSpeedUpdates = 0;

      displayRunStats();
      // Calculate the current radius based on extruder steps (0.01 mm per atep)
    

      // Adjust the arm speed based on the current radius
      // Speed increases as the radius increases to maintain spiral spacing
      float newArmSpeed = (2 * PI * currentRadius / spiralSpacing) * stepsPerRevolution / 60;
      armStepper.setSpeed(newArmSpeed);

   

    }

       // Move the arm
      if (currentRadius < dishRadius) {
        armStepper.move(stepsPerRevolution);  // Move outward
      }

      */
      armStepper.runSpeedToPosition();
      Serial.println(armStepper.speed());

    // Move the dish
    beltStepper.runSpeed();

    // move the plunger
    plungerStepper.runSpeedToPosition();
  } 
  
}