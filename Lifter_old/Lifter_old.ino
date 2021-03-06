#include <AccelStepper.h>
#include <Servo.h>

// #defines are macros for faster pin access than built-in digitalWrite().
//  *http://masteringarduino.blogspot.com/2013/10/fastest-and-smallest-digitalread-and.html

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

const boolean DEBUG = true;        // send stuff to serial monitor for testing

const byte ledPin = 13;            // on board LED
const byte holdPinArm = 12;        // enable pins for stepper drivers
const byte holdPinLift = 11;       // HIGH->disable, LOW->enables
const byte SERV1PIN = 10;          // servo control pin
const byte PINST1_ST = 6;          
const byte PINST1_DIR = 7;
const byte PINST2_ST = 4;
const byte PINST2_DIR = 5;
const byte REDLED = 9;
const byte GREENLED = 8;
const byte LIMIT_SW = 2;            // should be a hardware interrupt pin
const byte LIMIT_ARM = 3;     
const byte ONRESERVE = 32;          // max byte length of serial-delivered string
const byte PROX_PIN = A0;           // Analog prox. sensor Pin for card-stack-top
const byte CARD_PIN = A1;           // Analog prox. for 'card on-board'
const int cardThickness = 52;       // calculated 141 steps to lift a card. 608steps/turn
                                     // (20 turns per 86 cards, 141.3953 ...
const int proxSense2 = 750;         // threshold for OPB606A riding on fan housing
const int liftSenseTop = 700;        // above threshold nothing is close to face of OPB606A
const long int hoverArmPos = 18;      // pick up postion after zeroing against armLimit
const long int firstArmPos = 153;    // experimentally determined drop-off position
const long int secondArmPos = 300;   // 2nd drop and max Arm travel
const byte closeAng = 117;              // servo position: closest approach to card-stack
const byte openAng = 165;              // servo postion:
const byte bumpBinSteps = 36;        // for jiggle when grabTryCo17unt goes up.
const byte servoPosition = 137;      // initial setup() safe servo position & middle return
const byte extraServoBump = 16;      // travel either way a little further then settle back to low or high
int newAnalog1 = 0;                 // hold value from the PROX_PIN sensor 
int newAnalog2 = 0;                 // hold value from the CARD_PIN sensor
int oldAnalog1 = 0;                 // discover changes in the PROX_PIN sensor 
int oldAnalog2 = 0;                 // discover changes in the CARD_PIN sensor
long int destinationArmPos = firstArmPos;  // arm drop-off point (first or second)
byte grabTryCount = 0;              // bump the bin to help the grabber
byte oldServoPos = servoPosition;    // for tracking changes is servo. Are they continuous?
String inputString = "";
boolean LOADED = false;
boolean ARMZEROED = false;
boolean BACKINGOFF = false;
byte restServoPos = servoPosition;    // used for moving servo back off its maximum travel on either end; set in fanDown()
boolean stressedServo = false;       // flag for above
unsigned long timeStart = 0;
unsigned long timeCurrent = 0;
boolean UPDATE = true;                  // flag for printing updated info to serial monitor
unsigned long currentTime = millis();  // I can't seem to declare local variables
byte currentServoPos = 0;              

AccelStepper stepperArm(AccelStepper::DRIVER, PINST1_ST, PINST1_DIR);
AccelStepper stepperLift(AccelStepper::DRIVER, PINST2_ST, PINST2_DIR);

const long int liftUpToTop = -200000;     // arbitrary rise until limit switch hit.
long int liftBackOff = 500;              // back off the switch; zero is limit-switch position

Servo myserv;

volatile byte liftState = LOW;
volatile byte armState = LOW;


void setup()
{  
    myserv.attach(SERV1PIN); 
    myserv.write(servoPosition);
    
    stepperLift.setMaxSpeed(1300.0);
    stepperLift.setAcceleration(900.0);
    stepperArm.setMaxSpeed(1400.0);
    stepperArm.setAcceleration(700.0);
    
    pinAsOutput(holdPinArm);
    pinAsOutput(holdPinLift); 
    pinAsOutput(REDLED);
    pinAsOutput(GREENLED);
    pinAsInput(LIMIT_SW);
    pinAsInput(PROX_PIN);
    
    attachInterrupt(0, limitLift, CHANGE);          // int.0 on pin 2
    attachInterrupt(1, limitArm, CHANGE);          // int.1 on pin 3
    
    digitalLow(holdPinArm);                 // HIGH disables driver, LOW enables
    digitalLow(holdPinLift);                // HIGH disables driver, LOW enables
     
    Serial.begin(115200);
    inputString.reserve(ONRESERVE);
    Serial.println("Setup: liftLimit Switch is " + String(digitalState(LIMIT_SW)) + "\n");
    Serial.println("Setup: armLimit Switch is " + String(digitalState(LIMIT_ARM)) + "\n");
    Serial.println("A0: " + String(analogRead(PROX_PIN)));
    Serial.println("A1: " + String(analogRead(CARD_PIN)));
    stepperLift.moveTo(liftUpToTop);
    stepperArm.moveTo(-10234);    // should get Arm to limit-switch under any starting circumstance
    timeStart = millis();
    timeCurrent = millis();
}

void loop() {
    /*
    if (DEBUG) {  
      digitalWrite(GREENLED, digitalState(LIMIT_SW));     // glow when free. in loop 
      digitalWrite(REDLED, digitalState(LIMIT_ARM));      // glow when free. in loop
    }
    */
    
    // only handles limiting if bin is mostly empty
    if ((liftState == HIGH) && !BACKINGOFF) {
      delay(3);                             // de-bounce the switch hopefully
      if (digitalState(LIMIT_SW)) {
        stepperLift.setCurrentPosition(0);
        stepperLift.moveTo(liftBackOff);
        BACKINGOFF = true;
        UPDATE = true;
        liftState = LOW;
      }
    }
    else if (BACKINGOFF && !liftState) {   // deals with limit-switch hysterysis
      BACKINGOFF = false;
      liftBackOff = stepperLift.currentPosition();
      if (liftBackOff < 1)
        Serial.println("WARNING  WARNING  error with liftBackOff");
      stepperLift.setCurrentPosition(0);
    }
      
    
    // arm starts off moving towards limit switch in setup(), hits it, moves to ready-to-load         
    if ((armState == HIGH) && (stepperArm.targetPosition() < 0)){
      delay(4);                                  // de-bounce the switch hopefully
      if (digitalState(LIMIT_ARM)) {
        stepperArm.setCurrentPosition(0);
        delay(3);                             // deceleration allowance
        stepperArm.moveTo(firstArmPos);
        UPDATE = true;
        armState = LOW;
      }
    }
    
    newAnalog1 = analogRead(PROX_PIN);
    newAnalog2 = analogRead(CARD_PIN);
    
    // auto-servo position based on bin and sensor threshold
    fanDown(closeAng, openAng, destinationArmPos);      
    
    // loading by default, so moving down, one card at a time
    // newAnalog1 < senseTop means 'covered'  newAnalog1 >... is 'uncovered'
    if (!LOADED){
      if ((newAnalog1 < liftSenseTop) && (stepperLift.distanceToGo() < cardThickness)) {
        stepperLift.moveTo(stepperLift.currentPosition() + cardThickness);
      }
    }
    // loaded? Then move upwards a little bit
    else if ((newAnalog1 > liftSenseTop) && (stepperLift.distanceToGo() < 1))
        stepperLift.moveTo(stepperLift.currentPosition() - cardThickness);
 
    // TRACKING via SERIAL MONITOR without flooding it
    // analog sensors avoiding flutter values
   // comment out once everything is working perfectly 
    if (abs(oldAnalog1 - newAnalog1) > 30) {
      UPDATE = true;
      oldAnalog1 = newAnalog1;
    }
    else if (abs(oldAnalog2 - newAnalog2) > 30) {
      UPDATE = true;
      oldAnalog2 = newAnalog2;
    }
        
    if (abs(myserv.read() - oldServoPos) > 1) {
      UPDATE = true;
      oldServoPos = myserv.read();
    }
        
    if (UPDATE) {
        Serial.print("Liftpos: " + String(stepperLift.currentPosition()) + " Armpos: " + String(stepperArm.currentPosition()));
        Serial.println("  A0: " + String(newAnalog1) + "  A1: " + String(newAnalog2) + " srvo: " + String(oldServoPos) + " tm:" + String(millis()));
        UPDATE = false;
    }
    
    if (!digitalState(LIMIT_SW) || BACKINGOFF)
      stepperLift.run();
    stepperArm.run(); 
     
    // first character sent switches us into LOADED mode
    // 97="a", 98="b", 99="c", 100="d"                                
    if (Serial.available() > 0) {        
      inputString = String(Serial.read());
      if (!LOADED) {
        LOADED = true;
        stepperLift.setCurrentPosition(0);
        UPDATE = true;
      }        
      Serial.println("Rx: " + inputString);
      if (inputString == "97")               // 'a' hover over input stack
        stepperArm.moveTo(hoverArmPos);
      else if (inputString == "98") {        // 'b' ID'd pile
        stepperArm.moveTo(firstArmPos);
        destinationArmPos = firstArmPos;
      }
      else if (inputString == "99") {        // 'c' unidentified pile
        stepperArm.moveTo(secondArmPos);
        destinationArmPos = secondArmPos;
      }
      else if (inputString == "100") {       // 'd' Reset into loading mode until another character sent
        stepperArm.moveTo(firstArmPos);
        destinationArmPos = firstArmPos;
        LOADED = false;
      }
      inputString = "";
    }    
} //end void-loop

// interrupt function on pin 2, int.0
void limitLift() {  
  liftState = digitalState(LIMIT_SW);
}

// interrupt function on pin 3, int.1
void limitArm() {
  armState = digitalState(LIMIT_ARM);
}

// trying for non-blocking non-repetitive servo with delay between actuation attempts...
void fanDown(byte suck, byte drop, long int dropZone) {
  // go down to get card only if stopped in hover position over bin
  currentTime = millis();
  currentServoPos = myserv.read();
  
  // after allowing time for travel, check stress levels...
  if (((timeStart - currentTime) > 900)) {
    if (currentServoPos == (drop + extraServoBump)) {
      stressedServo = true;
      restServoPos = drop;
    }
    else if (currentServoPos == (suck - extraServoBump)) {
      restServoPos = suck;
      stressedServo = true;
    }
    else if ((currentServoPos == suck) || (currentServoPos == drop))
      stressedServo = false;
  } 
  
  if ((stepperArm.currentPosition() == hoverArmPos) && 
      (newAnalog2 > proxSense2) && 
      !stressedServo) {
        myserv.write(drop + extraServoBump);
        timeStart = currentTime;
    }
  else if ((stepperArm.currentPosition() == dropZone) && 
            (newAnalog2 < proxSense2) &&
            !stressedServo) {
              myserv.write(suck - extraServoBump);
              timeStart = currentTime;
   }
  else if (stressedServo && ((timeStart - currentTime) > 900)) {
    myserv.write(restServoPos);
    if (DEBUG)
      Serial.println("de-stressed to position: " + String(restServoPos));
  }
}

  
  
