#include <AccelStepper.h>
#include <Servo.h>

// #defines are macros for faster than digitalWrite() read & write.
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
const int cardThickness = 141;       // 141 steps to lift a card. 608steps/turn
                                    // (20 turns per 86 cards, 141.3953 ...
const int proxSense2 = 800;         // threshold for OPB606A riding on fan housing
const int liftSenseTop = 500;        // above threshold nothing is close to face of OPB606A
const long int hoverArmPos = 5;      // pick up postion after zeroing against armLimit
const long int firstArmPos = 142;    // experimentally determined drop-off position
const long int secondArmPos = 280;   // 2nd drop and max Arm travel
const byte lowAng = 147;              // servo position: closest approach to card-stack
const byte midAng = 117;              // servo position: move to clear roller
const byte highAng = 15;              // servo postion:
const byte bumpBinSteps = 36;        // for jiggle when grabTryCount goes up.
const byte servoPosition = 50;      // initial safe servo position
int newAnalog1 = 0;                 // hold value from the PROX_PIN sensor 
int newAnalog2 = 0;                 // hold value from the CARD_PIN sensor
int oldAnalog1 = 0;                 // discover changes in the PROX_PIN sensor 
int oldAnalog2 = 0;                 // discover changes in the CARD_PIN sensor
byte grabTryCount = 0;              // bump the bin to help the grabber             
String inputString = "";
boolean LOADED = false;
boolean ARMZEROED = false;
boolean BACKINGOFF = false;
unsigned long timeStart = 0;
unsigned long timeCurrent = 0;

AccelStepper stepperArm(AccelStepper::DRIVER, PINST1_ST, PINST1_DIR);
AccelStepper stepperLift(AccelStepper::DRIVER, PINST2_ST, PINST2_DIR);

const long int liftUpToTop = -200000;     // arbitrary rise until limit switch hit.
const int liftBackOff = 500;              // back off the switch; zero is limit-switch position

Servo myserv;

volatile byte liftState = LOW;
volatile byte armState = LOW;
volatile boolean UPDATE = true;            // flag for printing updated info

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
     
    // glow red when any stepper is enabled: 
    // digitalWrite(REDLED, (!digitalState(holdPinArm)) | (!digitalState(holdPinLift)));  
    
    Serial.begin(115200);
    inputString.reserve(ONRESERVE);
    Serial.println("Setup: liftLimit Switch is " + String(digitalState(LIMIT_SW)) + "\n");
    Serial.println("Setup: armLimit Switch is " + String(digitalState(LIMIT_ARM)) + "\n");
    Serial.println("A0: " + String(analogRead(PROX_PIN)));
    Serial.println("A1: " + String(analogRead(CARD_PIN)));
    stepperLift.moveTo(liftUpToTop);
    stepperArm.moveTo(-1023);
    timeStart = millis();
    timeCurrent = millis();
}

void loop() {
    if (DEBUG) {  
      digitalWrite(GREENLED, digitalState(LIMIT_SW));     // glow when free. in loop 
      digitalWrite(REDLED, digitalState(LIMIT_ARM));      // glow when free. in loop
    }
    
    // only handles limiting if bin is mostly empty
    if ((liftState == HIGH) && (stepperLift.targetPosition() < 0)) {
      delay(5);                                    // de-bounce the switch hopefully
      if (digitalState(LIMIT_SW)) {
        stepperLift.setCurrentPosition(0);
        delay(3);                                 // deceleration 
        stepperLift.moveTo(liftBackOff);
        BACKINGOFF = true;
        timeStart = millis();
        UPDATE = true;
        liftState = LOW;
      }
    }
    else if ((!liftState) && ((timeStart - millis()) > 1900))    // deals with switch hysterysis
      BACKINGOFF = false;
    else if ((liftState == HIGH) && (!BACKINGOFF)) {          // stopping for unforseen reasons
      delay(4);                                               // de-bounce
      if (digitalState(LIMIT_SW)) {
        stepperLift.stop();                                  // stop at maximum deceleration
        UPDATE = true;                                      // report position
        liftState = LOW;
      }
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
    
    fanDown(lowAng, midAng, highAng, firstArmPos);      // auto-servo position based on bin and sensor threshold
    
    // loading by default, so moving down, one card at a time
    if (!LOADED){
      if ((newAnalog1 < liftSenseTop) && (stepperLift.distanceToGo() < cardThickness)) {
        stepperLift.moveTo(stepperLift.currentPosition() + cardThickness);
      }
    }
    // loaded? Then move upwards a little bit
    else if ((newAnalog1 > liftSenseTop) && (stepperLift.distanceToGo() < 1)) {
        stepperLift.moveTo(stepperLift.currentPosition() - cardThickness);
    }
    else if ((grabTryCount % 5) == 2)
        stepperLift.moveTo(stepperLift.currentPosition() - cardThickness);
    else if ((grabTryCount % 5) == 4)
        stepperLift.moveTo(stepperLift.currentPosition() + cardThickness);
        
    // track the analog sensors avoiding flutter values  
    if (abs(oldAnalog1 - newAnalog1) > 40) {
      UPDATE = true;
      oldAnalog1 = newAnalog1;
      //myserv.write(map(newAnalog1, 0, 1023, 40, 135));  // servo won't be slaved here unless testing
    }
    if (abs(oldAnalog2 - newAnalog2) > 40) {
      UPDATE = true;
      oldAnalog2 = newAnalog2;
    }
    
    stepperLift.run();
    stepperArm.run();

    if (UPDATE) {
        Serial.print("Liftpos: " + String(stepperLift.currentPosition()) + " Armpos: " + String(stepperArm.currentPosition()));
        Serial.println("  A0: " + String(newAnalog1) + "  A1: " + String(newAnalog2) + " srvo: " + String(myserv.read()));
        UPDATE = false;
    }
     
    // first character sent switches us into LOADED mode
    // 97="a", 98="b", 99="c", 100="d"                                
    if (Serial.available() > 0) {        
      inputString = String(Serial.read());
      LOADED = true;        
      Serial.println("Rx: " + inputString);
      if (inputString == "97")
        stepperArm.moveTo(hoverArmPos);
      else if (inputString == "98")
        stepperArm.moveTo(firstArmPos);
      else if (inputString == "99")
        stepperArm.moveTo(secondArmPos);
      else if (inputString == "100") {       // Reset into loading mode until another character sent
        stepperArm.moveTo(firstArmPos);
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

// trying for non-blocking servo with delay between actuation attempts...
void fanDown(byte down, byte mid, byte drop, long int dropZone) {
  // go down to get card only if stopped in hover position over bin
  if ((millis() - timeStart) > 800){ 
    if ((newAnalog2 > liftSenseTop) && (stepperArm.currentPosition() == hoverArmPos)) {
      myserv.write(down);
      timeStart = millis();
      grabTryCount += 1;
    }
    else if ((stepperArm.currentPosition() == dropZone) && (newAnalog2 < proxSense2)) {
      myserv.write(drop);
      timeStart = millis();
      grabTryCount = 0;
    }
    else if (myserv.read() != mid) {
      myserv.write(mid);
      timeStart = millis();
    }
  }
}

  
  
