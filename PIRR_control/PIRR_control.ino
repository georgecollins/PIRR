/*
  PIRR CONTROL
  Pond inspection robot
  
  blink a ligh every two seconds
  
  turn on A4 A5
*/

#include <Esplora.h>

int outa = 3;
int outb = 11;

int walkdir;

#define FORWARD 1
#define STAND 0
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
// when you send a servo signal you switch mode by implication

#define SIG_SRV0_UP 1
#define SIG_SRV0_DN 2
#define SIG_SRV1_UP 3
#define SIG_SRV1_DN 4
#define SIG_SRV2_UP 5
#define SIG_SRV2_DN 6
#define SIG_SRV3_UP 7
#define SIG_SRV3_DN 8
#define SIG_SRV4_UP 9
#define SIG_SRV4_DN 10
#define SIG_SRV5_UP 11
#define SIG_SRV5_DN 12
#define SIG_SRV6_UP 13
#define SIG_SRV6_DN 14
#define SIG_SRV7_UP 15
#define SIG_SRV7_DN 16

#define SIG_FWD 128
#define SIG_BWD 130
#define SIG_RT 131
#define SIG_LT 132


void setup() {
  pinMode(outa, OUTPUT);
  pinMode(outb, OUTPUT);
  Serial.begin(28800);
  
}
// send a byte
void signal(int sval) {
  digitalWrite(outa, HIGH);
  digitalWrite(outb, LOW);   

  delay(1);
  int div =128;
  for (int loop=0; loop < 8; loop++) {
    int s = sval/div;
    if (s > 0) {
      sval-=div;
      digitalWrite(outa, HIGH);
    }
    else digitalWrite(outa, LOW);
   digitalWrite(outb, HIGH);
 
   delay(1);
    
   div=div/2;
   digitalWrite(outa, LOW);   
   digitalWrite(outb, LOW);
   delay(1);
  }
  // here is my alternate 
  // outa high
  // ouut b low
}



void OutputWalkdir(int xValue, int yValue) {
 
  if (yValue < - 400) { 
    walkdir = FORWARD;
  } else
  {
    if (yValue > 400) {
      walkdir = BACKWARD;
    }
    else
      if (xValue > 400) {
        walkdir = LEFT;
      }
      else {
        if (xValue < - 400) {
           walkdir = RIGHT;
        } else
            walkdir = STAND;
      }
  }  
  
  // y - 400 forward
  // y + 400 bacwar
  // x + 500 = left
  // x - 500 = right 
  switch (walkdir) {
    case STAND:
      Esplora.writeRGB(255,255,255); 
      signal(128);  
      break;
    case FORWARD:
      Esplora.writeRGB(0,255,0);
      signal(129);
      break;
    case BACKWARD:
      Esplora.writeRGB(255,0,0);
      signal(130);
      break;
    case LEFT:
      Esplora.writeRGB(128, 128, 0);
      signal(131);
      break;
    case RIGHT:
      Esplora.writeRGB(0, 128, 128);
      signal(132);
      break;  
  }
  


}


void loop() {
  
  int xValue = Esplora.readJoystickX();        // read the joystick's X position
  int yValue = Esplora.readJoystickY();        // read the joystick's Y position
/*
  Serial.print(" X :");
  Serial.print(xValue);
  Serial.print(" Y :");
  Serial.println(yValue);
  delay(200);
  */
  
  int upbutton = Esplora.readButton(SWITCH_UP);
  
  OutputWalkdir(xValue, yValue);
  return;
 

}
