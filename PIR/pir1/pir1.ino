/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define MAX_INC 5  // number of steps in moving a servo

#define NUM_SERVOS 8
#define MAX_STEPS 12

#define RFV_SERVO 0
#define RFH_SERVO 3
#define RBV_SERVO 5
#define RBH_SERVO 2

#define LFV_SERVO 7
#define LFH_SERVO 6
#define LBV_SERVO 4
#define LBH_SERVO 1

#define RFV_OFFSET 0
#define RFH_OFFSET 0
#define RBV_OFFSET 0
#define RBH_OFFSET 0

#define LFV_OFFSET 0
#define LFH_OFFSET 0
#define LBV_OFFSET -20
#define LBH_OFFSET 0


#define RFH_FWD  (550 - 50)
#define RFH_BWD  (375 + 50)
#define LFH_FWD  (360 + 50)
#define LFH_BWD  (510 - 50)

#define RBH_FWD  (600 - 50)
#define RBH_BWD  (300 + 50) 
#define LBH_FWD  (300 + 50)
#define LBH_BWD  (600 - 50)

#define RFH_MID  460 
#define LFH_MID  435
#define RBH_MID  500 // 450 
#define LBH_MID  400 // 450

#define RFH0 (RFH_MID - 30)
#define RFH1 (RFH_MID - 20)
#define RFH2 (RFH_MID - 10)
#define RFH3 RFH_MID
#define RFH4 (RFH_MID + 10)
#define RFH5 (RFH_MID + 20)
#define RFH6 (RFH_MID + 30)

#define RBH0 (RBH_MID - 30)
#define RBH1 (RBH_MID - 20)
#define RBH2 (RBH_MID - 10)
#define RBH3  RBH_MID
#define RBH4 (RBH_MID + 10)
#define RBH5 (RBH_MID + 20)
#define RBH6 (RBH_MID + 30)


#define LFH0 (LFH_MID + 30)
#define LFH1 (LFH_MID + 20)
#define LFH2 (LFH_MID + 10)
#define LFH3  LFH_MID
#define LFH4 (LFH_MID - 10)
#define LFH5 (LFH_MID - 20)
#define LFH6 (LFH_MID - 30)

#define LBH0 (LBH_MID + 30)
#define LBH1 (LBH_MID + 20)
#define LBH2 (LBH_MID + 10)
#define LBH3  LBH_MID
#define LBH4 (LBH_MID - 10)
#define LBH5 (LBH_MID - 20)
#define LBH6 (LBH_MID - 30)


#define RFV_UP 450
#define RFV_DN  600
#define RBV_UP  450
#define RBV_DN  600

#define LFV_UP  450
#define LFV_DN  310 //450
#define LBV_UP  420     //270
#define LBV_DN  270    // 420

//our servo # counter
uint8_t servonum = 0;


// D3 
// D11

int ina = A0;
int inb = A1;


int standpose[NUM_SERVOS];

typedef struct pose {
  int time;
  int servopos[NUM_SERVOS]; //[MAX_STEPS][NUM_SERVOS];

} Pose;

Pose walkdat[6];
Pose triwalk[8];

int walkphase;  // 0 - 6



void fillwalkdat() {

 
  walkdat[0].servopos[RFV_SERVO] = RFV_DN; // {0, 0, 0, 0, 0, 0, 0, 0};// RFV_DN;//[RFV_SERVO] = RFV_DN;
  walkdat[0].servopos[RFH_SERVO] = RFH_FWD;
  walkdat[0].servopos[LFV_SERVO] = LFV_DN;
  walkdat[0].servopos[LFH_SERVO] = LFH_BWD;
  walkdat[0].servopos[RBV_SERVO] = RBV_DN;
  walkdat[0].servopos[RBH_SERVO] = RBH_BWD;
  walkdat[0].servopos[LBV_SERVO] = LBV_DN;
  walkdat[0].servopos[LBH_SERVO] = LBH_FWD;
  walkdat[0].time = 2; // how many times *20 you do the change
  
  walkdat[1].servopos[RFV_SERVO] = RFV_DN; 
  walkdat[1].servopos[RFH_SERVO] = RFH_FWD;
  walkdat[1].servopos[LFV_SERVO] = LFV_UP;
  walkdat[1].servopos[LFH_SERVO] = LFH_BWD;
  walkdat[1].servopos[RBV_SERVO] = RBV_UP;
  walkdat[1].servopos[RBH_SERVO] = RBH_BWD;
  walkdat[1].servopos[LBV_SERVO] = LBV_DN;
  walkdat[1].servopos[LBH_SERVO] = LBH_FWD;
  walkdat[1].time = 2; // how many times *20 you do the change

  // move
  walkdat[2].servopos[RFV_SERVO] = RFV_DN; 
  walkdat[2].servopos[RFH_SERVO] = RFH_BWD;
  walkdat[2].servopos[LFV_SERVO] = LFV_UP;
  walkdat[2].servopos[LFH_SERVO] = LFH_FWD;
  walkdat[2].servopos[RBV_SERVO] = RBV_UP;
  walkdat[2].servopos[RBH_SERVO] = RBH_FWD;
  walkdat[2].servopos[LBV_SERVO] = LBV_DN;
  walkdat[2].servopos[LBH_SERVO] = LBH_BWD;
  walkdat[2].time = 10; // how many times *20 you do the change
  
  // all dn
  walkdat[3].servopos[RFV_SERVO] = RFV_DN; 
  walkdat[3].servopos[RFH_SERVO] = RFH_BWD;
  walkdat[3].servopos[LFV_SERVO] = LFV_DN;
  walkdat[3].servopos[LFH_SERVO] = LFH_FWD;
  walkdat[3].servopos[RBV_SERVO] = RBV_DN;
  walkdat[3].servopos[RBH_SERVO] = RBH_FWD;
  walkdat[3].servopos[LBV_SERVO] = LBV_DN;
  walkdat[3].servopos[LBH_SERVO] = LBH_BWD;
  walkdat[3].time = 2; // how many times *20 you do the change
  
// new up
  walkdat[4].servopos[RFV_SERVO] = RFV_UP; 
  walkdat[4].servopos[RFH_SERVO] = RFH_BWD;
  walkdat[4].servopos[LFV_SERVO] = LFV_DN;
  walkdat[4].servopos[LFH_SERVO] = LFH_FWD;
  walkdat[4].servopos[RBV_SERVO] = RBV_DN;
  walkdat[4].servopos[RBH_SERVO] = RBH_FWD;
  walkdat[4].servopos[LBV_SERVO] = LBV_UP;
  walkdat[4].servopos[LBH_SERVO] = LBH_BWD;
  walkdat[4].time = 2; // how many times *20 you do the change

  // move again
  walkdat[5].servopos[RFV_SERVO] = RFV_UP; 
  walkdat[5].servopos[RFH_SERVO] = RFH_FWD;
  walkdat[5].servopos[LFV_SERVO] = LFV_DN;
  walkdat[5].servopos[LFH_SERVO] = LFH_BWD;
  walkdat[5].servopos[RBV_SERVO] = RBV_DN;
  walkdat[5].servopos[RBH_SERVO] = RBH_BWD;
  walkdat[5].servopos[LBV_SERVO] = LBV_UP;
  walkdat[5].servopos[LBH_SERVO] = LBH_FWD;
  walkdat[5].time = 10; // how many times *20 you do the change

/*
  ******** TRI RIGHT *******
*/


  triright[7].servopos[RFV_SERVO] = RFV_DN; 
  triright[7].servopos[RFH_SERVO] = RFH1;
  triright[7].servopos[LFV_SERVO] = LFV_DN;
  triright[7].servopos[LFH_SERVO] = LFH_MID;
  triright[7].servopos[RBV_SERVO] = RBV_DN;
  triright[7].servopos[RBH_SERVO] = RBH5;
  triright[7].servopos[LBV_SERVO] = LBV_DN;
  triright[7].servopos[LBH_SERVO] = LBH_MID;
  triright[7].time = 1; // how many times *20 you do the change

  triright[6].servopos[RFV_SERVO] = RFV_DN; 
  triright[6].servopos[RFH_SERVO] = RFH2;
  triright[6].servopos[LFV_SERVO] = LFV_DN;
  triright[6].servopos[LFH_SERVO] = LFH_MID;
  triright[6].servopos[RBV_SERVO] = RBV_DN;
  triright[6].servopos[RBH_SERVO] = RBH6;
  triright[6].servopos[LBV_SERVO] = LBV_DN;
  triright[6].servopos[LBH_SERVO] = LBH_MID;
  triright[6].time = 1; // how many times *20 you do the change

  triright[5].servopos[RFV_SERVO] = RFV_DN; 
  triright[5].servopos[RFH_SERVO] = RFH3;
  triright[5].servopos[LFV_SERVO] = LFV_DN;
  triright[5].servopos[LFH_SERVO] = LFH_MID;
  triright[5].servopos[RBV_SERVO] = RBV_UP;
  triright[5].servopos[RBH_SERVO] = RBH3;
  triright[5].servopos[LBV_SERVO] = LBV_DN;
  triright[5].servopos[LBH_SERVO] = LBH_MID;
  triright[5].time = 1; // how many times *20 you do the change

  triright[4].servopos[RFV_SERVO] = RFV_DN; 
  triright[4].servopos[RFH_SERVO] = RFH4;
  triright[4].servopos[LFV_SERVO] = LFV_UP;
  triright[4].servopos[LFH_SERVO] = LFH_MID;
  triright[4].servopos[RBV_SERVO] = RBV_DN;
  triright[4].servopos[RBH_SERVO] = RBH0;
  triright[4].servopos[LBV_SERVO] = LBV_DN;
  triright[4].servopos[LBH_SERVO] = LBH_MID;
  triright[4].time = 1; // how many times *20 you do the change
  
  triright[3].servopos[RFV_SERVO] = RFV_DN; 
  triright[3].servopos[RFH_SERVO] = RFH5;
  triright[3].servopos[LFV_SERVO] = LFV_DN;
  triright[3].servopos[LFH_SERVO] = LFH_MID;
  triright[3].servopos[RBV_SERVO] = RBV_DN;
  triright[3].servopos[RBH_SERVO] = RBH1;
  triright[3].servopos[LBV_SERVO] = LBV_DN;
  triright[3].servopos[LBH_SERVO] = LBH_MID;
  triright[3].time = 1; // how many times *20 you do the change

  triright[2].servopos[RFV_SERVO] = RFV_DN; 
  triright[2].servopos[RFH_SERVO] = RFH6;
  triright[2].servopos[LFV_SERVO] = LFV_DN;
  triright[2].servopos[LFH_SERVO] = LFH_MID;
  triright[2].servopos[RBV_SERVO] = RBV_DN;
  triright[2].servopos[RBH_SERVO] = RBH2;
  triright[2].servopos[LBV_SERVO] = LBV_UP;
  triright[2].servopos[LBH_SERVO] = LBH_MID;
  triright[2].time = 1; // how many times *20 you do the change

  triright[1].servopos[RFV_SERVO] = RFV_UP; 
  triright[1].servopos[RFH_SERVO] = RFH3;
  triright[1].servopos[LFV_SERVO] = LFV_DN;
  triright[1].servopos[LFH_SERVO] = LFH_MID;
  triright[1].servopos[RBV_SERVO] = RBV_DN;
  triright[1].servopos[RBH_SERVO] = RBH3;
  triright[1].servopos[LBV_SERVO] = LBV_DN;
  triright[1].servopos[LBH_SERVO] = LBH_MID;
  triright[1].time = 1; // how many times *20 you do the change

  triright[0].servopos[RFV_SERVO] = RFV_DN; 
  triright[0].servopos[RFH_SERVO] = RFH0;
  triright[0].servopos[LFV_SERVO] = LFV_DN;
  triright[0].servopos[LFH_SERVO] = LFH_MID;
  triright[0].servopos[RBV_SERVO] = RBV_DN;
  triright[0].servopos[RBH_SERVO] = RBH4;
  triright[0].servopos[LBV_SERVO] = LBV_DN;
  triright[0].servopos[LBH_SERVO] = LBH_MIO;
  triright[0].time = 1; // how many times *20 you do the change


/*
  ********** TRI WALK **********
*/

  triwalk[7].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[7].servopos[RFH_SERVO] = RFH1;
  triwalk[7].servopos[LFV_SERVO] = LFV_DN;
  triwalk[7].servopos[LFH_SERVO] = LFH4;
  triwalk[7].servopos[RBV_SERVO] = RBV_DN;
  triwalk[7].servopos[RBH_SERVO] = RBH5;
  triwalk[7].servopos[LBV_SERVO] = LBV_DN;
  triwalk[7].servopos[LBH_SERVO] = LBH2;
  triwalk[7].time = 1; // how many times *20 you do the change

  triwalk[6].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[6].servopos[RFH_SERVO] = RFH2;
  triwalk[6].servopos[LFV_SERVO] = LFV_DN;
  triwalk[6].servopos[LFH_SERVO] = LFH5;
  triwalk[6].servopos[RBV_SERVO] = RBV_DN;
  triwalk[6].servopos[RBH_SERVO] = RBH6;
  triwalk[6].servopos[LBV_SERVO] = LBV_DN;
  triwalk[6].servopos[LBH_SERVO] = LBH3;
  triwalk[6].time = 1; // how many times *20 you do the change

  triwalk[5].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[5].servopos[RFH_SERVO] = RFH3;
  triwalk[5].servopos[LFV_SERVO] = LFV_DN;
  triwalk[5].servopos[LFH_SERVO] = LFH6;
  triwalk[5].servopos[RBV_SERVO] = RBV_UP;
  triwalk[5].servopos[RBH_SERVO] = RBH3;
  triwalk[5].servopos[LBV_SERVO] = LBV_DN;
  triwalk[5].servopos[LBH_SERVO] = LBH4;
  triwalk[5].time = 1; // how many times *20 you do the change

  triwalk[4].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[4].servopos[RFH_SERVO] = RFH4;
  triwalk[4].servopos[LFV_SERVO] = LFV_UP;
  triwalk[4].servopos[LFH_SERVO] = LFH3;
  triwalk[4].servopos[RBV_SERVO] = RBV_DN;
  triwalk[4].servopos[RBH_SERVO] = RBH0;
  triwalk[4].servopos[LBV_SERVO] = LBV_DN;
  triwalk[4].servopos[LBH_SERVO] = LBH5;
  triwalk[4].time = 1; // how many times *20 you do the change
  
  triwalk[3].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[3].servopos[RFH_SERVO] = RFH5;
  triwalk[3].servopos[LFV_SERVO] = LFV_DN;
  triwalk[3].servopos[LFH_SERVO] = LFH0;
  triwalk[3].servopos[RBV_SERVO] = RBV_DN;
  triwalk[3].servopos[RBH_SERVO] = RBH1;
  triwalk[3].servopos[LBV_SERVO] = LBV_DN;
  triwalk[3].servopos[LBH_SERVO] = LBH6;
  triwalk[3].time = 1; // how many times *20 you do the change

  triwalk[2].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[2].servopos[RFH_SERVO] = RFH6;
  triwalk[2].servopos[LFV_SERVO] = LFV_DN;
  triwalk[2].servopos[LFH_SERVO] = LFH1;
  triwalk[2].servopos[RBV_SERVO] = RBV_DN;
  triwalk[2].servopos[RBH_SERVO] = RBH2;
  triwalk[2].servopos[LBV_SERVO] = LBV_UP;
  triwalk[2].servopos[LBH_SERVO] = LBH3;
  triwalk[2].time = 1; // how many times *20 you do the change

  triwalk[1].servopos[RFV_SERVO] = RFV_UP; 
  triwalk[1].servopos[RFH_SERVO] = RFH3;
  triwalk[1].servopos[LFV_SERVO] = LFV_DN;
  triwalk[1].servopos[LFH_SERVO] = LFH2;
  triwalk[1].servopos[RBV_SERVO] = RBV_DN;
  triwalk[1].servopos[RBH_SERVO] = RBH3;
  triwalk[1].servopos[LBV_SERVO] = LBV_DN;
  triwalk[1].servopos[LBH_SERVO] = LBH0;
  triwalk[1].time = 1; // how many times *20 you do the change

  triwalk[0].servopos[RFV_SERVO] = RFV_DN; 
  triwalk[0].servopos[RFH_SERVO] = RFH0;
  triwalk[0].servopos[LFV_SERVO] = LFV_DN;
  triwalk[0].servopos[LFH_SERVO] = LFH3;
  triwalk[0].servopos[RBV_SERVO] = RBV_DN;
  triwalk[0].servopos[RBH_SERVO] = RBH4;
  triwalk[0].servopos[LBV_SERVO] = LBV_DN;
  triwalk[0].servopos[LBH_SERVO] = LBH1;
  triwalk[0].time = 1; // how many times *20 you do the change

}


/*************************

C O M M U N I C A T I O N

*************************/

int listen() {
  int cura, curb, olda, oldb;
  int sum;
  int b;
  int div=128;
  cura=digitalRead(ina);
  curb=digitalRead(inb);
  olda=cura;
  oldb=curb;
  do {
      cura=digitalRead(ina);
      curb=digitalRead(inb);

      if ((cura!=olda) || (curb!=oldb)) {
        /*
        Serial.print("IN A:" );
        Serial.print(olda);
        Serial.print(" IN B:");
        Serial.println(oldb);
        */
        olda=cura;
        oldb=curb; 
             
        if ((cura==0) and (curb==1)) {
          if (div == 0) {
            // we got the right number
           // Serial.print("sum ");
           // Serial.println(sum);
           return sum;
            // could be return sum
          }
          sum = 0;
          div = 128;
        }
        if (cura==1) {
          sum = sum + div*curb;
          div= div/2;
        } 
      
      }
    delay(1);
      
  } while(true);
  
  
}


void setup() {
  Serial.begin(28800);
  Serial.println("8 channel Servo test!");
  Serial.println("I will try this");


  
  pwm.begin();
  Serial.println("PWM begin");
 
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.println("Freq set to 60");

  fillwalkdat();
  walkphase=0;
  Serial.println("loaded walk data");
  
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


// 0 = rfv
// 1 = lbh

void stand() {
  // rfv up = 150, down -600
  pwm.setPWM(0, 0, 600);
  // lbh
  pwm.setPWM(1, 0, 300); // 425   600 = way back, 300 = forward
  // rbh
  pwm.setPWM(2, 0, 600); /// 450 300 = back 600 = forward
  // rfh fwd = 550, mid = 375
  pwm.setPWM(3, 0, 550);
  // lbv 200= down
  pwm.setPWM(4, 0, 270);
  // rbv 600 = down 300 = up 
  pwm.setPWM(5, 0, 600); 
  // 5100 = straight 
  pwm.setPWM(6, 0, 360); // lfh 510 
 // lfv 600 up,  300 = down
  pwm.setPWM(7,0, 300);  
}



void SwitchPose(int *p1, int *p2) {
  ((Pose *)p1)->time= 7;
  
  for (int n=0; n < NUM_SERVOS; n++) {
     pwm.setPWM(n, 0, ((Pose *)p1)->servopos[n]); // lfh 510 
/*
     pwm.setPWM(n, 0, walkdat[0].servopos[n]); // lfh 510 
    Serial.print("Servo #");
    Serial.print(n);
    Serial.print(" value ");
    Serial.println(((Pose *)p1)->servopos[n]);
*/
  }

}

int SwitchPose2(int *p1, int *p2, int n) {
/*
 n  = number from 0 to 19
 return the new n
*/

  
  for (int i=0; i < NUM_SERVOS; i++) {
     int val = (((Pose *)p2)->servopos[i]*n + ((Pose *)p1)->servopos[i]*(MAX_INC-n))/MAX_INC;
     
     if (i==1) {
       /*
       Serial.println(((Pose *)p1)->servopos[i]);
       Serial.println(((Pose *)p2)->servopos[i]);
       Serial.println(val);
       */
     }
     
     pwm.setPWM(i, 0, val); // lfh 510 
     delay(((Pose *)p2)->time);  
  }

  return n+1;
}



int BasicWalk(int w) {
  int asig;
  for (int i=w; i < 6; i++) {
    Serial.println(i);  
    //SwitchPose((int *)&walkdat[i], (int *) &walkdat[(i+1)%6]);
    for (int j=0; j < MAX_INC; j++)
      SwitchPose2((int *)&walkdat[i], (int *) &walkdat[(i+1)%6], j);
      
    asig = listen();
    if (asig!=129) {
      return i;  // the evwn # 
    }  
  }
  return 0;
}

int TriWalk(int w) {
  int asig;
  for (int i=w; i < 8; i++) {
    Serial.println(i);  
    //SwitchPose((int *)&walkdat[i], (int *) &walkdat[(i+1)%6]);
    for (int j=0; j < MAX_INC; j++)
      SwitchPose2((int *)&triwalk[i], (int *) &triwalk[(i+1)%8], j);
/*     
    asig = listen();
    if (asig!=129) {
      return i;  // the evwn # 
    } 
*/   
  }
  return 0;
}

 // do a count 
  // when the count gets to 20, listen 
  // then switch step
// do the timer



int TriBack(int w) {
  int asig;
 // Serial.println(w);
  for (int i=(w-1)%8; i > -1; i--) {

    for (int j=0; j < MAX_INC; j++) {
    
      //SwitchPose2((int *)&walkdat[(i-1)%6], (int *) &walkdat[i], j);
      SwitchPose2((int *)&triwalk[i], (int *) &triwalk[(i+1)%8], j);

    }
/*    
     asig = listen();
    if (asig!=130) {
      return i;  // the evwn # 
    }  
    */
  }
  return 7;
}

int TriLeft(int w) {
  return w;
}

int TriRight(int w) {
  return w;
}

void Stand(int w) {
     w= (w/3)*3;
     SwitchPose((int *)&walkdat[w], (int *) &walkdat[(w+1)%6]);
     delay(100);

}


void loop() {

  
  int asig, bsig;
  
  asig = listen();
  if (asig==0) {
    // get the message
    
  }
  /*
  Serial.print("Walkphase ");
  Serial.println(walkphase);
  */
  
  switch (asig) {
    case 128:
      // Stand
      Stand(0);  // walkphase
      break;
    case 129:
      // Forward
      walkphase = TriWalk(0);  //BasicWalk(walkphase);  
      break;
    case 130:
      // Backward
      walkphase = TriBack(walkphase);  // was walkphase
      break;
    case 131:
      // Left
      walkphase = TriLeft(walkphase);
      break;
    case 132:
      // right
      walkphase = TriRight(walkphase);
      break;
  }
  return; 

  

  servonum ++;
  if (servonum > 7) servonum = 0;
 
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
     pwm.setPWM(servonum, 0, pulselen);
    
  }
  
 }
