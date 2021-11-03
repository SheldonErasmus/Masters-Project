#include <MyDynamixel.h>

int i = 0;
uint8_t goalMovementBits[4] = {0, 0, 0, 0};
uint8_t ledOn = 0x01;
uint8_t ledOff = 0x00;
int DEPin = 19;
uint8_t retPack[7];

void setup() {
  Serial.begin(115200);
  Serial5.begin(57600);
  Serial5.transmitterEnable(DEPin);
}

MyDynamixel DYNA(&Serial5);
uint8_t newID[] = {1};

void loop() 
{
//    if (i % 2 == 0)
//  {
//    DYNA.WriteServos(1,LED, &ledOn, 1);//activateServos(servoID, ledOn);
//    delay(1000);
//    goalMovementBits[0] = 0x00;
//    goalMovementBits[1] = 0x02;
//    goalMovementBits[2] = 0x00;
//    goalMovementBits[3] = 0x02;
//  }
//  else
//  {
//    DYNA.WriteServos(1,LED, &ledOff, 1);//activateServos(servoID, ledOff);
//    delay(1000);
//    goalMovementBits[0] = 0x00;
//    goalMovementBits[1] = 0x00;
//    goalMovementBits[2] = 0x00;
//    goalMovementBits[3] = 0x02;
//  }
//
//  i++;
//  if (i > 1000)
//  {
//    i = 1;
//  }

  //WriteServos(GOAL_POSITION_L, goalMovementBits, 4);
  //Serial.println(ReadServos(PRESENT_POSITION_L, 1),HEX);
  //DYNA.ReadServos(1,PRESENT_TEMPERATURE,1,retPack);
  //DYNA.PresentPos(1);
  //delay(1000);
  //DYNA.MoveServos(1,0,8);
  //DYNA.MoveServos(1,150);

  DYNA.MoveServos(1,150,8);
  DYNA.MoveServos(2,0,8);
  
  float f_present_position = 0.0;
  float f_present_position2 = 0.0;

  while (abs(150 - f_present_position) > 2.0)
  {
  f_present_position = DYNA.PresentPos(1);
  f_present_position2 = DYNA.PresentPos(2);
  Serial.print("Present_Position: ");
  Serial.print(f_present_position); Serial.print(" "); Serial.println(f_present_position2);
  }
  delay(500);

  // Set Goal Position in DEGREE value
  DYNA.MoveServos(1,0,8);
  DYNA.MoveServos(2,150,8);
  
  while (abs(0 - f_present_position) > 2.0)
  {
    f_present_position = DYNA.PresentPos(1);
    f_present_position2 = DYNA.PresentPos(2);
    Serial.print("Present_Position: ");
    Serial.print(f_present_position); Serial.print(" "); Serial.println(f_present_position2);
  }
  delay(500); 
//  DYNA.WriteServos(2,ID, newID, 1); 
}
