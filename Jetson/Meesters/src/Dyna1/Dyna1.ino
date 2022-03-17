#include <MyDynamixel.h>

int i = 0;
uint8_t goalMovementBits[4] = {0, 0, 0, 0};
uint8_t ledOn = 0x01;
uint8_t ledOff = 0x00;
int DEPin = 19;
uint8_t retPack[7];

void setup()
{
  Serial.begin(115200);
}

#define DXL_SERIAL Serial5
MyDynamixel DYNA(DXL_SERIAL, 1000000, DEPin);
uint8_t newID[] = {17};
uint8_t newRetSpd[] = {5};

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
  //DYNA.ReadServos(254,PRESENT_TEMPERATURE,1,retPack);
  //Serial.println(retPack[5]);
  //Serial.println(DYNA.PresentPos(3));
  //delay(1000);
  //DYNA.MoveServos(1,0,8);
  //DYNA.MoveServos(1,150);

  //DYNA.WriteServos(1,ID, newID, 1);

  //DYNA.WriteServos(newID[0],RETURN_DELAY_TIME, newRetSpd, 1);

  DYNA.ReadServos(newID[2], RETURN_DELAY_TIME, 1, retPack);
  Serial.println(retPack[5]);

}
