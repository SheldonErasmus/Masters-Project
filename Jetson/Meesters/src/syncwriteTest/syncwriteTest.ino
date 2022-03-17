#include <MyDynamixel.h>
#define DXL_SERIAL Serial5
const uint8_t DEPin = 19; // DYNAMIXEL DIR PIN
MyDynamixel dxl(DXL_SERIAL, 1000000, DEPin);

float theta1[6] = {0,0,0,0,0,0};
float theta2[6] = {0,0,0,0,0,0};
float theta3[6] = {0,0,0,0,0,0};

long currentmillis = 0;
long prevmillis = millis();

void SetAngles(float* th1,float* th2,float* th3 ,float spd1=0,float spd2=0, float spd3=0);

void setup() 
{
  Serial.begin(115200);

}

long tic,toc;

void loop() 
{
  currentmillis = millis();
  if(currentmillis - prevmillis >= 10)
  {
    prevmillis = currentmillis;
    tic = micros();
    SetAngles(theta1,theta2,theta3,10,10,10);
    toc = micros();
    Serial.println(toc-tic);
  }

}

double Angle[18];
double Spd[18];
uint8_t Id[18];
uint8_t x = 0;

void SetAngles(float* th1,float* th2,float* th3 ,float spd1,float spd2, float spd3)
{
  
  for(int num = 0; num<18; num++)
  {
    if (num == 0) //thata11
    {
      Angle[num] = th1[0]*180/M_PI+150;
      Spd[num] = spd1;
      Id[num] = 0;
        
    }
    else if (num == 1)  //thata21
    {
      Angle[num] = -th2[0]*180/M_PI+150;
      Spd[num] = spd2;
      Id[num] = 1;
  
      dataStr[0] = 0; 
      itoa( millis(),buff,10); //convert long to charStr
      strcat(dataStr, buff); //add it to the end
      strcat(dataStr, ", "); //append the delimiter
      dtostrf(th2[0], 5, 5, buff);
      strcat(dataStr, buff);

    }
    else if (num == 2)  //thata31
    {
      Angle[num] = -th3[0]*180/M_PI+150;
      Spd[num] = spd3;
      Id[num] = 2;

    }
    else if (num == 3)  //thata12
    {
      Angle[num] = th1[1]*180/M_PI+150;
      Spd[num] = spd1;
      Id[num] = 3;
 
    }
    else if (num == 4)  //thata22
    {
      Angle[num] = th2[1];
      Spd[num] = spd2;
      Id[num] = 4;

    }
    else if (num == 5)  //thata32
    {
      Angle[num] = th3[1];
      Spd[num] = spd3;
      Id[num] = 5;

    }
    else if (num == 6)  //thata13
    {
      Angle[num] = th1[2];
      Spd[num] = spd1;
      Id[num] = 6;

    }
    else if (num == 7)  //thata23
    {
      Angle[num] = th2[2];
      Spd[num] = spd2;
      Id[num] = 7;

    }
    else if (num == 8)  //thata33
    {
      Angle[num] = th3[2];
      Spd[num] = spd3;
      Id[num] = 8;

    }
    else if (num == 9)  //thata14
    {
      Angle[num] = th1[3];
      Spd[num] = spd1;
      Id[num] = 9;

    }
    else if (num == 10) //thata24
    {
      Angle[num] = th2[3];
      Spd[num] = spd2;
      Id[num] = 10;

    }
    else if (num == 11) //thata34
    {
      Angle[num] = th3[3];
      Spd[num] = spd3;
      Id[num] = 11;

    }
    else if (num == 12) //thata15
    {
      Angle[num] = th1[4];
      Spd[num] = spd1;
      Id[num] = 12;

    }
    else if (num == 13) //thata25
    {
      Angle[num] = th2[4];
      Spd[num] = spd2;
      Id[num] = 13;

    }
    else if (num == 14) //thata35
    {
      Angle[num] = th3[4];
      Spd[num] = spd3;
      Id[num] = 14;

    }
    else if (num == 15) //thata16
    {
      Angle[num] = th1[5];
      Spd[num] = spd1;
      Id[num] = 15;

    }
    else if (num == 16) //thata26
    {
      Angle[num] = th2[5];
      Spd[num] = spd2;
      Id[num] = 16;

    }
    else if (num == 17) //thata36
    {
      Angle[num] = th3[5];
      Spd[num] = spd3;
      Id[num] = 17;

    }
  }

  dxl.SyncMove(Id,Angle,Spd,18);
  logdata.data = dataStr;
  pub_log.publish(&logdata);

}
