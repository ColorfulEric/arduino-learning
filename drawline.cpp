#include<Servo.h>

Servo base,lArm,rArm;

// 机械臂尺寸，用尺子量，单位 mm
const float L1 = 80.0;
const float L2 = 80.0;
const float L3 = 80.0;
const float H = 60.0;

// 舵机偏移，需要标定
const float BASE_OFFSET = 90.0;
const float LARM_OFFSET = 90.0;
const float RARM_OFFSET = 0.0;

// 限幅
const int baseMin = 0,  baseMax = 180;
const int lArmMin = 0, lArmMax = 90;
const int rArmMin = 90, rArmMax = 180;

void setup() 
{
  base.attach(9);
  lArm.attach(8);
  rArm.attach(7);
  Serial.begin(9600);
}

bool inverseKinematics(float x,float y,float z,int &sBase,int &sLArm,int &sRArm)
{
    float theta1 = atan2(y,x);
    float r = sqrt(x*x + y*y) - L3;

    float D = (r*r + z*z - L1*L1 - L2*L2)/(2*L1*L2);
    //D表示cosθ3
    if(D < -1.0 || D > 1.0) return false;

    float theta3 = atan2(sqrt(1 - D*D),D);
    float theta2 = atan2(L2 * sin(theta3),L1 + L2*cos(theta3)) + atan2(z - H,r);

    sBase = (int)(BASE_OFFSET - degrees(theta1));
    sLArm = (int)(LARM_OFFSET - (degrees(theta3) - degrees(theta2)));
    sRArm = (int)(RARM_OFFSET + degrees(theta2));

    if(sBase < baseMin || sBase > baseMax) return false;
    if(sLArm < lArmMin || sLArm > lArmMax) return false;
    if(sRArm < rArmMin || sRArm > rArmMax) return false;

    return true;
}

bool moveTo(float x,float y, float z)
{
    int sBase ,sLArm,sRArm;
    if(!inverseKinematics(x,y,z,sBase,sLArm,sRArm));
    {
        Serial.println("+Warning: Position out of reach");
        return false;
    }

    int fromBase = base.read();
    int fromLArm = lArm.read();
    int fromRArm = rArm.read();

    int steps = 20;
    for(int i = 0;i <= steps;i++)
    {
        base.write(fromBase + (sBase - fromBase) * i / steps);
        lArm.write(fromLArm + (sLArm - fromLArm) * i / steps);
        rArm.write(fromRArm + (sRArm - fromRArm) * i / steps);
        delay(20);
    }

    return true;
}

void drawline(float x1,float y1,float x2,float y2,float z,float steps)
{
    for(int i = 0;i <= steps;i++)
    {
        float t = (float)i / steps;
        float x = x1 + (x2 - x1) * t;
        float y = y1 + (y2 - y1) * t;
        moveTo(x,y,z);
    }
}

void loop()
{
    drawline();
    drawline();
    drawline();
    
    delay(3000);
}