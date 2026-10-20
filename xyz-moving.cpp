#include<Servo.h>

Servo base,lArm,rArm;

//单位mm
const float L1 = 80.0;//大臂(rArm)
const float L2 = 80.0;//小臂(lArm)
const float H = 50.0;//肩部桌面
const float R_MIN = 20;//最短距离


// 舵机初始偏移，需要标定
const float BASE_OFFSET = 90.0;
const float LARM_OFFSET = 90.0;
const float RARM_OFFSET = 180.0;

// 限幅
const int baseMin = 0, baseMax = 180;
const int lArmMin = 0, lArmMax = 90;
const int rArmMin = 90, rArmMax = 180;

int DSD = 15;

void setup() 
{
  base.attach(9,500,2500);
  base.write(90);
  delay(1000);
  lArm.attach(8,500,2500);
  lArm.write(90);
  delay(1000);
  rArm.attach(7,500,2500);
  rArm.write(90);
  delay(1000);
  Serial.begin(9600);
  Serial.println("Please Input Data: ");
}

void loop()
{
    if(Serial.available())
    {
        String serialCmd = Serial.readStringUntil('\n');
        serialCmd.trim();

        if(serialCmd.length()==0) return;

        int x,y,z;

        if(serialCmd.indexOf('x') != -1 && serialCmd.indexOf('y') != -1 && serialCmd.indexOf('z') != -1)
        {
            x = getValue(serialCmd,'x');
            y = getValue(serialCmd,'y');
            z = getValue(serialCmd,'z');

        }
        else Serial.println("+Warning: Wrong Command! ");

        int sBase,sLArm,sRArm;

        if(inverseKinematics(x,y,z,sBase,sLArm,sRArm))
        {
            moveServo(base,base.read(),sBase);
            moveServo(lArm,lArm.read(),sLArm);
            moveServo(rArm,rArm.read(),sRArm);
        }
        else Serial.println("+Warning: Out Of reach");
    }
}

int getValue(String data, char key)
{
    int index = data.indexOf(key);
    if(index == -1) return -1;

    int end = data.indexOf(',',index);
    if(end == -1) end = data.length();

    return data.substring(index + 1, end).toInt();
}

bool inverseKinematics(float x,float y,float z,int &sBase,int &sLArm,int &sRArm)
{
    float r = sqrt(x * x + y * y);
    if(r < 20.0) return false;

    float D = (r * r + z * z - L1 * L1 - L2 * L2)/(2.0 * L1 * L2);
    if(D < -1.0 || D > 1.0) return false;

    float d = z - H;

    float theta3 = atan2(sqrt(1.0 - D * D), D);
    float theta2;
    if(d <= 0)
    {
        d = 0.0 - d;
        theta2 = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3)) - atan2(d, r);
    }
    else
    {
        theta2 = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3)) + atan2(d, r);
    }
    float theta1 = atan2(y,x);

    //舵机实际需要到达的角度
    sBase = (int)(degrees(theta1));
    sLArm = (int)(LARM_OFFSET - degrees(theta3 - theta2));
    sRArm = (int)(RARM_OFFSET - degrees(theta2));

    float B = sBase,L = sLArm,R = sRArm;

    Serial.print(F(" need base = ")); Serial.print(sBase, 1);
    Serial.print(F(" lArm = "));          Serial.print(sLArm, 1);
    Serial.print(F(" rArm = "));          Serial.println(sRArm, 1);
    Serial.print(F(" r = "));             Serial.print(r);
    Serial.print(F(" theta1 = "));        Serial.print(theta1);
    Serial.print(F(" theta2 = "));        Serial.print(theta2);
    Serial.print(F(" theta3 = "));        Serial.println(theta3);
    Serial.print(F(" sBase = "));Serial.print(B);
    Serial.print(F(" sLArm = "));Serial.print(L);
    Serial.print(F(" sRArm = "));Serial.println(R);

    if(sBase < baseMin || sBase > baseMax) return false;
    if(sLArm < lArmMin || sLArm > lArmMax) return false;
    if(sRArm < rArmMin || sRArm > rArmMax) return false;

    return true;
}

void moveServo(Servo &servoName, int fromPos, int toPos)
{
    if(fromPos>toPos)
    {
        for(int i = fromPos; i >= toPos; i--)
        {
            servoName.write(i);
            delay(DSD);
        }
    }
    else
    {
        for(int i = fromPos; i <= toPos; i++)
        {
            servoName.write(i);
            delay(DSD);
        }
    }
}