//定义摇杆的模拟输入引脚
const int joyX = A0;
const int joyY = A1;
void setup()
{
    Serial.begin(9600);
}

void loop()
{
    //1、读取遥感的原始模拟值（0-1023）
    int rawX = analogRead(joyX);
    int rawY = analogRead(joyY);
    //2、讲原始值转化为舵机的PWM角度值（0-180）
    int pwmX = map(rawX,0,1023,0,180);
    int pwmY = map(rawY,0,1023,0,180);
    //3、将原始值的转化后的PWM值答应到串口监视器
    //打印格式：x原始值|x角度|y原始值|y角度
    Serial.print("x:");
    Serial.print(rawX);
    Serial.print("->");
    Serial.print(pwmX);
    Serial.print("°\t"); 

    Serial.print("Y: ");
    Serial.print(rawY);
    Serial.print(" -> ");
    Serial.print(pwmY);
    Serial.println("°");

    delay(100);
}