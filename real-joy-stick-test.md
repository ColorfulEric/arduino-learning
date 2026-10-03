```cpp
const int joyX1 = A0;
const int joyY1 = A1;
const int joyX2 = A2;
const int joyY2 = A3;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    int rawX1 = analogRead(joyX1);
    int rawY1 = analogRead(joyY1);
    int rawX2 = analogRead(joyX2);
    int rawY2 = analogRead(joyY2);

    int pwmX1 = map(rawX1,0,1023,0,180);
    int pwmY1 = map(rawY1,0,1023,0,180);
    int pwmX2 = map(rawX2,0,1023,0,180);
    int pwmY2 = map(rawY2,0,1023,0,180);

    Serial.print("x1:");
    Serial.print(rawX1);
    Serial.print("->");
    Serial.print(pwmX1);
    Serial.print("°\t"); 
    Serial.print("y1:");
    Serial.print(rawY1);
    Serial.print("->");
    Serial.print(pwmY1);
    Serial.print("°\t"); 

    Serial.print("x2: ");
    Serial.print(rawX2);
    Serial.print(" -> ");
    Serial.print(pwmX2);
    Serial.print("°\t");
    Serial.print("y2: ");
    Serial.print(rawY2);
    Serial.print(" -> ");
    Serial.print(pwmY2);
    Serial.println("°");

    delay(100);
}
```