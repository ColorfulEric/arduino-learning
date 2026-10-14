#incldue<Servo.h>

Servo myservo;

void setup()
{
    Serial.begin(9600);
    myservo.attach(6);
}

void loop()
{
    int data = analogRead(A1);
    myservo.write(data);
}