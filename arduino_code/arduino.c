const int trigPin = 9;  
const int echoPin = 10; 
const int outputPin = 8;
float duration, distance;
int counter1 = 0, counter0 = 0;
int curValue = LOW;
int prevValue = LOW; 

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);  
  pinMode(outputPin, OUTPUT);
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trigPin, LOW);  
  duration = pulseIn(echoPin, HIGH);  
  distance = (duration*.0343)/2;

  if (distance <= 30.0) {
    ++counter1;
    counter0 = 0;
    if(counter1 == 5)
    { 
      curValue = HIGH;
      counter1 = 0;
    }
  } else {
    counter0++;
    counter1 = 0;
    if(counter0 == 5)
    {
      curValue = LOW;
      counter0 = 0;
    }
  }

  if (curValue != prevValue) {
    digitalWrite(outputPin, curValue);
    prevValue = curValue;
  }

  int printPin = digitalRead(outputPin); 
  Serial.print("Pin curValue: ");  
  Serial.println(printPin);
  delay(50);  
}