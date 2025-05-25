#define RED PI_3
#define YELLOW PI_2
#define GREEN PI_7
#define BLUE PI_5

void setup() {
    pinMode(RED, OUTPUT);
    pinMode(YELLOW, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
}

void loop() {
  digitalWrite(RED, HIGH);
  delay(500);
  digitalWrite(YELLOW, HIGH);
  delay(500);
  digitalWrite(GREEN, HIGH);
  delay(500);
  digitalWrite(BLUE, HIGH);
  delay(500);

  digitalWrite(RED, LOW);
  delay(500);
  digitalWrite(YELLOW, LOW);
  delay(500);
  digitalWrite(GREEN, LOW);
  delay(500);
  digitalWrite(BLUE, LOW);
  delay(500);

}
