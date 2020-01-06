char t;

#define L_M_F D2
#define L_M_R D3
#define R_M_F D4
#define R_M_R D5

// LED_BUILTIN


void setup() {
  pinMode(L_M_F, OUTPUT);
  pinMode(L_M_R, OUTPUT);
  pinMode(R_M_F, OUTPUT);
  pinMode(R_M_R, OUTPUT);
  Serial.begin(9600);

}

void loop() {

  if (Serial.available()) {
    t = Serial.read();
    Serial.println(t);
  }

  if (t == '1') {          //move forward(all motors rotate in forward direction)
    Serial.println(t);
      Serial.println(L_M_F);
  Serial.println(L_M_R);
  Serial.println(R_M_F);
  Serial.println(R_M_R);
    digitalWrite(L_M_F, HIGH);
    digitalWrite(L_M_R, LOW);
    digitalWrite(R_M_F, HIGH);
    digitalWrite(R_M_R, LOW);
  }

  delay(100);
}
