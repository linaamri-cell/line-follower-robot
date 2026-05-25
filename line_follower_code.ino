  //Les capteurs
const int IR[6] = {A5,A1,A2,A3,A4,A0};
//weights
int weights[6] = {0,0,0,0,0,0};
 //threshold
int threshold[6] = {0,0,0,0,0,0};
//led
int led=9;
//boutton_go
int go = 2;
//obstacles done ?
bool obs2_done = false;
bool obs3_done = false;
bool obs4_done = false;
bool obs5_done = false;
  //pont H
 int IN1=12, IN2=13, ENA=11;
 int IN3=6, IN4=7, ENB=5;
// min speed
int min_right_speed = 70;
int min_left_speed = 70;
// PID coefficients (doit etre multiple de 10 pour qu'ils soient efficaces)
float Kp = 80.0;
float Ki = 0.0;
float Kd = 40.0 ;
float k_turn = 0.5;
float integral = 0;
float lastError = 0;
int32_t last_PID_time = 0;
float last_position = 0;

unsigned long startobs3=0;
unsigned long startTime;

void calibrate(){
  delay(1000);
  for(int i=0; i<20; i++){
    for(int j=0; j<6; j++){
      threshold[j] += analogRead(IR[j]);
      delay(1);
    }
  }
  digitalWrite(led, 1);
  delay(2000);
  digitalWrite(led, 0);
  for(int i=0; i<20; i++){
    for(int j=0; j<6; j++){
      threshold[j] += analogRead(IR[j]);
      delay(1);
    }
  }

  for(int j=0; j<6; j++){
    threshold[j] = threshold[j]/40;
  }
  delay(1500);
  digitalWrite(led, 1);
  delay(2000);
  digitalWrite(led, 0);

}

void set_weights(int w0,int w1,int w2,int w3,int w4,int w5){
weights[0]=w0;
weights[1]=w1;
weights[2]=w2;
weights[3]=w3;
weights[4]=w4;
weights[5]=w5;
}


void run_right_motor(int right_speed) {
  if (abs(right_speed) < min_right_speed) {
    analogWrite(ENB, 0);
    return;
  }
  if (right_speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, right_speed);
  }
  else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB,abs(right_speed));
  }
}

void run_left_motor(int left_speed) {
  if (abs(left_speed) < min_left_speed) {
    analogWrite(ENA, 0);
    return;
  }

  if (left_speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA,left_speed);
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA,abs(left_speed));
  }
}

//connaitre la position du robot
float get_error(int cerveau){
  float position=0;
  int somme=0;
  for(int i=0;i<6;i++){
    int val = analogRead(IR[i]);
      if(val>threshold[i]) //A0:147 A1:433 A2:445
        val=1 - cerveau;
      else
        val=cerveau;
    position += val*weights[i];
    somme += val;
  }
 // delayMicroseconds(50);
  if(somme != 0){
    position=position/somme;
    position=position/100; //rendre la correction convenable
    last_position = position;
    return position;
  }
  else 
    return last_position/100;
}


float PID(int cerveau) {
  float error= get_error(cerveau); 
  float dt = (micros() - last_PID_time)/1000000.0;
  float derivative = (error-lastError) /dt;
  float correction;
  integral += error;
  integral = constrain(integral, -100, 100);
  correction= Kp*error + Ki*integral + Kd*derivative;
  lastError=error;
  if(abs(integral)>=100){
    integral =0;
  }
  last_PID_time = micros();
  return correction;
}

// === Mouvement avec PID ===
void followLine(int w0, int w1, int w2, int w3, int w4, int w5, int base_speed_right, int base_speed_left, int cerveau) {
  set_weights(w0,w1,w2,w3,w4,w5);
  float correction = PID(cerveau);

  int left_speed  = constrain(base_speed_left + correction - abs(correction) * k_turn, -255, 255);
  int right_speed = constrain(base_speed_right - correction - abs(correction) * k_turn, -255, 255);

  run_right_motor(right_speed);
  run_left_motor(left_speed);
  //delay(25);
}

//  detection capteurs 
String detection_capteurs() {
  String pattern = "";

  for (int i = 0; i < 6; i++) {
    int val = analogRead(IR[i]);
    if (val > threshold[i]) {  // capteur sur noir
      pattern += "1";
    } else {
      pattern += "0";
    }
  }
  return pattern;
}

String detection_capteurs_w() {
  String pattern = "";

  for (int i = 0; i < 6; i++) {
    int val = analogRead(IR[i]);
    if (val < threshold[i]) {  // capteur sur noir
      pattern += "1";
    } else {
      pattern += "0";
    }
  }
  return pattern;
}

// condition 2
bool condition_obs2(){
  if((millis()-startTime > 5000) && (obs2_done == false) && ((detection_capteurs() == "111111") || (detection_capteurs() == "101101") || (detection_capteurs() == "101111") || (detection_capteurs() == "111101")))
    return true;
  else
    return false;
}

void reaction_obs2(){
  run_right_motor(0);
  run_left_motor(0);
  delay(300);
  unsigned long start = millis();
  run_right_motor(-85);
  run_left_motor(100);
  delay(400);
  unsigned long start_obs = millis();

  // We used this loop to avoid getting allBlack a second time just after reaching the hexaGone 
  for (int i=0;i<500;i++)
  {
    followLine(-40, -40, -100, 75, 250, 400,90,90,0);
  }

  while (detection_capteurs() != "111111"){
    followLine(-40, -40, -100, 75, 250, 400,90,90,0);
  }
}

// condition 3
bool condition_obs3(){
  if((millis()-startobs3> 7000)&&(obs2_done == true) && (obs3_done == false) && ((detection_capteurs() == "010011") || (detection_capteurs() == "001110") || (detection_capteurs() == "000001") || (detection_capteurs() == "001101"))){
    return true;
    
    }
  else
    return false;
}

void reaction_obs3(){
  unsigned long start_obs = millis();
  run_left_motor(0);
  run_right_motor(0);
  delay(500);
  run_left_motor(100);
  run_right_motor(120);
  delay(350);
  run_left_motor(100);
  run_right_motor(100);
  delay(150);
  for (int i=0;i<5000;i++)
  {
  followLine(0, -500, -75, 75, 250, 0,95,95,1);
  }
  while ((detection_capteurs_w()!="011111") && (detection_capteurs_w()!="111110") && (detection_capteurs_w()!="111100") && (detection_capteurs_w()!="011110")){
  followLine(0, -500, -75, 75, 250, 0,95,95,1); 
  }
}
/*

      // condition 4
bool condition_obs4(){
  if()
    return true;
  else
    return false;
}

void reaction_obs4(){
  
}

      // condition 5
bool condition_obs5(){
  if()
    return true;
  else
    return false;
}

void reaction_obs5(){
  
}*/

void setup() {
Serial.begin(9600);
  for(int i=0;i<6;i++){
    pinMode(IR[i],INPUT);
  }
 
 //Pont H
 //Moteur 1 droite
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
//moteur 2 gauche
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(go, INPUT_PULLUP);

calibrate();
startTime = millis();
last_PID_time = micros();
int go_boutton = digitalRead(go);
while(go_boutton == HIGH){
  go_boutton = digitalRead(go);
}
for (int i = 0; i < 250 ; i++){
  run_left_motor(115);
  run_right_motor(115);
}
}


void loop() {
  if(obs2_done)
    followLine(-215, -100, -35, 35, 165, 260,100,100,0);
  else
    followLine(-250, -100, -40, 40, 100, 250,110,110,0);   


  
  // Parfaite !!!!!
  if(condition_obs2()){
    digitalWrite(led,HIGH);
    reaction_obs2();
    digitalWrite(led,LOW);
    obs2_done=true;
    startobs3 = millis();
    run_right_motor(0);
    run_left_motor(0);
    delay(300);
    run_right_motor(-85);
    run_left_motor(85);
    delay(400);
    }
  
  

  if(condition_obs3()){
    digitalWrite(led,HIGH);
    reaction_obs3();
    digitalWrite(led,LOW);
    run_right_motor(0);
    run_left_motor(0);
    delay(300);
    run_right_motor(150);
    run_left_motor(90);
    delay(550);
    run_right_motor(0);
    run_left_motor(0);
    delay(30000);


  }
  

}