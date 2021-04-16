#define WIFI_NAME "Cristo salva"
#define WIFI_PASSWORD ""
#define DEVICE_ID 4
#define DEVICE_NAME "Robo Salvavidas"
#define TOKEN "~991902_jNysKCl7ke3LDlgi2NXNxND7"


#include <RemoteMe.h>
#include <RemoteMeSocketConnector.h>
#include <RemoteMeDirectWebSocketConnector.h> // !important library needs WebSockets by Markus Sattler Please install it from Library manager

#include <ESP8266WiFi.h>


#define ENA D3
#define IN1 D1
#define IN2 D2

#define IN3 D5
#define IN4 D6
#define ENB D7

           //pra frente
int a = 0;

int motor = 0;

int aceleracao = 0;
int acelerado = 0;

int antacelerado = 0;

int parada = 0;


unsigned long lastExecutedMillis1 = 0;
unsigned long lastExecutedMillis2 = 0;
unsigned long lastExecutedMillis3 = 0;

RemoteMe& remoteMe = RemoteMe::getInstance(TOKEN, DEVICE_ID);


int vertical;
int horizontal;

int motorA;
int motorB;


//***** CODE FOR COMFORTABLE VARIABLE SET *******

inline void setJoystick(int16_t i1, int16_t i2) {remoteMe.getVariables()->setSmallInteger2("Joystick", i1, i2); }

inline void setBackward(boolean b) {remoteMe.getVariables()->setBoolean("Backward", b); }
inline void setForward(boolean b) {remoteMe.getVariables()->setBoolean("Forward", b); }
inline void setLeft(boolean b) {remoteMe.getVariables()->setBoolean("Left", b); }
inline void setRight(boolean b) {remoteMe.getVariables()->setBoolean("Right", b); }
inline void setSpeed(int32_t i) {remoteMe.getVariables()->setInteger("Speed", i); }
inline void setWiFi_strength(int32_t i) {remoteMe.getVariables()->setInteger("WiFi strength", i); }


//***** IMPLEMENT FUNCTIONS BELOW *******


void onWiFi_strengthChange(int32_t i) {
}

void onJoystickChange(int16_t i1, int16_t i2) {
  //Serial.printf("onJoystickChange: i1: %d, i2: %d\n",i1, i2);

   vertical = i2;
   horizontal = i1;

   

   //se para frente:
   
if (vertical >= 33){

    Serial.print("frente");
   Serial.println(vertical);

  //se para direita
  
  if (horizontal >= 33){
  
     Serial.print("direita");
     Serial.println(horizontal);

    //as duas rodas para frente:
    
     if (horizontal <= vertical){
    
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

     // motorA = vertical-horizontal;
      motorB = vertical+horizontal;
      motorA = motorB*3/4
      if (motorB >= 1023){
        motorB = 1023;
      }
      if (motorA < 0) {
      motorA = 0;
      }
      analogWrite(ENB, motorA); 
      analogWrite(ENA, motorB);
      }

      //uma roda para frente e uma para traz:
      
     if (horizontal >= vertical){

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      
      //motorA = horizontal-vertical;
      motorB = horizontal+vertical;
      motorA = motorB*3/4
      if (motorB >= 1023){
        motorB = 1023;
      }
      if (motorA < 0) {
      motorA = 0;
      }
      analogWrite(ENB, motorA); 
      analogWrite(ENA, motorB);
      
     }
    }
  
  //se para esquerda:
  
  if (horizontal <= 33){
  
     Serial.print("esquerda");
     Serial.println(horizontal);

      //as duas rodas para frente:
     
      if (-horizontal <= vertical){
  
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      
      motorA = vertical-horizontal;
      //motorB = vertical+horizontal;
      motorB = motorA*3/4
        if (motorA >= 1023){
        motorA = 1023;
      }
      if (motorB < 0) {
      motorB = 0;
      }
      analogWrite(ENB, motorA); 
      analogWrite(ENA, motorB);
      }

      //uma roda para frente e uma para traz:
      
      if (-horizontal >= vertical){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
     
      motorA = -horizontal-vertical;
      //motorB = -horizontal+vertical;
      motorB = motorA*3/4
      if (motorA >= 1023){
        motorA = 1023;
      }
      if (motorB < 0) {
      motorB = 0;
      }
      analogWrite(ENB, motorB); 
      analogWrite(ENA, motorA);
      }
    }
  }

   //se para tras:
   
if (vertical <= 33){

    Serial.print("tras");
   Serial.println(vertical);
   
  //se para a direita
  
  if (horizontal >= 33){
    
     Serial.print("direita");
     Serial.println(horizontal);

     //as duas rodas para traç
    
     if (horizontal <= -vertical){
    
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

      //motorA = -vertical-horizontal;
      motorB = -vertical+horizontal;
      motorA = motorB*3/4
      if (motorB >= 1023){
        motorB = 1023;
      }
      if (motorA < 0) {
      motorA = 0;
      }
      analogWrite(ENA, motorB); 
      analogWrite(ENB, motorA);
      }
    }


  //se para esquerda
  
  if (horizontal <= 33){

 Serial.print("esquerda");
 Serial.println(horizontal);

    //as duas para traz
 
    if (-horizontal <= -vertical){

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
   // motorA = -vertical+horizontal;
    motorB = -vertical-horizontal;
    motorA = motorB*3/4
    if (motorB >= 1023){
      motorB = 1023;
    }
    if (motorA < 0) {
    motorA = 0;
    }
    analogWrite(ENA, motorA); 
    analogWrite(ENB, motorB);
    }


 
  }
}
// parado, sem impulso
if (-33 < horizontal and vertical < 33){

    analogWrite(motorA, 0); 
    analogWrite(motorB, 0);
    delay(10);
    analogWrite(motorA, 0); 
    analogWrite(motorB, 0);
    delay(10);
    analogWrite(motorA, 0); 
    analogWrite(motorB, 0);
}

}


void onBackwardChange(boolean b) {
  Serial.printf("onBackwardChange: b: %d\n",b);
   digitalWrite(IN2, b?HIGH:LOW);
   digitalWrite(IN3, b?HIGH:LOW);

 a = b;
 /
 acelerado = 0;

if (!b)  // test for falsity
{
  parada = 0;  // do whatever...
}

 parada = b;  
}
void onForwardChange(boolean b) {
  
  Serial.printf("onForwardChange: b: %d\n",b);
 digitalWrite(IN1, b?HIGH:LOW);
 digitalWrite(IN4, b?HIGH:LOW);

 a = b;
 
 acelerado = 0;

if (!b)  // test for falsity
{
  parada = 0;  // do whatever...
}

 parada = 1;  
 
}
void onLeftChange(boolean b) {
  Serial.printf("onLeftChange: b: %d\n",b);
  digitalWrite(IN1, b?HIGH:LOW);
  digitalWrite(IN3, b?HIGH:LOW);
   a = b;
 
 acelerado = 0;

if (!b)  // test for falsity
{
  parada = 0;  // do whatever...
}
  parada = 1;  

}
void onRightChange(boolean b) {
  Serial.printf("onRightChange: b: %d\n",b);
  digitalWrite(IN2, b?HIGH:LOW);
  digitalWrite(IN4, b?HIGH:LOW);
   a = b;
 
 acelerado = 0;
 
if (!b)  // test for falsity
{
  parada = 0;  // do whatever...
}
  parada = 1;  
}
void onSpeedChange(int32_t i) {
  Serial.printf("onSpeedChange: i: %d\n",i);
  //analogWrite(ENA, i); 
  //analogWrite(ENB, i);
  aceleracao = i;
}


void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);

   analogWrite(ENA, 300); 
   analogWrite(ENB, 300);                          
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW); 
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, HIGH);
  delay(100);
   analogWrite(ENA, 300); 
   analogWrite(ENB, 300);  
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, LOW);
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, LOW);
  delay(50);

 analogWrite(ENA, 0); 
 analogWrite(ENB, 0);

  }

   analogWrite(ENA, 300); 
   analogWrite(ENB, 300);                          
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW); 
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, HIGH);
  delay(100);
   analogWrite(ENA, 300); 
   analogWrite(ENB, 300);  
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, LOW);
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, LOW);
  delay(50);

 analogWrite(ENA, 0); 
 analogWrite(ENB, 0);

  remoteMe.getVariables()->observeInteger("WiFi strength" ,onWiFi_strengthChange);

  remoteMe.getVariables()->observeSmallInteger2("Joystick" ,onJoystickChange);

  remoteMe.getVariables()->observeBoolean("Backward" ,onBackwardChange);
  remoteMe.getVariables()->observeBoolean("Forward" ,onForwardChange);
  remoteMe.getVariables()->observeBoolean("Left" ,onLeftChange);
  remoteMe.getVariables()->observeBoolean("Right" ,onRightChange);
  remoteMe.getVariables()->observeInteger("Speed" ,onSpeedChange);


  remoteMe.setConnector(new RemoteMeSocketConnector());
  remoteMe.setDirectConnector(new RemoteMeDirectWebSocketConnector());
  remoteMe.sendRegisterDeviceMessage(DEVICE_NAME);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

 analogWrite(ENA, 0); 
 analogWrite(ENB, 0);

}


void loop() {
  remoteMe.loop();

if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
}

if(millis() > lastExecutedMillis1 + 1000){
   lastExecutedMillis1 = millis();
   Serial.print("RRSI: ");
   Serial.println(WiFi.RSSI());
   setWiFi_strength(WiFi.RSSI());
    }

//////////////////////////////////////////////////////////////////////////////////////////////

 // app de botoes:

                 //menos que 500 (rapido)

if (acelerado < 500){

 if ((a == 1)&(acelerado < aceleracao)){
  
   acelerado = acelerado + 12;

   Serial.print("menos que 500 'rapido' ");
   Serial.println(acelerado);
   
    analogWrite(ENA, acelerado); 
    analogWrite(ENB, acelerado);
   delay(1);
  }

 
 if ((a == 1)&(acelerado > aceleracao)){
  
   acelerado = aceleracao;

   Serial.print("menos que 500 'rapido' ");
   Serial.println(acelerado);
   
    analogWrite(ENA, acelerado); 
    analogWrite(ENB, acelerado);
   delay(1);
  }
}
         // mais que 500 (devagar)

if (acelerado >= 500){

 if ((a == 1)&(acelerado < aceleracao)){
  
   acelerado = acelerado + 5;
   
    Serial.print("mais0 que 500 'devagar' ");
   Serial.println(acelerado);

    analogWrite(ENA, acelerado); 
    analogWrite(ENB, acelerado);
   delay(1);
  }
 
 if ((a == 1)&(acelerado > aceleracao)){
  
   acelerado = aceleracao;
   
   Serial.print("mais0 que 500 'devagar' ");
   Serial.println(acelerado);
   
   analogWrite(ENA, acelerado); 
   analogWrite(ENB, acelerado);
   delay(1);
  }
 }
 
  if (a == 0){
   acelerado = 10;
   delay(1);
   
   if(millis() > lastExecutedMillis2 + 500){
   lastExecutedMillis2 = millis();
   Serial.print("parada");
   Serial.println(  );
    }
  }
 

if (parada == 0){
   acelerado = 10;
   delay(1);

   if(millis() > lastExecutedMillis3 + 500){
   lastExecutedMillis3 = millis();
   Serial.print("stop2");
   Serial.println(  );
    }
  }
 
/////////////////////////////////////////////////////////////////////////////////////////





}
