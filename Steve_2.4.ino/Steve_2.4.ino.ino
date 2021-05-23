#define WIFI_NAME "Cristo salva"
#define WIFI_PASSWORD ""
#define DEVICE_ID 4
#define DEVICE_NAME "Robo Salvavidas"
#define TOKEN "~991902_jNysKCl7ke3LDlgi2NXNxND7"

#include <RemoteMe.h>
#include <RemoteMeSocketConnector.h>
#include <RemoteMeDirectWebSocketConnector.h> // !important library needs WebSockets by Markus Sattler Please install it from Library manager

#include <WiFi.h>


const int ENA = 16; //D3
#define IN1 17//D1
#define IN2 18//D2

#define IN3 19//D5
#define IN4 21//D6
const int ENB = 22; //D7

// setting PWM properties
const int freq = 10000;
const int speedmA = 0;
const int speedmB = 0;
const int resolution = 8;

//pra frente
int a = 0;

int aceleracao = 0;
int acelerado = 0;

// time
unsigned long lastExecutedMillis1 = 0;
unsigned long lastExecutedMillis2 = 0;
unsigned long lastExecutedMillis3 = 0;

int motorA;
int motorB;

RemoteMe& remoteMe = RemoteMe::getInstance(TOKEN, DEVICE_ID);

//*************** CODE FOR COMFORTABLE VARIABLE SET *********************

inline void setForward(boolean b) {remoteMe.getVariables()->setBoolean("Forward", b); }
inline void setBackward(boolean b) {remoteMe.getVariables()->setBoolean("Backward", b); }
inline void setLeft(boolean b) {remoteMe.getVariables()->setBoolean("Left", b); }
inline void setRight(boolean b) {remoteMe.getVariables()->setBoolean("Right", b); }
inline void setSpeed(int32_t i) {remoteMe.getVariables()->setInteger("Speed", i); }
inline void setJoystick(int16_t i1, int16_t i2) {remoteMe.getVariables()->setSmallInteger2("Joystick", i1, i2); }
inline void setWiFi_strength(int32_t i) {remoteMe.getVariables()->setInteger("WiFi strength", i); }

//*************** IMPLEMENT FUNCTIONS BELOW *********************

void onWiFi_strengthChange(int32_t i) {
}

void onJoystickChange(int16_t i1, int16_t i2) {


if ((i2 == 0) and (i1 == 0)) {
  
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
}
else {
  
  if (i2 > 0) {   //as duas rodas para frente:

    if (i1 > 0) {    //se para direita
      if (i1 * 2 <= i2) {
        
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      ledcWrite(speedmB, i2);
      ledcWrite(speedmA, i2);

      Serial.println("frente");
      Serial.println(i2);
      }
      
      if (i2 <= i1 / 3) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        ledcWrite(speedmB, i1);
        ledcWrite(speedmA, i1);

        Serial.println("direita");
        Serial.println(i1);
      }
      
      if ((i1 * 2 > i2) and (i2 >= i1 / 3)) {

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        if (i2 > i1) {
          motorB = i2;
        } 
        else if (i2 <= i1) {
          motorB = i1;
        }
        
        motorA = motorB/2;

        Serial.println("frente direita");
        Serial.println(motorA);
        Serial.println(motorB);
        
        ledcWrite(speedmB, motorB);
        ledcWrite(speedmA, motorA);
        
      }
    }

    if (i1 < 0) {    //se para esquerda:

      if (-i1 * 2 <= i2) {
        
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

      ledcWrite(speedmB, i2);
      ledcWrite(speedmA, i2);

      Serial.print("frente");
      Serial.println(i2);
      }
            
      if (i2 < -i1 / 3) {    //uma roda para frente e uma para traz:

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        ledcWrite(speedmB, -i1);
        ledcWrite(speedmA, -i1);
        
        Serial.println("esquerda");
        Serial.println(-i1);
      }
      
      if ((-i1 * 2 > i2) and (i2 >= -i1 / 3)) {

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        if (i2 > -i1) {
          motorA = i2;
        } 
        else if (i2 <= -i1) {
          motorA = -i1;
        }
        
        motorB = motorA/2;

        Serial.println("frente esquerda");
        Serial.println(motorA);
        Serial.println(motorB);
    
        ledcWrite(speedmB, motorB);
        ledcWrite(speedmA, motorA);
      }
    }
  }

  if (i2 < 0) {           //as duas para traz:
      
    if (i1 < 0) {     //se para esquerda
 
      if (-i1 * 2 <= -i2) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorA = -i2;

        ledcWrite(speedmB, motorA);
        ledcWrite(speedmA, motorA);
        
        Serial.println("traz");
        Serial.println(motorA);
      }
      
      if (-i2 < -i1 / 3) {    //uma roda para frente e uma para traz:

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        ledcWrite(speedmB, -i1);
        ledcWrite(speedmA, -i1);
        
        Serial.println("esquerda");
        Serial.println(-i1);
      }
      
      if ((-i1 * 2 > -i2) and (-i2 >= -i1 / 3)) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        if (-i2 > -i1) {
          motorB = -i2;
        } 
        else if (-i2 <= -i1) {
          motorB = -i1;
        }
        
        motorA = motorB/2;

        ledcWrite(speedmA, motorA);
        ledcWrite(speedmB, motorB);
        Serial.println("traz esquerda");
        Serial.println(motorA);
        Serial.println(motorB);
      }
    }
    
     if (i1 > 0) {    //se para direita

        if (i1 * 2 <= -i2) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        motorA = -i2;

        ledcWrite(speedmB, motorA);
        ledcWrite(speedmA, motorA);
        
        Serial.println("traz");
        Serial.println(motorB);
      }

      if (-i2 <= i1 / 3) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        ledcWrite(speedmB, i1);
        ledcWrite(speedmA, i1);

        Serial.println("direita");
        Serial.println(i1);
      }
      
      if ((i1 * 2 > -i2) and (-i2 > i1 / 3)) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        if (-i2 > i1) {
          motorB = -i2;
        } 
        else if (-i2 <= i1) {
          motorB = i1;
        }
       
        motorA = motorB / 2;
      
        ledcWrite(speedmA, motorA);
        ledcWrite(speedmB, motorB);
        
      Serial.println("traz direita");
        Serial.println(motorA);
        Serial.println(motorB);
      }
    }
  }
}

}

void onBackwardChange(boolean b) {
  Serial.printf("onBackwardChange: b: %d\n", b);
  digitalWrite(IN2, b ? HIGH : LOW);
  digitalWrite(IN3, b ? HIGH : LOW);

  a = b;

  acelerado = 0;
}
void onForwardChange(boolean b) {

  Serial.printf("onForwardChange: b: %d\n", b);
  digitalWrite(IN1, b ? HIGH : LOW);
  digitalWrite(IN4, b ? HIGH : LOW);

  a = b;

  acelerado = 0;
}
void onLeftChange(boolean b) {
  Serial.printf("onLeftChange: b: %d\n", b);
  digitalWrite(IN1, b ? HIGH : LOW);
  digitalWrite(IN3, b ? HIGH : LOW);
  a = b;

  acelerado = 0;
}
void onRightChange(boolean b) {
  Serial.printf("onRightChange: b: %d\n", b);
  digitalWrite(IN2, b ? HIGH : LOW);
  digitalWrite(IN4, b ? HIGH : LOW);
  a = b;

  acelerado = 0;
}
void onSpeedChange(int32_t i) {
  Serial.printf("onSpeedChange: i: %d\n", i);

  aceleracao = i;
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  remoteMe.getVariables()->observeBoolean("Forward" ,onForwardChange);
  remoteMe.getVariables()->observeBoolean("Backward" ,onBackwardChange);
  remoteMe.getVariables()->observeBoolean("Left" ,onLeftChange);
  remoteMe.getVariables()->observeBoolean("Right" ,onRightChange);
  remoteMe.getVariables()->observeInteger("Speed" ,onSpeedChange);
  remoteMe.getVariables()->observeSmallInteger2("Joystick" ,onJoystickChange);
  remoteMe.getVariables()->observeInteger("WiFi strength" ,onWiFi_strengthChange);

  remoteMe.setConnector(new RemoteMeSocketConnector());
  remoteMe.setDirectConnector(new RemoteMeDirectWebSocketConnector());
  remoteMe.sendRegisterDeviceMessage(DEVICE_NAME);

  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // configure PWM functionalitites
  ledcSetup(speedmA, freq, resolution);
  ledcSetup(speedmB, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENA, speedmA);
  ledcAttachPin(ENB, speedmB);


  ledcWrite(speedmA, 100);
  ledcWrite(speedmB, 100);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, HIGH);
  delay(100);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  delay(50);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, HIGH);
  delay(100);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  delay(50);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, HIGH);
  delay(100);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(50);
 
  ledcWrite(speedmA, 0);
  ledcWrite(speedmB, 0);
}

void loop() {
  remoteMe.loop();

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  if (millis() > lastExecutedMillis1 + 1000) {
    lastExecutedMillis1 = millis();
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
    setWiFi_strength(WiFi.RSSI());
  }

  /////////////////////////////////////////////////////////////////////////////////////////////

  // app de botoes:

  //menos que 100 (rapido)

  if (acelerado < 100) {

    if ((a == 1) & (acelerado < aceleracao)) {

      acelerado = acelerado + 12;

      Serial.print("menos que 500 'rapido' ");
      Serial.println(acelerado);

      ledcWrite(speedmA, acelerado);
      ledcWrite(speedmB, acelerado);
      delay(1);
    }

    if ((a == 1) & (acelerado > aceleracao)) {

      acelerado = aceleracao;

      Serial.print("menos que 500 'rapido' ");
      Serial.println(acelerado);

      ledcWrite(speedmA, acelerado);
      ledcWrite(speedmB, acelerado);
      delay(1);
    }
  }
  // mais que 100 (devagar)

  if (acelerado >= 100) {

    if ((a == 1) & (acelerado < aceleracao)) {

      acelerado = acelerado + 5;

      Serial.print("mais0 que 500 'devagar' ");
      Serial.println(acelerado);

      ledcWrite(speedmA, acelerado);
      ledcWrite(speedmB, acelerado);
      delay(1);
    }

    if ((a == 1) & (acelerado > aceleracao)) {

      acelerado = aceleracao;

      Serial.print("mais0 que 500 'devagar' ");
      Serial.println(acelerado);

      ledcWrite(speedmA, acelerado);
      ledcWrite(speedmB, acelerado);
      delay(1);
    }
  }

}
