#define WIFI_NAME "JoseRenato_2G"
#define WIFI_PASSWORD "09011971"
#define DEVICE_ID 12
#define DEVICE_NAME "Esp1"
#define TOKEN "~991902_g5R8vugG7opMLNjBWyy8uPHW"


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

int F;
int B;
int R;
int L;

RemoteMe& remoteMe = RemoteMe::getInstance(TOKEN, DEVICE_ID);

//*************** CODE FOR COMFORTABLE VARIABLE SET *********************

inline void setBackward(boolean b) {remoteMe.getVariables()->setBoolean("Backward", b); }
inline void setForward(boolean b) {remoteMe.getVariables()->setBoolean("Forward", b); }
inline void setLeft(boolean b) {remoteMe.getVariables()->setBoolean("Left", b); }
inline void setRight(boolean b) {remoteMe.getVariables()->setBoolean("Right", b); }
inline void setSpeed(int32_t i) {remoteMe.getVariables()->setInteger("Speed", i); }

//*************** IMPLEMENT FUNCTIONS BELOW *********************

void onBackwardChange(boolean b) {
  Serial.printf("onBackwardChange: b: %d\n",b);
   digitalWrite(IN1, b?HIGH:LOW);
   digitalWrite(IN4, b?HIGH:LOW);
}
void onForwardChange(boolean b) {
  Serial.printf("onForwardChange: b: %d\n",b);
  digitalWrite(IN2, b?HIGH:LOW);
  digitalWrite(IN3, b?HIGH:LOW);
}
void onLeftChange(boolean b) {
  Serial.printf("onLeftChange: b: %d\n",b);
  digitalWrite(IN2, b?HIGH:LOW);
  digitalWrite(IN4, b?HIGH:LOW);
}
void onRightChange(boolean b) {
  Serial.printf("onRightChange: b: %d\n",b);
  digitalWrite(IN1, b?HIGH:LOW);
  digitalWrite(IN3, b?HIGH:LOW);
}
void onSpeedChange(int32_t i) {
  Serial.printf("onSpeedChange: i: %d\n",i);
  analogWrite(ENA, i); 
  analogWrite(ENB, i);
}




void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

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
}
