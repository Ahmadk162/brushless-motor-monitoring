#include <PID_v1_bc.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

// PWM config
double pwm_cmd2 = 0;
int pwm_cmd1=0;
int pwm_percentage_1, pwm_percentage_2 = 0;

//variable of PWM
int dc=0;
int ac=0;

//pwm valeu
char pwm1[8]={'0','0','0','0','0','0','0','0'};
char pwm2[8]={'0','0','0','0','0','0','0','0'};

// Update these with values suitable for your network.
const char* ssid = "your wifi ssid";
const char* password = "your wifi password";
const char* mqtt_server = "your mqtt server link";
#define mqtt_port your_MQTT_port
#define MQTT_USER "your_mqtt_user_name"
#define MQTT_PASSWORD "your_mqqtt_password"
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_PUBLISH_CH_pwm1 "/icircuit/ESP32/serialdata/tx/pwm1"
#define MQTT_SERIAL_PUBLISH_CH_pwm2 "/icircuit/ESP32/serialdata/tx/pwm2"
#define MQTT_SERIAL_PUBLISH_CH_speed "/icircuit/ESP32/serialdata/tx/speed"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"
#define MQTT_SERIAL_RECEIVER_CH_pwm1 "/icircuit/ESP32/serialdata/rx/pwm1"
#define MQTT_SERIAL_RECEIVER_CH_pwm2 "/icircuit/ESP32/serialdata/rx/pwm2"
#define MQTT_SERIAL_RECEIVER_motor_speed "/icircuit/ESP32/serialdata/rx/motor/speed"
#define MQTT_SERIAL_RECEIVER_kp "/icircuit/ESP32/serialdata/rx/pid/kp"
#define MQTT_SERIAL_RECEIVER_kd "/icircuit/ESP32/serialdata/rx/pid/kd"
#define MQTT_SERIAL_RECEIVER_ki "/icircuit/ESP32/serialdata/rx/pid/ki"
#define Vac 32
#define PWM_ch1 5
#define PWM_ch2 16
#define hall_sensor 34

const char* test_root_ca= \
          "-----BEGIN CERTIFICATE-----\n"\
      "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\n" \
      "-----END CERTIFICATE-----\n";

WiFiClientSecure wifiClient;
//WiFiClient wifiClient;
PubSubClient client(wifiClient);
//varial used for interrupt and motor speed
char speed_of_motor[8]={'0','0','0','0','0','0','0','0'};
double            speed_of_motor_0;
double            speed_of_motor_in=0;
float             speed_of_motor_1=0;
const byte        interruptPin = 23;              // Assign the interrupt pin
volatile uint64_t StartValue;                     // First interrupt value
volatile uint64_t PeriodCount;                    // period in counts of 0.000001 of a second
float             Freq;                           // frequency     
float             Freq_1;                           // frequency     
char              str[21];                        // for printing uint64_t values
#define R 0.2
#define nb_mag 30

float time_tuning=0;
boolean x=0;
boolean y=0;
int i=0;
//int NewSampleTime=100;
volatile uint64_t PeriodCount_1=0;
hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t 
hw_timer_t * My_timer = NULL;                        // pointer to a variable of type hw_timer_t 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?

//PID controller
//Specify the links and initial tuning parameters
double Kp=30, Ki=1, Kd=0.1;
PID myPID(&speed_of_motor_0, &pwm_cmd2, &speed_of_motor_in, Kp, Ki, Kd, DIRECT);
int WindowSize = 255;
unsigned long windowStartTime;
void IRAM_ATTR function_ISR() 
{  
      uint64_t TempVal= timerRead(timer);         // value of timer at interrupt
      PeriodCount= TempVal - StartValue;          // period count between rising edges in 0.000001 of a second
      StartValue = TempVal;                       // puts latest reading as start for next calculation
}
void IRAM_ATTR onTimer() 
{  
       x=1;
}
void setup() {
  Serial.begin(115200);
  
  Serial.setTimeout(500);// Set time out for 
  setup_wifi();
  wifiClient.setCACert(test_root_ca);  //test_root_ca 
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  client.publish(MQTT_SERIAL_PUBLISH_CH_pwm1, pwm1);
  
  pinMode(interruptPin, INPUT_PULLUP);                                            // sets pin high
  //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING); // attaches pin to interrupt on Falling Edge
  attachInterrupt(digitalPinToInterrupt(interruptPin), function_ISR, RISING); // attaches pin to interrupt on Falling Edge
  timer = timerBegin(0, 80, true);                                                // this returns a pointer to the hw_timer_t global variable
  timerStart(timer);                                                              // starts the timer

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 100000, true);
  timerAlarmEnable(My_timer);

  windowStartTime = millis();
  //initialize the variables we're linked to
  speed_of_motor_in = 0;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  //myPID.SetSampleTime(NewSampleTime);
  myPID.SetMode(AUTOMATIC);
}

//loop
void loop() { 
  Freq =1000000.0/PeriodCount;
  if(PeriodCount==0)
  {
    Freq=1;
  }
  Freq/=30;
  speed_of_motor_1=Freq*22.6*R;  
  if (speed_of_motor_1>=1 && speed_of_motor_1<=100)
  {
    speed_of_motor_0=speed_of_motor_1;
    if(x==1)
    {
      time_tuning+=0.1;
      Serial.print(time_tuning);
      Serial.print(",");
      //Serial.print(pwm_cmd2);
      //Serial.print(",");
      Serial.println(speed_of_motor_0);
      //Serial.print(",");
      //Serial.println(speed_of_motor_in);
      x=0;
    }    
    dtostrf(speed_of_motor_0, 1, 2, speed_of_motor);
    client.publish(MQTT_SERIAL_PUBLISH_CH_speed, speed_of_motor);
    myPID.Compute();
    analogWrite(PWM_ch2,pwm_cmd2);
    }
   client.loop();

   if (Serial.available() > 0) {
     char mun[501];
     memset(mun,0, 501);
     Serial.readBytesUntil( '\n',mun,500);
     publishSerialData(mun);
   }
 }
