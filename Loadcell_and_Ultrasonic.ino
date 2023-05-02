#include <Arduino.h>
#include "HX711.h"
#include <ros.h>
#include <std_msgs/Bool.h>

#define NUM_SNSR 4
#define PRINT_FREQ 1000

struct us_snsr{
  // pin configs
  int trigPin;
  int echoPin;
  // relevant data used
  long duration = 0;
  long distance = 0;
};

// sensor objs
us_snsr front;
us_snsr back;
us_snsr left;
us_snsr right;
// add to ptr arr
us_snsr* snsr_arr[NUM_SNSR] = {&front, &back, &left, &right};

// flag vars
bool is_obstructed = 0;
// 10 is default... 0 is taken for front :/
int obstructed_dir = 10;

unsigned long time_now = 0;

const int timepassed = 3000;
unsigned long lastTimePublish = 0;

const int weightlimit = 5500; //max weight capped at 5.5kg

void USinit (us_snsr us) {
  pinMode (us.trigPin, OUTPUT);
  pinMode (us.echoPin, INPUT);
}

void USdist (us_snsr* us) {
  // off trigPin pulse
  digitalWrite (us->trigPin, LOW);
  delayMicroseconds(2);
  // generate 10ms pulse
  digitalWrite (us->trigPin, HIGH);
  delayMicroseconds(10);
  // pulse off
  digitalWrite (us->trigPin, LOW);

  //measure pulse duration from echoPin
  us->duration = pulseIn (us->echoPin, HIGH);
  // calc distance
  us->distance = (us->duration/2) / 29.1;
}

void dirStr (int i) {
  switch(i){
    case(0): 
      Serial.print("front\0");
      break;
    case(1):
      Serial.print("back\0");
      break;
    case(2):
      Serial.print("left\0");
      break;
    case(3):
      Serial.print("right\0");
      break;
    default:
      Serial.print("error\0");
      break;
  }
}

ros::NodeHandle nh; //rose node handler
std_msgs::Bool limit_value;
std_msgs::Bool is_obstructed_bool;
ros::Publisher Ultra("Ultra", &is_obstructed_bool);
ros::Publisher Limit("Limit", &limit_value);

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

void setup() {
  front.trigPin = 22;
  front.echoPin = 23;
  back.trigPin = 24;
  back.echoPin = 25;
  left.trigPin = 26;
  left.echoPin = 27;
  right.trigPin = 28;
  right.echoPin = 29;
  for (int i = 0; i < NUM_SNSR; i++)
    USinit (*snsr_arr[i]);
  
  nh.initNode(); //initialise ros node
  nh.advertise(Limit); //advertise LOADCELL publisher

  nh.initNode();
  nh.advertise(Ultra); //advertise ULTRA publisher

  Serial.begin(57600); //baud rate
  //Serial.println("HX711 Demo");
  //Serial.println("Initializing the scale");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.println(scale.read());      // print a raw reading from the ADC
  scale.set_scale(224.27);
  scale.tare();               // reset the scale to 0
  //Serial.println("Readings:");
}

void loop() {
  //LOADCELL
  unsigned long currentTime = millis();

  if(currentTime - lastTimePublish >= timepassed) {
    lastTimePublish = currentTime;
    int limit = 0;
    //Serial.print("one reading:\t");
    Serial.println(scale.get_units(), 1);

    if (scale.get_units(10) > weightlimit) 
    {
      limit_value.data = true;
      //Serial.println("LIMIT REACHED");
    }
    else
    {
      limit_value.data = false;
    }

    Limit.publish(&limit_value);
    nh.spinOnce(); //this function handles incoming messages on ROS topics and services
  }


  //ULTRASENSOR
  if (millis() > time_now + PRINT_FREQ) {
    time_now = millis();
    if (is_obstructed == 0) {
      is_obstructed_bool.data = 0;
      Ultra.publish( &is_obstructed_bool );
      nh.spinOnce();
    }
  }

    // distance measuring behaviour
  for (int i = 0; i < NUM_SNSR; i++){
    USdist (snsr_arr[i]);
    if (snsr_arr[i]->distance < 50){
      is_obstructed = 1;
      obstructed_dir = i;
      // send obstructed message to ROS machine
      is_obstructed_bool.data = 1;
      Ultra.publish( &is_obstructed_bool );
      nh.spinOnce();
    } else {
      // reset values
      is_obstructed = 0;
      obstructed_dir = 10;
    }
    delayMicroseconds(25);
  }
}
