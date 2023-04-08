/*
 * serial publisher
 * collate US vals from n snsrs
 * then send out signal if (virtual) robot needs stop
 */
// includes

#include <ros.h>
#include <std_msgs/Bool.h>

// defines

#define NUM_SNSR 4
#define PRINT_FREQ 1000

// structs

struct us_snsr{
	// pin configs
	int trigPin;
	int echoPin;
	// relevant data used
	long duration = 0;
	long distance = 0;
};

// functions

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

// global vars

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

// non-blking delay vars
unsigned long time_now = 0;

// ros objs
ros::NodeHandle  nh;
std_msgs::Bool is_obstructed_bool;
ros::Publisher Ultra("Ultra", &is_obstructed_bool);


// init code

void setup()
{
  // USsnsr inits.
  // The structs can't be inited outside, so they here.
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
    
  nh.initNode();
  nh.advertise(Ultra);
}

void loop()
{
  // if not obstructed - tell ros that surroundings are clear
  // however this will unecessarily spam when nthing is around
  // so is published every second
  if (millis() > time_now + PRINT_FREQ) {
    time_now = millis();
    if (is_obstructed == 0) {
      is_obstructed_bool.data = 0;
      Ultra.publish( &is_obstructed_bool );
      nh.spinOnce();
    }
  }
  

  // if obstructed behaviour
  /*
  if (is_obstructed == 1) {
    // tell motor to halt movement
    // halt movement should hold for ~5 seconds
    // ^ is it possible to only affect motors and let mcu resume the dist measuring?
    delay(5000);
    // mcu resumes to measure dist, if obs still there it will flag again.
    // ^ possible to tell motor to hold on a little more?
    is_obstructed = 0;
    obstructed_dir = 10;
  }
  */
  
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

  // debugging - show current status every 1000ms
  /*
  if(millis() > time_now + PRINT_FREQ) {
    time_now = millis();
    for (int i = 0; i < NUM_SNSR; i++){
      dirStr(i);
      Serial.print(snsr_arr[i]->distance);
      Serial.print('\n');
    }
   if(is_obstructed == 1){
    Serial.print("Obstructed!\n");
    Serial.print("Direction: ");
    dirStr(obstructed_dir);
    Serial.print('\n');
    }
  }
  */
}
