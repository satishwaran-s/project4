/*
 * rosserial publisher
 * collate US vals from n snsrs
 * then sort out logic if robot needs stop
 */

#include <ros.h>
#include <std_msgs/String.h>

struct us_snsr {
	// position of the USsnsr - this is to clarify things!
	char pos [8];
	// pin configs
	int trigPin;
	int echoPin;
	// relevant data used
	long duration = 0;
	long distance = 0;
};

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher ussnsr_info("ultrasonic_sensor_info", &str_msg);

static us_snsr snsr_arr[] = {
	{"Front\0", 22, 23}
};

void USinit (us_snsr us) {
	pinMode (us.trigPin, OUTPUT);
	pinMode (us.echoPin, INPUT);
}

void USdist (us_snsr us) {
	digitalWrite (us.trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite (us.trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite (us.trigPin, LOW);

	us.duration = pulseIn (echoPin, HIGH);
	us.distance = (duration/2) / 29.1;
}

void setup()
{
	//USinit ();
	nh.initNode();
	nh.advertise(ussnsr_info);
}

void loop()
{
	char distStr[5] = "test\0";
	//itoa(distance, distStr, 10);
  
	str_msg.data = distStr;
	ussnsr_info.publish( &str_msg );
	nh.spinOnce();
	delay(1000);
}
