#define ACTIVATION_MODE     		OTAA
#define CLASS						CLASS_A
#define SPREADING_FACTOR    		9
#define ADAPTIVE_DR         		true
#define CONFIRMED           		false
#define PORT                		1

#define SEND_BY_PUSH_BUTTON 		false
#define FRAME_DELAY         		10000
//#define PAYLOAD_HELLO				  true
//#define PAYLOAD_TEMPERATURE   false
//#define CAYENNE_LPP_          false
#define LOW_POWER           		false

String devEUI = "0000000000000000";

// Configuration for ABP Activation Mode
String devAddr = "00000000";
String nwkSKey = "00000000000000000000000000000000";
String appSKey = "00000000000000000000000000000000";

// Configuration for OTAA Activation Mode
String appKey = "00000000000000000000000000000000"; 
String appEUI = "0000000000000000";
