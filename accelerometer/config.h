#define ACTIVATION_MODE     		ABP
#define CLASS						        CLASS_A
#define SPREADING_FACTOR    		7
#define ADAPTIVE_DR         		true
#define CONFIRMED           		false
#define PORT                		15

#define SEND_BY_PUSH_BUTTON 		false
#define FRAME_DELAY         		10000
//#define PAYLOAD_HELLO				    true
//#define PAYLOAD_TEMPERATURE    	false
//#define CAYENNE_LPP_         		false
#define LOW_POWER           		false

String devEUI = "70B3D57ED005BD23";

// Configuration for ABP Activation Mode
String devAddr = "260BF792";
String nwkSKey = "54A76EBAF7FA3C86B630F403274C1B3A";
String appSKey = "A5E5C9B3A4C1418F38B4EEFE3AA08A96";

// Configuration for OTAA Activation Mode
String appKey = "00000000000000000000000000000000"; 
String appEUI = "0000000000000000";