// the classroom (less-precise) edition of controlled greenhouse
// THIS CODE / PROJECT MUST USE A LEONARDO -- not an UNO 
// (ANDEEs SPI on an Uno would also use pins 10-13)
#include <SPI.h>
#include "DHT.h"
#include <math.h>
#include <Wire.h>
#include <Servo.h>
#include <Andee.h>
// Backpack Interface labelled "YwRobot Arduino LCM1602 IIC V1"
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#include <LiquidCrystal_I2C.h>

#define OUTER_DHT_PIN      A0
//#define HEATER_PIN         A1  // optional
//#define SOIL_TEMP          A2  // optional
#define SOIL_MOISTURE_PIN  A3
#define OUTER_BRIGHT_PIN   A4
//#define INNER_BRIGHT_PIN   A5  // optional
//#define PROXIMITY_PIN      A5  // optional
//#define SERIAL_RX_PIN       0  // reserved by Serial
//#define SERIAL_TX_PIN       1  // reserved by Serial
//#define I2C_TWI_SDA_PIN     2  // reserved by I2C -- Data
//#define I2C_TWI_SCL_PIN     3  // reserved by I2C -- Clock
#define FAN_DIRECTION       4  // reserved by MOTOR shield & romeo
#define FAN_SPEED           5  // reserved by MOTOR shield & romeo
#define PUMP_SPEED          6  // reserved by MOTOR shield & romeo
#define PUMP_DIRECTION      7  // reserved by MOTOR shield & romeo
//#define ANDEE_PIN         8  // reserved by ANDEE shield
//#define LED_RED_PIN         9  // needed for RGB LED control
//#define LED_GREEN_PIN      10  // needed for RGB LED control
//#define LED_BLUE_PIN       11  // needed for RGB LED control
#define LAMP_BRIGHT_PIN    11  // needed for one pin lamp brigthness
#define WINDOW_PIN         12
#define INNER_DHT_PIN      13

//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
#define DHTTYPE          DHT11   // DHT 11 

DHT inner_dht(INNER_DHT_PIN, DHTTYPE);
DHT outer_dht(OUTER_DHT_PIN, DHTTYPE);

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd( 0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );  // Set the LCD I2C address
//LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

// servo settings
Servo window;

// define inputs for sliders
AndeeHelper titleSetting;
AndeeHelper lampSetting;
AndeeHelper lowLightSetting;
AndeeHelper dayStartSetting;
AndeeHelper nightStartSetting;
AndeeHelper minAirHumiditySetting;
AndeeHelper maxAirHumiditySetting;
AndeeHelper dayAirTempSetting;
AndeeHelper nightAirTempSetting;
AndeeHelper minSoilMoistureSetting; // Discrete input slider
AndeeHelper maxSoilMoistureSetting; // Discrete input slider
AndeeHelper SoilTempSetting;
AndeeHelper nightSoilTempSetting;
AndeeHelper fanRunMinSetting;
AndeeHelper fanOffMinSetting;
AndeeHelper resevoirAlertSetting;
AndeeHelper buttonResetPosition;

// This is where you change your device name:
char newBluetoothName[] = "ClimeCase-Control"; // New device name
char cmdReply[64]; // String buffer
char commandString[100]; // String to store the new device name and device command into one
char user_title_in[32];
char project_title[40] = "ClimeCase-Control";
char project_title_default[40] = "ClimeCase-Control";
char day_start_title[20];
char night_start_title[20];

// We'll use Analog Input Pin A0 to read our analog input.
// Change the pin number if you are using another pin.
const int analogInputPin = A0;
const int soil_moisture_pin = A5;

int   ss_in                     = 0;
int   ss_default                = 0;
int   lamp_in                   = 100;
int   lamp_default              = 100;
int   low_light_default         = 75;
float day_start_default         = 6.5;
int   day_start_hh_default      = 06;
int   day_start_mm_default      = 30;
float night_start_default       = 19.0;
int   night_start_hh_default    = 19;
int   night_start_mm_default    = 30;

int   min_air_humidity_default  = 50;
int   max_air_humidity_default  = 75;
float day_air_temp_default      = 22.0;
float night_air_temp_default    = 14.0;

int   min_soil_moisture_default = 25;
int   max_soil_moisture_default = 75;
float day_soil_temp_default     = 25.0;
float night_soil_temp_default   = 18.0;

float fan_run_min_default       = 5.0;
float fan_off_min_default       = 10.0;
int   resevoir_alert_default    = 20;

int   low_light_in;
float day_start_in;
int   day_start_hh_in;
int   day_start_mm_in;
int   day_start_ss_in;
float night_start_in;
int   night_start_hh_in;
int   night_start_mm_in;
int   night_start_ss_in;
int   min_air_humidity_in;
int   max_air_humidity_in;
float day_air_temp_in;
float night_air_temp_in;
int   min_soil_moisture_in;
int   max_soil_moisture_in;
float soil_temp_in;
float night_soil_temp_in;
float fan_run_min_in;
float fan_off_min_in;
int   resevoir_alert_in;

int wait_delay = 4000;

// define sensing values
float
box_temp,
box_humidity,
room_temp,
room_humidity,
soil_temp,
soil_humidity;

void setup() {
  Andee.begin();
  Andee.clear();

  // We need to combine the new device name with the device command
  sprintf(commandString, "SET BT NAME %s", newBluetoothName);
  // Send command to change device name
  Andee.sendCommand(commandString, cmdReply);
  
  setInitialData(); // Define ANDEE object types and their appearance
  
  Serial.begin( 96000 );
  Serial.println("BioBox Serial Defined:\n");
  delay(wait_delay);  
  config_lcd();  
  lcd.clear();
}

void config_lcd() {
  /*
  // initialize the lcd   
  //lcd.init();                      
  // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.begin(20,4);
  // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++) {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight();
  lcd.home();
  lcd.setCursor(0, 1);
  lcd.print("Hello world");
  lcd.setCursor(0, 2);
  lcd.print("arduinos!");
  */
  
  // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.begin(20,4);
  // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++) {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  Serial.println( "LCD blinked 3 times" );
  // finish with backlight on
  lcd.backlight();   
  //-------- Write characters on the display ------------------
  // NOTE: Cursor Position: (CHAR, LINE) start at 0  
  lcd.clear();
  lcd.setCursor(7,0); //Start at character 8 on line 0
  lcd.print("ClimeCase");
  lcd.setCursor(3,1);
  lcd.print("Student Edition");
  Serial.println("Wrote BioBox header");
  lcd.setCursor(1,3);
  lcd.print("Intializing...");  
  Serial.println("wrote Intializing...");
  lcd.print("LCD");  
  Serial.println( "initializing message on LCD" );
  delay(wait_delay);  
  Serial.println( "Leaving LCD Intialize" );
}

void lcd_display() {
  lcd.setCursor(7,0); //Start at character 8 on line 0
  lcd.print("ClimeCase");
  lcd.setCursor(3,1);
  lcd.print("Student Edition");
  lcd.setCursor(7,3);
  lcd.print("Looping");  
  Serial.println( "LCD - loop" );
}

// This is the function meant to define the types and the apperance of
// all the objects on your smartphone
void setInitialData() {  
  // Set Project Title
  titleSetting.setId(0);
  titleSetting.setType(KEYBOARD_IN); // Sets object as a text input button
  titleSetting.setLocation(0,0,ONE_QUART);
  titleSetting.setTitle( project_title_default );
  titleSetting.setKeyboardType(ALPHA_NUMERIC); 
  
  // LAMP Setting
  lampSetting.setId(10);
  lampSetting.setType(SLIDER_IN);
  lampSetting.setLocation(1,0,ONE_QUART);
  lampSetting.setTitle("Lamp Brightness");
  lampSetting.setSliderMinMax(0, 100, 0); // Display 2 decimal places
  lampSetting.setSliderInitialValue( lamp_default );  // Set slider position to 50
  lampSetting.setSliderNumIntervals(100); // Set to 0 for continuous slider
  lampSetting.setSliderReportMode(ON_VALUE_CHANGE);

  // LIGHT Settings
  lowLightSetting.setId(11);
  lowLightSetting.setType(SLIDER_IN);
  lowLightSetting.setLocation(1,1,ONE_QUART);
  lowLightSetting.setTitle("Lamp Trigger");
  lowLightSetting.setSliderMinMax(0, 100, 0); // Display 2 decimal places
  lowLightSetting.setSliderInitialValue( low_light_default );  // Set slider position to 50
  lowLightSetting.setSliderNumIntervals(100); // Set to 0 for continuous slider
  lowLightSetting.setSliderReportMode(ON_VALUE_CHANGE);
  //Daylight Start
  dayStartSetting.setId(12);
  dayStartSetting.setType(TIME_IN); // Sets object as a time input button
  dayStartSetting.setLocation(1,2,ONE_QUART);
  sprintf(day_start_title, "Day Start: %02d:%02d", day_start_hh_default, day_start_mm_default);
  dayStartSetting.setTitle( day_start_title );
  // Optional Setting. When the user opens up the time picker,
  dayStartSetting.setDefaultTime(day_start_hh_default, day_start_mm_default, ss_default); // Format: hour, minute, second
  //Daylight End
  nightStartSetting.setId(13);
  nightStartSetting.setType(TIME_IN); // Sets object as a time input button
  nightStartSetting.setLocation(1,3,ONE_QUART);
  sprintf( night_start_title, "Night Start: %02d:%02d", night_start_hh_default, night_start_mm_default);
  nightStartSetting.setTitle( night_start_title );
  // Optional Setting. When the user opens up the time picker,
  nightStartSetting.setDefaultTime( night_start_hh_default, night_start_mm_default, ss_default); // Format: hour, minute, second
  
  // AIR CONTROL SETTINGS
  minAirHumiditySetting.setId(20);  
  minAirHumiditySetting.setType(SLIDER_IN); // Set object as a slider
  minAirHumiditySetting.setLocation(2, 0, ONE_QUART); 
  minAirHumiditySetting.setTitle("Min Air Humidity");
  minAirHumiditySetting.setSliderMinMax( 0, 100, 0 );
  minAirHumiditySetting.setSliderInitialValue( min_air_humidity_default );
  minAirHumiditySetting.setSliderNumIntervals(100);
  minAirHumiditySetting.setSliderReportMode(ON_VALUE_CHANGE);
  // Let's draw a continuous slider! ///////////////////////////////////////
  maxAirHumiditySetting.setId(21);
  maxAirHumiditySetting.setType(SLIDER_IN);
  maxAirHumiditySetting.setLocation(2,1,ONE_QUART);
  maxAirHumiditySetting.setTitle("Max Air Humidity");
  maxAirHumiditySetting.setSliderMinMax(0, 100, 0); // Display 2 decimal places
  maxAirHumiditySetting.setSliderInitialValue( max_air_humidity_default );  // Set slider position to 50
  maxAirHumiditySetting.setSliderNumIntervals(100); // Set to 0 for continuous slider
  maxAirHumiditySetting.setSliderReportMode(ON_VALUE_CHANGE);
  //Day Air Temp
  dayAirTempSetting.setId(22);
  dayAirTempSetting.setType(SLIDER_IN);
  dayAirTempSetting.setLocation(2,2,ONE_QUART);
  dayAirTempSetting.setTitle("Day Air Temp");
  dayAirTempSetting.setSliderMinMax(-40, 50, 1); // Display 2 decimal places
  dayAirTempSetting.setSliderInitialValue( day_air_temp_default );  // Set slider position to 50
  dayAirTempSetting.setSliderNumIntervals(0); // Set to 0 for continuous slider
  dayAirTempSetting.setSliderReportMode(ON_VALUE_CHANGE);
  //Night Air Temp
  nightAirTempSetting.setId(23);
  nightAirTempSetting.setType(SLIDER_IN);
  nightAirTempSetting.setLocation(2,3,ONE_QUART);
  nightAirTempSetting.setTitle("Night Air Temp");
  nightAirTempSetting.setSliderMinMax(-40, 50, 1); // Display 2 decimal places
  nightAirTempSetting.setSliderInitialValue( night_air_temp_default );  // Set slider position to 50
  nightAirTempSetting.setSliderNumIntervals(0); // Set to 0 for continuous slider
  nightAirTempSetting.setSliderReportMode(ON_VALUE_CHANGE);

  // SOIL SETTINGS
  minSoilMoistureSetting.setId(1);  
  minSoilMoistureSetting.setType(SLIDER_IN); // Set object as a slider
  minSoilMoistureSetting.setLocation(0, 1, ONE_QUART); 
  minSoilMoistureSetting.setTitle("Min Soil Moisture");
  minSoilMoistureSetting.setSliderMinMax( 0, 100, 0 );
  minSoilMoistureSetting.setSliderInitialValue( min_soil_moisture_default );
  minSoilMoistureSetting.setSliderNumIntervals(100);
  minSoilMoistureSetting.setSliderReportMode(ON_VALUE_CHANGE);
  minSoilMoistureSetting.setSliderColor(THEME_GREY_DARK); // Slider bar colour
  minSoilMoistureSetting.setColor(THEME_GREY); // Slider background colour
  // Let's draw a continuous slider! ///////////////////////////////////////
  maxSoilMoistureSetting.setId(2);
  maxSoilMoistureSetting.setType(SLIDER_IN);
  maxSoilMoistureSetting.setLocation(0,2,ONE_QUART);
  maxSoilMoistureSetting.setTitle("Max Soil Moisture");
  maxSoilMoistureSetting.setSliderMinMax(0, 100, 0); // Display 2 decimal places
  maxSoilMoistureSetting.setSliderInitialValue( max_soil_moisture_default );  // Set slider position to 50
  maxSoilMoistureSetting.setSliderNumIntervals(100); // Set to 0 for continuous slider
  maxSoilMoistureSetting.setSliderReportMode(ON_FINGER_UP);
  maxSoilMoistureSetting.setSliderColor(THEME_MIDNIGHT_DARK);
  maxSoilMoistureSetting.setColor(THEME_MIDNIGHT);
  //Day Soil Temp
  SoilTempSetting.setId(3);
  SoilTempSetting.setType(SLIDER_IN);
  SoilTempSetting.setLocation(0,3,ONE_QUART);
  SoilTempSetting.setTitle("Soil Temp");
  SoilTempSetting.setSliderMinMax(-40, 50, 1); // Display 2 decimal places
  SoilTempSetting.setSliderInitialValue( day_soil_temp_default );  // Set slider position to 50
  SoilTempSetting.setSliderNumIntervals(0); // Set to 0 for continuous slider
  SoilTempSetting.setSliderReportMode(ON_VALUE_CHANGE);
  
  //Fan on (min)
  fanRunMinSetting.setId(30);
  fanRunMinSetting.setType(SLIDER_IN);
  fanRunMinSetting.setLocation(3,0,ONE_QUART);
  fanRunMinSetting.setTitle("Fan On (mins)");
  fanRunMinSetting.setSliderMinMax(0, 60, 1); // Display 2 decimal places
  fanRunMinSetting.setSliderInitialValue( fan_run_min_default );  // Set slider position to 50
  fanRunMinSetting.setSliderNumIntervals(0); // Set to 0 for continuous slider
  fanRunMinSetting.setSliderReportMode(ON_VALUE_CHANGE); 
  // Fan off (min)
  fanOffMinSetting.setId(31);
  fanOffMinSetting.setType(SLIDER_IN);
  fanOffMinSetting.setLocation(3,1,ONE_QUART);
  fanOffMinSetting.setTitle("Fan Off (mins)");
  fanOffMinSetting.setSliderMinMax(0, 60, 1); // Display 2 decimal places
  fanOffMinSetting.setSliderInitialValue( fan_off_min_default );  // Set slider position to 50
  fanOffMinSetting.setSliderNumIntervals(0); // Set to 0 for continuous slider
  fanOffMinSetting.setSliderReportMode(ON_VALUE_CHANGE);
 
  // Resovoir Alert Level (%)
  resevoirAlertSetting.setId(32);
  resevoirAlertSetting.setType(SLIDER_IN);
  resevoirAlertSetting.setLocation(3,2,ONE_QUART);
  resevoirAlertSetting.setTitle("Resevoir Alert");
  resevoirAlertSetting.setSliderMinMax(0, 100, 0); // Display 2 decimal places
  resevoirAlertSetting.setSliderInitialValue( resevoir_alert_default );  // Set slider position to 50
  resevoirAlertSetting.setSliderNumIntervals(100); // Set to 0 for continuous slider
  resevoirAlertSetting.setSliderReportMode(ON_VALUE_CHANGE);
  
  // Button to reset slider positions 
  buttonResetPosition.setId(33); // Don't forget to assign a unique ID number
  buttonResetPosition.setType(BUTTON_IN); // Defines object as a button
  buttonResetPosition.setLocation(3,2,ONE_QUART);
  buttonResetPosition.setTitle("Reset 2 Defaults");
  buttonResetPosition.setColor(THEME_RED);
  buttonResetPosition.setTitleColor(THEME_RED_DARK);
  
}

// Arduino will run instructions here repeatedly until you power it off.
void loop() {
  
  lcd_display();
  
  if( buttonResetPosition.isPressed() ) {
    buttonResetPosition.ack();
    titleSetting.setTitle( project_title_default );
    lowLightSetting.moveSliderToValue(low_light_default);
    //dayStartSetting.moveSliderToValue( day_start_default );
    sprintf( day_start_title, "Day Start: %02d:%02d", day_start_hh_default, day_start_mm_default);
    dayStartSetting.setTitle( day_start_title );
    dayStartSetting.setDefaultTime(day_start_hh_default, day_start_mm_default, ss_default); // Format: hour, minute, secon
    //nightStartSetting.moveSliderToValue( night_start_default );
    sprintf( night_start_title, "Night Start: %02d:%02d", night_start_hh_default, night_start_mm_default);
    dayStartSetting.setTitle( day_start_title );
    dayStartSetting.setDefaultTime(night_start_hh_default, night_start_mm_default, ss_default); // Format: hour, minute, secon
    //
    lampSetting.moveSliderToValue( lamp_default );
    minAirHumiditySetting.moveSliderToValue( min_air_humidity_default );
    maxAirHumiditySetting.moveSliderToValue( max_air_humidity_default );
    dayAirTempSetting.moveSliderToValue( day_air_temp_default );
    nightAirTempSetting.moveSliderToValue( night_air_temp_default );
    minSoilMoistureSetting.moveSliderToValue( min_soil_moisture_default );
    maxSoilMoistureSetting.moveSliderToValue( max_soil_moisture_default );
    SoilTempSetting.moveSliderToValue( day_soil_temp_default );
    nightSoilTempSetting.moveSliderToValue( night_soil_temp_default );
    fanRunMinSetting.moveSliderToValue( fan_run_min_default );
    fanOffMinSetting.moveSliderToValue( fan_off_min_default );
    resevoirAlertSetting.moveSliderToValue( resevoir_alert_default );
  } 
  
  if ( titleSetting.isPressed() ) {
    memset( user_title_in, 0x00, 32 ); // Empty the contents of the string before receiving user input
    titleSetting.ack(); // Acknowledge button press or else phone will be left waiting
    titleSetting.getKeyboardMessage( user_title_in ); // Display keyboard and store input into userInput
    sprintf( project_title, "Title: %s", user_title_in );
    titleSetting.setTitle( project_title );   
    sprintf(commandString, "SET BT NAME %s", user_title_in );
    Andee.sendCommand( commandString, cmdReply );
  }
  titleSetting.update();
  
  if( dayStartSetting.isPressed() ) { 
    dayStartSetting.ack();
    // Display time picker and store it in the three variables - hh, mm, and ss
    dayStartSetting.getTimeInput(&day_start_hh_in, &day_start_mm_in, &ss_in); 
    // String the three variables into a single time string
    sprintf(day_start_title, "Day Start: %02d:%02d", day_start_hh_in, day_start_mm_in);
    dayStartSetting.setTitle( day_start_title );
    // Optional Setting. When the user opens up the time picker,
    dayStartSetting.setDefaultTime(day_start_hh_in, day_start_mm_in, ss_default); // Format: hour, minute, second
  }
  dayStartSetting.update();
  
  if( nightStartSetting.isPressed() ) { 
    nightStartSetting.ack();
    // Display time picker and store it in the three variables - hh, mm, and ss
    nightStartSetting.getTimeInput(&night_start_hh_in, &night_start_mm_in, &ss_in); 
    // String the three variables into a single time string
    sprintf( night_start_title, "Night Start: %02d:%02d", night_start_hh_in, night_start_mm_in);
    nightStartSetting.setTitle( night_start_title );
    // Optional Setting. When the user opens up the time picker,
    nightStartSetting.setDefaultTime(night_start_hh_in, night_start_mm_in, ss_default); // Format: hour, minute, second
  }
  nightStartSetting.update();
  
  lamp_in               = lampSetting.getSliderValue( INT );
  low_light_in          = lowLightSetting.getSliderValue( INT );
  day_start_in          = dayStartSetting.getSliderValue( FLOAT );
  night_start_in        = nightStartSetting.getSliderValue( FLOAT );
  min_air_humidity_in   = minAirHumiditySetting.getSliderValue( INT ); // Retrieve integer value
  max_air_humidity_in   = maxAirHumiditySetting.getSliderValue( INT ); // Retrieve float value
  day_air_temp_in       = dayAirTempSetting.getSliderValue( FLOAT );
  night_air_temp_in     = nightAirTempSetting.getSliderValue( FLOAT );
  min_soil_moisture_in  = minSoilMoistureSetting.getSliderValue( INT ); // Retrieve integer value
  max_soil_moisture_in  = maxSoilMoistureSetting.getSliderValue( INT ); // Retrieve float value
  soil_temp_in          = SoilTempSetting.getSliderValue( FLOAT );
  night_soil_temp_in    = nightSoilTempSetting.getSliderValue( FLOAT );
  fan_run_min_in        = fanRunMinSetting.getSliderValue( FLOAT );
  fan_off_min_in        = fanOffMinSetting.getSliderValue( FLOAT );
  resevoir_alert_in     = resevoirAlertSetting.getSliderValue( INT );
  
  lampSetting.update();
  lowLightSetting.update();
  dayStartSetting.update();
  nightStartSetting.update();
  minAirHumiditySetting.update();
  maxAirHumiditySetting.update();
  dayAirTempSetting.update();
  nightAirTempSetting.update();
  minSoilMoistureSetting.update();
  maxSoilMoistureSetting.update();
  SoilTempSetting.update();
  nightSoilTempSetting.update();
  fanRunMinSetting.update();
  fanOffMinSetting.update();
  resevoirAlertSetting.update();
  buttonResetPosition.update();
  
  delay(1000); // Always leave a short delay for Bluetooth communication
}
