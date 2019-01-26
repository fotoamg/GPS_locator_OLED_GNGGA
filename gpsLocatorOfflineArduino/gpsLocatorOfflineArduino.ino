
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <SoftwareSerial.h>

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

/*  setup tuning variables: */
const byte minSats = 7;
const unsigned int maxValidDist = 8000;
const unsigned int screenUpdateMillis = 1000;
const unsigned int linkAlert = 8000;
const boolean serialDebug = true;

/*  config variables: */
/* 2,3 should be left out for sofserial... when upgraded with home gps unit*/
const byte SOFT_RX = 2; // onboard GPS receiver pin
const byte SOFT_TX = 3;
const byte BTN_PIN = 5;
const byte SAT_PIN = 6;
const byte HOMESET_PIN = 7;
const byte LINK_PIN = 8;
const byte BUZZER_PIN = 9;
const char SPC = ' ';
const char COMMA = ',';

SoftwareSerial gpsSerial(SOFT_RX, SOFT_TX);

/*  debug variables: */
// setup-ban hívom ezekkel meg mivel a hw serialon nincs jelenleg GPS bekötve
/*
const String ggaMsgHEAD = "$GPGGA,212937.00,";
const String ggaMsgOtthon    = "4614.70289,N,02007.73038,E"; // otthon
const String ggaMsgMatyaster = "4614.42873,N,02008.12940,E"; // matyaster
const String ggaMsgUton      = "4614.62873,N,02008.00040,E"; // valahol kozte
const String ggaMsgFix = ",1,"; // 3D fix means 1
const String ggaMsgSat = "09"; // nr of sats
const String ggaMsgHdop = ",2.13,";
const String ggaMsgAlt1 = "83.2,";
const String ggaMsgAlt2 = "392.4,";
const String ggaMsgTAIL = "M,37.7,M,,*";
*/
// alábbi teszt sorok felvannak fentebb darabolva hogy kevesebb memoria menjen el a stringre késöbb konkatenálom akkor
// már nem fagy szét....
//String ggaMsgExample2 = "$GNGGA,130554.80,4614.42873,N,02008.12940,E,1,09,1.61,392.4,M,37.7,M,,*45";// Matyaster
//String ggaMsgExample3 = "$GNGGA,130554.80,4614.52873,N,02007.71040,E,1,09,1.61,392.4,M,37.7,M,,*47";

/*  status variables: */
double HOME_LAT = 46.24505;
double HOME_LON = 20.12884;
int HOME_ALT = 83;
boolean homeSet = false;
unsigned long homeSetMilllis = millis();
unsigned long lastScreenUpdate = millis();
unsigned long lastLink = millis();
unsigned long lastRadio = millis();
unsigned long lastGPS = millis();

/* stat counters*/
unsigned int radioRestart = 0;
unsigned int gpsRestart = 0;
unsigned int radioChkFail = 0;
unsigned int gpsChkFail = 0;
String lastProcMsg = "";

/* Button action */
byte buttonPushCounter = 0;   // counter for the number of button presses
boolean buttonState = false;         // current state of the button
boolean lastButtonState = false;     // previous state of the button
unsigned long buttonBounceEnd = millis() + 500;

/* tracked device most important data */
double last_valid_lat = 46.24505;
double last_valid_lon = 20.12884;
float last_valid_alt = 83.0;
byte last_radio_sat = 0;

/* variables holding parsed message BEG */
double msg_lat = 46.24505;
double msg_lon = 20.12884;
int msg_alt = 83;
int msg_sat = 0;
String msg_utc = "";
/* variables holding parsed message END */

int relative_alt = 0;

/* home GPS unit collected data */
double gps_lat = 46.24505;
double gps_lon = 20.12884;
byte gps_sat = 0;

/* serial parsing buffer of home GPS */
char inDataGps[85];
byte indexGps = 0;
unsigned int gpsCheck = 0;
boolean gpsStar = false;

/* serial parsing buffer for radio link */
char inData[85];
byte index = 0;
unsigned int inCheck = 0;
boolean checkStar = false;


void setup() {
  pinMode(BTN_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(SAT_PIN, OUTPUT);
  digitalWrite(SAT_PIN, LOW);

  pinMode(LINK_PIN, OUTPUT);
  digitalWrite(LINK_PIN, LOW);

  pinMode(HOMESET_PIN, OUTPUT);
  digitalWrite(HOMESET_PIN, LOW);

  Wire.begin();
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.set400kHz();
  //oled.setFont(Adafruit5x7);
  oled.setFont(ZevvPeep8x16);
  oled.clear();
  // oled.println("------------------");
  oled.println("  INITIALIZING");
  // oled.println("------------------");

  Serial.begin(9600);
  // Serial.println("SETUP!!! ");

  // az alábbiak élesben kikerülnek mert a HW serialon is lesz egy gps link, de most nem tudok 2 helyen lenni egyszerre így mockolom

  /* test messages to mock radio GPS coords... */
  // ezt a hívást kikellett szednem mert nem birta memoriával:
/*
  parseSentence(ggaMsgHEAD + ggaMsgOtthon + ggaMsgFix +  ggaMsgSat +  ggaMsgHdop + ggaMsgAlt1 + ggaMsgTAIL);// + "6A", 106);
  trackedSentenceCheck();
  processStatus();

  parseSentence(ggaMsgHEAD + ggaMsgMatyaster + ggaMsgFix +  ggaMsgSat +  ggaMsgHdop + ggaMsgAlt2 + ggaMsgTAIL);//  + "5F", 95);
  //parseSentence("$GNGGA,130554.80,4614.42873,N,02008.12940,E,1,09,1.61,392.4,M,37.7,M,,*73");
  trackedSentenceCheck();

  processStatus();

  // ezt a hívást kikellett szednem mert nem birta memoriával:
  parseSentence(ggaMsgHEAD + ggaMsgUton + ggaMsgFix +  ggaMsgSat +  ggaMsgHdop + ggaMsgAlt1 + ggaMsgTAIL);//  + "62", 98);
  gpsSentenceCheck();
  */  
  
 /* parseSentence("$GPGGA,212937.00,4614.70289,N,02007.73038,E,1,05,2.13,83.2,M,37.7,M,,*66");
  gpsSentenceCheck();
  processStatus();*/
  
  /* comment it out in real test until this line !!! */

  gpsSerial.begin(9600);
}

void loop() { //$GPGGA,212937.00,4614.70289,N,02007.73038,E,1,05,2.13,83.2,M,37.7,M,,*66 //otthon
        ////Serial.print(".");
  char aChar;
  /* checking tracked GPS BEG */
  while (Serial.available() > 0)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    // buzzer hangjelzés csipog ahogy jönnek az adatok a cél tárgytól így hallani hogy van adat kapcsolat
    // az FPV sisakban is mivel ez egy dron nyomkövető rendszer
    // //Serial.println(" AVAIL! ");
    aChar = Serial.read();
    if (serialDebug) {
        Serial.write(aChar);
       // Serial.print(inCheck);
    }
    if (aChar == '$')
    {
      if (index >42) { // possibly recoverable data
        String tempStr = String(inData);
        if (tempStr.startsWith("$GNGGA,") || tempStr.startsWith("$GPGGA,")) {
           if (serialDebug) {
              Serial.println(" ");
              Serial.println(" !!!!!!!!!! Possible recovarable data chunk: ");
              Serial.println(tempStr);
              Serial.println(" !!!!!! ");
            }
        }
      }
      index = 1;
      inData[0] = aChar;
      inData[index] = '\0';;
      inCheck = 0;
      checkStar = false;
    } else if (aChar == '\n' || aChar == '\r' || aChar == ' ')
    {
      // End of record detected. Time to parse
      if (index > 50) {
        lastProcMsg = String(inData);
        if (checkSentence(lastProcMsg, inCheck, true)) {
          if (parseSentence(lastProcMsg)) {
            trackedSentenceCheck();
            processStatus();
            displayLastRadioMsg();
            
            if (serialDebug) {
              Serial.println(" ");
              Serial.print("  Processed! [");
              Serial.print(String(inData));
              Serial.println("] ");
            }
          }
          /* parseSentence(String(inData));
          trackedSentenceCheck();
          processStatus();*/
        }
      }
      index = 0;
      inData[index] = '\0';;
      inCheck = 0;
      checkStar = false;
    }
    else
    {
      if (aChar == '*') {
        checkStar = true;
      }
      else if (!checkStar) {
        inCheck = inCheck ^ aChar;
      }

      inData[index] = aChar;
      // digitalWrite(BUZZER_PIN, (aChar % 2) == 0 ? HIGH : LOW ); 
      if (index < 83) { index++; }
      inData[index] = '\0'; // Keep the string NULL terminated
    }
  }
  /* checking tracked GPS END */

  digitalWrite(BUZZER_PIN, LOW);

  while (gpsSerial.available() > 0)
  {
    aChar = gpsSerial.read();
    //Serial.print(aChar);
    if (aChar == '$')
    {
      // End of record detected. Time to parse
      if (indexGps > 50) {
        if (checkSentence(String(inDataGps), gpsCheck, false)) {
          if (parseSentence(String(inDataGps))) {
            gpsSentenceCheck();
            processStatus();
          }
          // parseSentence(String(inDataGps));
          // gpsSentenceCheck();
          // processStatus();
        }
      }
      indexGps = 1;
      inDataGps[0] = aChar;
      inDataGps[indexGps] = '\0';
      gpsCheck = 0;
      gpsStar = false;
    }
    else
    {
      if (aChar == '*') {
        gpsStar = true;
      }
      else if (!gpsStar) {
        gpsCheck = gpsCheck ^ aChar;
      }


      inDataGps[indexGps] = aChar;
      if (indexGps < 83) { indexGps++; }
      inDataGps[indexGps] = '\0'; // Keep the string NULL terminated
    }
  }

  /* Process status here!! */
  processStatus();
  /* Update display here! */
 
  
  if (screenUpdateMillis + lastScreenUpdate < millis()) {
    processStatus();
    displayStatusFont8x16();
    lastScreenUpdate = millis();
  }
}


/* Reading power supply voltage in uV */
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

/*************************************************************************
* //Function to calculate the distance between two waypoints
*************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  //I've to split all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat = radians(flat2 - flat1);
  float lat1Rad = radians(flat1);
  float lat2Rad = radians(flat2);
  //flat1 = radians(flat1);
  //flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat / 2.0)*sin(diflat / 2.0));
  dist_calc2 = cos(lat1Rad);
  dist_calc2 *= cos(lat2Rad);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

  dist_calc *= 6371000.0; //Converting to meters
              ////Serial.println(dist_calc);
  return dist_calc;
}

/* Return bearing in degrees for 2 coordinapes */
int getBearing(float lat, float lon, float lat2, float lon2) {
  float teta1 = radians(lat);
  float teta2 = radians(lat2);
  //float delta1 = radians(lat2 - lat);
  float delta2 = radians(lon2 - lon);
  //==================Heading Formula Calculation================//
  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
  float brng = atan2(y, x);
  brng = degrees(brng);// radians to degrees
  brng = ((360 + (int)brng) % 360);

  return round(brng);
}

/* check GPS sentence for checksum and filter for GGA */
boolean checkSentence(String ggaMsg, unsigned int checksum, boolean radioCheck) {
  String checkIn = String(checksum, HEX);
  checkIn.toUpperCase();
  int starIndex = ggaMsg.indexOf('*');
  String checkMsg = ggaMsg.substring(starIndex + 1);
  boolean result = false;
  if (checkMsg && checkMsg.length() > 2) checkMsg = checkMsg.substring(0, 2);
  if (checkMsg && checkIn && checkMsg == checkIn) {
    if (ggaMsg.startsWith("$GNGGA,") || ggaMsg.startsWith("$GPGGA,")) {
      result = true;
    }
  }
  else {
    if (radioCheck) {
      radioChkFail++;
      if (serialDebug) {
        Serial.println();
        Serial.print("   Checksum: ");
        Serial.println(checkIn);
        Serial.print("   ");
        Serial.println(ggaMsg);
      }
    }
    else {
      gpsChkFail++;
    }
  }
  return result;
}

/* Parsing of GGA messages into global variables,
TODO v2: coordinate and alt sanitation!!!
return false if no fix data so not doing anything at all*/
boolean parseSentence(String ggaMsg) {
  boolean result = false;
  String tmpStr = "";
  byte idx = 0;
  byte countr = 0;
  byte commas[10];

  //"$GNGGA,130554.80,4614.42873,N,02008.12940,E,1,09,1.61,392.4,M,37.7,M,,*73";

  for (idx = 0; (idx < ggaMsg.length()) && (countr < 10); idx++) {
    if (ggaMsg.charAt(idx) == COMMA) {
      commas[countr] = idx;
      countr++;
    }
  }

  if (countr == 10) {
    tmpStr = ggaMsg.substring(0, commas[0]);
    if (tmpStr && tmpStr.endsWith("GGA")) {
      digitalWrite(LINK_PIN, HIGH);
      String tmpStr = ggaMsg.substring(commas[5] + 1, commas[6]);
      if (tmpStr && ((tmpStr.charAt(0) - '0') >= 1) ) {
        msg_utc = ggaMsg.substring(commas[0] + 1, commas[1]).substring(0, 6);

        tmpStr = ggaMsg.substring(commas[1] + 1, commas[2]);
        msg_lat = tmpStr.substring(0, 2).toDouble() + (tmpStr.substring(2).toDouble() / 60.0);
        if (ggaMsg.substring(commas[2] + 1, commas[3]) == "S") {
          msg_lat = msg_lat * -1.0;
        }

        tmpStr = ggaMsg.substring(commas[3] + 1, commas[4]);
        msg_lon = tmpStr.substring(0, 3).toDouble() + (tmpStr.substring(3).toDouble() / 60.0);
        if (ggaMsg.substring(commas[4] + 1, commas[5]) == "W") {
          msg_lon = msg_lon * -1.0;
        }

        msg_sat = ggaMsg.substring(commas[6] + 1, commas[7]).toInt();
        msg_alt = round(ggaMsg.substring(commas[8] + 1, commas[9]).toFloat());
        result = true;
      } else if (serialDebug) {
        Serial.println(" --- Fix type failed!!!");
      }
    }
    else {
      //Serial.println("No GGA header ending! ");
    }
  }

  return result;
}

void processStatus() {
  if (homeSet) {
    /* process HOME LED and buzzer sound */
    if ((lastLink + linkAlert) < millis()) {
      
      digitalWrite(LINK_PIN, LOW);
    }
  }
  else {
    if (last_radio_sat >= minSats) {
      HOME_LAT = last_valid_lat;
      HOME_LON = last_valid_lon;
      HOME_ALT = last_valid_alt;
      homeSetMilllis = millis();
      homeSet = true;
      digitalWrite(HOMESET_PIN, HIGH);
      oled.clear();
    }
  }



  if ((lastGPS + linkAlert) < millis()) {
    gpsRestart++;
    lastGPS = millis();
    gpsSerial.flush();
    gpsSerial.begin(9600);
  }
  if (((lastLink + linkAlert) < millis()) && ((lastRadio + linkAlert) < millis())) {
    if (serialDebug) {
        Serial.println("radioRestart");
        
        Serial.print(" lastLink + linkAlert:");
        Serial.print(lastLink + linkAlert);

        Serial.print(" lastRadio + linkAlert:");
        Serial.print(lastRadio + linkAlert);
        
        Serial.print(" millis(): ");
        Serial.println( millis() );
      }
    
    radioRestart++;
    lastRadio = millis();
    Serial.flush();
    Serial.begin(9600);
  }

  buttonState = digitalRead(BTN_PIN);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    if (buttonBounceEnd < millis()) {
      // if the state has changed, increment the counter
      if (buttonState) {
        // if the current state is HIGH then the button went from off to on:
        buttonPushCounter++;
        oled.clear();
        if (buttonPushCounter > 3) buttonPushCounter = 0;
        displayLastRadioMsg();
      }
      lastButtonState = buttonState;
    }
  }
  else {
    buttonBounceEnd = millis() + 200;
  }
}

String bearingToHeading(int bear) {
  //String directions[] = {"  N", " NE", "  E", " SE", "  S", " SW", "  W", " NW", "  N"};
  String directions[] = { "  N", "NNE" ," NE", "NEE", "  E",  "EES", " SE",  "SSE", "  S", "SSW", " SW", "SWW", "  W",  "NWW", " NW", "NNW" ,"  N" };
  //int idx = round(( bear % 360) / 45);
  int idx = round((bear % 360) / 22.5);
  return " " + directions[idx];
}

String alertStr() {
  long secs = millis() / 100;
  return (((lastLink + linkAlert) < millis()) && (secs % 3 == 1)) ? "!" : " ";
}


void displayLastRadioMsg() {
  byte i;
  if (buttonPushCounter == 2) {
     // if(lastProcMsg.indexOf("GGA") > 0) {
      oled.set1X();
      oled.setCursor(0, 0);
    
        for (i = 0; i < 62; i++) {
          if (i < lastProcMsg.length() && lastProcMsg.charAt(i) != '\n' && lastProcMsg.charAt(i) != '\r') {
            oled.print(lastProcMsg.charAt(i));
          }
          else {
            oled.print(" ");
          }
          if (i == 15 || i == 30 || i == 45) oled.println();
        }
     // }
   }
}

void displayStatusFont8x16() {
  oled.setFont(ZevvPeep8x16);
  static char str[16];
  if (buttonPushCounter == 3) {
    byte i;
    oled.set1X();
    oled.setCursor(0, 0);
    if (indexGps < 2) {
      oled.println("Empty GPS buff");
    }
    else {
      for (i = 0; i < 62; i++) {
        if (i < indexGps && inDataGps[i] != '\r' && inDataGps[i] != '\n') {
          oled.print(inDataGps[i]);
        }
        else {
          oled.print(" ");
        }
        if (i == 15 || i == 30 || i == 45) oled.println();
      }
    }
  }
  else if (buttonPushCounter == 2) {
   /* byte i;
    oled.set1X();
    oled.setCursor(0, 0);
    
    if ((index < 2 && homeSet)) { //|| ((lastLink + linkAlert) < millis())) && homeSet) {
      oled.print(String(HOME_LAT, 6));
      oled.println("<HOME");
      oled.println(String(HOME_LON, 6));
      oled.print(String(last_valid_lat, 6));
      oled.println("<DRONE");
      oled.print(String(last_valid_lon, 6));
      oled.print(" B");
      oled.println(getBearing(HOME_LAT, HOME_LON, last_valid_lat, last_valid_lon));

    }
    else {
      for (i = 0; i < 62; i++) {
        if (i < index && inData[i] != '\n' && inData[i] != '\r') {
          oled.print(inData[i]);
        }
        else {
          oled.print(" ");
        }
        if (i == 15 || i == 30 || i == 45) oled.println();
      }
    }*/
  }
  else if (buttonPushCounter == 1) {
    oled.set1X();
    oled.setCursor(0, 0);
    oled.print(readVcc() * 0.001f);
    sprintf(str, "V S:%02d/%02d", last_radio_sat, gps_sat);
    oled.println(str);

    oled.print("RR:");
    oled.print(radioRestart, DEC);
    oled.print(" C:");
    oled.print(radioChkFail, DEC);
    oled.print(" i:");
    oled.println(index, DEC);

    oled.print("GR:");
    oled.print(gpsRestart, DEC);
    oled.print(" C:");
    oled.print(gpsChkFail, DEC);
    oled.print(" i:");
    oled.println(indexGps, DEC);
    oled.println(millis(), DEC);
  }
  else {
    if (homeSet) {
      relative_alt = last_valid_alt - HOME_ALT;
      int distance = round(calc_dist(gps_lat, gps_lon, last_valid_lat, last_valid_lon));
      /*int range = relative_alt + distance;*/
      int bear = getBearing(HOME_LAT, HOME_LON, last_valid_lat, last_valid_lon);
      int gpsBear = getBearing(gps_lat, gps_lon, last_valid_lat, last_valid_lon);
      //  oled.clear();
      oled.set1X();
      oled.setCursor(0, 0);
      if (last_valid_lat > 0.0) oled.print(SPC);
      oled.print(String(last_valid_lat, 6));
      sprintf(str, " S: %02d", last_radio_sat);
      oled.println(str);

      if (last_valid_lon > 0.0) oled.print(SPC);
      oled.print(String(last_valid_lon, 6));
      oled.print(" B:");
      sprintf(str, "%3d", bear);
      oled.println(str);

      //oled.print(" ");
      //oled.print(alertStr());
      sprintf(str, "D:%5dm", distance);
      oled.print(str);
      oled.print(bearingToHeading(gpsBear));
      oled.print(SPC);
      //readVcc()
      sprintf(str, "%3d", gpsBear);
      oled.println(str);

      oled.print(msg_utc.substring(0, 2));
      oled.print(":");
      oled.print(msg_utc.substring(2, 4));
      oled.print(":");
      oled.print(msg_utc.substring(4, 6));
      oled.print(alertStr());
      sprintf(str, "A:%4dm", relative_alt);
      oled.println(str);

      // sprintf(str, "%10ul$", millis() );
      // oled.println(str);
    }
    else {
      //Serial.println("  ----------- SCREEN NO HOME SET YET -------------");
      oled.clear();
      oled.println("   - NO HOME -");
      oled.println("   - SET YET -");
      oled.print("  Sats Nr: ");
      oled.println(last_radio_sat);
    }
  }
}

void trackedSentenceCheck() {
  lastLink = millis();
  lastRadio = millis();

  /* if (serialDebug) {
        Serial.println("trackedSentenceCheck()");
 
        Serial.print(" lastLink set to:");
        Serial.print(lastLink);

        Serial.print(" lastRadio set to:");
        Serial.println(lastRadio);
      }*/
  
  /* Here could be some validation and sanity check... */
  if (homeSet) {
    /* could be also checking with last coordinates... */
    if (calc_dist(msg_lat, msg_lon, HOME_LAT, HOME_LON) < maxValidDist) {
      last_valid_lat = msg_lat;
      last_valid_lon = msg_lon;
      last_valid_alt = msg_alt;
      last_radio_sat = msg_sat;
    }
    else {
      // //Serial.println("  --- SANITY max DIST FAIL -----");       countert   
    }
  }
  else {
    last_valid_lat = msg_lat;
    last_valid_lon = msg_lon;
    last_valid_alt = msg_alt;
    last_radio_sat = msg_sat;
  }
}


void gpsSentenceCheck() {
  lastGPS = millis();
  gps_lat = msg_lat;
  gps_lon = msg_lon;
  gps_sat = msg_sat;
}
