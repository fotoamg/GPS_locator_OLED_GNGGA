
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <SoftwareSerial.h>

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

#define SERIAL_RX_BUFFER_SIZE 256

/*  setup tuning variables: */
const byte minSats = 7;
const unsigned int maxValidDist = 8000;
const unsigned int screenUpdateMillis = 1000;
const unsigned int linkAlert = 8000;
const boolean serialDebug = false;

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
//String ggaMsgExample2 = "$GNGGA,130554.80,4614.42873,N,02008.12940,E,1,09,1.61,392.4,M,37.7,M,,*45";// Matyaster
//String ggaMsgExample3 = "$GNGGA,130554.80,4614.52873,N,02007.71040,E,1,09,1.61,392.4,M,37.7,M,,*47";

/*  status variables: */
double HOME_LAT = 46.24505;
double HOME_LON = 20.12884;
int HOME_ALT = 83;
unsigned int maxDist = 0;
byte kph = 0;
byte maxKph = 0;
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
char msg_utc[] = "120102";
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
byte gpsStar = 0;

/* serial parsing buffer for radio link */
char inData[85];
byte index = 0;
unsigned int inCheck = 0;
byte checkStar = 0;


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
    aChar = Serial.read();
    if (serialDebug) {
        Serial.write(aChar);
       // Serial.print(inCheck);
    }
    if (aChar == '$') {
     /* if (index >42) { // possibly recoverable data
        String tempStr = String(inData);
        if (tempStr.startsWith("$GNGGA,") || tempStr.startsWith("$GPGGA,")) {
           if (serialDebug) {
              Serial.println(" ");
              Serial.println(" !!!!!!!!!! Possible recovarable data chunk: ");
              Serial.println(tempStr);
              Serial.println(" !!!!!! ");
            }
        }
      }*/
      index = 1;
      inData[0] = aChar;
      inData[index] = '\0';;
      inCheck = 0;
      checkStar = 0;
    } else if (aChar == '\n' || aChar == '\r' || aChar == ' ') {
      // End of record detected. Time to parse
      if (index > 50) {
        inData[index] = '\0';
        if (checkAndParseSentence(inData, inCheck, index, checkStar, true)) {
            displayGpsMsg(lastProcMsg,2);
        }
      }
      index = 0;
      inData[index] = '\0';
      inCheck = 0;
      checkStar = 0;
    }
    else
    {
      if (aChar == '*') {
        checkStar = index;
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
    if (aChar == '$') {
      indexGps = 1;
      inDataGps[0] = aChar;
      inDataGps[indexGps] = '\0';
      gpsCheck = 0;
      gpsStar = 0;
    } else if (aChar == '\n' || aChar == '\r' || aChar == ' ') {
      // End of record detected. Time to parse
      if (indexGps > 50) {
        inDataGps[indexGps] = '\0';
        if (checkAndParseSentence(inDataGps, gpsCheck, indexGps, gpsStar, false)) {
          ;
        }
      }
      indexGps = 0;
      inDataGps[indexGps] = '\0';
      gpsCheck = 0;
      gpsStar = 0;   
    }
    else
    {
      if (aChar == '*') {
        gpsStar = indexGps;
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
    displayStatusFont8x16();
    lastScreenUpdate = millis();
  }
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

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

void parseLat(String latMsg, char sphere) {
  msg_lat = latMsg.substring(0, 2).toDouble();
  msg_lat += (latMsg.substring(2).toDouble() / 60.0);
      if (sphere == 'S') {
        msg_lat = msg_lat * -1.0;
      }
}

void parseLon(String lonMsg, char sphere) {
  if(lonMsg.charAt(0) == '0') {
    msg_lon = lonMsg.substring(1, 3).toDouble();
  } else {
    msg_lon = lonMsg.substring(0, 3).toDouble();
  }
  msg_lon += (lonMsg.substring(3).toDouble() / 60.0);
  if (sphere == 'W') {
    msg_lon = msg_lon * -1.0;
  }
}

boolean checkAndParseSentence(char msgBuffer[], unsigned int checksum, byte bufferSize, byte checkSumIdx, boolean radioCheck) {
  char chr;
  boolean result = false;
  boolean ggaMsg = true;
  boolean rmcMsg = true;
  byte idx = 0;
  byte countr = 0;
  byte commas[10];
  
  //"$GNGGA,130554.80,4614.42873,N,02008.12940,E,1,09,1.61,392.4,M,37.7,M,,*73";

  if (bufferSize < checkSumIdx +2 ) { // checksum start found and buffer size enough for checksum
      result = false;
   } else {

      unsigned int chkIn = (msgBuffer[checkSumIdx+1] - '0') *16;
      unsigned int tmpInt = (msgBuffer[checkSumIdx+2] - '0');
      if (tmpInt < 10) {
        chkIn+= tmpInt;
      } else {
        chkIn+= (tmpInt - 7);
      }

      
      if ( (msgBuffer[checkSumIdx] == '*') && (msgBuffer[0] == '$') && (chkIn == checksum) ) {
        for (idx = 0; (idx < checkSumIdx) && (countr < 10); idx++) {
          // process char by char here!!!
          chr = msgBuffer[idx];
          if (chr == COMMA) {
              commas[countr] = idx; // saving each comma's position in the buffer
              countr++;
          } else if (countr == 0 && idx == 1 && chr != 'G' ) {
               ggaMsg = false;
               rmcMsg = false;
          } else if (countr == 0 && idx == 3 && (ggaMsg || rmcMsg) ) {
             if (chr != 'G' ) {
                ggaMsg = false;
             }
             if (chr != 'R' ) {
                rmcMsg = false;
             }
          } else if (countr == 0 && idx == 4 && (ggaMsg || rmcMsg) ) {
             if (chr != 'G' ) {
                ggaMsg = false;
             }
             if (chr != 'M' ) {
                rmcMsg = false;
             }
            
          } else if (countr == 0 && idx == 5 && (ggaMsg || rmcMsg) ) {
             if (chr != 'A' ) {
                ggaMsg = false;
             }
             if (chr != 'C' ) {
                rmcMsg = false;
             }
            
          } else if (countr == 1 && (ggaMsg || rmcMsg) && idx < 13 ) { // UTC timestamp part
               msg_utc[idx-commas[0]-1] = chr;
           
          }
          
        } // end for   

        if (ggaMsg) { // GGA message header               
            digitalWrite(LINK_PIN, HIGH);
            if ((msgBuffer[commas[5]+1] - '0') >= 1) { // 3D fix or better precision
                  
                String lastMsg = String(msgBuffer);
                parseLat(lastMsg.substring(commas[1] + 1, commas[2]), msgBuffer[commas[2]+1]);
                parseLon(lastMsg.substring(commas[3] + 1, commas[4]), msgBuffer[commas[4]+1]);
                msg_sat = lastMsg.substring(commas[6] + 1, commas[7]).toInt();
                msg_alt = round(lastMsg.substring(commas[8] + 1, commas[9]).toFloat());

                if (radioCheck) {
                    lastProcMsg = lastMsg;
                    trackedSentenceCheck(true);
                    processStatus();
                  } else {
                    displayGpsMsg(lastMsg, 3);
                    gpsSentenceCheck(true);
                    processStatus();
                  }
                result = true;
            }
        } else if (rmcMsg) { // RMC message header
            digitalWrite(LINK_PIN, HIGH);
            if (msgBuffer[commas[1]+1] == 'A') { // Active status
              
                  String lastMsg = String(msgBuffer);
                  parseLat(lastMsg.substring(commas[2] + 1, commas[3]), msgBuffer[commas[3]+1]);
                  parseLon(lastMsg.substring(commas[4] + 1, commas[5]), msgBuffer[commas[5]+1]);

                  if (radioCheck) {
                    kph = round(lastMsg.substring(commas[6] + 1, commas[7]).toFloat() * 1.85);
                    if (kph > maxKph) { maxKph = kph; }
                    lastProcMsg = lastMsg;
                    trackedSentenceCheck(false);
                    processStatus();
                  } else {
                    gpsSentenceCheck(false);
                    processStatus();
                  }
                  result = true;
           }
                       
        } else {
          //Serial.println("No RMC or GGA header ending! ");
        }

      } else { // checksum failed
         if (radioCheck) {
             radioChkFail++;
          } else {
             gpsChkFail++;
          }
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
        displayGpsMsg(lastProcMsg,2);
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

void displayGpsMsg(String gpsMsg, byte panel) {
  byte i;
  if (buttonPushCounter == panel) {
      oled.set1X();
      oled.setCursor(0, 0);
    
        for (i = 0; i < 62; i++) {
          if (i < gpsMsg.length() && gpsMsg.charAt(i) != '\n' && gpsMsg.charAt(i) != '\r') {
            oled.print(gpsMsg.charAt(i));
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
  int distance = 0;
  if (homeSet) {
      relative_alt = last_valid_alt - HOME_ALT;
      distance = round(calc_dist(gps_lat, gps_lon, last_valid_lat, last_valid_lon));
      if (distance > maxDist) {
        maxDist = distance;
      }
  }
  oled.setFont(ZevvPeep8x16);
  static char str[16];
  if (buttonPushCounter == 3) {
  }
  else if (buttonPushCounter == 2) {
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
    oled.print(millis(), DEC);
    oled.print(" MD:");
    oled.println(maxDist, DEC);
  }
  else {
    if (homeSet) {
      /*int range = relative_alt + distance;*/
      //int bear = getBearing(HOME_LAT, HOME_LON, last_valid_lat, last_valid_lon);
      int gpsBear = getBearing(gps_lat, gps_lon, last_valid_lat, last_valid_lon);
      //int gpsBear = 111;
      
      //  oled.clear();
      oled.set1X();
      oled.setCursor(0, 0);
      if (last_valid_lat > 0.0) oled.print(SPC);
      oled.print(String(last_valid_lat, 6));
      sprintf(str, " S: %02d", last_radio_sat);
      oled.println(str);

      if (last_valid_lon > 0.0) oled.print(SPC);
      oled.print(String(last_valid_lon, 6));
      //oled.print(" B:");
      oled.print(" V:");
      sprintf(str, "%3d", maxKph);
      //sprintf(str, "%3d", bear);
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


      oled.print(msg_utc[0]);
      oled.print(msg_utc[1]);
      oled.print(":");
      oled.print(msg_utc[2]);
      oled.print(msg_utc[3]);
      oled.print(":");
      oled.print(msg_utc[4]);
      oled.print(msg_utc[5]);

      oled.print(alertStr());
      sprintf(str, "A:%4dm", relative_alt);
      oled.println(str);
    }
    else {
      oled.clear();
      oled.println("   - NO HOME -");
      oled.println("   - SET YET -");
      oled.print("  Sats Nr: ");
      oled.println(last_radio_sat);
    }
  }
}

void trackedSentenceCheck(boolean gga) {
  lastLink = millis();
  lastRadio = millis();
  /* Here could be some validation and sanity check... */
  if (homeSet) {
    /* could be also checking with last coordinates... */
    if (calc_dist(msg_lat, msg_lon, HOME_LAT, HOME_LON) < maxValidDist) {
      last_valid_lat = msg_lat;
      last_valid_lon = msg_lon;
      if (gga) {
                last_valid_alt = msg_alt;
                last_radio_sat = msg_sat;
      }
    }
    else {
     //Serial.println("  --- SANITY max DIST FAIL -----");    //  TODO add counter
    }
  }
  else {
    last_valid_lat = msg_lat;
    last_valid_lon = msg_lon;
    if (gga) {
                last_valid_alt = msg_alt;
                last_radio_sat = msg_sat;
             }
  }
}


void gpsSentenceCheck(boolean gga) {
  lastGPS = millis();
  gps_lat = msg_lat;
  gps_lon = msg_lon;
  if (gga) gps_sat = msg_sat;
}
