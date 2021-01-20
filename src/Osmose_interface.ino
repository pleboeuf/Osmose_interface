/*
 * Project Osmose_interface
 * Description: Osmose operation data logging
 * Author: Pierre Leboeuf
 * Date: 24 dec 2020
 * 
 */

// STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
#include "../lib/ITEADLIB_Nextion/src/ITEADLIB_Nextion.h"
#include "math.h"
#define dbSerial Serial
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define pumpDebounceDelay 100 // Debounce time in milliseconds for pump mechanical start/stop switch
#define pumpONstate 0         // Pump signal is active low.
#define pumpOFFstate 1        // Pump signal is active low.
#define histLength 6          // Number of line in data history
#define lineLength 100        // Max length of each line of history

#define sysOnMsg "En marche. Selectionner l'operation"
#define sysOffMsg "Pret"
#define sysOsm1234 "Osmose en cour. Seq: 1-2-3-4"
#define sysOsm4321 "Osmose en cour. Seq: 4-3-2-1"
#define sysLav "Lavage en cour"
#define sysRinse "Rincage en cour"

#define operData 0 // Identify Osmose operation data
#define brixData 1 // Identify Brix concentration data
#define summaryData 2 // Identify Brix concentration data

String sysStateMsg = sysOffMsg;

// Firmware version et date
#define FirmwareVersion "1.0.1" // Version du firmware du capteur.
String F_Date = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time; // Date et heure de compilation UTC
String VersionNo = FirmwareVersion;
String VersionStr = "tVers.txt=\"Osmose Logger V" + VersionNo + "\"\xFF\xFF\xFF";

USARTSerial &nexSerial = Serial1;

// Définition des éléments du HMI Netion que l'on veut accéder
// Home page
NexText tDateTime = NexText(0, 18, "tDateTime");
NexText tVers = NexText(0, 19, "tVers");
NexText tstatus = NexText(0, 5, "status");
NexText tSeqName = NexText(0, 23, "tSeqName");
NexText tEnCourCntr = NexText(0, 26, "tEnCourCntr");
NexText tSeqCntr = NexText(0, 27, "tSeqCntr");
NexText tSLavCntr = NexText(0, 28, "tSLavCntr");

NexButton ind = NexButton(0, 7, "ind");
NexButton bGotoOsmose = NexButton(0, 13, "bOsmose");
NexButton bGotoBRIX = NexButton(0, 14, "bBrix");
NexButton bGotoSommaire = NexButton(0, 15, "bSom");
NexButton bGotoLavage = NexButton(0, 16, "bLavage");
NexButton bGotoRincage = NexButton(0, 17, "bRince");

// Page Données d'opération
NexText col1 = NexText(1, 17, "col1");
NexText col2 = NexText(1, 18, "col2");
NexText col3 = NexText(1, 19, "col3");
NexText col4 = NexText(1, 20, "col4");
NexText conc = NexText(1, 21, "conc");
NexText temp = NexText(1, 22, "temp");
NexText pres = NexText(1, 23, "pres");
NexText seq = NexText(1, 24, "seq");
NexButton bOkOSM = NexButton(1, 43, "bOk");

// Page Données de concentration
NexVar fieldSelect = NexVar(2, 16, "flsel");
NexText brixSeve = NexText(2, 17, "brixSeve");
NexText brixConc = NexText(2, 18, "brixConc");
NexButton bm = NexButton(2, 34, "bm");
NexButton bp = NexButton(2, 33, "bp");
NexButton bOkBRIX = NexButton(2, 25, "bOk");

// Page Lavage
NexButton bOkLAV = NexButton(3, 4, "bOk");
NexDSButton rNorm = NexDSButton(3, 13, "rNorm");
NexDSButton rRecirc = NexDSButton(3, 14, "rRecirc");

// Page Rinçage
NexButton bOkRINC = NexButton(4, 4, "bOk");

// Sommaire des données
NexText tl0 = NexText(5, 10, "tl0");
NexText tl1 = NexText(5, 11, "tl1");
NexText tl2 = NexText(5, 12, "tl2");
NexText tl3 = NexText(5, 13, "tl3");
NexText tl4 = NexText(5, 14, "tl4");
NexText tl5 = NexText(5, 15, "tl5");
// Sommaire d'opération
NexText ts0 = NexText(5, 16, "ts0");
NexText ts1 = NexText(5, 17, "ts1");
NexText ts2 = NexText(5, 18, "ts2");
NexText ts3 = NexText(5, 19, "ts3");
NexText ts4 = NexText(5, 20, "ts4");
NexText ts5 = NexText(5, 21, "ts5");
NexButton bOkSUMM = NexButton(5, 1, "bOk");
// NexButton bSommaire = NexButton(5, 7, "bSommaire");


// Liste des objets que l'on veut surveiller
NexTouch *nex_listen_list[] =
    {
        &bp, &bm, &bOkOSM, &bOkBRIX, &bOkLAV, &bOkRINC, &bOkSUMM, 
        &bGotoOsmose, &bGotoBRIX, &bGotoSommaire, &bGotoLavage, &bGotoRincage,
        // &col1, &col2, &col3, &col4, &conc, &temp, &pres, 
        &seq,
        &rNorm, &rRecirc,
        NULL};

// Variables globale
enum Mode {
  concentration1234, //0
  concentration4321, //1
  lavage_normal,     //2
  lavage_recirc,     //3
  rinsage,           //4
  indefini           //5
};
String modeText[] = {
  "concentration 1-2-3-4", //0
  "concentration 4-3-2-1", //1
  "lavage_normal", //2
  "lavage_recirc", //3
  "rinsage",       //4
  "indefini"       //5
};
enum Etat {
  marche,
  arret
};
String alarmText[] = {
  "Pas alarme",
  "SVP entrer les donnees!!!",
  "Seq. en oper. depuis plus de 4h!!!",
  "Plus de 12h depuis le lavage!!!"
};
struct runTime {
  time32_t totalSec;
  int hour;
  int min;
  int sec;
};
runTime secToHrMinSec(unsigned debut);

Mode System_mode = indefini;
Etat System_state = arret;

String formatDateTime(time32_t t);

char now[12];
int previousSec = 0;
unsigned long currentTime = 0;
unsigned long lastUpdateTime = 0;
unsigned startTime;
unsigned endTime;
unsigned tempsOperOsmTotal = 0;
unsigned tempsOperOsmSequence = 0;
unsigned tempsOperLavage = 0;
unsigned tempsOperRinsage = 0;
unsigned nextMinute = 60;
static uint32_t tempsOperEnCour = 0;
static uint32_t tempsSeq1234 = 0;
static uint32_t tempsSeq4321 = 0;
static uint32_t tempsDepuisLavage = 0;
const uint32_t seqTimeLimit = 4 * 60 * 60; // 4 hours
const uint32_t depuisLavageTimeLimit = 12 * 60 * 60; // 4 hours
const uint32_t dataEntryTimeLimit = 10 * 60; // 1 min

bool operDataValid = false;
bool brixDataValid = false;
double bSeve = 0, bConc = 0;
double Col1 = 0, Col2 = 0, Col3 = 0, Col4 = 0, dConc = 0;
double Temp = 0, Pres = 0;
String filterSeq = "";

String hist_data[histLength];
String hist_perf[histLength];
bool pushDataFlag = false;

int heater = D3; // Contrôle le transistor du chauffage
int alarmBuzzer = D6; // Contrôle le buzzer d'alerte
int optoInput = A1;
int alarmStatus = 0;

bool PumpCurrentState = pumpOFFstate; // Initialize pump in the OFF state
bool PumpOldState = pumpOFFstate;     // Pour déterminer le chanement d'état

// volatile unsigned long changeTime = 0; // Moment du dernier changement d'état de la pompe

/* Define a log handler on Serial1 for log messages */
SerialLogHandler logHandler(115200, LOG_LEVEL_INFO, {
                                                        // Logging level for non-application messages
                                                        {"app", LOG_LEVEL_NONE} // Logging level for application messages
                                                    });

// Variables liés aux publications
uint32_t noSerie = 0;
long unsigned int newGenTimestamp = 0;
String StopEventName = "Osmose/Stop";         // Name of the event to be sent to the cloud
String StartEventName = "Osmose/Start";       // Name of the event to be sent to the cloud

/*
  Attach interrupt handler to pin A1 to monitor pump Osmose Start/Stop
*/
class PumpState_A1 {
public:
  PumpState_A1() {
    pinMode(A1, INPUT);
    attachInterrupt(A1, &PumpState_A1::A1Handler, this, CHANGE);
  }
  void A1Handler() {
    // IMPORTANT: Pump is active LOW. Pump is ON when PumpCurrentState == false
    delayMicroseconds(pumpDebounceDelay * 1000);
    PumpCurrentState = digitalRead(A1);
    // changeTime = millis();
  }
};
PumpState_A1 ThePumpState; // Instantiate the class A1State


// **************************************************************
//                  *** Main program ***
// **************************************************************

// **************************************************************
// setup() runs once, when the device is first turned on.
// **************************************************************
void setup()
{
  char tmp[12];
  nexInit();
  setBaudrate(115200);
  dbSerialBegin(115200);
  pinMode(heater, OUTPUT);
  pinMode(alarmBuzzer, OUTPUT);
  analogWrite(heater, 0);
  digitalWrite(alarmBuzzer, LOW);
  Time.zone(-5);

  nexSerial.print("rest\xFF\xFF\xFF"); // Reset
  delay(500);
  writeNextionTextData(tstatus, "Démarrage...");
  // Attach callback routine to Nextion objects
  bGotoOsmose.attachPop(bGotoOsmoseCallBack); // Enter Osmose operation data
  bGotoBRIX.attachPop(bGotoBrixCallBack); // Enter Brix concentration data
  bGotoSommaire.attachPop(bGotoSommairePopCallback); // Display sommaire panel
  bGotoLavage.attachPop(bGotoLavageCallback);
  bGotoRincage.attachPop(bGotoRincageCallback);
  
  bm.attachPop(bmPopCallback); // Handle button - (minus)
  bp.attachPop(bpPopCallback); // Handle button - (plus)
  bOkBRIX.attachPush(bOkBRIXPushCallback);
  bOkOSM.attachPush(bOkOSMPushCallback);
  bOkLAV.attachPop(bOkLAVPopCallback);
  bOkRINC.attachPop(bOkRINCPopCallback);
  bOkSUMM.attachPop(bOkSUMMPopCallback);
  seq.attachPop(seqPOPCallback);

  // Enregistrement des fonctions et variables disponible par le nuage
  Log.info("(setup) Enregistrement des variables et fonctions\n");
  Particle.variable("Version", FirmwareVersion);
  Particle.variable("Date", FirmwareDate);

    //Wait until Photon receives time from Particle Cloud (or connection to Particle Cloud is lost)
  if (Particle.connected()) {
    Log.info("Connecté au nuage. :)");
    Particle.syncTime();
    Log.info("(setup) Syncing time ");
    waitUntil(Particle.syncTimeDone);
    if (not(Time.isValid()) || Time.year() < 2020) {
      Log.info("(setup) Syncing time ");
      Particle.syncTime();
      waitUntil(Particle.syncTimeDone);
    }
    Log.info("(setup) syncTimeDone " + Time.timeStr());
    currentTime = Time.now();
    newGenTimestamp = currentTime; // Init generation at boot time
    lastUpdateTime = currentTime;
    previousSec = Time.second();
    System_mode = indefini;
    System_state = arret;
    initSummaryPanel();
    nexSerial.print(VersionStr);
    nexSerial.print("ind.txt=\"Off\"\xFF\xFF\xFF");
    nexSerial.print("ind.bco=RED\xFF\xFF\xFF");
    // Clear counter;
    sprintf(tmp, "%02d:%02d:%02d", 0, 0, 0);
    writeNextionTextData(tSeqCntr, tmp);
    writeNextionTextData(tSLavCntr, tmp);
    // Init System state
    PumpCurrentState = digitalRead(A1);
    if (PumpCurrentState == pumpONstate) {
      System_state = marche;
      sysStateMsg = sysOnMsg;
      startTime = Time.now();
      Log.info("Setup: Système en marche!");
    }
    checkSysState();
    Log.info("Setup complete!\n\n");
  }
}


// **************************************************************
// loop() runs over and over again, as quickly as it can execute.
// **************************************************************
void loop()
{
  // The core of your code will likely live here.
  nexLoop(nex_listen_list);
  currentTime = Time.now();
  // Update time once every second
  if (Time.second() != previousSec) {
    previousSec = Time.second();
    writeNextionTextData(tDateTime, Time.timeStr());
    checkSysState(); // Check for system state and opr. mode
    if (System_state == marche) {
      checkAlarm();
    }
  }

  // Publish durations every minute if running
  if (tempsOperEnCour == nextMinute) {
    nextMinute = tempsOperEnCour + 60;
    if (System_state == marche) {
      publishTimeCounters();
      if (alarmStatus > 0) {
        publishAlarm(currentTime);
      }
    }
  }

  // Sync time once a day
  if (currentTime > (lastUpdateTime + ONE_DAY_MILLIS)) {
    Particle.syncTime();
    lastUpdateTime = currentTime;
  }

  Particle.process();
}

// **************************************************************
// Read text data from sourceField and convert it to double
// **************************************************************
double readNexTionData(NexText sourceField) {
  double data;
  char buffer[20] = {0};
  sourceField.getText(buffer, sizeof(buffer));
  data = atof(buffer);
  Log.info("readNexTionData: buffer= %s", buffer);
  return data;
}

// **************************************************************
// Read text data from sourceField
// **************************************************************
String readNextionText(NexText sourceField) {
  String outText = "";
  char buffer[20] = {0};
  sourceField.getText(buffer, sizeof(buffer));
  Log.info("readNextionText: buffer= %s", buffer);
  outText = String(buffer);
  return outText;
}

// **************************************************************
// Write data to destField after conversion to text from double
// **************************************************************
void writeNextionData(NexText destField, double data) {
  char buffer[20] = {0};
  sprintf(buffer, "%.1f", data);
  destField.setText(buffer);
}

// **************************************************************
// Write data to destField after conversion to text from double
// **************************************************************
void writeNextionTextData(NexText destField, String textData) {
  char buffer[100] = {0};
  sprintf(buffer, "%s", textData.c_str());
  destField.setText(buffer);
}

// **************************************************************
// Button gotoOsmose(bOsmose) Call back
// **************************************************************
void bGotoOsmoseCallBack(void *ptr) {
  if (System_state == marche){
    nexSerial.print("page 1\xFF\xFF\xFF");
  }
}

// **************************************************************
// Button gotoOsmose(bOsmose) Call back
// **************************************************************
void bGotoBrixCallBack(void *ptr) {
  if (System_state == marche) {
    nexSerial.print("page 2\xFF\xFF\xFF");
  }
}

// **************************************************************
// Button PLUS(+) POP callback
// **************************************************************
void bpPopCallback(void *ptr) {
  uint32_t flsel;
  double bSeve = 0;
  double bConc = 0;

  fieldSelect.getValue(&flsel);
  if (flsel == 17) {
    bSeve = readNexTionData(brixSeve) + 0.1;
    writeNextionData(brixSeve, bSeve);
    Log.info("bpPopCallback: Bouton(+), seve: %.1f", bSeve);
  } else {
    bConc = readNexTionData(brixConc) + 0.1;
    writeNextionData(brixConc, bConc);
    Log.info("bpPopCallback: Bouton(+), conc: %.1f", bConc);
  }
}

// **************************************************************
// Button MINUS(-) POP callback
// **************************************************************
void bmPopCallback(void *ptr) {
  uint32_t flsel;
  double bSeve = 0;
  double bConc = 0;
  fieldSelect.getValue(&flsel);
  if (flsel == 17) {
    bSeve = readNexTionData(brixSeve) - 0.1;
    if (bSeve < 0)
      bSeve = 0;
    writeNextionData(brixSeve, bSeve);
    Log.info("bmPopCallback: Bouton(-), seve: %.1f", bSeve);
  } else {
    bConc = readNexTionData(brixConc) - 0.1;
    if (bConc < 0)
      bConc = 0;
    writeNextionData(brixConc, bConc);
    Log.info("bmPopCallback: Bouton(-), conc: %.1f", bConc);
  }
}

// ***************************************************************
// BRIX PANEL button Ok PUSH callback
// ***************************************************************
void bOkBRIXPushCallback(void *ptr) {
  brixDataValid = false;
  bSeve = readNexTionData(brixSeve);
  bConc = readNexTionData(brixConc);
  if (bSeve > 0 && bConc > 0) {
    brixDataValid = true;
    delay(500);
    nexSerial.print("page 0\xFF\xFF\xFF");
  }
  if (operDataValid && brixDataValid && System_state == marche) {
    publishData(brixData, "");
    publishEvent(StartEventName, startTime);
  }
  Log.info("bOkBRIXPushCallback: brixDataValid: %d, brix Seve= %.1f, Concentré= %.1f", brixDataValid, bSeve, bConc);
  // Send event
}

// ***************************************************************
// OSMOSE PANEL button Ok PUSH callback
// ***************************************************************
void bOkOSMPushCallback(void *ptr) {
  operDataValid = false;
  Col1 = readNexTionData(col1);delay(1);
  Col2 = readNexTionData(col2);delay(1);
  Col3 = readNexTionData(col3);delay(1);
  Col4 = readNexTionData(col4);delay(1);
  dConc = readNexTionData(conc);delay(1);
  Temp = readNexTionData(temp);delay(1);
  Pres = readNexTionData(pres);delay(1);
  filterSeq = readNextionText(seq);
  
  if (Col1 > 0 && Col2 > 0 && Col3 > 0 && Col4 > 0 && dConc > 0 && Temp > 0 && Pres > 0 && filterSeq.length() > 6) {
    operDataValid = true;
    delay(100);
    if (System_state == marche && filterSeq != " - - - ") {
      delay(500);
      nexSerial.print("vis cOsm.id,1\xFF\xFF\xFF");
    }
    nexSerial.print("page 0\xFF\xFF\xFF");
    if (filterSeq == "1-2-3-4") {
      System_mode = concentration1234;
      sysStateMsg = sysOsm1234;
      tempsSeq1234 = tempsSeq1234 + tempsOperEnCour;
      tempsSeq4321 = 0;
      nexSerial.print("tSeqName.txt=\"1-2-3-4:\"\xFF\xFF\xFF");      
    } else {
      System_mode = concentration4321;
      sysStateMsg = sysOsm4321;
      tempsSeq1234 = 0;
      tempsSeq4321 = tempsSeq4321 + tempsOperEnCour;
      nexSerial.print("tSeqName.txt=\"4-3-2-1:\"\xFF\xFF\xFF");
    }
    publishData(operData, "");
  }
  Log.info("bOkOSMPushCallback: c1= %.1f, c2= %.1f, c3= %.1f, c4= %.1f, Conc= %.1f, Temp=%.1f, Pres= %.0f, Seq: %s",
                                Col1, Col2, Col3, Col4, dConc, Temp, Pres, filterSeq.c_str());
  Log.info("bOkOSMPushCallback: operDataValid= %d, brixDataValid= %d", operDataValid, brixDataValid);
  // Send event
}

// **************************************************************
// Field seq (sequence) POP callback
// **************************************************************
void seqPOPCallback(void *ptr) {
  filterSeq = readNextionText(seq);
  Log.info("seqPOPCallback: filterSeq= %s", filterSeq.c_str());
}

// **************************************************************
// Sommaire POP callback
// **************************************************************
void bGotoSommairePopCallback(void *ptr) {
  delay(100);
  nexSerial.print("page 5\xFF\xFF\xFF");
  Log.info("bGotoSommairePopCallback!");
}

// **************************************************************
// Go to Lavage POP callback
// **************************************************************
void bGotoLavageCallback(void *ptr) {
  if (System_state == marche) {
    nexSerial.print("page 3\xFF\xFF\xFF");
  }
  Log.info("bGotoLavageCallback!");
}

// **************************************************************
// Go to Rinçage POP callback
// **************************************************************
void bGotoRincageCallback(void *ptr) {
  if (System_state == marche) {
    nexSerial.print("page 4\xFF\xFF\xFF");
    tempsDepuisLavage = tempsDepuisLavage - tempsOperEnCour;
    System_mode = rinsage;
  }
  Log.info("bGotoRincageCallback!");
}

// **************************************************************
// Lavage PANEL button Ok POP callback
// **************************************************************
void bOkLAVPopCallback(void *ptr) {
  uint32_t val;
  // lire le Radio bouton de sélection
  if (System_state == marche) {
    sysStateMsg = sysLav;
    rNorm.getValue(&val);
    if (val == 1) {
      System_mode = lavage_normal;
    } else {
      System_mode = lavage_recirc;
    }
  nexSerial.print("page 0\xFF\xFF\xFF");
  nexSerial.print("vis cLav.id=1\"\xFF\xFF\xFF");
  publishEvent(StartEventName, startTime);
  }
  nexSerial.print("page 0\xFF\xFF\xFF");
  Log.info("bOkLAVPopCallback! val = %lu", val);
}

// **************************************************************
// Rinçage PANEL button Ok POP callback
// **************************************************************
void bOkRINCPopCallback(void *ptr) {
  if (System_state == marche) {
    System_mode = rinsage;
    sysStateMsg = sysRinse;
    nexSerial.print("page 0\xFF\xFF\xFF");
    nexSerial.print("vis cRinc.id=1\"\xFF\xFF\xFF");
    publishEvent(StartEventName, startTime);
  }
  nexSerial.print("page 0\xFF\xFF\xFF");
  Log.info("bOkRINCPushCallback!");
}

// **************************************************************
// Sommaire PANEL button Ok POP callback
// **************************************************************
void bOkSUMMPopCallback(void *ptr) {
  Log.info("bOkSUMMPopCallback!");
}

// **************************************************************
// Home PANEL button Sommaire POP callback
// **************************************************************
void bSommairePopCallback(void *ptr) {
  nexSerial.print("page 5\xFF\xFF\xFF");
  String start_time_str = startDateTimeStr(startTime);
  String end_time_str = stopTimeStr(startTime, endTime);
  writeResultsLines(pushDataFlag, start_time_str, end_time_str);
  writeOperSummaryLines(pushDataFlag, start_time_str, end_time_str);
  Log.info("bSommairePopCallback!");
}

// **************************************************************
// History data management
// **************************************************************
void pushHistory(String hist[]) {
  if (hist[2].startsWith("Donnees"))
    hist[2] = "";
  for (int i = histLength - 1; i > 2; i--)
  {
    hist[i] = hist[i - 1];
    Log.info("pushHistory: hist[%d] = %s", i, hist[i].c_str());
  }
}

// ***************************************************************
// Format a time string
// ***************************************************************
String startDateTimeStr(time32_t t) {
  char thisDatetime[12];
  sprintf(thisDatetime, "%02d/%02d %2d:%02d", Time.month(t), Time.day(t), Time.hour(t), Time.minute(t));
  Log.info("startDateTimeStr: thisDatetime = %s", thisDatetime);
  return thisDatetime;
}

// ***************************************************************
// Format a time string
// ***************************************************************
String stopTimeStr(time32_t st, time32_t et) {
  char thisTime[18] = "";
  int timeInterval = et - st;
  int hour = floor(timeInterval / 3600);
  int min = floor((timeInterval - hour * 3600) / 60);
  // if (hour > 0 || min > 0) {
    sprintf(thisTime, "%2d:%02d", hour, min);
  // }
  Log.info("startDateTimeStr: thisTime = %s", thisTime);
  return thisTime;
}

// ***************************************************************
// Show a summary af data at stop time
// ***************************************************************
void showSummary() {
  if (operDataValid && brixDataValid) {
    delay(500);
    nexSerial.print("page 5\xFF\xFF\xFF");
    String start_time_str = startDateTimeStr(startTime);
    String end_time_str = stopTimeStr(startTime, endTime);
    writeResultsLines(pushDataFlag, start_time_str, end_time_str);
    writeOperSummaryLines(pushDataFlag, start_time_str, end_time_str);
    publishData(summaryData, end_time_str);
    delay(100);
    Log.info("showSummary: Show and publish results");
  } else {
    Log.info("showSummary: invalid data!");
  }
}

// ***************************************************************
// write to all data summary field
// ***************************************************************
void writeResultsLines(bool pushData, String start, String end) {
  char tmp[105];
  // Entête  ligne 0 et 1
  hist_data[0] = "    Debut    Densite (Brix)            Debits (GPM)             Temp.   Pres.     Seq.     Duree";
  hist_data[1] = "date  heure  Seve   Conc.   Col1   Col2   Col3   Col4   Conc.  (deg C)  (PSI)              hr:min";
  writeNextionTextData(tl0, hist_data[0]);
  writeNextionTextData(tl1, hist_data[1]);
  // Ligne 2: Données
  if (operDataValid == true && brixDataValid == true){
    if (pushData) pushHistory(hist_data);
    sprintf(tmp, "%11s %5.1f %6.1f  %6.1f  %5.1f  %5.1f  %5.1f  %5.1f  %6.1f  %6.0f  %10s %7s",
                  start.c_str(), bSeve, bConc, Col1, Col2, Col3, Col4, dConc, Temp, Pres, filterSeq.c_str(), end.c_str());
    hist_data[2] = String(tmp);
    writeNextionTextData(tl2, hist_data[2]);
    Log.info("writeResultsLines: Wrote allData field!: tmp= %s", hist_data[2].c_str());
  } 
  
  writeNextionTextData(tl2, hist_data[2]);
  writeNextionTextData(tl3, hist_data[3]);
  writeNextionTextData(tl4, hist_data[4]);
  writeNextionTextData(tl5, hist_data[5]);
}

// ***************************************************************
// write to all data summary field
// ***************************************************************
void writeOperSummaryLines(bool pushData, String start, String end)
{
  char tmp[100];
  float debitFiltratgpm;
  float debitTotalgpm;
  float pcConc;
  // Entête  ligne 0 et 1
  hist_perf[0] = "    Debut             Concentre             Filtrat         Total          Duree";
  hist_perf[1] = "date  heure    % concen.  Debit (GPH)     Debit (GPH)    Debit (GPH)       hr:min";
  writeNextionTextData(ts0, hist_perf[0]);
  writeNextionTextData(ts1, hist_perf[1]);
  // Ligne 2: Données
  if (operDataValid == true && brixDataValid == true) {
    if (pushData) {
      pushHistory(hist_perf);
      pushDataFlag = false;
    }
    debitFiltratgpm = Col1 + Col2 + Col3 + Col4;
    debitTotalgpm = debitFiltratgpm + dConc;
    pcConc = 100 * debitFiltratgpm / debitTotalgpm;
    sprintf(tmp, "%11s  %8.0f %11.0f  %14.0f  %13.0f %15s", start.c_str(), pcConc, 60 * dConc, 60 * debitFiltratgpm, 60 * debitTotalgpm, end.c_str());
    hist_perf[2] = String(tmp);
    writeNextionTextData(ts2, hist_perf[2]);
    Log.info("writeOperSummaryLines: Wrote allData field!: tmp= %s", hist_perf[2].c_str());
  } 

  writeNextionTextData(ts2, hist_perf[2]);
  writeNextionTextData(ts3, hist_perf[3]);
  writeNextionTextData(ts4, hist_perf[4]);
  writeNextionTextData(ts5, hist_perf[5]);
}

// ***************************************************************
// Initialize Results Summary Panel
// ***************************************************************
void initSummaryPanel() {
  nexSerial.print("page 5\xFF\xFF\xFF");
  // Entête  ligne 0 et 1
  hist_data[0] = "    Debut    Densite (Brix)            Debits (GPM)             Temp.   Pres.     Seq.     Duree";
  hist_data[1] = "date  heure  Seve   Conc.   Col1   Col2   Col3   Col4   Conc.  (deg C)  (PSI)              hr:min";
  writeNextionTextData(tl0, hist_data[0]);
  writeNextionTextData(tl1, hist_data[1]);
  writeNextionTextData(tl2, hist_data[2]);
  writeNextionTextData(tl3, hist_data[3]);
  writeNextionTextData(tl4, hist_data[4]);
  writeNextionTextData(tl5, hist_data[5]);
  // Entête  ligne 0 et 1
  hist_perf[0] = "    Debut             Concentre             Filtrat         Total          Duree";
  hist_perf[1] = "date  heure    % concen.  Debit (GPH)     Debit (GPH)    Debit (GPH)       hr:min";
  writeNextionTextData(ts0, hist_perf[0]);
  writeNextionTextData(ts1, hist_perf[1]);
  writeNextionTextData(ts2, hist_perf[2]);
  writeNextionTextData(ts3, hist_perf[3]);
  writeNextionTextData(ts4, hist_perf[4]);
  writeNextionTextData(ts5, hist_perf[5]);
  nexSerial.print("page 0\xFF\xFF\xFF");
}

// ***************************************************************
// Handle system state changes and display time counter
// ***************************************************************
void checkSysState() {
  // À faire seulement au changement d'état
  if (PumpCurrentState != PumpOldState) {
    PumpOldState = PumpCurrentState;
    if (PumpCurrentState == pumpONstate) {
    // À faire au démarrage
      startTime = Time.now();
      System_state = marche;
      // timeOffset = Time.second(startTime);
      sysStateMsg = sysOnMsg;
      // Mise à jour des compteurs
      tempsOperEnCour = 0;
      nextMinute = tempsOperEnCour + 60;
      // Publish start time
      publishEvent(StartEventName, startTime);
      Log.info("checkSysState: Système en marche!");
    } else {
    // À faire à l'arrêt
      endTime = Time.now();
      System_state = arret;
      sysStateMsg = sysOffMsg;
      nexSerial.print("status.pco=GREEN\xFF\xFF\xFF");

      if (System_mode == lavage_normal || System_mode == lavage_recirc) {
        tempsDepuisLavage = 0;
        tempsSeq1234 = 0;
        tempsSeq4321 = 0;
        Log.info("checkSysState: System_mode = %s", modeText[System_mode].c_str());        
      }

      showSummary();
      pushDataFlag = true;
      publishEvent(StopEventName, endTime);      // Publish stop time
      operDataValid = false;
      brixDataValid = false;
      System_mode = indefini;
      Log.info("checkSysState: Système en arrêt!");
    }
  } 
  // À faire à toute les boucles -- Affichage
  if (PumpCurrentState == pumpONstate) {
    // Turn indicator On
    nexSerial.print("ind.txt=\"On\"\xFF\xFF\xFF");
    nexSerial.print("ind.bco=GREEN\xFF\xFF\xFF");
    // Increment secToHrMinSec counter
    nexSerial.print("vis tEnCourCntr.id,1\xFF\xFF\xFF");
    updateTimeCounters();
  } else {
    // Turn indicator Off
    nexSerial.print("ind.txt=\"Off\"\xFF\xFF\xFF");
    nexSerial.print("ind.bco=RED\xFF\xFF\xFF");
  }
  // Update status message
  if (alarmStatus > 0) {
    writeNextionTextData(tstatus, alarmText[alarmStatus].c_str());
  } else {
    writeNextionTextData(tstatus, sysStateMsg.c_str());
  }

  // Mettre les indicateurs à ON ou OFF
  if (operDataValid) {
    nexSerial.print("vis cOsm.id,1\xFF\xFF\xFF");
  } else {
    nexSerial.print("vis cOsm.id,0\xFF\xFF\xFF");
  }
  if (brixDataValid) {
    nexSerial.print("vis cBrix.id,1\xFF\xFF\xFF");
  } else {
    nexSerial.print("vis cBrix.id,0\xFF\xFF\xFF");
  }
  if (System_mode == lavage_normal || System_mode == lavage_recirc) {
    nexSerial.print("vis cLav.id,1\xFF\xFF\xFF");
    tempsDepuisLavage = 0;
  } else {
    nexSerial.print("vis cLav.id,0\xFF\xFF\xFF");
  }
  if (System_mode == rinsage) {
    nexSerial.print("vis cRinc.id,1\xFF\xFF\xFF");
  } else {
    nexSerial.print("vis cRinc.id,0\xFF\xFF\xFF");
  }
}

// ***************************************************************
// Formattage des événements sous forme JSON pour publication
// ***************************************************************
String makeJSON(uint32_t numSerie, uint32_t timeStamp, uint32_t timer, uint32_t startStopTime, int eData, String mode, String eName){
  char publishString[200];
  sprintf(publishString,"{\"noSerie\": %lu,\"generation\": %lu,\"timestamp\": %lu,\"timer\": %lu,\"start-stop-time\": %lu,\"eData\":%d,\"mode\": \"%s\",\"eName\": \"%s\"}",
                            numSerie, newGenTimestamp,          timeStamp,        timer, startStopTime, eData, mode.c_str(), eName.c_str());
  // Log.info ("(makeJSON) - makeJSON: %s",publishString);
  return publishString;
}

// ***************************************************************
// Publication des événements Start/Stop
// ***************************************************************
bool publishEvent(String eName, uint32_t evenTime) {
  noSerie++;
  String msg = makeJSON(noSerie, Time.now(), millis(), evenTime, System_mode, modeText[System_mode], eName.c_str());
  bool pubSuccess = Particle.publish(eName, msg, PRIVATE, NO_ACK);
  Log.info("publishEvent: %s", eName.c_str());
  return pubSuccess;
}

// ***************************************************************
// Publication des alarmes
// ***************************************************************
bool publishAlarm(uint32_t evenTime) {
  noSerie++;
  String eName = "Osmose/alarm";
  String msg = makeJSON(noSerie, Time.now(), millis(), evenTime, alarmStatus, alarmText[alarmStatus], eName.c_str());
  bool pubSuccess = Particle.publish(eName, msg, PRIVATE, NO_ACK);
  Log.info("publishAlarm: %s", eName.c_str());
  return pubSuccess;
}


// ***************************************************************
// Publication des données
// ***************************************************************
bool publishData(int dataType, String end) {
  String eName;
  noSerie++;
  char msg[200];
  if (dataType == operData) {
    eName = "Osmose/operData";
    sprintf(msg,"{\"noSerie\": %lu,\"generation\": %lu,\"eName\": \"%s\",\"Col1\": %.1f,\"Col2\": %.1f,\"Col3\": %.1f,\"Col4\": %.1f,\"Conc\": %.1f,\"Temp\": %.1f,\"Pres\": %.1f,\"seq\": %s}",
                    noSerie,        newGenTimestamp,   eName.c_str(),     Col1,        Col2,         Col3,        Col4,        dConc,         Temp,            Pres,          filterSeq.c_str());
  } else if (dataType == brixData) {
    eName = "Osmose/concData";
    sprintf(msg,"{\"noSerie\": %lu,\"generation\": %lu,\"eName\": \"%s\",\"Seve\": %.1f,\"Conc\": %.1f}",
                    noSerie,        newGenTimestamp,   eName.c_str(),     bSeve,        bConc);
  } else if (dataType == summaryData) {
    eName = "Osmose/summaryData";
    sprintf(msg,"{\"noSerie\": %lu,\"generation\": %lu,\"eName\": \"%s\",\"PC_Conc\": %d,\"GPH_Conc\": %d,\"GPH_Filtrat\": %d,\"GPH_Total\": %d,\"Durée\": %s}",
                    noSerie,        newGenTimestamp,   eName.c_str(), 67, 300, 600,  900, end.c_str());
  }
  bool pubSuccess = Particle.publish(eName, msg, PRIVATE, NO_ACK);
  Log.info("publishData: %s", eName.c_str());
  return pubSuccess;
}

// ***************************************************************
// Publication des temps d'opération
// ***************************************************************
bool publishTimeCounters() {
  String eName;
  noSerie++;
  char msg[200];
  eName = "Osmose/timeCounter";
  sprintf(msg,"{\"noSerie\": %lu,\"generation\": %lu,\"eName\": \"%s\",\"TempsOperEnCour\": %lu,\"TempsSeq1234\": %lu,\"TempsSeq4321\": %lu,\"TempsDepuisLavage\": %lu",
                    noSerie,        newGenTimestamp,   eName.c_str(),     tempsOperEnCour,        tempsSeq1234,         tempsSeq4321,        tempsDepuisLavage);
  bool pubSuccess = Particle.publish(eName, msg, PRIVATE, NO_ACK);
  Log.info("publishTimeCounters: ");
  return pubSuccess;
}

// ***************************************************************
// Convert time counter to hour, minutes and seconds
// ***************************************************************
runTime secToHrMinSec(unsigned timeValue) {
  runTime t;
  t.totalSec = timeValue;
  t.hour = floor(t.totalSec / 3600);
  t.min = floor((t.totalSec - t.hour * 3600) / 60);
  t.sec = t.totalSec - (t.hour * 3600 + t.min * 60);
  return t;
}

// ***************************************************************
// Update screen counter
// ***************************************************************
void updateTimeCounters() {
  char tmp0[12];
  char tmp1[12];
  char tmp2[12];
  runTime rt0, rt1, rt2;

  if (System_state == marche) {
    rt0 = secToHrMinSec(tempsOperEnCour);
    sprintf(tmp0, "%02d:%02d:%02d", rt0.hour, rt0.min, rt0.sec);
    tempsOperEnCour++;

    if (System_mode == concentration1234) {
      rt1 = secToHrMinSec(tempsSeq1234);
      sprintf(tmp1, "%02d:%02d:%02d", rt1.hour, rt1.min, rt1.sec);
      tempsSeq1234++;
    } else if (System_mode == concentration4321) {
      rt1 = secToHrMinSec(tempsSeq4321);
      sprintf(tmp1, "%02d:%02d:%02d", rt1.hour, rt1.min, rt1.sec);
      tempsSeq4321++;
    } else {
      sprintf(tmp1, "%02d:%02d:%02d", 0, 0, 0);
      nexSerial.print("tSeqName.txt=\" - - - :\"\xFF\xFF\xFF");
    }
    
    rt2 = secToHrMinSec(tempsDepuisLavage);
    sprintf(tmp2, "%02d:%02d:%02d", rt2.hour, rt2.min, rt2.sec);
    if (System_mode != rinsage) {
      tempsDepuisLavage++;
    }

  }
  writeNextionTextData(tEnCourCntr, tmp0);
  writeNextionTextData(tSeqCntr, tmp1);
  writeNextionTextData(tSLavCntr, tmp2);

  // Log.info("updateTimeCounters: tempsOperEnCour = %lu, tempsSeq1234 = %lu, tempsSeq4321 = %lu, tempsDepuisLavage = %lu", tempsOperEnCour, tempsSeq1234, tempsSeq4321, tempsDepuisLavage);
}

void checkAlarm() {
  if (tempsDepuisLavage > depuisLavageTimeLimit) {
    nexSerial.print("status.pco=63488\xFF\xFF\xFF");
    alarmStatus = 3;
    digitalWrite(alarmBuzzer, HIGH);
    delay(50);
    digitalWrite(alarmBuzzer, LOW);
    delay(100);
    digitalWrite(alarmBuzzer, HIGH);
    delay(50);
    digitalWrite(alarmBuzzer, LOW);
    delay(100);
    digitalWrite(alarmBuzzer, HIGH);
    delay(50);
    digitalWrite(alarmBuzzer, LOW);
    delay(50);
  } else if (tempsSeq1234 > seqTimeLimit || tempsSeq4321 > seqTimeLimit) {
    nexSerial.print("status.pco=63488\xFF\xFF\xFF");
    alarmStatus = 2;
    digitalWrite(alarmBuzzer, HIGH);
    delay(50);
    digitalWrite(alarmBuzzer, LOW);
    delay(50);
  } else if ((operDataValid == false || brixDataValid == false) && tempsOperEnCour > dataEntryTimeLimit) {
    nexSerial.print("status.pco=63488\xFF\xFF\xFF");
    alarmStatus = 1;
    digitalWrite(alarmBuzzer, HIGH);
    delay(25);
    digitalWrite(alarmBuzzer, LOW);
    delay(25); 
    digitalWrite(alarmBuzzer, HIGH);
    delay(150);
    digitalWrite(alarmBuzzer, LOW);
    delay(25);
  } else {
    alarmStatus = 0;
  }
}