/*
 * Project Osmose_interface
 * Description: Osmose operation data logging
 * Author: Pierre Leboeuf
 * Date: 24 dec 2020
 * 
 */


// STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
#include "../lib/ITEADLIB_Nextion/src/ITEADLIB_Nextion.h"
#define dbSerial Serial
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define pumpDebounceDelay 100      // Debounce time in milliseconds for pump mechanical start/stop switch
#define pumpONstate 0             // Pump signal is active low.
#define pumpOFFstate 1            // Pump signal is active low.
#define histLength 5              // Number of line in data history
#define lineLength 100            // Max length of each line of history

// Firmware version et date
#define FirmwareVersion "0.5.0"   // Version du firmware du capteur.
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time; // Date et heure de compilation UTC


USARTSerial& nexSerial = Serial1;

// Définition des éléments du HMI Netion que l'on veut accéder
NexText tDateTime = NexText(0, 19, "tDateTime");

NexText col1 = NexText(1, 17, "col1");
NexText col2 = NexText(1, 18, "col2");
NexText col3 = NexText(1, 19, "col3");
NexText col4 = NexText(1, 20, "col4");
NexText conc = NexText(1, 21, "conc");
NexText temp = NexText(1, 22, "temp");
NexText pres = NexText(1, 23, "pres");
NexText seq = NexText(1, 24, "seq");

NexText brixSeve = NexText(2, 17, "brixSeve");
NexText brixConc = NexText(2, 18, "brixConc");

// Sommaire des données
NexText tl0 = NexText(5, 8, "tl0");
NexText tl1 = NexText(5, 9, "tl1");
NexText tl2 = NexText(5, 10, "tl2");
NexText tl3 = NexText(5, 11, "tl3");
NexText tl4 = NexText(5, 12, "tl4");
// Sommaire d'opération
NexText ts0 = NexText(5, 13, "ts0");
NexText ts1 = NexText(5, 14, "ts1");
NexText ts2 = NexText(5, 15, "ts2");
NexText ts3 = NexText(5, 16, "ts3");
NexText ts4 = NexText(5, 17, "ts4");

NexButton bGotoBRIX(0, 15, "bBrix");
NexButton bGotoSommaire(0, 16, "bSom");

NexButton bm = NexButton(2, 34, "bm");
NexButton bp = NexButton(2, 33, "bp");

NexButton bOkOSM = NexButton(1, 43, "bOk");
NexButton bOkBRIX = NexButton(2, 25, "bOk");
NexButton bOkLAV = NexButton(3, 4, "bOk");
NexButton bOkRINC = NexButton(4, 4, "bOk");
NexButton bOkSUMM = NexButton(5, 1, "bOk");
NexButton bSommaire = NexButton(5, 7, "bSommaire");

NexVar fieldSelect = NexVar(2, 16, "flsel");

// Liste des objets que l'on veut surveiller
NexTouch *nex_listen_list[] = 
{
  &bp, &bm, &bOkOSM, &bOkBRIX, &bOkLAV, &bOkRINC, 
  &bOkSUMM, &bSommaire, &bGotoSommaire, &seq,
  NULL
};

// Variables globale
enum Mode {concentration, lavage_normal, lavage_recirc, rinse, indefini};
enum Etat {marche, arret};

Mode System_mode = indefini;
Etat System_etat = arret;

unsigned tempsOperTotal;
unsigned tempsOperSequence;
unsigned tempsOperLavage;
unsigned tempsOperRinsage;

bool operDataValid = false;
bool brixDataValid = false;
double Seve = 0, Conc = 0; 
double Col1 = 0, Col2 = 0, Col3 = 0, Col4 = 0;
double Temp = 0, Pres = 0;
String filterSeq = "";

String hist_data[histLength];
String hist_perf[histLength];

int heater = D3; // Contrôle le transistor du chauffage
int optoInput = A1;

bool PumpCurrentState = pumpOFFstate;      // Initialize pump in the OFF state
bool PumpOldState = pumpOFFstate;          // Pour déterminer le chanement d'état

unsigned long currentTime;
unsigned long lastUpdateTime;
int previousSec = 0;
volatile unsigned long changeTime = 0;     // Moment du dernier changement d'état de la pompe

/* Define a log handler on Serial1 for log messages */
SerialLogHandler logHandler(115200, LOG_LEVEL_INFO, {   // Logging level for non-application messages
    { "app", LOG_LEVEL_INFO }                      // Logging level for application messages
});


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
        changeTime = Time.now();
      }
  };

  PumpState_A1 ThePumpState; // Instantiate the class A1State


// *** Main program ***
// setup() runs once, when the device is first turned on.
void setup() {
  nexInit();
  setBaudrate(115200);
  dbSerialBegin(115200);
  pinMode(heater, OUTPUT);
  analogWrite(heater, 0);
  Time.zone(-5);

  //Wait until Photon receives time from Particle Cloud (or connection to Particle Cloud is lost)
  if(Particle.connected()){
    Log.info("Connecté au nuage. :)");
    Particle.syncTime();
    if (not(Time.isValid()) || Time.year() < 2020) {
      Log.info("(setup) Syncing time ");
      Particle.syncTime();
      waitUntil(Particle.syncTimeDone);
      Log.info("(setup) syncTimeDone " + Time.timeStr());
      currentTime = Time.now();
      lastUpdateTime = currentTime;
      previousSec = Time.second();
    }
  }
  // Enregistrement des fonctions et variables disponible par le nuage
  Log.info("(setup) Enregistrement des variables et fonctions\n");
  Particle.variable("Version", FirmwareVersion);
  Particle.variable("Date", FirmwareDate);

  // Attach callback routine to Nextion objects
  bm.attachPop(bmPopCallback);
  bp.attachPop(bpPopCallback);
  bOkBRIX.attachPush(bOkBRIXPushCallback);
  bOkOSM.attachPush(bOkOSMPushCallback);
  bOkLAV.attachPop(bOkLAVPopCallback);
  bOkRINC.attachPop(bOkRINCPopCallback);
  bOkSUMM.attachPop(bOkSUMMPopCallback);
  bSommaire.attachPop(bSommairePopCallback);
  bGotoSommaire.attachPop(bGotoSommairePopCallback);
  seq.attachPop(seqPOPCallback);

  delay(500);
  nexSerial.print("rest\xFF\xFF\xFF");
  delay(500);
  nexSerial.print("page 0\xFF\xFF\xFF");

  Log.info("Setup complete!\n\n");
}
// **************************************************************


// **************************************************************
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  nexLoop(nex_listen_list);
  currentTime = Time.now();
  // Update time once every second
  if (Time.second() != previousSec) {
    writeNextionTextData(tDateTime, Time.timeStr());
    previousSec = Time.second();
  }

  if (PumpCurrentState != PumpOldState){
    if (PumpCurrentState == pumpONstate){
      // changer l'état du système
      System_etat = marche;
      // Turn On indicator
      // Set message to "Systeme en operation"
      // Démarrer le compteur de temps global, le mode d'opération n'est pas défini.
    } else {
      // changer l'état du système
      System_etat = marche;
      // Turn Off indicator
      // Set message to "Systeme en arret" 
      // Arrêter le compteur global et le compteur pour le mode d'opération courant
    }
  }
  // Sync time once a day
  if (currentTime > (lastUpdateTime + ONE_DAY_MILLIS)) {
    Particle.syncTime();
    lastUpdateTime = currentTime;
  }
}
// **************************************************************


// Read text data from sourceField and convert it to double
double readNexTionData(NexText sourceField) {
  double data;
  char buffer[20] = {0};
  sourceField.getText(buffer, sizeof(buffer));
  data = atof(buffer);
  Log.info("readNexTionData: buffer= %s", buffer);
  return data;
}


// Read text data from sourceField
String readNextionText(NexText sourceField) {
  String outText = "";
  char buffer[20] = {0};
  sourceField.getText(buffer, sizeof(buffer));
  Log.info("readNextionText: buffer= %s", buffer);
  outText = String(buffer);
  return outText;
}


// Write data to destField after conversion to text from double
void writeNextionData(NexText destField, double data) {
  char buffer[20] = {0};
  sprintf(buffer, "%.1f", data);
  destField.setText(buffer);
}


// Write data to destField after conversion to text from double
void writeNextionTextData(NexText destField, String textData) {
  char buffer[100] = {0};
  sprintf(buffer, "%s", textData.c_str());
  destField.setText(buffer);
}


// Button PLUS(+) POP callback
void bpPopCallback(void *ptr) {
  uint32_t flsel;
  double bSeve = 0;
  double bConc = 0;

  fieldSelect.getValue(&flsel);
  if (flsel == 17){
    bSeve = readNexTionData(brixSeve) + 0.1;
    writeNextionData(brixSeve, bSeve);
    Log.info("bpPopCallback: Bouton(+), seve: %.1f", bSeve);
  } else {
    bConc = readNexTionData(brixConc) + 0.1;
    writeNextionData(brixConc, bConc);
    Log.info("bpPopCallback: Bouton(+), conc: %.1f", bConc);
  }
}


// Button MINUS(-) POP callback
void bmPopCallback(void *ptr) {
  uint32_t flsel;
  double bSeve = 0;
  double bConc = 0;

  fieldSelect.getValue(&flsel);
  if (flsel == 17){
    bSeve = readNexTionData(brixSeve) - 0.1;
    if(bSeve < 0) bSeve = 0;
    writeNextionData(brixSeve, bSeve);
    Log.info("bmPopCallback: Bouton(-), seve: %.1f", bSeve);
  } else {
    bConc = readNexTionData(brixConc) - 0.1;
    if(bConc < 0) bConc = 0;
    writeNextionData(brixConc, bConc);
    Log.info("bmPopCallback: Bouton(-), conc: %.1f", bConc);
  }
}


void seqPOPCallback(void *ptr) {
  filterSeq = readNextionText(seq);
  Log.info("seqPOPCallback: filterSeq= %s", filterSeq.c_str());
}


// BRIX PANEL button Ok PUSH callback
void bOkBRIXPushCallback(void *ptr) {  
  brixDataValid = false;
  Seve = readNexTionData(brixSeve);
  Conc = readNexTionData(brixConc);

  if (Seve > 0 && Conc > 0) {
    brixDataValid = true;
  }
  
  if (operDataValid && brixDataValid) {
    delay (500);
    nexSerial.print("page 5\xFF\xFF\xFF");
    writeSummaryLine(true);
    delay (100);
    nexSerial.print("click bSommaire,0\xFF\xFF\xFF");
  }
  Log.info("bOkBRIXPushCallback: brix Seve= %.1f, Concentré= %.1f", Seve, Conc);
  // Send event
}


// OSMOSE PANEL button Ok PUSH callback
void bOkOSMPushCallback(void *ptr) {
  operDataValid = false;
  Col1 = readNexTionData(col1);
  Col2 = readNexTionData(col2);
  Col3 = readNexTionData(col3);
  Col4 = readNexTionData(col4);
  // delay(5);
  Conc = readNexTionData(conc);
  Temp = readNexTionData(temp);
  Pres = readNexTionData(pres);
  // filterSeq = readNextionText(seq);
  if (Col1 > 0 && Col2 > 0 && Col3 > 0 && Col4 >0 && Conc > 0 && Temp > 0 && Pres > 0 && filterSeq.length() > 6){
    operDataValid = true;
    nexSerial.print("bOkOSM.pco=GREEN\xFF\xFF\xFF");
  
  }
  Log.info("bOkOSMPushCallback: c1= %.1f, c2= %.1f, c3= %.1f, c4= %.1f, Conc= %.1f, Temp=%.1f, Pres= %.0f, Seq: %s", \
                                Col1, Col2, Col3, Col4, Conc, Temp, Pres, filterSeq.c_str());

  if (operDataValid && brixDataValid) {
    delay (100);
    nexSerial.print("page 5\xFF\xFF\xFF");
    writeSummaryLine(true);
    delay (100);
    // nexSerial.print("vis bSommaire,0\xFF\xFF\xFF");
  }
  Log.info("bOkOSMPushCallback: operDataValid= %d, brixDataValid= %d", operDataValid, brixDataValid);
  // Send event
}


// Sommaire button Ok POP callback
void bGotoSommairePopCallback(void *ptr) {
  delay (100);
  nexSerial.print("page 5\xFF\xFF\xFF");
  writeSummaryLine(true);
  delay (100);
  nexSerial.print("click bSommaire,0\xFF\xFF\xFF");
  // nexSerial.print("vis bSommaire,0\xFF\xFF\xFF");
  Log.info("bGotoSommairePopCallback!");
}

// Lavage PANEL button Ok POP callback
void bOkLAVPopCallback(void *ptr) {
  Log.info("bOkLAVPushCallback!");
}


// Rinçage PANEL button Ok POP callback
void bOkRINCPopCallback(void *ptr) {
  Log.info("bOkRINCPushCallback!");
}


// Rinçage PANEL button Ok POP callback
void bOkSUMMPopCallback(void *ptr) {
  Log.info("bOkSUMMPopCallback!");
}


// Rinçage PANEL button Ok POP callback
void bSommairePopCallback(void *ptr) {
  nexSerial.print("page 5\xFF\xFF\xFF");
  writeSummaryLine(true);
  Log.info("bSommairePopCallback!");
}

// History data management
void pushHistory(String hist[]) {
  for (int i = histLength - 1; i>2; i--) {
    hist[i] = hist[i-1];
    Log.info("pushHistory: hist[%d] = %s", i, hist[i].c_str());
  }
}


// write to all data summary field
void writeSummaryLine(bool pushData) {
  char tmp[100];
  // Entête  ligne 0 et 1
  hist_data[0] = "  Densite                    Debits                  Temperature   Pression   Sequence    Duree";
  hist_data[1] = "Seve   Conc.     Col1   Col2   Col3   Col4    Conc.      deg C     ls/po.ca.              hr:min";
  writeNextionTextData(tl0, hist_data[0]);
  writeNextionTextData(tl1, hist_data[1]);
  // Ligne 2: Données
  if (operDataValid == true && brixDataValid == true){
    if (pushData){ pushHistory(hist_data); }
    sprintf(tmp, "%4.1f %6.1f  %8.1f  %5.1f  %5.1f  %5.1f  %6.1f  %9.1f  %9.0f       %s    %s", 
                  Seve, Conc, Col1, Col2, Col3, Col4, Conc, Temp, Pres, filterSeq.c_str(), "02:30");
    hist_data[2] = String(tmp);
    writeNextionTextData(tl2, hist_data[2]);
    Log.info("writeSummaryLine: Wrote allData field!: tmp= %s", hist_data[2].c_str());
  } else {
    hist_data[2] = "Donnees invalides";
    writeNextionTextData(tl2, hist_data[2]);
  }
  writeNextionTextData(tl3, hist_data[3]);
  writeNextionTextData(tl4, hist_data[4]);
}