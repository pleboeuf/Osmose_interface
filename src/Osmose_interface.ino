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

#define sysOnMsg "System ON"
#define sysOffMsg "System OFF"
#define sysOsm "Osmose en cour"
#define sysLav "Lavage en cour"
#define sysRinse "Rincage en cour"

// Firmware version et date
#define FirmwareVersion "0.8.5" // Version du firmware du capteur.
String F_Date = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time; // Date et heure de compilation UTC

USARTSerial &nexSerial = Serial1;

// Définition des éléments du HMI Netion que l'on veut accéder
// Home page
NexText tDateTime = NexText(0, 19, "tDateTime");
NexText tstatus = NexText(0, 5, "status");
NexText duree = NexText(0, 6, "duree");
NexButton bGotoSommaire(0, 16, "bSom");
NexButton bGotoBRIX(0, 15, "bBrix");

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
NexButton bSommaire = NexButton(5, 7, "bSommaire");


// Liste des objets que l'on veut surveiller
NexTouch *nex_listen_list[] =
    {
        &bp, &bm, &bOkOSM, &bOkBRIX, &bOkLAV, &bOkRINC,
        &bOkSUMM, &bSommaire, &bGotoSommaire,
        &col1, &col2, &col3, &col4, &conc,
        &temp, &pres, &seq,
        NULL};

// Variables globale
enum Mode {
  concentration,
  lavage_normal,
  lavage_recirc,
  rinse,
  indefini
};
enum Etat {
  marche,
  arret
};
struct runTime {
  int totalSec;
  int hour;
  int min;
  int sec;
};
struct runTime duration;

Mode System_mode = indefini;
Etat System_state = arret;

unsigned startTime;
unsigned endTime;
unsigned tempsOperOsmTotal = 0;
unsigned tempsOperOsmSequence = 0;
unsigned tempsOperLavage = 0;
unsigned tempsOperRinsage = 0;

bool operDataValid = false;
bool brixDataValid = false;
double bSeve = 0, bConc = 0;
double Col1 = 0, Col2 = 0, Col3 = 0, Col4 = 0, dConc = 0;
double Temp = 0, Pres = 0;
String filterSeq = "";

String hist_data[histLength];
String hist_perf[histLength];

int heater = D3; // Contrôle le transistor du chauffage
int optoInput = A1;

bool PumpCurrentState = pumpOFFstate; // Initialize pump in the OFF state
bool PumpOldState = pumpOFFstate;     // Pour déterminer le chanement d'état

unsigned long currentTime;
unsigned long lastUpdateTime;
char now[12];
int previousSec = 0;
volatile unsigned long changeTime = 0; // Moment du dernier changement d'état de la pompe

/* Define a log handler on Serial1 for log messages */
SerialLogHandler logHandler(115200, LOG_LEVEL_INFO, {
                                                        // Logging level for non-application messages
                                                        {"app", LOG_LEVEL_INFO} // Logging level for application messages
                                                    });

/*
  Attach interrupt handler to pin A1 to monitor pump Osmose Start/Stop
*/
class PumpState_A1
{
public:
  PumpState_A1()
  {
    pinMode(A1, INPUT);
    attachInterrupt(A1, &PumpState_A1::A1Handler, this, CHANGE);
  }
  void A1Handler()
  {
    // IMPORTANT: Pump is active LOW. Pump is ON when PumpCurrentState == false
    delayMicroseconds(pumpDebounceDelay * 1000);
    PumpCurrentState = digitalRead(A1);
    changeTime = millis();
  }
};
PumpState_A1 ThePumpState; // Instantiate the class A1State

// *** Main program ***
// setup() runs once, when the device is first turned on.
void setup()
{
  nexInit();
  setBaudrate(115200);
  dbSerialBegin(115200);
  pinMode(heater, OUTPUT);
  analogWrite(heater, 0);
  Time.zone(-5);

  //Wait until Photon receives time from Particle Cloud (or connection to Particle Cloud is lost)
  if (Particle.connected())
  {
    Log.info("Connecté au nuage. :)");
    Particle.syncTime();
    if (not(Time.isValid()) || Time.year() < 2020)
    {
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
  col1.attachPop(col1POPCallback);
  col2.attachPop(col2POPCallback);
  col3.attachPop(col3POPCallback);
  col4.attachPop(col4POPCallback);
  conc.attachPop(concPOPCallback);
  temp.attachPop(tempPOPCallback);
  pres.attachPop(presPOPCallback);
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
void loop()
{
  // The core of your code will likely live here.
  nexLoop(nex_listen_list);
  currentTime = Time.now();
  // Update time once every second
  if (Time.second() != previousSec)
  {
    writeNextionTextData(tDateTime, Time.timeStr());
    // Check for system state and opr. mode
    checkSysState();
    previousSec = Time.second();
  }

  // Sync time once a day
  if (currentTime > (lastUpdateTime + ONE_DAY_MILLIS))
  {
    Particle.syncTime();
    lastUpdateTime = currentTime;
  }
}
// **************************************************************

// Read text data from sourceField and convert it to double
double readNexTionData(NexText sourceField)
{
  double data;
  char buffer[20] = {0};
  sourceField.getText(buffer, sizeof(buffer));
  data = atof(buffer);
  Log.info("readNexTionData: buffer= %s", buffer);
  return data;
}

// Read text data from sourceField
String readNextionText(NexText sourceField)
{
  String outText = "";
  char buffer[20] = {0};
  sourceField.getText(buffer, sizeof(buffer));
  Log.info("readNextionText: buffer= %s", buffer);
  outText = String(buffer);
  return outText;
}

// Write data to destField after conversion to text from double
void writeNextionData(NexText destField, double data)
{
  char buffer[20] = {0};
  sprintf(buffer, "%.1f", data);
  destField.setText(buffer);
}

// Write data to destField after conversion to text from double
void writeNextionTextData(NexText destField, String textData)
{
  char buffer[100] = {0};
  sprintf(buffer, "%s", textData.c_str());
  destField.setText(buffer);
}

// Button PLUS(+) POP callback
void bpPopCallback(void *ptr)
{
  uint32_t flsel;
  double bSeve = 0;
  double bConc = 0;

  fieldSelect.getValue(&flsel);
  if (flsel == 17)
  {
    bSeve = readNexTionData(brixSeve) + 0.1;
    writeNextionData(brixSeve, bSeve);
    Log.info("bpPopCallback: Bouton(+), seve: %.1f", bSeve);
  }
  else
  {
    bConc = readNexTionData(brixConc) + 0.1;
    writeNextionData(brixConc, bConc);
    Log.info("bpPopCallback: Bouton(+), conc: %.1f", bConc);
  }
}

// Button MINUS(-) POP callback
void bmPopCallback(void *ptr)
{
  uint32_t flsel;
  double bSeve = 0;
  double bConc = 0;
  fieldSelect.getValue(&flsel);
  if (flsel == 17)
  {
    bSeve = readNexTionData(brixSeve) - 0.1;
    if (bSeve < 0)
      bSeve = 0;
    writeNextionData(brixSeve, bSeve);
    Log.info("bmPopCallback: Bouton(-), seve: %.1f", bSeve);
  }
  else
  {
    bConc = readNexTionData(brixConc) - 0.1;
    if (bConc < 0)
      bConc = 0;
    writeNextionData(brixConc, bConc);
    Log.info("bmPopCallback: Bouton(-), conc: %.1f", bConc);
  }
}

// BRIX PANEL button Ok PUSH callback
void bOkBRIXPushCallback(void *ptr)
{
  brixDataValid = false;
  bSeve = readNexTionData(brixSeve);
  bConc = readNexTionData(brixConc);
  if (bSeve > 0 && bConc > 0)
  {
    brixDataValid = true;
    delay(10);
    nexSerial.print("bBrix.pco=GREEN\xFF\xFF\xFF");
  }
  if (operDataValid && brixDataValid)
  {
    delay(500);
    nexSerial.print("page 5\xFF\xFF\xFF");
    nowTime();
    writeSummaryLine(true);
    writeOperSummaryLine(true);
    delay(100);
    nexSerial.print("click bSommaire,0\xFF\xFF\xFF");
  }
  Log.info("bOkBRIXPushCallback: brix Seve= %.1f, Concentré= %.1f", bSeve, bConc);
  // Send event
}

// OSMOSE PANEL button Ok PUSH callback
void bOkOSMPushCallback(void *ptr)
{
  operDataValid = false;
  Col1 = readNexTionData(col1);
  Col2 = readNexTionData(col2);
  Col3 = readNexTionData(col3);
  Col4 = readNexTionData(col4);
  delay(5);
  for (int i = 0; i < 10; i++)
  {
    dConc = readNexTionData(conc);
    if (dConc > 0.0)
      break;
  }
  for (int i = 0; i < 10; i++)
  {
    Temp = readNexTionData(temp);
    if (Temp > 0.0)
      break;
  }
  for (int i = 0; i < 10; i++)
  {
    Pres = readNexTionData(pres);
    if (Pres > 0.0)
      break;
  }

  // filterSeq = readNextionText(seq);
  if (Col1 > 0 && Col2 > 0 && Col3 > 0 && Col4 > 0 && dConc > 0 && Temp > 0 && Pres > 0 && filterSeq.length() > 6)
  {
    operDataValid = true;
    delay(10);
    nexSerial.print("bOsmose.pco=GREEN\xFF\xFF\xFF");
  }
  Log.info("bOkOSMPushCallback: c1= %.1f, c2= %.1f, c3= %.1f, c4= %.1f, Conc= %.1f, Temp=%.1f, Pres= %.0f, Seq: %s",
           Col1, Col2, Col3, Col4, dConc, Temp, Pres, filterSeq.c_str());

  // if (operDataValid && brixDataValid) {
  //   delay (100);
  //   nexSerial.print("page 5\xFF\xFF\xFF");
  //   writeSummaryLine(true);
  //   writeOperSummaryLine(true);
  //   delay (100);
  //   // nexSerial.print("vis bSommaire,0\xFF\xFF\xFF");
  // }
  Log.info("bOkOSMPushCallback: operDataValid= %d, brixDataValid= %d", operDataValid, brixDataValid);
  // Send event
}

void col1POPCallback(void *ptr)
{
  // Col1 = readNexTionData(col1);
  Log.info("col1POPCallback: c1= %.1f", Col1);
}

void col2POPCallback(void *ptr)
{
  // Col2 = readNexTionData(col2);
  Log.info("col2POPCallback: c2= %.1f", Col2);
}

void col3POPCallback(void *ptr)
{
  // Col3 = readNexTionData(col3);
  Log.info("col3POPCallback: c3= %.1f", Col3);
}

void col4POPCallback(void *ptr)
{
  // Col4 = readNexTionData(col4);
  Log.info("col4POPCallback: c4= %.1f", Col4);
}

void concPOPCallback(void *ptr)
{
  // Conc = readNexTionData(conc);
  Log.info("concPOPCallback: conc= %.1f", dConc);
}

void tempPOPCallback(void *ptr)
{
  // Temp = readNexTionData(temp);
  Log.info("tempPOPCallback: temp= %.1f", Temp);
}

void presPOPCallback(void *ptr)
{
  // Pres = readNexTionData(pres);
  Log.info("presPOPCallback: pres= %.1f", Pres);
}

void seqPOPCallback(void *ptr)
{
  filterSeq = readNextionText(seq);
  Log.info("seqPOPCallback: filterSeq= %s", filterSeq.c_str());
}

// Sommaire POP callback
void bGotoSommairePopCallback(void *ptr)
{
  delay(100);
  nexSerial.print("page 5\xFF\xFF\xFF");
  writeSummaryLine(false);
  writeOperSummaryLine(false);
  delay(100);
  nexSerial.print("click bSommaire,0\xFF\xFF\xFF");
  Log.info("bGotoSommairePopCallback!");
}

// Lavage PANEL button Ok POP callback
void bOkLAVPopCallback(void *ptr)
{
  Log.info("bOkLAVPushCallback!");
}

// Rinçage PANEL button Ok POP callback
void bOkRINCPopCallback(void *ptr)
{
  Log.info("bOkRINCPushCallback!");
}

// Sommaire PANEL button Ok POP callback
void bOkSUMMPopCallback(void *ptr)
{
  Log.info("bOkSUMMPopCallback!");
}

// Rinçage PANEL button Ok POP callback
void bSommairePopCallback(void *ptr)
{
  nexSerial.print("page 5\xFF\xFF\xFF");
  nowTime();
  writeSummaryLine(true);
  writeOperSummaryLine(true);
  Log.info("bSommairePopCallback!");
}

// History data management
void pushHistory(String hist[])
{
  if (hist[2].startsWith("Donnees"))
    hist[2] = "";
  for (int i = histLength - 1; i > 2; i--)
  {
    hist[i] = hist[i - 1];
    Log.info("pushHistory: hist[%d] = %s", i, hist[i].c_str());
  }
}

void nowTime()
{
  sprintf(now, "%2d/%02d %2d:%02d", Time.month(), Time.day(), Time.hour(), Time.minute());
}

// write to all data summary field
void writeSummaryLine(bool pushData)
{
  char tmp[105];
  // Entête  ligne 0 et 1
  hist_data[0] = "   Debut    Densite (Brix)            Debits (GPM)              Temp.   Pres.    Seq.      Duree";
  hist_data[1] = "date  heure  Seve   Conc.   Col1   Col2   Col3   Col4   Conc.  (deg C)  (PSI)              hr:min";
  writeNextionTextData(tl0, hist_data[0]);
  writeNextionTextData(tl1, hist_data[1]);
  // Ligne 2: Données
  if (operDataValid == true && brixDataValid == true)
  {
    if (pushData)
      pushHistory(hist_data);
    sprintf(tmp, "%11s %5.1f %6.1f  %6.1f  %5.1f  %5.1f  %5.1f  %5.1f  %6.1f  %6.0f  %8s %9s",
            now, bSeve, bConc, Col1, Col2, Col3, Col4, dConc, Temp, Pres, filterSeq.c_str(), "02:30");
    hist_data[2] = String(tmp);
    writeNextionTextData(tl2, hist_data[2]);
    Log.info("writeSummaryLine: Wrote allData field!: tmp= %s", hist_data[2].c_str());
  }
  else
  {
    hist_data[2] = "Donnees invalides";
    writeNextionTextData(tl2, hist_data[2]);
  }
  writeNextionTextData(tl3, hist_data[3]);
  writeNextionTextData(tl4, hist_data[4]);
  writeNextionTextData(tl5, hist_data[5]);
}

// write to all data summary field
void writeOperSummaryLine(bool pushData)
{
  char tmp[100];
  float debitFiltratgpm;
  float debitTotalgpm;
  float pcConc;
  // Entête  ligne 0 et 1
  hist_perf[0] = "   Debut             Concentre              Filtrat         Total          Duree";
  hist_perf[1] = "date  heure    % concen.  Debit (GPH)     Debit (GPH)    Debit (GPH)       hr:min";
  writeNextionTextData(ts0, hist_perf[0]);
  writeNextionTextData(ts1, hist_perf[1]);
  // Ligne 2: Données
  if (operDataValid == true && brixDataValid == true)
  {
    if (pushData)
      pushHistory(hist_perf);
    debitFiltratgpm = Col1 + Col2 + Col3 + Col4;
    debitTotalgpm = debitFiltratgpm + dConc;
    pcConc = 100 * debitFiltratgpm / debitTotalgpm;
    sprintf(tmp, "%11s  %8.0f %11.0f  %14.0f  %13.0f %15s", now, pcConc, 60 * dConc, 60 * debitFiltratgpm, 60 * debitTotalgpm, "02:30");
    hist_perf[2] = String(tmp);
    writeNextionTextData(ts2, hist_perf[2]);
    Log.info("writeOperSummaryLine: Wrote allData field!: tmp= %s", hist_perf[2].c_str());
  }
  else
  {
    hist_perf[2] = "Donnees invalides";
    writeNextionTextData(ts2, hist_perf[2]);
  }
  writeNextionTextData(ts3, hist_perf[3]);
  writeNextionTextData(ts4, hist_perf[4]);
  writeNextionTextData(ts5, hist_perf[5]);
}


void checkSysState()
{
  if (PumpCurrentState != PumpOldState)
  {
    if (PumpCurrentState == pumpONstate)
    {
      // changer l'état du système
      System_state = marche;
      // Turn On indicator
      nexSerial.print("ind.txt=ON\xFF\xFF\xFF");
      nexSerial.print("ind.pco=GREEN\xFF\xFF\xFF");
      // Set message to "Systeme en operation"
      writeNextionTextData(tstatus, sysOnMsg);
      nexSerial.print("vis duree.id,1\xFF\xFF\xFF");
      // Démarrer le compteur de temps global, le mode d'opération n'est pas défini.
      startTime = Time.now();
      Log.info("checkSysState: Système en marche!");
    }
    else
    {
      // changer l'état du système
      System_state = arret;
      // Turn Off indicator
      nexSerial.print("ind.txt=OFF\xFF\xFF\xFF");
      nexSerial.print("ind.pco=RED\xFF\xFF\xFF");
      // Set message to "Systeme en arret"
      writeNextionTextData(tstatus, sysOffMsg);
      // nexSerial.print("vis duree.id,0\xFF\xFF\xFF");
      // Arrêter le compteur global et le compteur pour le mode d'opération courant
      endTime = Time.now();
      Log.info("checkSysState: Système en arrêt!");
    }
    PumpOldState = PumpCurrentState;
  } else {
    // Afficher le temps d'opération
    if (PumpCurrentState == pumpONstate) {
      // update running time counter
      time32_t runTimeInSec = Time.now() - startTime;
      unsigned hour = floor(runTimeInSec / 3600);
      unsigned min = floor((runTimeInSec - hour * 3600) / 60);
      unsigned sec = runTimeInSec - (hour * 3600 + min * 60);
      char tmp[35];
      sprintf(tmp, "%02d:%02d:%02d", hour, min, sec);
      writeNextionTextData(duree, tmp);
      // Log.info("checkSysState: runtime= %.0f, hour= %d, min= %d, sec= %d", runTimeInSec, hour, min, sec);
    }
  }
}
