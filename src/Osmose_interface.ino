/*
 * Project Osmose_interface
 * Description:
 * Author:
 * Date:
 */

// STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
#include "../lib/ITEADLIB_Nextion/src/ITEADLIB_Nextion.h"
#define dbSerial Serial
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)

USARTSerial& nexSerial = Serial1;

// Définition des éléments du HMI Netion que l'on veut accéder
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

NexText allData = NexText(5, 7, "allData");
NexText oprData = NexText(5, 8, "oprData");

NexButton bm = NexButton(2, 34, "bm");
NexButton bp = NexButton(2, 33, "bp");

NexButton bOkOSM = NexButton(1, 36, "bOk");
NexButton bOkBRIX = NexButton(2, 25, "bOk");
NexButton bOkLAV = NexButton(3, 4, "bOk");
NexButton bOkRINC = NexButton(4, 4, "bOk");
NexButton bOkSUMM = NexButton(5, 1, "bOk");

NexVar fieldSelect = NexVar(2, 16, "flsel");

// Liste des objets que l'on veut surveiller
NexTouch *nex_listen_list[] = 
{
  &bp, &bm, &bOkOSM, &bOkBRIX, &bOkLAV, &bOkRINC, &bOkSUMM,
  NULL
};


// Variables globale
char buffer[200] = {0};
String filterSeq = "";

bool operDataValid = false;
bool brixDataValid = false;
double Seve = 0;
double Conc = 0; 
double Col1 = 0, Col2 = 0, Col3 = 0, Col4 = 0;
double Temp = 0;
double Pres = 0;

int heater = D3;                  // Contrôle le transistor du chauffage

/* Define a log handler on Serial1 for log messages */
SerialLogHandler logHandler(115200, LOG_LEVEL_INFO, {   // Logging level for non-application messages
    { "app", LOG_LEVEL_INFO }                      // Logging level for application messages
});


// Read text data from sourceField and convert it to double
double readNexTionData(NexText sourceField) {
  double data;
  memset(buffer, 0, sizeof(buffer)); // Clear buffer
  sourceField.getText(buffer, sizeof(buffer));
  data = atof(buffer);
  return data;
}


// Write data to destField after conversion to text from double
void writeNextionData(NexText destField, double data) {
  memset(buffer, 0, sizeof(buffer)); // Clear buffer
  sprintf(buffer, "%.1f", data);
  destField.setText(buffer);
}


// Write data to destField after conversion to text from double
void writeNextionTextData(NexText destField, String textData) {
  memset(buffer, 0, sizeof(buffer)); // Clear buffer
  sprintf(buffer, "%s", textData.c_str());
  destField.setText(buffer);
}


// Button PLUS(+) PUSH callback
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


// Button MINUS(-) PUSH callback
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


// BRIX PANEL button Ok PUSH callback
void bOkBRIXPushCallback(void *ptr) {  
  Seve = readNexTionData(brixSeve);
  Conc = readNexTionData(brixConc);
  brixDataValid = true;
  writeSummaryField();
  Log.info("bOkBRIXPushCallback: brix Sève= %.1f, Concentré= %.1f", Seve, Conc);
  // Send event
}


// OSMOSE PANEL button Ok PUSH callback
void bOkOSMPushCallback(void *ptr) {
  Col1 = readNexTionData(col1);
  Col2 = readNexTionData(col2);
  Col3 = readNexTionData(col3);
  Col4 = readNexTionData(col4);
  Conc = readNexTionData(conc);
  Temp = readNexTionData(temp);
  Pres = readNexTionData(pres);
  memset(buffer, 0, sizeof(buffer)); // Clear buffer
  seq.getText(buffer, sizeof(buffer));
  filterSeq = String(buffer);
  operDataValid = true;
  writeSummaryField();
  Log.info("bOkOSMPushCallback: c1= %.1f, c2= %.1f, c4= %.1f, c4= %.1f, Conc= %.1f, Temp=%.1f, Pres= %.0f, Seq: %s", Col1, Col2, Col3, Col4, Conc, Temp, Pres, buffer);
  // Send event
}


// Lavage PANEL button Ok PUSH callback
void bOkLAVPopCallback(void *ptr) {
  Log.info("bOkLAVPushCallback!");
}


// Rinçage PANEL button Ok PUSH callback
void bOkRINCPopCallback(void *ptr) {
  Log.info("bOkRINCPushCallback!");
}


// Rinçage PANEL button Ok PUSH callback
void bOkSUMMPopCallback(void *ptr) {
  Log.info("bOkSUMMPopCallback!");
}


// write to allData summary field
void writeSummaryField() {
  char tmp[80];
  if (operDataValid == true && brixDataValid == true){
    Log.info("  operDataValid= %d, brixDataValid= %d", operDataValid, brixDataValid);
    // sprintf(buffer, "%3.1f %8.1f  %6.1f  %6.1f  %6.1f  %6.1f  %6.1f  %5.1f  %5.0f  %s", Seve, Conc, Col1, Col2, Col3, Col4, Conc, Temp, Pres, filterSeq.c_str());
    sprintf(tmp, "%3.1f %8.1f  %6.1f  %6.1f  %6.1f  %6.1f  %6.1f  %5.1f  %5.0f", Seve, Conc, Col1, Col2, Col3, Col4, Conc, Temp, Pres);
    writeNextionTextData(allData, tmp);
    Log.info("  writeSummaryField: Wrote allData field!: buffer= %s", buffer);
  }
  Log.info("writeSummaryField: operDataValid= %d, brixDataValid= %d", operDataValid, brixDataValid);
}


// *** Main program ***
// setup() runs once, when the device is first turned on.
void setup() {
  nexInit();
  setBaudrate(115200);
  dbSerialBegin(115200);
  pinMode(heater, OUTPUT);
  analogWrite(heater, 0);
  Time.zone(-5);

// Attach callback routine to Nextion objects
  bm.attachPop(bmPopCallback);
  bp.attachPop(bpPopCallback);
  bOkBRIX.attachPush(bOkBRIXPushCallback);
  bOkOSM.attachPush(bOkOSMPushCallback);
  bOkLAV.attachPop(bOkLAVPopCallback);
  bOkRINC.attachPop(bOkRINCPopCallback);
  bOkSUMM.attachPop(bOkSUMMPopCallback);

  // nexSerial.print("rest");
  delay(2000);
  memset(buffer, 0, sizeof(buffer)); // Clear buffer
  // dbSerial.println("Démarrage");
  Log.info("Setup complete!");
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  nexLoop(nex_listen_list);
}