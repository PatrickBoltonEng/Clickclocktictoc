/*
 * Project Clickclocktictoc
 * Description:
 * Author:  PJB
 * Date:  05/21/21
 */

#include "Particle.h"
#include "Nextion.h"
#include "math.h"
#include "Adafruit_BME280.h"
#include "Seeed_HM330X.h"
#include "JsonParserGeneratorRK.h"

SYSTEM_THREAD(ENABLED);

USARTSerial& nexSerial = Serial1;

#define DEBUG_SERIAL_ENABLE
#define dbSerial Serial

#define UPDATE_INTERVAL 15000  //1 sec = 15000 millis

Adafruit_BME280 bme;
HM330X hm330;
uint8_t buf[30];
uint16_t buf2[30];

const char* hm330x_str[] = {"Sensor#: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                     "Particle Count >0.3um/L: ", "Particle Count >0.5um/L: ", "Particle Count >1.0um/L: ",
                     "Particle Count >2.5um/L: ", "Particle Count >5.0um/L: ", "Particle Count >10 um/L: ",
                    };

uint16_t hm330x_val[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

unsigned long updateInterval;

float TC, PPa, RH, TF, Ppsi, HI, DPF;
int PM10S, PM25S, PM100S, PM10E, PM25E, PM100E, PC03, PC05, PC10, PC25, PC50, PC100;
int hr_i, min_i, ampm_i, weekday_i, month_i, day_i, min_time, min_last;
float NO2=12.345, C2H5OH=12.345, VOC=12.345, CO=1.2345; 
int CO2=123, CO2TC=123;
float UVAP=1.23, UVI12=0.123;
int UVI16=123, VIS16=123, IR16=123;


String ampm_s, weekday_s, month_s;

char buffer[100] = {0};

JsonParserStatic<512, 25> Gas_parser;
JsonParserStatic<512, 25> Light_parser;

SerialLogHandler logHandler(LOG_LEVEL_INFO);

//page0
NexNumber n0 = NexNumber(0, 3, "n0");       //hour
NexNumber n1 = NexNumber(0, 4, "n1");       //minute
NexText t0 = NexText(0, 5, "t0");           //AM-PM 
NexText t1 = NexText(0, 6, "t1");           //weekday 
NexText t2 = NexText(0, 7, "t2");           //month 
NexNumber n2 = NexNumber(0, 8, "n2");       //day-date

//page 1
NexNumber n3 = NexNumber(1, 4, "n3");       //Integer Temperature x 10 (for x0 to be 0.1 precision) pg 1
NexNumber n4 = NexNumber(1, 7, "n4");       //Integer Pressure x 1000 (for x1 to be 0.001 precision) pg 1
NexNumber n5 = NexNumber(1, 10, "n5");       //Integer Humidity x 10 (for x2 to be 0.1 precision) pg 1
NexNumber n6 = NexNumber(1, 13, "n6");       //Integer Heat Index x 10 (for x3 to be 0.1 precision) pg 1
NexNumber n7 = NexNumber(1, 16, "n7");       //Integer Dew Point x 10 (for x4 to be 0.1 precision) pg 1

//page 2
NexNumber n8 = NexNumber(2, 3, "n8");         //PM1.0Std
NexNumber n9 = NexNumber(2, 10, "n9");        //PM2.5Std
NexNumber n10 = NexNumber(2, 13, "n10");      //PM10 Std                  
NexNumber n11 = NexNumber(2, 8, "n11");       //PM1.0env
NexNumber n12 = NexNumber(2, 11, "n12");      //PM2.5env 
NexNumber n13 = NexNumber(2, 14, "n13");      //PM10 env 
NexNumber n14 = NexNumber(2, 22, "n14");      //PC >0.3um 
NexNumber n15 = NexNumber(2, 23, "n15");      //PC >0.5um 
NexNumber n16 = NexNumber(2, 24, "n16");      //PC >1.0um 
NexNumber n17 = NexNumber(2, 25, "n17");      //PC >2.5um 
NexNumber n18 = NexNumber(2, 26, "n18");      //PC >5.0um 
NexNumber n19 = NexNumber(2, 27, "n19");      //PC >10 um

//page3
NexNumber n20 = NexNumber(3, 2, "n20");       //Si1145 UVI
NexNumber n21 = NexNumber(3, 7, "n21");       //Si1145 VIS
NexNumber n22 = NexNumber(3, 9, "n22");       //Si1145 IR
NexNumber n23 = NexNumber(3, 19, "n23");      //GUVA-S12D UVI 

//page4
NexNumber n24 = NexNumber(4, 7, "n24");      //GM-102B NO2
NexNumber n25 = NexNumber(4, 8, "n25");      //GM-302B EtOH
NexNumber n26 = NexNumber(4, 9, "n26");      //GM-502B VOC
NexNumber n27 = NexNumber(4, 10, "n27");     //GM-702B CO
NexNumber n28 = NexNumber(4, 15, "n28");     //MH-Z16 CO2

NexTouch *nex_listen_list[] = 
{ 
  NULL
};

void setup() 
{
  Particle.subscribe("GasesGracias", GasHandler, MY_DEVICES);
  Particle.subscribe("Inthelight", LightHandler, MY_DEVICES);
  
  Serial.begin(9600);
  delay(100);

  Serial1.begin(9600);
  delay(100);

  nexInit();
  delay(100);

  if (bme.begin()){Log.info("BME280 Sensor Initialized.");}
  else{Log.info("BME280 init failed");}

  if (hm330.init()) {
    Log.info("HM330X init failed!!!");
    while (1);
  }
  updateInterval = millis();
  min_last=Time.minute();
}

void loop() 
{
  nexLoop(nex_listen_list);
  

  if(Particle.disconnected()){return;}

  HI=1000.0f;
  DPF=1000.0f;

  if ((millis() - updateInterval) > UPDATE_INTERVAL)
  {
    timegrab();

    n0.setValue(hr_i);
    n1.setValue(min_i);
    t0.setText(ampm_s);
    t1.setText(weekday_s);
    t2.setText(month_s);
    n2.setValue(day_i);
     
    getBMEValues(TC, PPa, RH);
    Log.info("Temperature(C): %f", TC);
    Log.info("Pressure(Pa): %f", PPa);
    Log.info("RelativeHumidity(%%): %f", RH);

    TF = (9.0f * TC / 5.0f) + 32.0f;    // TF in Fahrenheit
    Log.info("Temperature(F): %f", TF);

    Ppsi = (PPa / 6894.75729f);         // Pressure converted from Pa to psia
    Log.info("Pressure(psia): %f", Ppsi);

    CalcHeatIndex(TF, RH, HI);
    Log.info("HeatIndex(F): %f", HI);

    CalcDewPoint(TC, RH, DPF);
    Log.info("DewPoint(F): %f", DPF);

    n3.setValue(int(TF*10));      //formating for display with 0.1 precision
    n4.setValue(int(Ppsi*1000));  //formating for display with 0.001 precision
    n5.setValue(int(RH*10));     //formating for display with 0.1 precision
    n6.setValue(int(HI*10));      //formating for display with 0.1 precision
    n7.setValue(int(DPF*10));     //formating for display with 0.1 precision

    getparticles();
   
    n8.setValue(PM10S);
    n9.setValue(PM25S);
    n10.setValue(PM100S);
    n11.setValue(PM10E);
    n12.setValue(PM25E);     
    n13.setValue(PM100E);
    n14.setValue(PC03);
    n15.setValue(PC05);
    n16.setValue(PC10);
    n17.setValue(PC25);
    n18.setValue(PC50);
    n19.setValue(PC100);

    n20.setValue(UVI16);
    n21.setValue(VIS16);
    n22.setValue(IR16);
    n23.setValue(int(UVI12*10000.0));

    n24.setValue(int(NO2*1000.0));
    n25.setValue(int(C2H5OH*1000.0));
    n26.setValue(int(VOC*1000.0));
    n27.setValue(int(CO*1000.0));
    n28.setValue(CO2);

    min_time=Time.minute();
    if((min_time!=min_last)&&(min_time==0||min_time==5||min_time==10||min_time==15||min_time==20||min_time==25||min_time==30||
        min_time==35||min_time==40||min_time==45||min_time==50||min_time==55))
    {
      createEventPayload(TF, Ppsi, RH, HI, DPF, PM10S, PM25S, PM100S, PM10E, PM25E, PM100E, PC03, PC05, PC10, PC25, PC50, PC100);
      min_last = min_time;
      Log.info("Last Update: %d", min_last);
    }
            
    updateInterval = millis();

  }
}

void GasHandler(const char *event, const char *data)
{
  Log.info(event);
  Gas_parser.addString(data);
  Gas_parser.parse();
  
  Gas_parser.getOuterValueByKey("NO2ppm", NO2);
  Log.info("NO2ppm: %f", NO2);

  Gas_parser.getOuterValueByKey("C2H5OHppm", C2H5OH);
  Log.info("C2H5OHppm: %f", C2H5OH);

  Gas_parser.getOuterValueByKey("VOCppm", VOC);
  Log.info("VOCppm: %f", VOC);

  Gas_parser.getOuterValueByKey("COppm", CO);
  Log.info("COppm: %f", CO);

  Gas_parser.getOuterValueByKey("CO2ppm", CO2);
  Log.info("CO2ppm: %d", CO2);
    
  Gas_parser.getOuterValueByKey("CO2_T(C)", CO2TC);
  Log.info("CO2_TC: %d", CO2TC);  
  
  Gas_parser.clear();

  return;
}

void LightHandler(const char *event, const char *data)
{
  Log.info(event);
  Light_parser.addString(data);

  Light_parser.parse();

  Light_parser.getOuterValueByKey("UVAP12", UVAP);
  Log.info("UVAP12: %f", UVAP);

  Light_parser.getOuterValueByKey("UVI12", UVI12);
  Log.info("UVI12: %f", UVI12);

  Light_parser.getOuterValueByKey("UVI16", UVI16);
  Log.info("UVI16: %d", UVI16);

  Light_parser.getOuterValueByKey("VIS16", VIS16);
  Log.info("VIS16: %d", VIS16);

  Light_parser.getOuterValueByKey("IR16", IR16);
  Log.info("IR16: %d", IR16);

  Light_parser.clear();
  return;
}

float getBMEValues(float &TC, float &PPa, float &RH)
{
  TC = bme.readTemperature();
  //T in C
  PPa = bme.readPressure();
  // Pressure in Pa
    RH = bme.readHumidity();
  // RH in %
  return 1;
}

float CalcHeatIndex(float &TF, float &RH, float &HI)
{   
  // Heat Index Calculations from nws
  // HIsimple if TF < 80
  if(TF<=80)
  {
    float HIsimple = (TF + 61.0f + (1.2f*(TF-68.0f)) + (0.094*RH)) / 2.0f;
    HI = HIsimple;
  }
  // Rothfusz regression base equation
  float HIbasic = -42.379f + (2.04901523f*TF) + (10.14333127f*RH) -(0.22475541f*TF*RH) 
       - (0.00683783f*TF*TF) - (0.05481717f*RH*RH) + (0.00122874f*TF*TF*RH) 
       + (0.00085282f*TF*RH*RH) - (0.00000199f*TF*TF*RH*RH);

  //High humidity add correction RH> 85%, 80F<T<87;
  if(TF>80.0f && TF<=87.0f && RH>=85.0f)
  {
    float HIwet = (RH-85.0f)*(78.0f-TF)/50.0f;
    HI = HIbasic + HIwet;
  }
    
  //Low humidity correction RH<13, T from 80 to 112
  // float HIhigh = ((13.0f-RH)/4.0f)*sqrt((17-abs(TF-95))/17.0f)
  // break into cases to avoid undefined abs() which appears to only be for integers
  // float Hihigh1 = ((13.0f-RH)/4.0f)*sqrt((17-(95-TF))/17.0f) for 78F<T<95F
  if(TF>80.0f && TF<=95.0f && RH<=13.0f)
  {
    float HIhigh1 = ((13.0f-RH)/4.0f)*sqrt((17-(95-TF))/17.0f);
    HI = HIbasic - HIhigh1;
  }

  if(TF>95.0f && TF<=112.0f && RH<=13.0f)
  {
    float HIhigh2 = ((13.0f-RH)/4.0f)*sqrt((17-(TF-95))/17.0f);
    HI = HIbasic - HIhigh2;
  }
  else
  {
    HI = HIbasic;
  }
  return HI;
}

float CalcDewPoint(float &TC, float &RH, float &DPF)
{
  //Dewpoint by Magus Equation with Sonntag 1990 constants
  // a=6.112 mbar, b=17.62, c=243.12 C with error < 0.1% for -45C to 60C (+/- 0.35C)
  float a = 6.112f;
  // in mbar
  float b = 17.62f;
  float c = 243.12f;
  float  gamma = log(RH/100.0f) + (b*TC/(c+TC));
  float Pact = a*exp(gamma);
  // actual vapor pressure in mbar
  float DPC = (c*log(Pact/a)) / (b - log(Pact/a));
  // dew point temperature in C
  DPF = ((9.0f * DPC) / 5.0f) + 32.0f;
  // dew point temperature in F
  return DPF;
}

void timegrab()
{
  Time.zone(-7);

  hr_i=Time.hourFormat12();

  min_i=Time.minute();

  ampm_i = Time.isPM();

  if(ampm_i==1){ampm_s=String("PM");}
  else{ampm_s=String("AM");}

  weekday_i=Time.weekday();

  if(weekday_i==1){weekday_s=String("SUN");}
  if(weekday_i==2){weekday_s=String("MON");}
  if(weekday_i==3){weekday_s=String("TUES");}
  if(weekday_i==4){weekday_s=String("WED");}
  if(weekday_i==5){weekday_s=String("THUR");}
  if(weekday_i==6){weekday_s=String("FRI");}
  if(weekday_i==7){weekday_s=String("SAT");}
            
  month_i=Time.month();

  if(month_i==1){month_s=String("JAN");}
  if(month_i==2){month_s=String("FEB");}
  if(month_i==3){month_s=String("MAR");}
  if(month_i==4){month_s=String("APR");}
  if(month_i==5){month_s=String("MAY");}
  if(month_i==6){month_s=String("JUNE");}
  if(month_i==7){month_s=String("JULY");}
  if(month_i==8){month_s=String("AUG");}
  if(month_i==9){month_s=String("SEPT");}
  if(month_i==10){month_s=String("OCT");}
  if(month_i==11){month_s=String("NOV");}
  if(month_i==12){month_s=String("DEC");}
  day_i=Time.day();

  /*
  Log.info("Hour: %d", hr_i);
  Log.info("Min: %d", min_i);
  Log.info("Is PM: %d", ampm_i);
  Log.info(ampm_s);
  Log.info("Weekday: %d", weekday_i);
  Log.info(weekday_s);
  Log.info("Month: %d", month_i);
  Log.info(month_s);
  Log.info("Day: %d", day_i);
  */

  return;
}

void getparticles()
{
  if (hm330.hm330x_read_sensor_value(buf, 29))
  {
    Log.info("HM330X read result failed!!!");
  }
  hm330x_parse_result_value(buf);
  hm330x_parse_result(buf);
  
  Log.info("PM10S: %d", PM10S);
  Log.info("PM25S: %d", PM25S);
  Log.info("PM100S: %d", PM100S);
  Log.info("PM10E: %d", PM10E);
  Log.info("PM25E: %d", PM25E);
  Log.info("PM100E: %d", PM100E);
  Log.info("P03: %d", PC03);
  Log.info("P05: %d", PC05);
  Log.info("P10: %d", PC10);
  Log.info("P25: %d", PC25);
  Log.info("P50: %d", PC50);
  Log.info("P100: %d", PC100);
  
  return;
}

HM330XErrorCode hm330x_print_result(const char* hm330x_str, uint16_t hm330x_value) 
{
    if (NULL == hm330x_str) {
        return ERROR_PARAM;
    }
    //Log.info(hm330x_str);
    //Log.info(String(hm330x_value));
    return NO_ERR;
}

HM330XErrorCode hm330x_parse_result(uint8_t* hm330x_data) 
{
    uint16_t hm330x_value = 0;
    if (NULL == hm330x_data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 14; i++) 
    {
       hm330x_value = (uint16_t) hm330x_data[i * 2] << 8 | hm330x_data[i * 2 + 1];
       hm330x_val[i-1] = hm330x_value;
       hm330x_print_result(hm330x_str[i-1], hm330x_val[i-1]);  
    }
    PM10S     =   hm330x_val[1];
    PM25S     =   hm330x_val[2];
    PM100S    =   hm330x_val[3];
    PM10E     =   hm330x_val[4];
    PM25E     =   hm330x_val[5];
    PM100E    =   hm330x_val[6];
    PC03      =   hm330x_val[7];
    PC05      =   hm330x_val[8];
    PC10      =   hm330x_val[9];
    PC25      =   hm330x_val[10];
    PC50      =   hm330x_val[11];
    PC100     =   hm330x_val[12];
    return NO_ERR;
}

HM330XErrorCode hm330x_parse_result_value(uint8_t* hm330x_data) 
{
    if (NULL == hm330x_data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        //Log.info(String(data[i], HEX));
        
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += hm330x_data[i];
    }
    if (sum != hm330x_data[28]) {
        Log.info("HM330x CheckSum Error");
    }
    return NO_ERR;
}

void createEventPayload(float TF, float Ppsi, float RH, float HI, float DPF, int PM10S, int PM25S, 
                        int PM100S, int PM10E, int PM25E, int PM100E, int PC03, int PC05, int PC10, 
                        int PC25, int PC50, int PC100)
{
  JsonWriterStatic<512> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("Temperature(°F)",TF);
    jw.insertKeyValue("Pressure(psia)", Ppsi);
    jw.insertKeyValue("Humidity(%)", RH);
    jw.insertKeyValue("HeatIndex(°F)", HI);
    jw.insertKeyValue("DewPoint(°F)", DPF);
    jw.insertKeyValue("PM1_0S(ug/m3)", PM10S);
    jw.insertKeyValue("PM2_5S(ug/m3)", PM25S);
    jw.insertKeyValue("PM10_0S(ug/m3)", PM100S);
    jw.insertKeyValue("PM1_0E(ug/m3)", PM10E);
    jw.insertKeyValue("PM2_5E(ug/m3)", PM25E);
    jw.insertKeyValue("PM10_0E(ug/m3)", PM100E);
    jw.insertKeyValue("PC>0_3um(#/L)", PC03);
    jw.insertKeyValue("PC>0_3um(#/L)", PC03);
    jw.insertKeyValue("PC>0_5um(#/L)", PC05);
    jw.insertKeyValue("PC>1_0um(#/L)", PC10);
    jw.insertKeyValue("PC>2_5um(#/L)", PC25);
    jw.insertKeyValue("PC>5_0um(#/L)", PC50);
    jw.insertKeyValue("PC>10 um(#/L)", PC100);
  }
  Particle.publish("Clickclocktictoc", jw.getBuffer(), PRIVATE);
}