#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>

// ===== CONFIGURAÇÃO =====
RTC_DS3231 rtc;

// pinos (ajuste conforme sua placa)
// Viveiro (originais)
const int chipSelect    = 4;
const int pinoSensor1   = A0; // viveiro sensor 1
const int pinoSensor2   = A1; // viveiro sensor 2
const int pinoValvula   = 9;  // válvula do viveiro
// Horta (3 sensores)
const int pinoSensorH1  = A2; // horta sensor 1
const int pinoSensorH2  = A3; // horta sensor 2
const int pinoSensorH3  = A4; // horta sensor 3
const int pinoValvulaH  = 10; // válvula solenoide da horta

const int pinoFan       = 8;

// --- NOVOS: leitura da bateria e controle do relé de comutação ---
const float R_DIV_TOP = 100000.0;
const float R_DIV_BOT = 33000.0;
const int pinBatSense = A6;
const int pinRelayPower = 7; // relé IN
const bool RELAY_ACTIVE_HIGH = true;
const float BAT_LOW = 11.5;
const float BAT_RESTORE = 12.0;
unsigned long lastSwitchMillis = 0;
const unsigned long MIN_SWITCH_INTERVAL = 30000UL; // 30s

// lógica do relé/válvula: ajuste conforme seu módulo
const int VALVE_OPEN_LEVEL   = LOW;  // ajuste conforme seu módulo
const int VALVE_CLOSED_LEVEL = HIGH; // ajuste conforme seu módulo

// fan: ajuste conforme sua lógica (aqui HIGH = ligado)
const int FAN_ON_LEVEL  = HIGH;
const int FAN_OFF_LEVEL = LOW;

// parâmetros de umidade e tempos
const int limiarSeco = 70;                 // quando iniciar rega (< limiarSeco)
const int TARGET_HUM = 75;                 // objetivo para parar a rega (>= TARGET_HUM)
const unsigned long CHECK_INTERVAL = 3600000UL; // 1 hora (mantido por compatibilidade)
const unsigned long IRR_POLL_INTERVAL = 5000UL; // enquanto irrigando, checar sensores a cada 5s
const unsigned long SAFETY_TIMEOUT = 10UL * 60UL * 1000UL; // timeout de segurança 10 min (ajuste)
const unsigned long MIN_EVENT_INTERVAL = 2000UL; // evitar logs idênticos dentro desse intervalo (ms)
const int SAMPLE_COUNT = 5;               // nº de amostras rápidas para estabilizar leitura
const unsigned long SAMPLE_DELAY = 120UL; // ms entre amostras rápidas

// display e SD
LiquidCrystal_I2C lcd(0x27, 20, 4);
const int addrHeaderFlag = 0; // EEPROM flag para cabeçalho
const int addrRTCSetFlag = 1; // EEPROM flag para RTC ajustado

// estados Viveiro
bool irrigando = false;
unsigned long irrigationStartMillis = 0;
unsigned long lastCheckMillis = 0;
unsigned long lastIrrPollMillis = 0;

// estados Horta
bool irrigandoHorta = false;
unsigned long irrigationStartMillisH = 0;
unsigned long lastCheckMillisH = 0;
unsigned long lastIrrPollMillisH = 0;

String lastEvent = "";
unsigned long lastEventMillis = 0;

// ---------- utilitárias ----------
String timestamp(const DateTime &dt) {
  char b[20];
  snprintf(b, sizeof(b), "%04d/%02d/%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
  return String(b);
}

// safeLog unificado: origem é "viveiro" ou "horta" (ou "system")
// Valores < 0 serão gravados como campo vazio no CSV.
void safeLog(const String &ts, const char* origem, int u1, int u2, int u3, int u4, const char* ev) {
  unsigned long nowMs = millis();
  String evStr = String(ev);
  if (evStr == lastEvent && (nowMs - lastEventMillis) < MIN_EVENT_INTERVAL) {
    Serial.println("Ignorado log duplicado: " + evStr);
    return;
  }
  File f = SD.open("log.csv", FILE_WRITE);
  if (f) {
    f.print(ts); f.print(',');
    f.print(origem); f.print(',');
    if (u1 >= 0) f.print(u1);
    f.print(',');
    if (u2 >= 0) f.print(u2);
    f.print(',');
    if (u3 >= 0) f.print(u3);
    f.print(',');
    if (u4 >= 0) f.print(u4);
    f.print(',');
    f.println(ev);
    f.close();
    Serial.println(String(ev) + " (" + String(origem) + ") gravado: " + ts);
  } else {
    Serial.println("Falha ao abrir log.csv para escrita");
  }
  lastEvent = evStr;
  lastEventMillis = nowMs;
}

// lê 2 sensores (viveiro) com SAMPLE_COUNT amostras e retorna média (0..100)
void readSensorsAvgTwo(int pin1, int pin2, int &u1_out, int &u2_out, float &med_out) {
  long sum1 = 0;
  long sum2 = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    wdt_reset();
    int r1 = analogRead(pin1);
    int r2 = analogRead(pin2);
    sum1 += map(r1, 1023, 0, 0, 100);
    sum2 += map(r2, 1023, 0, 0, 100);
    delay(SAMPLE_DELAY);
  }
  int u1 = sum1 / SAMPLE_COUNT;
  int u2 = sum2 / SAMPLE_COUNT;
  float med = (u1 + u2) / 2.0;
  u1_out = u1;
  u2_out = u2;
  med_out = med;
}

// lê 3 sensores (horta) com SAMPLE_COUNT amostras e retorna médias (0..100)
void readSensorsAvgThree(int p1, int p2, int p3, int &s1_out, int &s2_out, int &s3_out, float &med_out) {
  long sum1 = 0;
  long sum2 = 0;
  long sum3 = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    wdt_reset();
    int r1 = analogRead(p1);
    int r2 = analogRead(p2);
    int r3 = analogRead(p3);
    sum1 += map(r1, 1023, 0, 0, 100);
    sum2 += map(r2, 1023, 0, 0, 100);
    sum3 += map(r3, 1023, 0, 0, 100);
    delay(SAMPLE_DELAY);
  }
  int s1 = sum1 / SAMPLE_COUNT;
  int s2 = sum2 / SAMPLE_COUNT;
  int s3 = sum3 / SAMPLE_COUNT;
  float med = (s1 + s2 + s3) / 3.0;
  s1_out = s1; s2_out = s2; s3_out = s3; med_out = med;
}

// abre/fecha válvula viveiro
void openValve() {
  digitalWrite(pinoValvula, VALVE_OPEN_LEVEL);
}
void closeValve() {
  digitalWrite(pinoValvula, VALVE_CLOSED_LEVEL);
}

// abre/fecha válvula horta
void openValveHorta() {
  digitalWrite(pinoValvulaH, VALVE_OPEN_LEVEL);
}
void closeValveHorta() {
  digitalWrite(pinoValvulaH, VALVE_CLOSED_LEVEL);
}

// ---------- leitura e controle da bateria ----------
float readBatteryVoltage() {
  const int SAMPLES = 10;
  long sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += analogRead(pinBatSense);
    delay(5);
  }
  float adc = (float)sum / (float)SAMPLES;
  float vout = adc * (5.0 / 1023.0); // tensão no ponto do divisor (A6)
  float vin = vout * ((R_DIV_TOP + R_DIV_BOT) / R_DIV_BOT);
  return vin;
}

// seleciona bateria (true) ou carregador (false)
// já encapsula active-high/low dos módulos de relé
void selectBatterySource(bool useBattery) {
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(pinRelayPower, useBattery ? HIGH : LOW);
  } else {
    digitalWrite(pinRelayPower, useBattery ? LOW : HIGH);
  }
}

// lógica a rodar periodicamente:
void checkBatteryAndSwitch() {
  static bool usingBattery = false; // estado corrente
  unsigned long now = millis();
  if (now - lastSwitchMillis < MIN_SWITCH_INTERVAL) return; // evita trocas rápidas

  float vbatt = readBatteryVoltage();
  Serial.print("Vbat: "); Serial.println(vbatt);

  if (usingBattery) {
    if (vbatt <= BAT_LOW) {
      selectBatterySource(false); // vai para carregador
      usingBattery = false;
      lastSwitchMillis = now;
      safeLog(timestamp(rtc.now()), "system", -1, -1, -1, -1, "power_switch_to_charger");
      Serial.println("Switch -> CHARGER (bateria baixa)");
    }
  } else {
    if (vbatt >= BAT_RESTORE) {
      selectBatterySource(true); // volta pra bateria
      usingBattery = true;
      lastSwitchMillis = now;
      safeLog(timestamp(rtc.now()), "system", -1, -1, -1, -1, "power_switch_to_battery");
      Serial.println("Switch -> BATTERY (bateria ok)");
    }
  }
}

// ---------- Funções agendadas ----------
void performViveiroCheck(unsigned long nowMillis, const DateTime &now) {
  if (!irrigando) {
    int u1, u2; float med;
    readSensorsAvgTwo(pinoSensor1, pinoSensor2, u1, u2, med);

    Serial.print("Viveiro Scheduled Check: u1="); Serial.print(u1);
    Serial.print(" u2="); Serial.print(u2);
    Serial.print(" med="); Serial.println(med);

    if (med < limiarSeco) {
      irrigando = true;
      irrigationStartMillis = nowMillis;
      lastIrrPollMillis = nowMillis;
      openValve();
      safeLog(timestamp(now), "viveiro", u1, u2, -1, -1, "valve_open_viveiro");
      Serial.println("Viveiro: Iniciando rega.");
    } else {
      Serial.println("Viveiro: Solo ok - sem rega neste check.");
    }
  }
}

void performHortaCheck(unsigned long nowMillis, const DateTime &now) {
  if (!irrigandoHorta) {
    int hs1, hs2, hs3; float hmed;
    readSensorsAvgThree(pinoSensorH1, pinoSensorH2, pinoSensorH3, hs1, hs2, hs3, hmed);

    Serial.print("Horta Scheduled Check: h1="); Serial.print(hs1);
    Serial.print(" h2="); Serial.print(hs2);
    Serial.print(" h3="); Serial.print(hs3);
    Serial.print(" med="); Serial.println(hmed);

    if (hmed < limiarSeco) {
      irrigandoHorta = true;
      irrigationStartMillisH = nowMillis;
      lastIrrPollMillisH = nowMillis;
      openValveHorta();
      safeLog(timestamp(now), "horta", hs1, hs2, hs3, -1, "valve_open_horta");
      Serial.println("Horta: Iniciando rega.");
    } else {
      Serial.println("Horta: Solo ok - sem rega neste check.");
    }
  }
}

// ---------- setup ----------
void setup() {
  // watchdog (reinicia se travar)
  wdt_enable(WDTO_8S);

  Serial.begin(9600);
  Wire.begin();

  // RTC
  if (!rtc.begin()) {
    Serial.println("Erro: RTC não encontrado");
    while (1) { /* trava pra indicar erro */ }
  }

  // ajustar RTC apenas se nunca tiver sido configurado antes
  bool needSet = false;
  if (EEPROM.read(addrRTCSetFlag) != 1) needSet = true;

  if (needSet) {
    rtc.adjust(DateTime(__DATE__, __TIME__));
    EEPROM.write(addrRTCSetFlag, 1);
    Serial.println("RTC ajustado para hora de upload (compile-time).");
  } else {
    Serial.println("RTC já configurado previamente. Mantendo hora.");
  }

  // SD: cabeçalho apenas uma vez (unificado: origem + 4 sensores)
  pinMode(chipSelect, OUTPUT);
  if (SD.begin(chipSelect)) {
    File f = SD.open("log.csv", FILE_READ);
    bool needHeader = (!f || f.size() == 0);
    if (f) f.close();
    if (needHeader) {
      File h = SD.open("log.csv", FILE_WRITE);
      if (h) {
        h.println("datetime,origem,umid1,umid2,umid3,umid4,evento");
        h.close();
        EEPROM.write(addrHeaderFlag, 1);
        Serial.println("Cabeçalho gravado em log.csv");
      }
    } else {
      Serial.println("log.csv já existe");
    }
  } else {
    Serial.println("Falha ao inicializar SD");
  }

  // LCD
  lcd.init();
  lcd.backlight();

  // pinos de saída
  pinMode(pinoValvula, OUTPUT);
  closeValve();
  pinMode(pinoValvulaH, OUTPUT);
  closeValveHorta();
  pinMode(pinoFan, OUTPUT);
  digitalWrite(pinoFan, FAN_OFF_LEVEL);

  // pinos para controle de energia
  pinMode(pinRelayPower, OUTPUT);
  // inicial: assume preferencia ao carregador (relé OFF -> NC -> charger)
  selectBatterySource(false);

  // inicializa temporizadores
  lastCheckMillis = millis();
  lastIrrPollMillis = millis();
  lastCheckMillisH = millis();
  lastIrrPollMillisH = millis();

  Serial.println("Comandos Serial: 'SET YYYY-MM-DD HH:MM:SS' | 'T' | 'GET' | 'TESTLOG'");
}

// ---------- Serial: processa comando de entrada ----------
void processSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("T")) {
    // ajuste manual apenas se necessário
    rtc.adjust(DateTime(__DATE__, __TIME__));
    EEPROM.write(addrRTCSetFlag, 1);
    Serial.println("RTC ajustado manualmente para hora de compilacao/upload.");
    return;
  }

  if (line.equalsIgnoreCase("GET")) {
    DateTime now = rtc.now();
    Serial.println("RTC: " + timestamp(now));
    return;
  }

  // TESTLOG -> força gravação manual no SD (grava um registro do viveiro e outro da horta)
  if (line.equalsIgnoreCase("TESTLOG")) {
    int v1, v2; float vm;
    readSensorsAvgTwo(pinoSensor1, pinoSensor2, v1, v2, vm);
    safeLog(timestamp(rtc.now()), "viveiro", v1, v2, -1, -1, "manual_test_viveiro");

    int h1, h2, h3; float hm;
    readSensorsAvgThree(pinoSensorH1, pinoSensorH2, pinoSensorH3, h1, h2, h3, hm);
    safeLog(timestamp(rtc.now()), "horta", h1, h2, h3, -1, "manual_test_horta");

    Serial.println("Log manual (viveiro + horta) gravado no SD.");
    return;
  }

  // SET YYYY-MM-DD HH:MM:SS
  if (line.startsWith("SET ")) {
    String payload = line.substring(4);
    int y=0, M=0, d=0, h=0, m=0, s=0;
    if (sscanf(payload.c_str(), "%d-%d-%d %d:%d:%d", &y, &M, &d, &h, &m, &s) == 6) {
      if (y>=2000 && M>=1 && M<=12 && d>=1 && d<=31 && h>=0 && h<24 && m>=0 && m<60 && s>=0 && s<60) {
        rtc.adjust(DateTime(y, M, d, h, m, s));
        EEPROM.write(addrRTCSetFlag, 1);
        Serial.print("RTC ajustado para: ");
        Serial.println(payload);
      } else {
        Serial.println("Valores fora do intervalo valido.");
      }
    } else {
      Serial.println("Formato inválido. Use: SET YYYY-MM-DD HH:MM:SS");
    }
    return;
  }

  Serial.println("Comando desconhecido.");
}

// ---------- loop ----------
void loop() {
  // reseta watchdog
  wdt_reset();

  // processa comandos Serial (ajuste de RTC, GET, TESTLOG)
  processSerialCommands();

  // manter backlight
  lcd.backlight();

  DateTime now = rtc.now();
  int h = now.hour();
  String ts = timestamp(now);

  // Controle da fan (10–18h)
  if (h >= 10 && h < 18) digitalWrite(pinoFan, FAN_OFF_LEVEL);
  else digitalWrite(pinoFan, FAN_ON_LEVEL);

  unsigned long nowMillis = millis();

  // ---------- Monitor de bateria e comutação ----------
  checkBatteryAndSwitch();

  // ---------- Agendamento via RTC: Viveiro no minuto 0, Horta no minuto 30 ----------
  static long lastViveiroMinuteIndex = -1;
  static long lastHortaMinuteIndex = -1;
  uint32_t minuteIndex = now.unixtime() / 60UL;

  if (now.minute() == 0 && minuteIndex != lastViveiroMinuteIndex) {
    lastViveiroMinuteIndex = minuteIndex;
    performViveiroCheck(nowMillis, now);
  }

  if (now.minute() == 30 && minuteIndex != lastHortaMinuteIndex) {
    lastHortaMinuteIndex = minuteIndex;
    performHortaCheck(nowMillis, now);
  }

  // =========================
  // Irrigation polling (enquanto irrigando)
  // Viveiro polling
  if (irrigando) {
    if (nowMillis - lastIrrPollMillis >= IRR_POLL_INTERVAL) {
      lastIrrPollMillis = nowMillis;
      int u1, u2; float med;
      readSensorsAvgTwo(pinoSensor1, pinoSensor2, u1, u2, med);

      Serial.print("Viveiro Irriga poll: u1="); Serial.print(u1);
      Serial.print(" u2="); Serial.print(u2);
      Serial.print(" med="); Serial.println(med);

      if (med >= TARGET_HUM) {
        closeValve();
        safeLog(timestamp(rtc.now()), "viveiro", u1, u2, -1, -1, "valve_close_target_viveiro");
        irrigando = false;
        Serial.println("Viveiro: Parou rega: alvo de umidade atingido.");
      } else if (SAFETY_TIMEOUT > 0 && (nowMillis - irrigationStartMillis) >= SAFETY_TIMEOUT) {
        closeValve();
        safeLog(timestamp(rtc.now()), "viveiro", u1, u2, -1, -1, "valve_close_timeout_viveiro");
        irrigando = false;
        Serial.println("Viveiro: Parou rega: safety timeout.");
      }
    }
  }

  // Horta polling
  if (irrigandoHorta) {
    if (nowMillis - lastIrrPollMillisH >= IRR_POLL_INTERVAL) {
      lastIrrPollMillisH = nowMillis;
      int hs1, hs2, hs3; float hmed;
      readSensorsAvgThree(pinoSensorH1, pinoSensorH2, pinoSensorH3, hs1, hs2, hs3, hmed);

      Serial.print("Horta Irriga poll: h1="); Serial.print(hs1);
      Serial.print(" h2="); Serial.print(hs2);
      Serial.print(" h3="); Serial.print(hs3);
      Serial.print(" med="); Serial.println(hmed);

      if (hmed >= TARGET_HUM) {
        closeValveHorta();
        safeLog(timestamp(rtc.now()), "horta", hs1, hs2, hs3, -1, "valve_close_target_horta");
        irrigandoHorta = false;
        Serial.println("Horta: Parou rega: alvo de umidade atingido.");
      } else if (SAFETY_TIMEOUT > 0 && (nowMillis - irrigationStartMillisH) >= SAFETY_TIMEOUT) {
        closeValveHorta();
        safeLog(timestamp(rtc.now()), "horta", hs1, hs2, hs3, -1, "valve_close_timeout_horta");
        irrigandoHorta = false;
        Serial.println("Horta: Parou rega: safety timeout.");
      }
    }
  }

  // =========================
  // Atualiza LCD (mostra médias e status)
  // =========================
  int v1_disp, v2_disp; float vmed_disp;
  readSensorsAvgTwo(pinoSensor1, pinoSensor2, v1_disp, v2_disp, vmed_disp);
  int h1_disp, h2_disp, h3_disp; float hmed_disp;
  readSensorsAvgThree(pinoSensorH1, pinoSensorH2, pinoSensorH3, h1_disp, h2_disp, h3_disp, hmed_disp);

  // escolher o texto de status
  char statusBuf[21];
  if (irrigando) {
    snprintf(statusBuf, sizeof(statusBuf), "Regando: Viveiro");
  } else if (irrigandoHorta) {
    snprintf(statusBuf, sizeof(statusBuf), "Regando: Horta");
  } else {
    snprintf(statusBuf, sizeof(statusBuf), "Solo OK");
  }

  char buf[21];
  lcd.setCursor(0, 0);
  // linha 0: média do viveiro
  snprintf(buf, sizeof(buf), "Viveiro:%3d%%", (int)vmed_disp);
  lcd.print(buf);
  lcd.setCursor(0, 1);
  // linha 1: média da horta
  snprintf(buf, sizeof(buf), "Horta:  %3d%%", (int)hmed_disp);
  lcd.print(buf);
  lcd.setCursor(0, 2);
  // linha 2: data e hora curto
  char dateBuf[21];
  snprintf(dateBuf, sizeof(dateBuf), "%04d/%02d/%02d", now.year(), now.month(), now.day());
  lcd.print(dateBuf);
  lcd.setCursor(11, 2);
  char timeBuf[10];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  lcd.print(timeBuf);
  lcd.setCursor(0, 3);
  // linha 3: status (Regando Viveiro / Regando Horta / Solo OK)
  lcd.print(statusBuf);

  delay(100);
}
