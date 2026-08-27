#include <WiFi.h>
#include <base64.h>

// ============================================================
// WIFI
// ============================================================

const char* ssid     = "BABAYO";
const char* password = "tukangkabel";


// ============================================================
// NTRIP CASTER
// ============================================================

const char* casterHost = "36.95.202.211";
const int   casterPort = 2001;

const char* mountPoint = "Nearest-rtcm3";

const char* ntripUser  = "mmdynss";
const char* ntripPass  = "Malaikat02#";


// ============================================================
// ZED-F9P UART
// ============================================================

#define RXD2 16
#define TXD2 17

#define GPS_SERIAL Serial2

const uint32_t GPS_BAUD = 115200;


// ============================================================
// NTRIP CLIENT
// ============================================================

WiFiClient client;


// ============================================================
// TIMER
// ============================================================

unsigned long lastNtripAttempt = 0;
unsigned long lastGgaSend      = 0;
unsigned long lastMonitor      = 0;
unsigned long lastWiFiCheck    = 0;

const unsigned long NTRIP_RECONNECT_INTERVAL = 3000;
const unsigned long GGA_INTERVAL              = 1000;
const unsigned long MONITOR_INTERVAL          = 2000;
const unsigned long WIFI_CHECK_INTERVAL       = 2000;


// ============================================================
// WIFI STATE
// ============================================================

bool wifiConnecting = false;

unsigned long wifiReconnectStart = 0;


// ============================================================
// NMEA BUFFER
// ============================================================

String nmeaBuffer = "";


// ============================================================
// GGA
// ============================================================

String latestGGA = "";

bool haveValidGGA = false;

unsigned long ggaReceived = 0;

unsigned long lastGgaMillis = 0;

float gpsHz = 0.0;


// ============================================================
// GPS DATA
// ============================================================

double latitudeDec  = 0.0;
double longitudeDec = 0.0;

double altitude = 0.0;

double hdop = 0.0;
double pdop = 0.0;

double speedKmh = 0.0;

int satellites = 0;

int fixQuality = 0;

String fixStatus = "MENUNGGU GPS";


// ============================================================
// RTCM MONITORING
// ============================================================

unsigned long rtcmBytes2s = 0;

unsigned long rtcmTotal = 0;


// ============================================================
// FLAG
// ============================================================

bool newGPSData = false;


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  GPS_SERIAL.begin(
    GPS_BAUD,
    SERIAL_8N1,
    RXD2,
    TXD2
  );

  delay(100);

  Serial.println();
  Serial.println("================================================");
  Serial.println(" ESP32 + ZED-F9P NTRIP CLIENT");
  Serial.println("================================================");

  Serial.println("UART GPS : 115200");
  Serial.println("GPS RATE : 5 Hz");
  Serial.println("GGA RATE : 1 Hz");
  Serial.println("RTCM     : F9P <- ESP32");
  Serial.println();

  connectWiFi();

  Serial.println();
  Serial.println("Menunggu GGA dari F9P...");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // 1. HANDLE WIFI
  // ----------------------------------------------------------

  handleWiFi();


  // ----------------------------------------------------------
  // 2. HANDLE NMEA DARI F9P
  // ----------------------------------------------------------

  readGPS();


  // ----------------------------------------------------------
  // 3. CONNECT NTRIP
  // ----------------------------------------------------------

  handleNTRIP();


  // ----------------------------------------------------------
  // 4. TERIMA RTCM
  // ----------------------------------------------------------

  readRTCM();


  // ----------------------------------------------------------
  // 5. KIRIM GGA 1 Hz
  // ----------------------------------------------------------

  sendGGA();


  // ----------------------------------------------------------
  // 6. MONITORING
  // ----------------------------------------------------------

  monitorSystem();
}


// ============================================================
// READ GPS
// ============================================================

void readGPS()
{
  while (GPS_SERIAL.available())
  {
    char c = GPS_SERIAL.read();


    // --------------------------------------------------------
    // Awal kalimat
    // --------------------------------------------------------

    if (c == '$')
    {
      nmeaBuffer = "$";
      continue;
    }


    // --------------------------------------------------------
    // Akhir kalimat
    // --------------------------------------------------------

    if (c == '\n')
    {
      if (nmeaBuffer.length() > 6)
      {
        parseNMEA(nmeaBuffer);
      }

      nmeaBuffer = "";

      continue;
    }


    // --------------------------------------------------------
    // Abaikan CR
    // --------------------------------------------------------

    if (c == '\r')
    {
      continue;
    }


    // --------------------------------------------------------
    // Tambahkan ke buffer
    // --------------------------------------------------------

    nmeaBuffer += c;


    // --------------------------------------------------------
    // Proteksi buffer
    // --------------------------------------------------------

    if (nmeaBuffer.length() > 160)
    {
      nmeaBuffer = "";
    }
  }
}


// ============================================================
// PARSE NMEA
// ============================================================

void parseNMEA(String sentence)
{
  // ==========================================================
  // GGA
  // ==========================================================

  if (
    sentence.startsWith("$GNGGA") ||
    sentence.startsWith("$GPGGA")
  )
  {
    parseGGA(sentence);
  }


  // ==========================================================
  // GSA
  // ==========================================================

  else if (
    sentence.startsWith("$GNGSA") ||
    sentence.startsWith("$GPGSA")
  )
  {
    parseGSA(sentence);
  }


  // ==========================================================
  // RMC
  // ==========================================================

  else if (
    sentence.startsWith("$GNRMC") ||
    sentence.startsWith("$GPRMC")
  )
  {
    parseRMC(sentence);
  }
}


// ============================================================
// PARSE GGA
// ============================================================

void parseGGA(String sentence)
{
  String latRaw =
    getValue(sentence, ',', 2);

  String latDir =
    getValue(sentence, ',', 3);

  String lonRaw =
    getValue(sentence, ',', 4);

  String lonDir =
    getValue(sentence, ',', 5);

  String quality =
    getValue(sentence, ',', 6);

  String sat =
    getValue(sentence, ',', 7);

  String hdopValue =
    getValue(sentence, ',', 8);

  String altValue =
    getValue(sentence, ',', 9);


  // ----------------------------------------------------------
  // Validasi posisi
  // ----------------------------------------------------------

  if (
    latRaw.length() == 0 ||
    lonRaw.length() == 0 ||
    latDir.length() == 0 ||
    lonDir.length() == 0
  )
  {
    return;
  }


  // ----------------------------------------------------------
  // Simpan GGA asli
  // ----------------------------------------------------------

  latestGGA = sentence;

  haveValidGGA = true;

  ggaReceived++;


  // ----------------------------------------------------------
  // Latitude
  // ----------------------------------------------------------

  latitudeDec =
    convertToDecimal(
      latRaw,
      latDir
    );


  // ----------------------------------------------------------
  // Longitude
  // ----------------------------------------------------------

  longitudeDec =
    convertToDecimal(
      lonRaw,
      lonDir
    );


  // ----------------------------------------------------------
  // Fix quality
  // ----------------------------------------------------------

  if (quality.length() > 0)
  {
    fixQuality =
      quality.toInt();
  }


  // ----------------------------------------------------------
  // Satellite
  // ----------------------------------------------------------

  if (sat.length() > 0)
  {
    satellites =
      sat.toInt();
  }


  // ----------------------------------------------------------
  // HDOP
  // ----------------------------------------------------------

  if (hdopValue.length() > 0)
  {
    hdop =
      hdopValue.toDouble();
  }


  // ----------------------------------------------------------
  // Altitude
  // ----------------------------------------------------------

  if (altValue.length() > 0)
  {
    altitude =
      altValue.toDouble();
  }


  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------

  updateFixStatus();


  // ----------------------------------------------------------
  // GPS RATE
  // ----------------------------------------------------------

  unsigned long now =
    millis();

  if (lastGgaMillis != 0)
  {
    unsigned long dt =
      now - lastGgaMillis;

    if (dt > 0)
    {
      gpsHz =
        1000.0 /
        (float)dt;
    }
  }

  lastGgaMillis = now;


  // ----------------------------------------------------------
  // DATA BARU
  // ----------------------------------------------------------

  newGPSData = true;
}


// ============================================================
// UPDATE FIX STATUS
// ============================================================

void updateFixStatus()
{
  switch (fixQuality)
  {
    case 0:
      fixStatus = "NO FIX";
      break;

    case 1:
      fixStatus = "GNSS FIX";
      break;

    case 2:
      fixStatus = "DGNSS";
      break;

    case 4:
      fixStatus = "RTK FIX";
      break;

    case 5:
      fixStatus = "RTK FLOAT";
      break;

    default:
      fixStatus =
        "UNKNOWN (" +
        String(fixQuality) +
        ")";
      break;
  }
}


// ============================================================
// PARSE GSA
// ============================================================

void parseGSA(String sentence)
{
  // GSA:
  // field 15 = PDOP

  String pdopValue =
    getValue(sentence, ',', 15);

  if (pdopValue.length() > 0)
  {
    pdop =
      pdopValue.toDouble();
  }
}


// ============================================================
// PARSE RMC
// ============================================================

void parseRMC(String sentence)
{
  // field 7 = speed knots

  String speedKnots =
    getValue(sentence, ',', 7);

  if (speedKnots.length() > 0)
  {
    double knots =
      speedKnots.toDouble();

    speedKmh =
      knots * 1.852;
  }
}


// ============================================================
// GET NMEA FIELD
// ============================================================

String getValue(
  String data,
  char separator,
  int index
)
{
  int found = 0;

  int strIndex[] =
  {
    0,
    -1
  };

  int maxIndex =
    data.length() - 1;


  for (
    int i = 0;
    i <= maxIndex &&
    found <= index;
    i++
  )
  {
    if (
      data.charAt(i) == separator ||
      i == maxIndex
    )
    {
      found++;

      strIndex[0] =
        strIndex[1] + 1;

      strIndex[1] =
        (
          i == maxIndex
          ? i + 1
          : i
        );
    }
  }


  if (found > index)
  {
    return data.substring(
      strIndex[0],
      strIndex[1]
    );
  }


  return "";
}


// ============================================================
// NMEA COORDINATE
// ============================================================

double convertToDecimal(
  String nmeaCoord,
  String dir
)
{
  if (nmeaCoord.length() < 4)
  {
    return 0.0;
  }


  int dotIndex =
    nmeaCoord.indexOf('.');


  if (dotIndex < 2)
  {
    return 0.0;
  }


  String degStr =
    nmeaCoord.substring(
      0,
      dotIndex - 2
    );


  String minStr =
    nmeaCoord.substring(
      dotIndex - 2
    );


  double decimal =
    degStr.toDouble() +
    (
      minStr.toDouble() /
      60.0
    );


  if (
    dir == "S" ||
    dir == "W"
  )
  {
    decimal *= -1.0;
  }


  return decimal;
}


// ============================================================
// WIFI INITIAL
// ============================================================

void connectWiFi()
{
  Serial.print("Menghubungkan WiFi");


  WiFi.mode(WIFI_STA);

  WiFi.setAutoReconnect(true);

  WiFi.persistent(false);


  WiFi.begin(
    ssid,
    password
  );


  unsigned long start =
    millis();


  while (
    WiFi.status() != WL_CONNECTED
  )
  {
    delay(300);

    Serial.print(".");


    if (
      millis() - start >
      15000
    )
    {
      Serial.println();

      Serial.println(
        "WiFi timeout."
      );

      wifiConnecting = true;

      wifiReconnectStart =
        millis();

      return;
    }
  }


  Serial.println();

  Serial.println(
    "WiFi CONNECTED"
  );

  Serial.print(
    "IP ESP32 : "
  );

  Serial.println(
    WiFi.localIP()
  );


  wifiConnecting = false;
}


// ============================================================
// WIFI AUTO RECONNECT
// ============================================================

void handleWiFi()
{
  unsigned long now =
    millis();


  if (
    now - lastWiFiCheck <
    WIFI_CHECK_INTERVAL
  )
  {
    return;
  }


  lastWiFiCheck =
    now;


  // ----------------------------------------------------------
  // WIFI NORMAL
  // ----------------------------------------------------------

  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    if (wifiConnecting)
    {
      wifiConnecting = false;

      Serial.println();

      Serial.println(
        "WiFi RECONNECTED"
      );

      Serial.print(
        "IP ESP32 : "
      );

      Serial.println(
        WiFi.localIP()
      );
    }

    return;
  }


  // ----------------------------------------------------------
  // WIFI PUTUS
  // ----------------------------------------------------------

  if (!wifiConnecting)
  {
    Serial.println();

    Serial.println(
      "WiFi DISCONNECTED"
    );

    Serial.println(
      "Mencoba reconnect WiFi..."
    );


    // Putuskan NTRIP
    client.stop();


    WiFi.disconnect();

    delay(50);


    WiFi.begin(
      ssid,
      password
    );


    wifiConnecting = true;

    wifiReconnectStart =
      now;
  }


  // ----------------------------------------------------------
  // WIFI RECONNECT TIMEOUT
  // ----------------------------------------------------------

  else if (
    now - wifiReconnectStart >
    15000
  )
  {
    Serial.println(
      "WiFi reconnect timeout."
    );


    WiFi.disconnect();

    delay(50);


    WiFi.begin(
      ssid,
      password
    );


    wifiReconnectStart =
      now;
  }
}


// ============================================================
// NTRIP HANDLER
// ============================================================

void handleNTRIP()
{
  // Tidak ada WiFi
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return;
  }


  // Sudah connected
  if (client.connected())
  {
    return;
  }


  // Jangan reconnect terlalu cepat
  if (
    millis() - lastNtripAttempt <
    NTRIP_RECONNECT_INTERVAL
  )
  {
    return;
  }


  lastNtripAttempt =
    millis();


  connectNTRIP();
}


// ============================================================
// CONNECT NTRIP
// ============================================================

void connectNTRIP()
{
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return;
  }


  Serial.println();

  Serial.println(
    "Menghubungkan NTRIP Caster..."
  );


  client.stop();


  if (
    !client.connect(
      casterHost,
      casterPort
    )
  )
  {
    Serial.println(
      "TCP CONNECT gagal."
    );

    return;
  }


  Serial.println(
    "TCP CONNECTED"
  );


  // ----------------------------------------------------------
  // AUTH
  // ----------------------------------------------------------

  String auth =
    String(ntripUser) +
    ":" +
    String(ntripPass);


  String auth64 =
    base64::encode(
      auth
    );


  // ----------------------------------------------------------
  // REQUEST
  // ----------------------------------------------------------

  client.print(
    "GET /" +
    String(mountPoint) +
    " HTTP/1.1\r\n"
  );


  client.print(
    "Host: " +
    String(casterHost) +
    "\r\n"
  );


  client.print(
    "Ntrip-Version: Ntrip/2.0\r\n"
  );


  client.print(
    "User-Agent: NTRIP ESP32Client/1.0\r\n"
  );


  client.print(
    "Connection: keep-alive\r\n"
  );


  client.print(
    "Authorization: Basic " +
    auth64 +
    "\r\n"
  );


  client.print(
    "\r\n"
  );


  // ----------------------------------------------------------
  // WAIT RESPONSE
  // ----------------------------------------------------------

  unsigned long timeout =
    millis();


  while (
    client.available() == 0
  )
  {
    if (
      WiFi.status() !=
      WL_CONNECTED
    )
    {
      client.stop();

      return;
    }


    if (
      millis() - timeout >
      5000
    )
    {
      Serial.println(
        "Caster timeout."
      );

      client.stop();

      return;
    }


    delay(5);
  }


  String response =
    client.readStringUntil('\n');


  response.trim();


  Serial.print(
    "Caster response: "
  );

  Serial.println(
    response
  );


  // ----------------------------------------------------------
  // CHECK RESPONSE
  // ----------------------------------------------------------

  if (
    response.indexOf("200 OK") == -1
  )
  {
    Serial.println(
      "NTRIP ditolak caster."
    );

    client.stop();

    return;
  }


  // ----------------------------------------------------------
  // BUANG HEADER
  // ----------------------------------------------------------

  while (
    client.connected()
  )
  {
    if (
      client.available()
    )
    {
      String line =
        client.readStringUntil('\n');

      line.trim();

      if (line.length() == 0)
      {
        break;
      }
    }
    else
    {
      break;
    }
  }


  Serial.println(
    "NTRIP CONNECTED"
  );


  // ----------------------------------------------------------
  // RESET RTCM MONITOR
  // ----------------------------------------------------------

  rtcmBytes2s = 0;


  // ----------------------------------------------------------
  // KIRIM GGA PERTAMA
  // ----------------------------------------------------------

  sendGGAImmediately();
}


// ============================================================
// SEND GGA
// ============================================================

void sendGGA()
{
  if (
    !client.connected()
  )
  {
    return;
  }


  if (
    !haveValidGGA
  )
  {
    return;
  }


  if (
    millis() - lastGgaSend <
    GGA_INTERVAL
  )
  {
    return;
  }


  lastGgaSend =
    millis();


  client.print(
    latestGGA
  );

  client.print(
    "\r\n"
  );
}


// ============================================================
// SEND GGA IMMEDIATELY
// ============================================================

void sendGGAImmediately()
{
  if (
    !client.connected()
  )
  {
    return;
  }


  if (
    !haveValidGGA
  )
  {
    Serial.println(
      "Belum ada GGA valid."
    );

    return;
  }


  Serial.println();

  Serial.println(
    "GGA PERTAMA -> CASTER"
  );

  Serial.println(
    latestGGA
  );


  client.print(
    latestGGA
  );

  client.print(
    "\r\n"
  );


  lastGgaSend =
    millis();
}


// ============================================================
// READ RTCM
// ============================================================

void readRTCM()
{
  if (
    !client.connected()
  )
  {
    return;
  }


  int availableBytes =
    client.available();


  while (
    availableBytes > 0
  )
  {
    uint8_t buffer[256];


    int toRead =
      availableBytes;


    if (toRead > 256)
    {
      toRead = 256;
    }


    int received =
      client.read(
        buffer,
        toRead
      );


    if (received <= 0)
    {
      break;
    }


    // --------------------------------------------------------
    // Forward RTCM langsung ke F9P
    // --------------------------------------------------------

    GPS_SERIAL.write(
      buffer,
      received
    );


    rtcmBytes2s +=
      received;


    rtcmTotal +=
      received;


    availableBytes =
      client.available();
  }
}


// ============================================================
// MONITORING
// ============================================================

void monitorSystem()
{
  if (
    millis() - lastMonitor <
    MONITOR_INTERVAL
  )
  {
    return;
  }


  lastMonitor =
    millis();


  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.println(
    "              MONITORING RTK"
  );

  Serial.println(
    "================================================"
  );


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  Serial.print(
    "WiFi       : "
  );

  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    Serial.println(
      "CONNECTED"
    );
  }
  else
  {
    Serial.println(
      "DISCONNECTED"
    );
  }


  // ----------------------------------------------------------
  // NTRIP
  // ----------------------------------------------------------

  Serial.print(
    "NTRIP      : "
  );

  if (
    client.connected()
  )
  {
    Serial.println(
      "CONNECTED"
    );
  }
  else
  {
    Serial.println(
      "DISCONNECTED"
    );
  }


  // ----------------------------------------------------------
  // GPS STATUS
  // ----------------------------------------------------------

  Serial.print(
    "GPS STATUS : "
  );

  Serial.println(
    fixStatus
  );


  // ----------------------------------------------------------
  // POSITION
  // ----------------------------------------------------------

  Serial.print(
    "LAT        : "
  );

  Serial.println(
    latitudeDec,
    8
  );


  Serial.print(
    "LON        : "
  );

  Serial.println(
    longitudeDec,
    8
  );


  // ----------------------------------------------------------
  // SAT
  // ----------------------------------------------------------

  Serial.print(
    "SAT        : "
  );

  Serial.println(
    satellites
  );


  // ----------------------------------------------------------
  // HDOP
  // ----------------------------------------------------------

  Serial.print(
    "HDOP       : "
  );

  Serial.println(
    hdop,
    2
  );


  // ----------------------------------------------------------
  // PDOP
  // ----------------------------------------------------------

  Serial.print(
    "PDOP       : "
  );

  Serial.println(
    pdop,
    2
  );


  // ----------------------------------------------------------
  // ALTITUDE
  // ----------------------------------------------------------

  Serial.print(
    "ALT        : "
  );

  Serial.print(
    altitude,
    2
  );

  Serial.println(
    " m"
  );


  // ----------------------------------------------------------
  // SPEED
  // ----------------------------------------------------------

  Serial.print(
    "SPEED      : "
  );

  Serial.print(
    speedKmh,
    2
  );

  Serial.println(
    " km/h"
  );


  // ----------------------------------------------------------
  // GPS RATE
  // ----------------------------------------------------------

  Serial.print(
    "GPS RATE   : "
  );

  Serial.print(
    gpsHz,
    2
  );

  Serial.println(
    " Hz"
  );


  // ----------------------------------------------------------
  // GGA
  // ----------------------------------------------------------

  Serial.print(
    "GGA RX     : "
  );

  Serial.println(
    ggaReceived
  );


  // ----------------------------------------------------------
  // RTCM
  // ----------------------------------------------------------

  Serial.print(
    "RTCM RX    : "
  );

  Serial.print(
    rtcmBytes2s
  );

  Serial.println(
    " byte / 2 detik"
  );


  Serial.print(
    "RTCM TOTAL : "
  );

  Serial.println(
    rtcmTotal
  );


  // ----------------------------------------------------------
  // GGA TERAKHIR
  // ----------------------------------------------------------

  Serial.print(
    "GGA VALID   : "
  );

  if (haveValidGGA)
  {
    Serial.println(
      "YES"
    );
  }
  else
  {
    Serial.println(
      "NO"
    );
  }


  Serial.println(
    "================================================"
  );


  // ----------------------------------------------------------
  // RESET COUNTER 2 DETIK
  // ----------------------------------------------------------

  rtcmBytes2s = 0;
}