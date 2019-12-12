#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <EEPROM.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define ATD_NO_ERROR           0x00
#define ATD_CONVERSION_ONGOING 0x01
#define ATD_POSITIVE_OVERRANGE 0x02
#define ATD_NEGATIVE_OVERRANGE 0x04
#define ATD_STALLED            0x08

#define ATD_CONVERSION_TIME_MS           69
#define ATD_SCLK_WAIT_MICROSEC           1

#define CONNECT_TIMEOUT_SECONDS 300

#define SCLK_PIN 4
#define MISO_PIN 13
#define LED_PIN 5
#define RST_PIN 12

#define WEBSERVER_SSID "ESP8266"
// Lens are standard + 1, because strings are \0 terminated
#define SSID_MAX_LEN 33
#define PASSWORD_MAX_LEN 64
#define TARGET_MAX_LEN 129
#define DEV_NAME_MAX_LEN 65
// Total 292 bytes of EEPROM used (including first status byte)

#define MAC_ADDR_LEN 6

#define CONFIG_LEN 6
#define RESET_PATTERN 0xAA

#define EEPROM_SIZE 512
#define EEPROM_VERIFY_PATTERN 0xAA

#define UINT16_VALUE(x) *((uint16_t *)x)

typedef struct atd_sample_t {
  union {
    struct {
      unsigned int eoc_bit : 1;
      unsigned int dmy_bit : 1;
      unsigned int sig_bit : 1;
      signed int   value   : 24;
      unsigned int sub_lsb : 5;
    };
    uint32_t field;
  };
} atd_sample_t;

typedef struct atd_average_t {
  unsigned long delta_ms;
  double sample_avg;
} atd_average_t;

atd_sample_t atd_glob_sample;
unsigned char atd_err_char;
unsigned long atd_last_time_ms;
unsigned char atd_read_i;
double atd_average_accum;
float atd_average_counter;

const char* HTML_RESP_FORM = "<!doctypehtml><form action=/ >SSID:<br><input name=ssid><br>Password:<br><input name=pass><br>Target:<br><input name=target><br><input name=dev_name><br><input type=submit value=Connect></form>";

char ssid[SSID_MAX_LEN];
char pass[PASSWORD_MAX_LEN];
char target[TARGET_MAX_LEN];
char dev_name[DEV_NAME_MAX_LEN];
char dev_config[CONFIG_LEN];
uint16_t samples_per_avg;
uint16_t avg_per_post;
uint8_t *avg_buffer;
uint16_t avg_buffer_ind;
atd_average_t avg_alias;
uint16_t connect_watchdog;

uint32_t WS_IP;
uint32_t WS_GATEWAY;
uint32_t WS_SUBNET;

ESP8266WebServer webserver(80);
HTTPClient http;

unsigned char server_complete;

void atd_spi_read(){
    if(digitalRead(MISO_PIN)){
        // SDO high, conversion hasn't finished
        atd_err_char |= ATD_CONVERSION_ONGOING;
        return;
    }
    // Two pretty much identical while loops to minimize branching
    atd_read_i = 0;
    while(atd_read_i < 32){
        digitalWrite(SCLK_PIN, 1);
        atd_glob_sample.field = (atd_glob_sample.field << 1) | digitalRead(MISO_PIN);
        atd_read_i++;
        delayMicroseconds(ATD_SCLK_WAIT_MICROSEC);
        digitalWrite(SCLK_PIN, 0);
        delayMicroseconds(ATD_SCLK_WAIT_MICROSEC);
    }
    if(!digitalRead(MISO_PIN)){
        // SDO low after reading 32 bits, something strange going on
        atd_err_char = ATD_STALLED;
    }
}

void atd_get_average(){
    avg_alias = ((atd_average_t *)(avg_buffer + 1 + MAC_ADDR_LEN + DEV_NAME_MAX_LEN))[avg_buffer_ind];
    atd_average_accum = 0;
    atd_average_counter = 0;
    while(atd_average_counter < samples_per_avg){
        atd_spi_read();
        if((atd_err_char & ATD_CONVERSION_ONGOING) > 0){
            continue;
        }
        if(atd_glob_sample.sig_bit && (atd_glob_sample.value >> 23)){
          atd_err_char |= ATD_POSITIVE_OVERRANGE;
        } else if(!atd_glob_sample.sig_bit && !(atd_glob_sample.value >> 23)){
          atd_err_char |= ATD_NEGATIVE_OVERRANGE;
        }
        atd_average_accum += atd_glob_sample.value;
        atd_average_counter++;
        delay(ATD_CONVERSION_TIME_MS);
    }
    avg_alias.sample_avg = atd_average_accum / atd_average_counter;
    avg_alias.delta_ms = millis() - atd_last_time_ms;
    atd_last_time_ms += avg_alias.delta_ms;
}

void server_root_handler() {
  if (webserver.hasArg("ssid") && webserver.hasArg("pass") && webserver.hasArg("target") && webserver.hasArg("dev_name")) {
    strcpy(ssid, webserver.arg("ssid").c_str());
    strcpy(pass, webserver.arg("pass").c_str());
    strcpy(target, webserver.arg("target").c_str());
    strcpy(dev_name, webserver.arg("dev_name").c_str());
    Serial.println(ssid);
    Serial.println(pass);
    Serial.println(target);
    Serial.println(dev_name);
    server_complete = 1;
  } else {
    webserver.send(200, "text/html", HTML_RESP_FORM);
  }
}


uint16_t eeprom_index;
void eeprom_read(void *dest, uint16_t offset, uint16_t len) {
  for (eeprom_index = offset; eeprom_index < offset + len; eeprom_index++) {
    ((unsigned char *)dest)[eeprom_index - offset] = EEPROM.read(eeprom_index);
  }
}

void eeprom_write(char *src, uint16_t offset, uint16_t len) {
  for (eeprom_index = offset; eeprom_index < offset + len; eeprom_index++) {
    EEPROM.write((int)eeprom_index, (unsigned char)(src[eeprom_index - offset]));
  }
}

uint16_t glob_eeprom_rw_index;
void setup() {
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, INPUT_PULLUP);

  glob_eeprom_rw_index = 1;
  // Should not get these from EEPROM because there is a chance the values could be 0xFFFF and screw up everything.
  // These defaults mean post requests happen approx every 10 seconds, with averages every 840 milliseconds.
  samples_per_avg = 12;
  avg_per_post = 12;

  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(9600);

  if (EEPROM.read(0) == EEPROM_VERIFY_PATTERN && digitalRead(RST_PIN)) {
    eeprom_read(ssid,     glob_eeprom_rw_index, SSID_MAX_LEN);     glob_eeprom_rw_index += SSID_MAX_LEN;
    eeprom_read(pass,     glob_eeprom_rw_index, PASSWORD_MAX_LEN); glob_eeprom_rw_index += PASSWORD_MAX_LEN;
    eeprom_read(target,   glob_eeprom_rw_index, TARGET_MAX_LEN);   glob_eeprom_rw_index += TARGET_MAX_LEN;
    eeprom_read(dev_name, glob_eeprom_rw_index, DEV_NAME_MAX_LEN); glob_eeprom_rw_index += DEV_NAME_MAX_LEN;
  } else {
    digitalWrite(LED_PIN, 1);

    WS_IP = 0x01010101;
    WS_GATEWAY = 0x00000000;
    WS_SUBNET = 0x00000000;
    server_complete = 0;

    WiFi.softAPConfig(IPAddress(WS_IP), IPAddress(WS_GATEWAY), IPAddress(WS_SUBNET));
    WiFi.softAP(WEBSERVER_SSID);
    Serial.println(WiFi.softAPIP());
    webserver.on("/", server_root_handler);
    webserver.begin();
    while (!server_complete) {
      webserver.handleClient();
    }
    webserver.close();
    WiFi.softAPdisconnect(1);

    EEPROM.write(0, EEPROM_VERIFY_PATTERN);
    eeprom_write(ssid,     glob_eeprom_rw_index, SSID_MAX_LEN);     glob_eeprom_rw_index += SSID_MAX_LEN;
    eeprom_write(pass,     glob_eeprom_rw_index, PASSWORD_MAX_LEN); glob_eeprom_rw_index += PASSWORD_MAX_LEN;
    eeprom_write(target,   glob_eeprom_rw_index, TARGET_MAX_LEN);   glob_eeprom_rw_index += TARGET_MAX_LEN;
    eeprom_write(dev_name, glob_eeprom_rw_index, DEV_NAME_MAX_LEN); glob_eeprom_rw_index += DEV_NAME_MAX_LEN;
    EEPROM.commit();

    digitalWrite(LED_PIN, 0);
  }

  WiFi.begin(ssid, pass);
  Serial.print("Connecting to wifi");
  connect_watchdog = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if(connect_watchdog > (CONNECT_TIMEOUT_SECONDS * 2)){
      // If connection fails after timeout, then invalidate EEPROM and reset into user configured startup.
      EEPROM.write(0, 0x00);
      EEPROM.commit();
      ESP.reset();
    }
    delay(500);
    Serial.print(".");
    connect_watchdog++;
  }
  Serial.println("connected");

  avg_buffer = (uint8_t *)malloc(1 + MAC_ADDR_LEN + DEV_NAME_MAX_LEN + (sizeof(atd_average_t) * avg_per_post));
  WiFi.macAddress(1 + avg_buffer);
  strcpy((char *)(1 + avg_buffer + MAC_ADDR_LEN), dev_name);
}

void loop() {
  // If Wifi isn't connected, it probably means that there was some minor fluctuation in the WiFi connection.
  // In this case, a reset into automatic startup should be enough to resolve it.
  // If it was disconnected, or some information changed, then the loop in setup will catch that.
  if(WiFi.status() != WL_CONNECTED){
    ESP.reset();
  }

  avg_buffer[0] = atd_err_char;
  atd_err_char = ATD_NO_ERROR;
  
  for (avg_buffer_ind = avg_per_post; avg_buffer_ind > 0; avg_buffer_ind--) {
    atd_get_average();
  }
  http.begin(target);
  http.addHeader("Content-Type", "text/plain");
  http.POST(avg_buffer, 1 + MAC_ADDR_LEN + DEV_NAME_MAX_LEN + (sizeof(atd_average_t) * avg_per_post));

  strcpy(dev_config, http.getString().c_str());
  http.end();

  samples_per_avg = UINT16_VALUE(dev_config);
  if(avg_per_post != UINT16_VALUE(dev_config + 2)){
    avg_per_post = UINT16_VALUE(dev_config + 2);
    avg_buffer = (uint8_t *)realloc(avg_buffer, 1 + MAC_ADDR_LEN + DEV_NAME_MAX_LEN + (sizeof(atd_average_t) * avg_per_post));
  }

  if(dev_config[4] == RESET_PATTERN){
    ESP.reset();
  }
}
