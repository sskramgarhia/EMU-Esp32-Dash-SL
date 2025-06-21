#include <Arduino.h>
#define LV_COLOR_16_SWAP 0
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "BluetoothSerial.h"
#include <Adafruit_NeoPixel.h>
#include <string>
#include <stdexcept>
using namespace std;

//#define USE_NAME
const char *pin = "1234";
String myBtName = "ESP32-BT-Master";

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
String slaveName = "EMUCANBT_SPP";
#else
uint8_t address[6] = { 0x98, 0xDA, 0x20, 0x07, 0x49, 0x06 }; // Updated MAC address: 98:DA:07:49:06
#endif

const int backLightPin = 27;
const int buzzerPin = 22;
bool buzzerOn = false;
bool btIconSts = false;
static lv_style_t style_bt;
static bool style_initialized = false;

int rpm = 0;
int spd = 0;
float afr = 0.0f;
float mapR = 0.0f;
float boost = 0.0f;
float boostPsi = 0.0f;
int tps = 0;
int clt = 0;
int ign = 0;
int inj = 0;
float bat = 0.0f;
int cel = 0;

unsigned long previousMillis = 0;
const unsigned long reconnectInterval = 5000;

// Flash red screen variables
bool flashRed = false;
unsigned long flashStartMillis = 0;
const unsigned long flashDuration = 300; // milliseconds

// NeoPixel shiftlight config
#define NUMPIXELS 8
#define NEOPIXEL_PIN 22 // GPIO22
int STARTRPM = 1200;
int ENDRPM = 6200;
int SHIFT_RPM = 4200;
int FLASH_END_RPM = 4700;
int step;
unsigned int ShiftLightsOn[NUMPIXELS];
const uint32_t LEDcolour[NUMPIXELS] = {
  Adafruit_NeoPixel::Color(0, 120, 0),
  Adafruit_NeoPixel::Color(0, 120, 0),
  Adafruit_NeoPixel::Color(0, 120, 0),
  Adafruit_NeoPixel::Color(0, 120, 0),
  Adafruit_NeoPixel::Color(255, 125, 0),
  Adafruit_NeoPixel::Color(255, 125, 0),
  Adafruit_NeoPixel::Color(255, 0, 0),
  Adafruit_NeoPixel::Color(255, 0, 0),
};
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

LV_FONT_DECLARE(lv_font_montserrat_18);
LV_FONT_DECLARE(lv_font_montserrat_28);

lv_obj_t *bt_icon_label;

// Display & LVGL setup
TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];
lv_obj_t *table;

// Forward declarations for event callbacks
void my_table_event_cb(lv_event_t *e);
static void table_event_cb_bg(lv_event_t *e);
void create_bt_icon();
void update_bt_icon_color(bool is_connected, bool firstTime);
int decodeCheckEngine(uint16_t value);
void connectToBt();

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t w = area->x2 - area->x1 + 1;
  uint16_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void create_table() {
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(30, 30, 30), LV_PART_MAIN);

  table = lv_table_create(lv_scr_act());
  lv_obj_align(table, LV_ALIGN_CENTER, -1, -1);
  lv_obj_set_style_text_opa(table, LV_OPA_COVER, 0);
  lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(table, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_text_color(table, lv_color_white(), LV_PART_ITEMS);
  lv_obj_set_style_bg_color(table, lv_color_make(30, 30, 30), LV_PART_MAIN);

  static lv_style_t style_cell0;
  lv_style_init(&style_cell0);
  lv_style_set_pad_top(&style_cell0, 12.8);
  lv_style_set_pad_bottom(&style_cell0, 12.8);
  lv_style_set_pad_left(&style_cell0, 4);
  lv_style_set_pad_right(&style_cell0, 4);
  lv_obj_add_style(table, &style_cell0, LV_PART_ITEMS);

  lv_table_set_col_cnt(table, 4);
  lv_table_set_row_cnt(table, 6);

  lv_obj_set_style_border_width(table, 1, LV_PART_ITEMS);
  lv_obj_set_style_border_color(table, lv_color_white(), LV_PART_ITEMS);
  lv_obj_set_style_border_side(table, LV_BORDER_SIDE_FULL, LV_PART_ITEMS);

  lv_table_set_col_width(table, 0, 53);
  lv_table_set_col_width(table, 1, 105);
  lv_table_set_col_width(table, 2, 47);
  lv_table_set_col_width(table, 3, 115);

  lv_table_add_cell_ctrl(table, 5, 1, LV_TABLE_CELL_CTRL_MERGE_RIGHT);
  lv_table_add_cell_ctrl(table, 5, 2, LV_TABLE_CELL_CTRL_MERGE_RIGHT);
  lv_table_add_cell_ctrl(table, 5, 3, LV_TABLE_CELL_CTRL_MERGE_RIGHT);

  lv_table_set_cell_value(table, 0, 0, "RPM");
  lv_table_set_cell_value(table, 0, 2, "SPD");
  lv_table_set_cell_value(table, 1, 0, "AFR");
  lv_table_set_cell_value(table, 1, 2, "CLT");
  lv_table_set_cell_value(table, 2, 0, "TPS");
  lv_table_set_cell_value(table, 2, 2, "BAT");
  lv_table_set_cell_value(table, 3, 0, "MAP");
  lv_table_set_cell_value(table, 3, 2, "BST");
  lv_table_set_cell_value(table, 4, 0, "INJ");
  lv_table_set_cell_value(table, 4, 2, "IGN");
  lv_table_set_cell_value(table, 5, 0, "CEL");

  lv_obj_add_event_cb(table, my_table_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
  lv_obj_add_event_cb(table, table_event_cb_bg, LV_EVENT_DRAW_PART_BEGIN, NULL);

  create_bt_icon();

  lv_timer_handler();
}

void setup() {
  tft.init();
  pinMode(backLightPin, OUTPUT);
  digitalWrite(backLightPin, LOW);
  uint16_t darkGray = ((30 & 0xF8) << 8) | ((30 & 0xFC) << 3) | (30 >> 3);
  tft.fillScreen(darkGray);
  tft.setRotation(1);
  Serial.begin(1000000);

  pinMode(buzzerPin, OUTPUT);

  // Initialize LVGL
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  // Setup LVGL Display Driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  SerialBT.begin(myBtName, true);

  create_table();
  digitalWrite(backLightPin, HIGH);
  connectToBt();

  // NeoPixel setup
  pixels.begin();
  pixels.clear();
  pixels.fill(pixels.Color(10, 0, 0)); // Faint red
  pixels.show();

  step = (ENDRPM - STARTRPM) / NUMPIXELS;
  for (int i = 0; i < NUMPIXELS; i++) {
    ShiftLightsOn[i] = STARTRPM + (step * i);
  }
}

void connectToBt() {
  bool connected;
#ifndef USE_NAME
  SerialBT.setPin(pin);
#endif

#ifdef USE_NAME
  connected = SerialBT.connect(slaveName);
#else
  connected = SerialBT.connect(address);
#endif

  if (connected) {
    Serial.println("Connected Successfully!");
  } else {
    Serial.println("Initial connect failed. Will retry in loop...");
  }
  update_bt_icon_color(SerialBT.hasClient(), false);
}

void updateShiftlight(int rpm) {
  static bool amberFlashState = false;
  static unsigned long lastAmberFlash = 0;
  const unsigned long amberFlashInterval = 120; // ms

  if (rpm < SHIFT_RPM) {
    // Normal operating range
    for (int i = 0; i < NUMPIXELS; i++) {
      if (rpm > ShiftLightsOn[i]) {
        pixels.setPixelColor(i, LEDcolour[i]);
      } else {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
    }
    pixels.show();
  }
  else if (rpm >= SHIFT_RPM && rpm < FLASH_END_RPM) {
    // Flash all amber between 4200 and 4700
    unsigned long now = millis();
    if (now - lastAmberFlash > amberFlashInterval) {
      amberFlashState = !amberFlashState;
      lastAmberFlash = now;
      if (amberFlashState) {
        pixels.fill(Adafruit_NeoPixel::Color(255, 125, 0)); // Amber
      } else {
        pixels.fill(pixels.Color(0, 0, 0));
      }
      pixels.show();
    }
  }
  else if (rpm >= FLASH_END_RPM) {
    // After 4700, turn off all
    pixels.fill(pixels.Color(0, 0, 0));
    pixels.show();
  }
}

void loop() {
  uint8_t frame[5];
  uint8_t channel;
  uint16_t value;
  int chData;
  unsigned long currentMillis = millis();

  if (!SerialBT.connected()) {
    // Attempt reconnection every 5 seconds
    if (currentMillis - previousMillis >= reconnectInterval) {
      previousMillis = currentMillis;
      connectToBt();
    }
  }

  update_bt_icon_color(SerialBT.hasClient(), false);

  // Wait until at least 5 bytes are available
  while (SerialBT.available() >= 5) {
    SerialBT.readBytes(frame, 5);  // Read exactly 5 bytes
    channel = frame[0];
    value = (frame[2] << 8) | frame[3];
    chData = static_cast<int>(channel);
    if (chData == 1) {
      rpm = static_cast<int>(value);
      lv_table_set_cell_value(table, 0, 1, String(rpm).c_str());
      // Flash red when rpm reaches 4200
      if (rpm >= SHIFT_RPM && !flashRed) {
        flashRed = true;
        flashStartMillis = millis();
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(255, 0, 0), LV_PART_MAIN);
      }
      updateShiftlight(rpm); // <-- NeoPixel shiftlight
    } else if (chData == 28) {
      spd = (static_cast<int>(value));
      lv_table_set_cell_value(table, 0, 3, (String(spd) + " KM/H").c_str());
    } else if (chData == 12) {
      afr = (static_cast<float>(value) / 10);
      lv_table_set_cell_value(table, 1, 1, String(afr).c_str());
    } else if (chData == 2) {
      mapR = (static_cast<float>(value) / 100);
      boost = (mapR - 1.0132f);
      boostPsi = boost * 14.5038f; // Convert bar to psi
      lv_table_set_cell_value(table, 3, 1, (String(mapR, 2) + " BAR").c_str());
      lv_table_set_cell_value(table, 3, 3, (String(boostPsi, 1) + " PSI").c_str());
    } else if (chData == 3) {
      tps = static_cast<int>(value);
      lv_table_set_cell_value(table, 2, 1, (String(tps) + " %").c_str());
    } else if (chData == 24) {
      clt = static_cast<int>(value);
      lv_table_set_cell_value(table, 1, 3, (String(clt) + " °C").c_str());
    } else if (chData == 6) {
      ign = (static_cast<int>(value) / 2);
      lv_table_set_cell_value(table, 4, 3, (String(ign) + " °").c_str());
    } else if (chData == 19) {
      inj = (static_cast<int>(value) / 2);
      lv_table_set_cell_value(table, 4, 1, (String(inj) + " %").c_str());
    } else if (chData == 5) {
      bat = (static_cast<float>(value) / 37);
      lv_table_set_cell_value(table, 2, 3, (String(bat) + " V").c_str());
    } else if (chData == 255) {
      cel = decodeCheckEngine(value);
    }
  }

  // Reset screen background after flash duration
  if (flashRed && (millis() - flashStartMillis > flashDuration)) {
    flashRed = false;
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(30, 30, 30), LV_PART_MAIN);
  }

  buzzerOn = (cel > 0 || clt > 105 || rpm > 7000 || boostPsi > 16.0 || (bat < 12.00 && bat > 1.00));
  digitalWrite(buzzerPin, (millis() % 600 < 300) && buzzerOn);

  lv_obj_invalidate(table);
  lv_timer_handler();
}

int decodeCheckEngine(uint16_t value) {
  int cel_codes = 0;
  string cel_names = "";
  if (value == 0) {
    lv_table_set_cell_value(table, 5, 1, "");
    return 0;
  } else {
    if (value & (1 << 0)) {
      cel_codes++;  // Bit 0
      cel_names = "CLT ";
    }
    if (value & (1 << 1)) {
      cel_codes++;  // Bit 1
      cel_names += "IAT ";
    }
    if (value & (1 << 2)) {
      cel_codes++;  // Bit 2
      cel_names += "MAP ";
    }
    if (value & (1 << 3)) {
      cel_codes++;  // Bit 3
      cel_names += "WBO ";
    }
    if (value & (1 << 8)) {
      cel_codes++;  // Bit 8
      cel_names += "FF SENSOR ";
    }
    if (value & (1 << 9)) {
      cel_codes++;  // Bit 9
      cel_names += "DBW ";
    }
    if (value & (1 << 10)) {
      cel_codes++;  // Bit 10
      cel_names += "FPR ";
    }

    lv_table_set_cell_value(table, 5, 1, cel_names.c_str());
    return cel_codes;
  }
}

void my_table_event_cb(lv_event_t * e) {
  lv_obj_t * table = lv_event_get_target(e);
  lv_obj_draw_part_dsc_t * dsc = (lv_obj_draw_part_dsc_t *)lv_event_get_param(e);

  if (dsc->part == LV_PART_ITEMS) {
    uint16_t row = dsc->id / lv_table_get_col_cnt(table);
    uint16_t col = dsc->id % lv_table_get_col_cnt(table);

    if (col == 0 || col == 2) {
      dsc->label_dsc->font = &lv_font_montserrat_18;
    } else if (col == 1 || col == 3) {
      dsc->label_dsc->font = &lv_font_montserrat_20;
    }
    dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
    if ((row == 0 && col == 1) || (row == 0 && col == 3) || (row == 1 && col == 1) || (row == 1 && col == 3) || (row == 2 && col == 1) || (row == 2 && col == 3) || (row == 3 && col == 1) || (row == 3 && col == 3) ||
        (row == 4 && col == 1) || (row == 4 && col == 3)) {
      dsc->label_dsc->align = LV_TEXT_ALIGN_RIGHT;
    }
    if (row == 5 && col == 1) {
      dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
    }
  }
}

static void table_event_cb_bg(lv_event_t *e) {
  lv_obj_t *table = lv_event_get_target(e);
  lv_obj_draw_part_dsc_t *dsc = (lv_obj_draw_part_dsc_t *)lv_event_get_param(e);

  // Ensure dsc and rect_dsc are valid
  if (!dsc || !dsc->rect_dsc) return;

  // Only modify table cell backgrounds
  if (dsc->part == LV_PART_ITEMS) {
    uint16_t row = dsc->id / lv_table_get_col_cnt(table);
    uint16_t col = dsc->id % lv_table_get_col_cnt(table);

    const char *value_str = lv_table_get_cell_value(table, row, col);

    // Check if value_str is null or empty before conversion
    float value = 0.0f;  // Default value
    if (value_str != nullptr && value_str[0] != '\0') {
      try {
        value = std::stof(value_str);  // Convert string to float safely
      } catch (...) {
        value = 0.0f;  // Handle invalid conversions
      }
    }

    // Default cell color
    lv_color_t bg_color = lv_color_make(30, 30, 30);
    lv_color_t text_color = lv_color_white();

    if (row == 0 && col == 1 && value > 7000.00) {
      bg_color = lv_color_make(0, 0, 255);
      text_color = lv_color_white();
    }
    if (row == 1 && col == 3 && value > 100.00) {
      bg_color = lv_color_make(0, 0, 255);
      text_color = lv_color_white();
    }
    if (row == 1 && col == 3 && value < 55.00 && value > 1.00) {
      bg_color = lv_color_make(0, 255, 255);
      text_color = lv_color_black();
    }
    if (row == 2 && col == 3 && value < 12.00 && value > 1.00) {
      bg_color = lv_color_make(0, 0, 255);
      text_color = lv_color_white();
    }
    if (row == 3 && col == 3 && value > 16.0) { // Now in PSI
      bg_color = lv_color_make(0, 0, 255);
      text_color = lv_color_white();
    }
    if (row == 5 && col == 1 && value_str != nullptr && value_str[0] != '\0') {
      bg_color = lv_color_make(0, 0, 255);
      text_color = lv_color_white();
    }

    // Apply background color to the cell
    dsc->rect_dsc->bg_color = bg_color;
    dsc->rect_dsc->bg_opa = LV_OPA_COVER;
    dsc->label_dsc->color = text_color;
  }
}

void update_bt_icon_color(bool is_connected, bool firstTime) {
  if (btIconSts != is_connected || firstTime) {
    if (!style_initialized) {
      lv_style_init(&style_bt);
      style_initialized = true;
    }
    if (is_connected) {
      lv_style_set_text_color(&style_bt, lv_color_make(0, 255, 0)); // Green
    } else {
      lv_style_set_text_color(&style_bt, lv_color_make(0, 0, 255)); // Red
    }
    lv_obj_add_style(bt_icon_label, &style_bt, 0);
    btIconSts = is_connected;
  }
}

void create_bt_icon() {
  bt_icon_label = lv_label_create(lv_scr_act());
  lv_label_set_text(bt_icon_label, LV_SYMBOL_BLUETOOTH);
  lv_obj_set_style_text_font(bt_icon_label, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_align(bt_icon_label, LV_ALIGN_BOTTOM_RIGHT, -3, -5);
  update_bt_icon_color(SerialBT.hasClient(), true);
}