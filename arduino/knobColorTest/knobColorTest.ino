#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01  _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;

public:
  LGFX(void) {
    { // SPI bus config
      auto cfg = _bus.config();
      cfg.spi_host   = SPI2_HOST;   // ESP32-C3 uses SPI2
      cfg.spi_mode   = 0;
      cfg.freq_write = 27000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = true;        // no MISO
      cfg.use_lock   = true;

      cfg.pin_sclk = 1;
      cfg.pin_mosi = 0;
      cfg.pin_miso = -1;
      cfg.pin_dc   = 4;

      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    { // Panel config
      auto cfg = _panel.config();
      cfg.pin_cs   = 10;
      cfg.pin_rst  = 2;
      cfg.pin_busy = -1;

      cfg.panel_width  = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;

      cfg.invert = true;   // if colors look wrong later, we’ll toggle this
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;

      _panel.config(cfg);
    }

    { // Backlight config (we already know BL=8 works)
      auto cfg = _light.config();
      cfg.pin_bl = 8;
      cfg.invert = true;       // if backlight behaves backwards later, flip this
      cfg.freq   = 44100;
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }

    setPanel(&_panel);
  }
};

LGFX lcd;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Boot");

  lcd.init();
  lcd.setBrightness(255);
  lcd.setRotation(0);

  Serial.println("LCD init done");
}

void loop() {
  Serial.println("RED");   lcd.fillScreen(TFT_RED);   delay(600);
  Serial.println("GREEN"); lcd.fillScreen(TFT_GREEN); delay(600);
  Serial.println("BLUE");  lcd.fillScreen(TFT_BLUE);  delay(600);
  Serial.println("WHITE"); lcd.fillScreen(TFT_WHITE); delay(600);
  Serial.println("BLACK"); lcd.fillScreen(TFT_BLACK); delay(600);
}