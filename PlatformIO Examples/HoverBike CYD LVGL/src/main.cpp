// based on https://github.com/Kafkar/ESP32_2432S028R-LVGL-tutorial
#include <Arduino.h>

#include "../include/hoverserial.h"

// rx pin on 4pin serial JST1.5mm heder left to USB-C is not working because TX output CH340C pulls to high on idle :-(
// solder rx and tx directly to the ESP32 WROOM module, which are pulled through the rgb led to 3.3V with 1 kOhm resistor.
// solder gnd to that 4pin serial header, 5V from Hoverboard Uart to the VIn pin of same JST port (or to the connected little smd transistor = easier)
//#define SERIAL2_RX 16   // uncomment these 2 lines if other pins are used.
//#define SERIAL2_TX 17   // uncomment these 2 lines if other pins are used.
HardwareSerial oSerialHover(2);
#define HOVER_BAUDRATE 115200   // you need to change RemoteUart/RemoteUartBus baudrate also in config.h : #define REMOTE_BAUD 115200

SerialHover2Server oHoverFeedback;

#include <SPI.h>

// include the installed LVGL- Light and Versatile Graphics Library - https://github.com/lvgl/lvgl
#include <lvgl.h>

// include the installed "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
#include <TFT_eSPI.h>

// include the installed the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
#include <XPT2046_Touchscreen.h>


// Create a instance of the TFT_eSPI class
TFT_eSPI tft = TFT_eSPI();

// Set the pius of the xpt2046 touchscreen
#define XPT2046_IRQ 36  // T_IRQ
#define XPT2046_MOSI 32 // T_DIN
#define XPT2046_MISO 39 // T_OUT
#define XPT2046_CLK 25  // T_CLK
#define XPT2046_CS 33   // T_CS

// Create a instance of the SPIClass and XPT2046_Touchscreen classes
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// Define the size of the TFT display
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// Define the size of the buffer for the TFT display
#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

// Create variables for the LVGL objects
lv_obj_t * sw;
lv_obj_t * slider;
lv_obj_t * btn_label;
lv_obj_t * text0_label;
lv_obj_t * text_label;
lv_obj_t * chart;
#define SER_COUNT 6
lv_chart_series_t * aSer[SER_COUNT];


// Create a variable to store the LED state
bool ledsOff = false;
bool rightLedOn = true;

// Create a buffer for drawing
uint32_t draw_buf[DRAW_BUF_SIZE / 4];


// Get the Touchscreen data
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    z = p.z;

    data->state = LV_INDEV_STATE_PRESSED;

    // Set the coordinates
    data->point.x = x;
    data->point.y = y;
  }
  else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = (lv_obj_t*)lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        LV_UNUSED(obj);
        String s;
        if(lv_obj_has_state(obj, LV_STATE_CHECKED)==true)
        {
          DEBUGN("eventhandler",1)
          s = "speed on";
        }
        else
        {
          DEBUGN("eventhandler",0)
          s = "speed off";
        }
        lv_label_set_text(text_label, s.c_str());    
        //LV_LOG_USER("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
    }
}


static lv_obj_t * slider_label;
// Callback that prints the current slider value on the TFT display and Serial Monitor for debugging purposes
static void slider_event_callback(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t*) lv_event_get_target(e);
  int iSlider = (int)lv_slider_get_value(slider);
  char buf[12];
  lv_snprintf(buf, sizeof(buf), iSlider<100 ? "%d" : "#ff0000 %d", iSlider);
  lv_label_set_text(slider_label, buf);
  //lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}


static lv_style_t style_large;
void lv_create_main_gui(void) 
{
  LV_FONT_DECLARE(lv_font_montserrat_28);
  lv_style_init(&style_large);
  lv_style_set_text_font(&style_large, &lv_font_montserrat_28); // Bigger font than default

  sw = lv_switch_create(lv_screen_active());
  lv_switch_set_orientation(sw,LV_SWITCH_ORIENTATION_VERTICAL);
  lv_obj_set_width(sw, 30);    // Set smaller width to make the lines wrap  
  lv_obj_set_height(sw,55);

  lv_obj_add_event_cb(sw, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(sw, LV_OBJ_FLAG_EVENT_BUBBLE);
  lv_obj_align(sw, LV_ALIGN_TOP_LEFT , 5, 5);
  //lv_obj_add_state(sw, LV_STATE_CHECKED);

  text0_label = lv_label_create(lv_screen_active());
  //lv_label_set_long_mode(text0_label, LV_LABEL_LONG_WRAP);    // Breaks the long lines
  lv_label_set_recolor(text0_label, true);
  lv_label_set_text(text0_label, "#0000ff Hello, HoverBike V1.0 :-)");
  //lv_obj_add_style(text0_label, &style_large, 0);  // Apply the style to the label
  lv_obj_set_width(text0_label, 270);    // Set smaller width to make the lines wrap
  lv_obj_set_height(text0_label, 20);    // Set smaller width to make the lines wrap
  lv_obj_set_style_text_align(text0_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(text0_label, LV_ALIGN_TOP_MID, 0, 0);

  //lv_obj_align_to(text0_label, sw, LV_ALIGN_OUT_RIGHT_TOP, 0, -5);

  text_label = lv_label_create(lv_screen_active());
  //lv_label_set_long_mode(text_label, LV_LABEL_LONG_WRAP);    // Breaks the long lines
  lv_label_set_recolor(text_label, true);
  lv_label_set_text(text_label, "#ff0000 www.2China.de");
  lv_obj_add_style(text_label, &style_large, 0);  // Apply the style to the label
  lv_obj_set_width(text_label, 270);    // Set smaller width to make the lines wrap
  lv_obj_set_height(text_label, 30);    // Set smaller width to make the lines wrap
  lv_obj_set_style_text_align(text_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(text_label, LV_ALIGN_TOP_MID, 0, 20);
  //lv_obj_align_to(text_label, sw, LV_ALIGN_OUT_RIGHT_TOP, 0, -5);



  
  // Create a slider aligned in the center bottom of the TFT display
  slider = lv_slider_create(lv_screen_active());
  lv_slider_set_orientation(slider,LV_SLIDER_ORIENTATION_VERTICAL);
  lv_obj_set_width(slider, 30);    // Set smaller width to make the lines wrap  
  lv_obj_set_height(slider,170);
  lv_obj_align(slider, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_add_event_cb(slider, slider_event_callback, LV_EVENT_VALUE_CHANGED, NULL);
  lv_slider_set_range(slider, 0, 100);
  lv_obj_set_style_anim_duration(slider, 2000, 0);

  // Create a label below the slider to display the current slider value
  slider_label = lv_label_create(lv_screen_active());
  lv_obj_add_style(slider_label, &style_large, 0);  // Apply the style to the label
  lv_label_set_recolor(slider_label, true);
  lv_obj_set_style_text_align(slider_label, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_text(slider_label, "0");
  //lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  lv_obj_align(slider_label, LV_ALIGN_TOP_RIGHT, 0, 0);

  /*Create a stacked_area_chart.obj*/
  chart = lv_chart_create(lv_screen_active());
  lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
  lv_chart_set_axis_range(chart,LV_CHART_AXIS_PRIMARY_Y,-400,400);
  lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);
  int width = 280, height = 180;
  lv_obj_set_size(chart, width, height);
  //lv_obj_center(chart);
  lv_obj_align(chart, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  lv_chart_set_point_count(chart, 100);
  aSer[0] = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
  aSer[1] = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
  aSer[2] = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
  aSer[3] = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
  aSer[4] = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
  aSer[5] = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
  //for(i = 0; i < 100; i++)  lv_chart_set_next_value(chart, ser, (int32_t)lv_rand(10, 90));  // Prefill with data

}

void setup() {
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.begin(115200);
  Serial.println(LVGL_Arduino);
  
  #ifdef SERIAL2_RX
    oSerialHover.begin(HOVER_BAUDRATE, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
  #else
    oSerialHover.begin(HOVER_BAUDRATE);
  #endif

  // Start LVGL
  lv_init();

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 0: touchscreen.setRotation(0);
  touchscreen.setRotation(2);

  // Create a display object
  lv_display_t *disp;

  // Initialize the TFT display using the TFT_eSPI library
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    
  // Initialize an LVGL input device object (Touchscreen)
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);

  // Set the callback function to read Touchscreen input
  lv_indev_set_read_cb(indev, touchscreen_read);

  // Function to draw the GUI (text, buttons and sliders)
  lv_create_main_gui();
}

unsigned long iTimeHoverLastRx = 0;
unsigned long iTimeHoverSend = 0;
unsigned long iTimeLVGL = 0;
void loop() 
{
  unsigned long iNow = millis();

  if (HoverReceive(oSerialHover,oHoverFeedback))
  //if (1)
  {
    DEBUGT("millis",iNow-iTimeHoverLastRx);
    DEBUGN("mpu",oHoverFeedback.iAmpR);

    float fCircumference = 2.0420352; // in meters
    //2,45044224 = (90,0/270,0) *2,0420352*3,6   = (hall_hoverMotor/hall_wheel) * wheel_diameter*PI * 3.6 kmh/revs
    float fSpeedKmh = 2.45044224*oHoverFeedback.iSpeedL/100.0;  // conversion from revs/s to km/h for my ebike hub motor with internal 3:1 planetary gearbox
    float fDistance = (fCircumference/270) * oHoverFeedback.iOdomL;

    if (iNow > 2000)
    {
      String sMess = String(oHoverFeedback.iGyroX) + " , " + String(oHoverFeedback.iGyroY) + " , " + String(oHoverFeedback.iGyroZ)
          + "    " + String(oHoverFeedback.iAccelX) + " , " + String(oHoverFeedback.iAccelY) + " , " + String(oHoverFeedback.iAccelZ) + "   ";
      lv_label_set_text(text0_label, sMess.c_str());    

      //sMess = "#ff0000 " + String(fSpeedKmh,1) + "# #00ff00 " + String(oHoverFeedback.iVolt/100.0,1) + // " , " + String(oHoverFeedback.iAmpL/100.0)
      //    + " # #0000ff " + String(fDistance,fDistance < 10 ? 1 : 0);

    //fSpeedKmh = 23.12, oHoverFeedback.iAmpR = 23452, oHoverFeedback.iSpeedR = (uint16_t) 52232, oHoverFeedback.iAmpL = 8;
     sMess = "#000000 " + String(oHoverFeedback.iAmpR) + "# #ff0000 " + String((int16_t)oHoverFeedback.iSpeedR) + " # #0000ff " + String((int16_t)oHoverFeedback.iOdomR)
          + " # #000000 " + String(oHoverFeedback.iAmpL/100) + "   ";
           
      lv_label_set_text(text_label, sMess.c_str());    
    }

    int32_t ai[SER_COUNT] = {oHoverFeedback.iGyroX/2+200,oHoverFeedback.iGyroY/2+200,oHoverFeedback.iGyroZ*2+200
          ,oHoverFeedback.iAccelX/64-200,oHoverFeedback.iAccelY/64-200,oHoverFeedback.iAccelZ/64-200};
    uint32_t p = lv_chart_get_point_count(chart);
    for (int i=0; i<SER_COUNT; i++)
    {
      lv_chart_set_next_value(chart, aSer[i], ai[i]);

      uint32_t s = lv_chart_get_x_start_point(chart, aSer[i]);
      int32_t * a = lv_chart_get_series_y_array(chart, aSer[i]);
      a[(s + 1) % p] = LV_CHART_POINT_NONE;
      a[(s + 2) % p] = LV_CHART_POINT_NONE;

    }
    //lv_chart_series_t * ser = NULL;
    //ser = lv_chart_get_series_next(chart, ser);


/*
    a[(s + 3) % p] = LV_CHART_POINT_NONE;
    lv_chart_refresh(chart);
*/


    //DEBUGT("iSpeed",iSpeed);
    //DEBUGT("iSteer",iSteer);
    //HoverLog(oHoverFeedback);
    iTimeHoverLastRx = iNow;
  }

  /* paththrough for autodetect 
  while (Serial.available())
  {
    char c = Serial.read();
    oSerialHover.write(c);   // send it to hoverboard (autodetect)
  }
  while (oSerialHover.available())
  {
    char c = oSerialHover.read();
    Serial.write(c);   // read it and send it out Serial (USB)
  }
  */

  if (iNow > iTimeLVGL)
  {
    lv_task_handler();  // let the GUI do its work
    lv_tick_inc(iNow - iTimeLVGL);     // tell LVGL how much time has passed
    iTimeLVGL = iNow +5;

  }

  if (iNow > iTimeHoverSend)
  {
    iTimeHoverSend = iNow + 100;
    boolean bOn = lv_obj_has_state(sw, LV_STATE_CHECKED);
    int iSpeed = bOn ? 10 * lv_slider_get_value(slider) : 0;
    HoverSend(oSerialHover,0,iSpeed);
  }

}