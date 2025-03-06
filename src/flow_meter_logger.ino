#include "gpio_general.h"
#include "i2cLCD.h"
#include <Wire.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "statechart.h"
#include "esp_spiffs.h"
#include <WiFi.h>
#include "esp_mac.h"
#include "esp_http_server.h"
#include "webserver_setup.h"
#include "logger.h"
#include "global_config.h"

input_data UP;
input_data DOWN;
input_data MENU;
input_data ESC;

LiquidCrystal_I2C lcd_i2c(0x27, 16, 2);
state_chart ui_chart;


char menu_list_up[][16]= {
    "Meas. Vol:",
    "Pulse Cnt:",
    "(1) HF mL/p:",
    "(2) LF mL/p",
    "(3) Vol THRE:",
    "(4) Max Vol:",
    "(5) Reset Vol"
};

char menu_list_down[][16] = {
  "0",                 //measured volume
  "0",                 //pulse count
  "0",                 //high flow ml/pulse value
  "0",                 //low flow ml/pulse
  "0",                 //threshold value
  "0",                 //max volume
  "   YES     NO  "    //reset volume menu
};

// 0 -> measured volume page
// 1 -> pulse count page
// 2 -> high flow mL/pulse
// 3 -> low flow ml/pulse
// 4 -> threshold (alarm)
// 5 -> max vol setting
// 6 -> reset volume
int8_t menu_position = 0;


int32_t param_value[MENU_PAGE] = { //match it with menu_position
  0, //volume measured
  0, //count pulse
  0, //HF ml/pulse value
  0, //LF ml/pulse value
  0, //threshold value
  1  //max vol setting
};

int32_t max_value_param[MENU_PAGE] = {
  0,      //meas volume (not configurable)
  0,      //count pulse  (not configurable)
  50000,  //ml/pulse max value is 20.000
  50000,  //high ml/pulse max value is 20.000
  100,    //100%
  250,    //maximal volume 
};


int32_t min_value_param[MENU_PAGE] = {
  0,        //meas volume (not configurable)
  0,        // count pulse (not configurable)
  0,        //ml/pulse min value is 0.000
  0,        //high ml/pulse min value is 0.000
  0,        //min alarm value 0%
  1,        //min volume 0L
};

const char* param_key_nvs[MENU_PAGE] = {
  "param0", //meas volume
  "param5", //pulse count 
  "param1", //mL/p value
  "param2", //scale value
  "param3", //threshold value
  "param4",  //max vol
  ""
};

byte solid_black[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

int16_t beep_tone[30] = 
{
  5000, 0, 5000, 0, 0,
  5000, 0, 5000, 0, 0,
  5000, 0, 5000, 0, 0,
  5000, 0, 5000, 0, 0,
  5000, 0, 5000, 0, 0,
  5000, 0, 5000, 0, 0
};

int32_t meas_vol_shdw = 0;
const char* shdw_nvs_key = "param1_shdw";
int tone_index = 0;
nvs_handle_t my_handle_nvs;
int flow_pulse = 0; //this one for counter intr only
intr_handle_t isr_handle_counter;      // Handle for custom ISR
hw_timer_t *timer = NULL;      // Timer handle
bool intr_flag_1s = 0;
int flow_pulse_acc = 0;
bool warning_global = false;
volatile bool interruptAllowed = true;
static esp_timer_handle_t limit_timer;

void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Wire.begin(21,22,200000);
  Serial.println("FW VERSION: 1.0.0");
  lcd_i2c.begin();
  lcd_i2c.createChar(12, solid_black);  // Store the custom character in index 0
  lcd_i2c.backlight();
  spiffs_setup();
  nvs_setup();
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 0);
  buzzer_init(); //passive buzzer need PWM signal
  setup_interrupt_flow();
  general_gpio_init(&UP, UP_PIN);
  general_gpio_init(&DOWN, DOWN_PIN);
  general_gpio_init(&MENU, MENU_PIN);
  general_gpio_init(&ESC, ESC_PIN);
  print_lcd(menu_list_up[menu_position], menu_list_down[menu_position],false);
  init_wifi();
  setup_server();
  NTP_config();
  //5th parameter is priority, the higher the number the higher the priority
  xTaskCreatePinnedToCore(state_metry_task, "state_metry_task", 4096, NULL, 3, NULL, 1);  
  xTaskCreatePinnedToCore(housekeep_task, "housekeep_task", 4096, NULL, 2, NULL, 1);
}

// the loop routine runs over and over again forever:
void loop() {
  vTaskDelay(10);
}


void print_lcd(char* up, char* down, bool blink_pointer){
  char up_formatted[17];  // 16 characters + 1 for null terminator
  char down_formatted[17];
  snprintf(up_formatted, sizeof(up_formatted), "%-16s", up);  // Left-align and pad with spaces
  snprintf(down_formatted, sizeof(down_formatted), "%-16s", down);  // Left-align and pad with spaces
  
  lcd_i2c.setCursor(0,0);
  lcd_i2c.print(up_formatted);
  lcd_i2c.setCursor(0,1);
  lcd_i2c.print(down_formatted);
  if(blink_pointer){
    int str_len = strlen(menu_list_down[menu_position]);
    lcd_i2c.setCursor(str_len,1);
    lcd_i2c.write(byte(12));
  }
}

void change_menu_position (int lowest_menu, int highest_menu, bool up){
  menu_position += up ? -1 : 1;
  menu_position = (menu_position < lowest_menu) ? highest_menu 
                : (menu_position > highest_menu) ? lowest_menu 
                : menu_position;
}

void statechart(){
  int up_button_logic = read_button(&UP, ACTIVE_LOW, 500, 1000);
  int down_button_logic = read_button(&DOWN, ACTIVE_LOW, 500, 1000);
  int menu_button_logic = read_button(&MENU, ACTIVE_LOW, 500, 1000);
  int esc_button_logic = read_button(&ESC, ACTIVE_LOW, 500, 1000);

  int arrow_logic = up_button_logic | down_button_logic;
  #ifdef DEBUG
  if(up_button_logic != 0)  Serial.println("up_button_logic: " +String (up_button_logic));
  if(down_button_logic !=0) Serial.println("down_button_logic: " +String (down_button_logic));
  #endif

  if(ui_chart.event_trans(esc_button_logic && warning_global, MAIN_STATE, MAIN_STATE)){
    //temporary volume reset
    meas_vol_shdw = 0;
    nvs_set_i32(my_handle_nvs, shdw_nvs_key, meas_vol_shdw);
  }
  if(ui_chart.event_trans(menu_button_logic == 1, MAIN_STATE, CONF_STATE)){
    menu_trans_menu(MAIN_STATE, CONF_STATE);
  }
  if(ui_chart.event_trans(esc_button_logic == 1, CONF_STATE, MAIN_STATE)){
    menu_trans_menu(CONF_STATE, MAIN_STATE);
  }
  if(ui_chart.event_trans(menu_button_logic == 1, CONF_STATE, CHANGE_PARAM_STATE)){
    menu_trans_menu(CONF_STATE, CHANGE_PARAM_STATE);
  }
  if(ui_chart.event_trans((esc_button_logic || menu_button_logic), CHANGE_PARAM_STATE, CONF_STATE)){
    if(esc_button_logic) exit_trans_conf(0, false);
    if(menu_button_logic)save_trans_conf();
  }
  if(ui_chart.event_trans(arrow_logic== 2 && menu_position != NON_CONFIG_PAGE-1+5, CHANGE_PARAM_STATE, HOLD_PARAM_STATE) ){
   param_trans_hold();
  }
  if(ui_chart.event_trans(UP.pos_trig || DOWN.pos_trig, HOLD_PARAM_STATE, CHANGE_PARAM_STATE) ){
    hold_trans_param();
  }
  if(ui_chart.event_trans(arrow_logic ==1, MAIN_STATE, MAIN_STATE)){
    menu_trans_menu(MAIN_STATE,MAIN_STATE);
  }
  if(ui_chart.event_trans(arrow_logic==1,CONF_STATE, CONF_STATE)){
    menu_trans_menu(CONF_STATE, CONF_STATE);
  }
  if(ui_chart.event_trans(arrow_logic==1, CHANGE_PARAM_STATE, CHANGE_PARAM_STATE)){
    param_trans_update();
  }

  //execute task based on state
  if((!UP.level || !DOWN.level ) && ui_chart.is_state(HOLD_PARAM_STATE)) {
    holding_state(0,false);
  }
  if(ui_chart.is_state(CHANGE_PARAM_STATE) && menu_position != NON_CONFIG_PAGE -1 + 5){
    blink_pointer_state(false, 0);
  }
  if(!ui_chart.is_state(MAIN_STATE)){
    lcd_i2c.backlight();
  }
  ui_chart.reset_transition();
}
 
void change_param_value(int32_t *base, int32_t adjust_val){
  long temp = (long)(*base) + adjust_val;  // Correctly modify the value
  if(temp >= max_value_param[menu_position]){
    *base = max_value_param[menu_position];
  }else if (temp <= min_value_param[menu_position]){
    *base = min_value_param[menu_position];
  }else{
    *base = (int) temp;
  }
}


//will change number to display character based on parameter specs.
//display character to lcd 
void update_param_display(int pos, bool logic, bool blink_pointer){
  if(pos == 0 || pos == NON_CONFIG_PAGE-1+4){
    int vol_percent = 100*param_value[0]/param_value[NON_CONFIG_PAGE-1+4];
    sprintf(menu_list_down[0], (param_value[0] * 0.001 >= 100) ? "%.1fL (%.1f%%)" 
                              : (param_value[0] *0.001 >= 10) ?  "%.2fL (%.1f%%)" : "%.3fL (%.1f%%)", 
    param_value[0] / 1000.0, vol_percent / 1000.0);  
    //max volume config
    if(pos == NON_CONFIG_PAGE-1+4)sprintf(menu_list_down[NON_CONFIG_PAGE-1+4], "%dL", param_value[NON_CONFIG_PAGE-1+4]);
  }else if(pos == NON_CONFIG_PAGE-1+1 || pos == NON_CONFIG_PAGE-1+2){ //mL/pulse value
    sprintf(menu_list_down[pos], "%.3f", param_value[pos] *0.001); 
  }else if (pos == NON_CONFIG_PAGE-1+3){
    sprintf(menu_list_down[pos], "%d%%", param_value[pos]); 
  }else if(pos == 1 ){
    sprintf(menu_list_down[pos], "%d", param_value[pos]);
  }
  if(logic) print_lcd(menu_list_up[pos], menu_list_down[pos], blink_pointer);
}


int64_t calculate_inst_flow(uint64_t pulse_in){
  //Pulse = a * Q - o
  // Q = (pulse+o)/a
  //Serial.printf("param %d %d\n", param_value[2], param_value[3]);
  if(pulse_in == 0) return 0;
  int64_t inst_flow = (long)pulse_in * param_value[2];
  uint64_t high_pulse = 35000/param_value[2]; //70ml per 2 second is higher volume
                                          //need to find the pulse with current low mL/p
 
  if(pulse_in < high_pulse){
    inst_flow = (long)pulse_in * param_value[3];
  }
  #ifdef DEBUG
  //ESP_LOGE("pulse->", "%d", pulse_in);
  Serial.println("pulse-> " + String(pulse_in));
  #endif
  return inst_flow; //return is like 2000 for mL unit
}

void threshold_warning(){
  static int i = 1;
  warning_global = meas_vol_shdw >= param_value[NON_CONFIG_PAGE-1+4]*10*param_value[NON_CONFIG_PAGE-1+3];
  buzzer_tone(warning_global ? beep_tone[tone_index]: 0);
  tone_index = (tone_index >= 29 || !warning_global) ? 0 : tone_index+1;
  i = warning_global && menu_position == 0 ? i+1 : 1;
  i%=2;
  if(i == 0) lcd_i2c.noBacklight();
  if(i == 1) lcd_i2c.backlight();
}

void calculate_total_vol(){
  //integrator difference equation
  //y[n]=y[n−1]+0.5(x[n]+x[n−1])
  //static int inst_flow_z = 0;
  int64_t inst_flow = calculate_inst_flow(flow_pulse_acc);  //result is uL
  #ifdef DEBUG
  //Serial.println("inst flow ret [mL/s]: " + String(inst_flow));
  #endif
  int64_t inst_vol = (long)round(inst_flow*0.001);//((long) inst_flow + inst_flow_z )>>1;
  //update nvs
  param_value[0] += inst_vol; //mL unit
  param_value[1] += flow_pulse_acc;
  meas_vol_shdw += inst_vol;
  
  #ifdef DEBUG
  //Serial.println("new total volume is " + String(param_value[0]));
  #endif
  if(flow_pulse_acc != 0){
    nvs_set_i32(my_handle_nvs, shdw_nvs_key, meas_vol_shdw);
    nvs_set_i32(my_handle_nvs, param_key_nvs[0], param_value[0]); //write meas volume to nvs
    nvs_set_i32(my_handle_nvs, param_key_nvs[1], param_value[1]); //write count pulse to nvs
  }
  update_param_display(0,menu_position == 0, false);
  update_param_display(1,menu_position == 1, false);
  //total volume unit is mL
}

// Custom ISR for pulse counting
static void IRAM_ATTR pulse_ISR(void* arg) {
  if (interruptAllowed) {
      interruptAllowed = false;
      esp_timer_start_once(limit_timer, INTERRUPT_LIMIT_US);  // Start timer for 10ms
      flow_pulse++;
  }
}

// Timer ISR to reset and print pulse count every 1s
bool IRAM_ATTR timer_ISR(void* arg) {
  intr_flag_1s = true;
  flow_pulse_acc = flow_pulse;
  flow_pulse = 0;
  return true;
}

// Function to manually configure interrupt with priority
void setup_interrupt_flow() {
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << flow_sensor_pin),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE  // Rising edge trigger
  };
  gpio_config(&io_conf);
  // Install ISR service (if not already installed)
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);  // Priority level 3 (safe for user ISR)
  // Add ISR handler
  gpio_isr_handler_add((gpio_num_t) flow_sensor_pin, pulse_ISR, (void *)flow_sensor_pin);
  const esp_timer_create_args_t timer_args = {
        .callback = limit_timer_callback,
        .name = "limit_timer"
    };
  esp_timer_create(&timer_args, &limit_timer);
  
  timer_config_t config = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80 // 1 tick = 1µs
  };
  
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000000);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_ISR, NULL, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2);
  timer_start(TIMER_GROUP_0, TIMER_0);
}

void limit_timer_callback(void* arg) {
    interruptAllowed = true;
}


void buzzer_init() {
  ledc_timer_config_t timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 1000, // Default frequency
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {
      .gpio_num = BUZZER_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 512,  // 50% duty cycle
      .hpoint = 0
  };
  ledc_channel_config(&channel);
  buzzer_tone(0);
}

void buzzer_tone(uint32_t freq) {
  if (freq == 0) {
      ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
      return;
  }
  ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq);
}

void menu_trans_menu(state_name_t state1, state_name_t state2){
  if(state1 == MAIN_STATE && state2 == CONF_STATE){
    menu_position = NON_CONFIG_PAGE;
  }else if(state1 == CONF_STATE && state2 == MAIN_STATE){
    menu_position = 0;
  }else if(state1 == MAIN_STATE && state2 == MAIN_STATE){
    change_menu_position(0, 1, UP.pos_trig == 1);
  }else if(state1 == CONF_STATE && state2 == CONF_STATE){
    change_menu_position(NON_CONFIG_PAGE,MENU_PAGE-1, UP.pos_trig == 1);
  }else if(state1 == CONF_STATE && state2 == CHANGE_PARAM_STATE){
    exit_trans_conf(param_value[menu_position], true);
    blink_pointer_state(true, 0);
    if(menu_position == NON_CONFIG_PAGE-1+5){
      menu_list_down[menu_position][10] = '>'; 
      update_param_display(menu_position, false, false);
    }
  }
  print_lcd(menu_list_up[menu_position], menu_list_down[menu_position], false);
  #ifdef DEBUG
  Serial.println("change menu to " + String(menu_list_up[menu_position]) + "\nValue: " + String (menu_list_down[menu_position]));
  #endif
}


void param_trans_update (){
  //up down button will change value
  int32_t adj_val = UP.pos_trig  ? 1 : -1;
  if(menu_position != NON_CONFIG_PAGE-1 + 5)
  { 
    change_param_value(&param_value[menu_position], adj_val);
    #ifdef DEBUG
    Serial.println("change value to " + String(param_value[menu_position]));
    #endif
  }
  else
  {
    menu_list_down[menu_position][adj_val == 1 ? 2 : 10] = '>'; 
    menu_list_down[menu_position][adj_val == 1 ? 10 : 2] = ' ';
    #ifdef DEBUG
    Serial.println("change menu to " + String(menu_list_up[menu_position]) + "\nValue: " + String (menu_list_down[menu_position]));
    #endif
  }
  update_param_display(menu_position, true, menu_position != NON_CONFIG_PAGE-1+5);
  
}

void save_trans_conf(){
  //save to nvs
  if(strcmp(param_key_nvs[menu_position], "") != 0){ //will not save if nvs key is blank
    int32_t a;
    nvs_get_i32(my_handle_nvs, param_key_nvs[menu_position], &a); 
    if( a != param_value[menu_position]){
      nvs_set_i32(my_handle_nvs, param_key_nvs[menu_position], param_value[menu_position]);
    }
    
  }
     blink_pointer_state(true, 1);
  if(menu_position == NON_CONFIG_PAGE-1+5) { 
  if(menu_list_down[NON_CONFIG_PAGE-1+5][2] == '>'){
      //reset max volume
      param_value[0] = 0;
      param_value[1] = 0;
      meas_vol_shdw = 0;
      menu_list_down[NON_CONFIG_PAGE-1+5][2] = ' ';
      nvs_set_i32(my_handle_nvs, param_key_nvs[0], param_value[0]);
      nvs_set_i32(my_handle_nvs, param_key_nvs[1], param_value[1]);
      nvs_set_i32(my_handle_nvs, shdw_nvs_key, meas_vol_shdw);  
  }
  menu_list_down[NON_CONFIG_PAGE-1+5][10] = ' ';
  }
  update_param_display(menu_position, true, false);
  #ifdef DEBUG
  Serial.println("saving parameter at " + String(menu_list_up[menu_position]) + ", value: " + String(menu_list_down[menu_position]));
  #endif
}

void exit_trans_conf(int temp_param_val, bool update_temp){
  static int tmp = 0;
  if(update_temp){
    tmp = temp_param_val;
  }else{
    param_value[menu_position] = tmp;
    //clear arrow cursor in menu_position 5
    menu_list_down[NON_CONFIG_PAGE-1+5][10] = ' ';
    menu_list_down[NON_CONFIG_PAGE-1+5][2] = ' ';
    update_param_display(menu_position, true, false);  
    blink_pointer_state(true, 1);
    #ifdef DEBUG
    Serial.println("Esc without saving");
    #endif
    
  }
}

void param_trans_hold(){
//transition before hold state
  holding_state(0,true);
  #ifdef DEBUG
  Serial.println("entering hold button state");
  #endif
  //return UP.neg_hold_trig == 1 ? UP_HOLD_STATE : DOWN_HOLD_STATE;
}

void holding_state(int buff, bool reset){
//prescaling to 150ms
static uint64_t psc_hold = 0;
static uint64_t time_val = 0;
if(reset){
  time_val = millis();
  psc_hold = time_val - 150;
}
else if(millis() - psc_hold >= 150){
  psc_hold = millis();
  int32_t adj_value = UP.level == 0 ? 10 : -10;
  if (millis() - time_val >= 10000){
    adj_value = UP.level == 0 ? 1000 : -1000;
  }else if(millis() - time_val >= 5000){
    adj_value = UP.level == 0 ? 100 : -100;
  }
  change_param_value(&param_value[menu_position], adj_value);
  update_param_display(menu_position, true, true);
  #ifdef DEBUG
  Serial.println("(HOLD) value change to " + String (param_value[menu_position]));
  #endif
}//end of prescaling
}

void hold_trans_param(){
  //releasing hold
  #ifdef DEBUG
  Serial.println("hold button released");
  #endif
}

void blink_pointer_state(bool reset, uint8_t pointer_blink){
  static uint64_t time_val = 0;
  static uint8_t _pointer = 0;
  if(reset){
    _pointer = pointer_blink;
    time_val = millis() - 500;
  }else if(millis() - time_val >= 500){
    time_val = millis();
    int a = strlen(menu_list_down[menu_position]);
    lcd_i2c.setCursor(a, 1);
    lcd_i2c.write(_pointer == 0 ? byte(12) : ' ');
    _pointer++;
    _pointer %= 2;    
  }
}

void WiFiEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("Wi-Fi started, connecting...");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to Wi-Fi!");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: 
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from Wi-Fi, reconnecting...");
      WiFi.reconnect();
      break;
    default:
      break;
  }
}


void init_wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.onEvent(WiFiEvent);
  
  WiFi.begin("FORMULATRIX_2.4GHz", "FmlxF@iry13");
  //WiFi.begin("FMLX-PPSK", "iotProd5321");
  //WiFi.begin("Ipong", "withmewmew");
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  printf("ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void state_metry_task(void *pvParameters){
  time_t now;
  while(1){
    //ESP_LOGE("TASK", "start_task");
    statechart();
    if(intr_flag_1s){
      //ESP_LOGE("TASK", "metry_task");
      calculate_total_vol();
      threshold_warning();
      //Serial.printf("Free heap: %d bytes\n", esp_get_free_heap_size());
      intr_flag_1s = false;
      time(&now);
      if (now > 951561638) { //less than year 2000
        epoch = now ;
      }
    }
    vTaskDelay(18);
  }
}

void housekeep_task(void *pvParameters){
  uint64_t tick = 0;
  esp_vfs_spiffs_conf_t spiffs_config = {
    .base_path = "/data",
    .partition_label = NULL, // if you have more than 1 partition
  };
  while(1){

    if((epoch - tick >= 60) && (epoch%60 == 0)){ //1 minute execution
      tick = epoch;
      if(epoch > 951561638 ){
        String file_name_local = epoch_to_date(epoch+TIME_OFFSET) + ".csv";
        String message = String(epoch) + ";" +String(param_value[0]) + ";" + ";" + "\n";
        if(!exists(file_name_local.c_str())) append_file(file_name_local.c_str(), HEADER_STRING);
        append_file(file_name_local.c_str(), message.c_str());
      } 

      size_t total = 0, used = 0;
      esp_err_t err_result = esp_spiffs_info(spiffs_config.partition_label, &total, &used);
      if (err_result == ESP_OK)
      {
          if(used >= 1000000){
            //enable housekeeping
            housekeeping();
          }
      }
    
    }
    vTaskDelay(1);
  }//end of while (1)
}