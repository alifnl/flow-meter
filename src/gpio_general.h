#ifndef GPIO_GENERAL_H
#define GPIO_GENERAL_H
#include "stdint.h"
#include <Arduino.h>
#include "esp_timer.h"
typedef struct
{
    uint8_t pin;
    uint8_t level;
    uint8_t neg_trig;      // this flag will be high if there is negative trigger
    uint8_t pos_trig;      // this flag will be high if there is positive trigger
    uint8_t pos_hold_trig; // this flag will be high if input is held high for certain time
    uint8_t neg_hold_trig; // this flag will be high if input is held
    uint64_t trigger_time;
    int read_logic_z;
    int hold_state;
}input_data;

typedef enum
{
    NEGATIVE_TRIGGER, // negative trigger means from 1 to 0
    POSITIVE_TRIGGER, // positive trigger means from 0 to 1
    POS_NEG_TRIGGER   // will trigger both positive and negative transition
} event_trigger_t;

typedef enum
{
    ACTIVE_LOW,
    ACTIVE_HIGH
} activation_type_t;

void general_gpio_init (input_data *pData, uint8_t pin);
int read_event(input_data *pData, event_trigger_t trigger_mode);
int read_button(input_data *pData, activation_type_t trigger_mode, uint64_t short_press_time, uint64_t hold_time);
uint64_t get_millis();

void general_gpio_init (input_data *pData, uint8_t pin){
  pData->pin = pin;
  pinMode(pData->pin, INPUT_PULLUP);
  pData->read_logic_z = digitalRead(pData->pin);
  return; 
}


int read_event(input_data *pData, event_trigger_t trigger_mode)
{
  pData->level = digitalRead(pData->pin); 
  int flag = 0;

  pData->neg_trig = 0;
  pData->pos_trig = 0;

    //will generate trigger if the logic read change from 0 to 1 or 1 to 0 based on trigger mode
    //NGT will result 1 or can check struct input_data->neg_trig;
    //PGT will result 2 or can check struct input_data->pos_trig;
  if (pData->level == 0 && pData->read_logic_z == 1 && (trigger_mode == NEGATIVE_TRIGGER || trigger_mode == POS_NEG_TRIGGER))
  {
    flag = 1;
    pData->neg_trig = 1;
    pData->pos_trig = 0;
  }
  else if (pData->level == 1 && pData->read_logic_z == 0 && (trigger_mode == POSITIVE_TRIGGER || trigger_mode == POS_NEG_TRIGGER))
  {
    flag = 2;
    pData->pos_trig = 1;
    pData->neg_trig = 0;
  }

    //shift data stored in struct
  pData->read_logic_z = pData->level;

  return flag;
}

//int read_long_press function can be used to read transition event based on trigger mode
//or can read long press detection based on hold time and trigger mode
// transition event will result 1 and long press event will result 2
int read_button(input_data *pData, activation_type_t trigger_mode, uint64_t short_press_time, uint64_t hold_time)
{ // hold time in millis
  int result = 0;
  read_event(pData, POS_NEG_TRIGGER);
  
  //trigger negative event if trigger mode is NEGATIVE_HOLD
  //trigger positive event if trigger_mode is POSTIVE_HOLD
  if (pData->hold_state == 0 && ((pData->neg_trig && trigger_mode == ACTIVE_LOW) || (pData->pos_trig  && trigger_mode == ACTIVE_HIGH)))
  { //
    // holding state
    pData->hold_state = 1;
    pData->trigger_time = get_millis();
    pData->neg_hold_trig = 0;
    pData->pos_hold_trig = 0;
    result = 0;
  }
  else if (pData->hold_state == 1)
  {
    if((pData->pos_trig && trigger_mode == ACTIVE_LOW
    || pData->neg_trig && trigger_mode == ACTIVE_HIGH)
    && get_millis() - pData->trigger_time <= short_press_time){
      pData->hold_state == 0;
      result = 1;
      //release button if the button pressed shorter than short_press_time 
      //will trigger short press event
    }else if ((pData->level == 0 && trigger_mode == ACTIVE_LOW) || 
              (pData->level == 1 && trigger_mode == ACTIVE_HIGH))
    {
      if (get_millis() - pData->trigger_time >= hold_time)
      {   //will trigger hold event if the hold time is exceeded
        pData->neg_hold_trig = (trigger_mode == ACTIVE_LOW);
        pData->pos_hold_trig = (trigger_mode == ACTIVE_HIGH);
        pData->hold_state = 0;
        result = 2;
      }
    }else{
      pData->hold_state = 0;
      result = 0;
    }
  }
  else{
        //reset if none of above conditions met
    pData->neg_hold_trig = 0;
    pData->pos_hold_trig = 0;
    pData->hold_state = 0;
    result = 0;
  }

  return result;
}

uint64_t get_millis()
{
	return esp_timer_get_time() * 0.001;
}
#endif




