#ifndef STATECHART_H
#define STATECHART_H
#include "stdint.h"
#include <Arduino.h>

typedef enum{
    MAIN_STATE,
    CONF_STATE,
    CHANGE_PARAM_STATE,
    HOLD_PARAM_STATE
}state_name_t;


class state_chart{
public:
    bool get_trans_executed();
    void reset_transition();
    bool event_trans(bool execute_logic, state_name_t last_state, state_name_t state); //check if there is transition event
    bool is_state(state_name_t state);

private:
    bool trans_executed = false;
    state_name_t current_state_class = MAIN_STATE;
    int menu_position = 0;
    state_name_t last_state_class = MAIN_STATE;
};


void state_chart::reset_transition(){
    trans_executed = false;
    last_state_class = current_state_class;
}

bool state_chart::get_trans_executed(){
    return trans_executed;
}

bool state_chart::event_trans(bool execute_logic, state_name_t last_state, state_name_t state){
    if(execute_logic && !trans_executed && current_state_class == last_state){
        trans_executed = true;
        current_state_class = state;
        return true;
    }
    return false;
}

bool state_chart::is_state(state_name_t state){
    return state == current_state_class ? true : false;
}


#endif