#ifndef UI_H
#define UI_H

// UI states
typedef enum {
    IDLE,
    RUNNING,
    ERROR
} ui_state_t;

extern volatile ui_state_t ui_state;

int ui_init(void);
ui_state_t ui_get_state(void);
void ui_set_state(ui_state_t new_ui_state);

#endif