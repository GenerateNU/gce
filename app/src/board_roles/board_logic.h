#ifndef BOARD_LOGIC_H
#define BOARD_LOGIC_H

/**
 * Board-specific state machine logic functions.
 * Each function implements the behavior for one board role.
 */

void handle_main_board_logic(void);
void handle_input_board_logic(void);
void handle_output_board_logic(void);

#endif // BOARD_LOGIC_H