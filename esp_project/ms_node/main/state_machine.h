#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STATE_INIT,
    STATE_DISCOVER,
    STATE_CANDIDATE,
    STATE_CH,
    STATE_MEMBER,
    STATE_SLEEP
} node_state_t;

extern node_state_t g_current_state;
extern bool g_is_ch;
extern uint32_t g_node_id;
extern uint64_t g_mac_addr;

/**
 * @brief Initialize state machine
 */
void state_machine_init(void);

/**
 * @brief Run state machine (call periodically)
 */
void state_machine_run(void);

/**
 * @brief Get current state name
 */
const char* state_machine_get_state_name(void);

#endif // STATE_MACHINE_H

