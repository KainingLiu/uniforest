#ifndef UNIFOREST_ACTIONS_H
#define UNIFOREST_ACTIONS_H

#include <stdint.h>

#define ACTION_GRAP1 1u
#define ACTION_GRAP2 2u
#define ACTION_GRAP3 3u
#define ACTION_BUILD 4u

#define ACTION_IDLE 0u
#define ACTION_RUNNING 1u
#define ACTION_DONE 2u
#define ACTION_CANCELLED 3u
#define ACTION_TIMEOUT 4u
#define ACTION_REJECTED 5u

typedef struct {
    uint32_t token;
    uint8_t id;
    uint8_t state;
    uint8_t stage;
} ActionStatus_t;

uint8_t Actions_Start(uint32_t token, uint8_t id, uint8_t flags);
void Actions_Update(void);
void Actions_Abort(uint8_t state);
uint8_t Actions_IsBusy(void);
ActionStatus_t Actions_GetStatus(void);

#endif
