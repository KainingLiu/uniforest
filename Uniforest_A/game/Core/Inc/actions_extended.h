#ifndef UNIFOREST_ACTIONS_EXTENDED_H
#define UNIFOREST_ACTIONS_EXTENDED_H
#include "actions.h"
/* Independent action status/token/sequence; legacy Actions_* stays untouched. */
uint8_t ActionsEx_Start(uint32_t token, uint8_t id, uint8_t flags);
void ActionsEx_Update(void);
void ActionsEx_Abort(uint8_t state);
uint8_t ActionsEx_IsBusy(void);
ActionStatus_t ActionsEx_GetStatus(void);
void ActionsEx_Reset(void);
#endif
