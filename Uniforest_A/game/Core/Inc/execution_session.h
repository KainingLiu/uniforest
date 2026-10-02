#ifndef UNIFOREST_EXECUTION_SESSION_H
#define UNIFOREST_EXECUTION_SESSION_H
#include "protocol.h"
#define EXEC_VERSION 1u
#define EXEC_CAPABILITIES 7u /* isolated session, full lift, clearing stop */
#define EXEC_LEASE_MS 200u
void ExecutionSession_Poll(void);
uint8_t ExecutionSession_Handle(const ProtoFrame_t *frame);
void ExecutionSession_LegacyTakeover(void);
uint32_t ExecutionSession_Id(void);
/* Only used after the outer dispatcher has chosen a command family. */
void Protocol_DispatchLegacy(const ProtoFrame_t *frame);
uint16_t Protocol_GetTelemetryRate(void);
void Protocol_RestoreTelemetryRate(uint16_t rate);
#endif
