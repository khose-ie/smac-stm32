#ifndef _STM32_CAN_H_
#define _STM32_CAN_H_

#include <smac-mcu.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
smacCanFd_t smac_can_fd_create_classic(void* handle);

smacRetCode_t smac_can_fd_set_event_classic(smacCanFd_t canfd, smacCanEvent_t* event,
                                            smacMcuEventData_t data);

smacRetCode_t smac_can_fd_transmit_classic(smacCanFd_t canfd, const smacCanMessage* message,
                                           uint32_t timeout);

smacRetCode_t smac_can_fd_receive_channel0_classic(smacCanFd_t canfd, smacCanMessage* message,
                                                   uint32_t timeout);

smacRetCode_t smac_can_fd_receive_channel1_classic(smacCanFd_t canfd, smacCanMessage* message,
                                                   uint32_t timeout);

smacRetCode_t smac_can_fd_async_transmit_classic(smacCanFd_t canfd, const smacCanMessage* message);

smacRetCode_t smac_can_fd_async_receive_channel0_classic(smacCanFd_t canfd,
                                                         smacCanMessage* message);

smacRetCode_t smac_can_fd_async_receive_channel1_classic(smacCanFd_t canfd,
                                                         smacCanMessage* message);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _STM32_CAN_H_
