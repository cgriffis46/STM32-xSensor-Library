/*
 * app.h
 *
 * The CAN joystick sender (app.cpp), called from CubeMX's freertos.c in its
 * USER CODE block, so regenerating code in CubeMX keeps the call:
 *   StartDefaultTask, USER CODE StartDefaultTask   app_default_task(argument);
 */
#ifndef APP_H_
#define APP_H_

#ifdef __cplusplus
extern "C" {
#endif

void app_default_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
