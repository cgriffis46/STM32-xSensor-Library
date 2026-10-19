/*
 * app.h
 *
 * The BMP390/SHT31 bring-up (app.cpp), called from CubeMX's main.c in its
 * USER CODE blocks, so regenerating code in CubeMX keeps the calls:
 *   USER CODE 2                  app_init();
 *   StartSensorTask, USER CODE 5 app_sensor_task(argument);
 *   StartConsoleTask             app_console_task(argument);
 */
#ifndef APP_H_
#define APP_H_

#ifdef __cplusplus
extern "C" {
#endif

void app_init(void);
void app_sensor_task(void *argument);
void app_console_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
