add_library(RTC-lib STATIC RTClib.cpp 
                        RTC_DS3231.cpp RTC_DS1307.cpp 
                        RTC_PCF8523.cpp RTC_PCF8563.cpp)
target_link_libraries(RTC-lib hardware_i2c pico_stdlib)