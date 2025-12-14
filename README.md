Core Changes

Added a thread-safe data hub so IMU/GPS writers and consumers stop sharing globals (components/application/app_state/app_state.c (line 15), header in app_state/include/app_state.h (line 5)). app_axis6 now just samples and posts to the hub (components/application/app_axis6/app_axis6.c (line 8) & :43) while app_sdcard owns a dedicated 25 Hz logger task (components/application/app_sdcard/app_sdcard.c (line 191) & :261) that also feeds ml_window. app_main bootstraps the hub once (main/main_app.c (line 12) & :20). This removes the SD-card jitter your collaborator flagged.
Refined the GPS stack: parsing lives in its own module with checksum protection and safe token copies (components/application/app_gps/app_gps_parser.c (line 20)–:352) and the runtime task only streams UART data into that parser (components/application/app_gps/app_gps.c (line 30)). The new app_gps.h (line 1) exposes a clean GNSS_Data while current_data/Deal_Data are gone.
Build/resource cleanup: main/CMakeLists.txt (line 1) now builds only main_app.c, application CMake pulls in the relocated wallpaper asset (components/application/CMakeLists.txt (line 5), file moved to components/application/app_gui/assets/wallpaper_image.c). I removed the unused temp_app_gui.txt and the entire components/drivers/esp_lcd_sh8601/test_apps/ tree per your request.
GUI polishing: dormant #if 0 blocks are gone from components/application/app_gui/app_gui.c (line 1) so only the live wallpaper/clock code remains, and all ML preview stubs were removed.
Hardware config is menuconfig-driven now: main/Kconfig.projbuild (line 1) defines the new JOFTMODE_* knobs, which feed directly into the I²C driver (components/application/interface/axis6_interface/axis6_interface.c (line 22)) and the power-button handler (components/application/app_power/app_power.c (line 11)).
Next Steps

Re-run idf.py menuconfig (or edit sdkconfig) to set the new JOFTMODE_* GPIO/frequency defaults, then rebuild with idf.py build to regenerate the binary and updated sdkconfig.
Flash/monitor on hardware to verify SD logging cadence, GPS parsing, and the GUI wallpaper asset that now lives inside the app component.
(Optional) add unit/system tests for the new parser/logger paths once you confirm the runtime behavior.
Let me know if you want me to tackle further cleanups or help validate on hardware.