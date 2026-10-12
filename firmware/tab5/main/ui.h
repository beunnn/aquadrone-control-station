#ifndef UI_H
#define UI_H

// Builds the selector screen and its refresh timer. Called once from app_main with the LVGL lock held (bsp_display_lock).
void uiStart( void );

#endif
