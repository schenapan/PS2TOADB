/* 
*/
#ifndef ADBKeyTable_h
#define ADBKeyTable_h

#include "ADBKeyCode.h"

/* 
*/
const uint8_t adb_key[][ 2 ] = {
                { ADB_KC_NUM, PS2_KEY_NUM },
                { ADB_KC_SCROLL, PS2_KEY_SCROLL },
                { ADB_KC_CAPS, PS2_KEY_CAPS },
                { ADB_KC_L_SHIFT, PS2_KEY_L_SHIFT },
                { ADB_KC_R_SHIFT, PS2_KEY_R_SHIFT },
                { ADB_KC_CTRL, PS2_KEY_L_CTRL },
                { ADB_KC_ALT, PS2_KEY_L_ALT },
                { ADB_KC_SYSRQ, PS2_KEY_SYSRQ },
                { ADB_KC_ESC, PS2_KEY_ESC },
                { ADB_KC_BS, PS2_KEY_BS },
                { ADB_KC_TAB, PS2_KEY_TAB },
                { ADB_KC_ENTER, PS2_KEY_ENTER },
                { ADB_KC_SPACE, PS2_KEY_SPACE },
                { ADB_KC_KP0, PS2_KEY_KP0 },
                { ADB_KC_KP1, PS2_KEY_KP1 },
                { ADB_KC_KP2, PS2_KEY_KP2 },
                { ADB_KC_KP3, PS2_KEY_KP3 },
                { ADB_KC_KP4, PS2_KEY_KP4 },
                { ADB_KC_KP5, PS2_KEY_KP5 },
                { ADB_KC_KP6, PS2_KEY_KP6 },
                { ADB_KC_KP7, PS2_KEY_KP7 },
                { ADB_KC_KP8, PS2_KEY_KP8 },
                { ADB_KC_KP9, PS2_KEY_KP9 },
                { ADB_KC_KP_DOT, PS2_KEY_KP_DOT },
                { ADB_KC_KP_PLUS, PS2_KEY_KP_PLUS },
                { ADB_KC_KP_MINUS, PS2_KEY_KP_MINUS },
                { ADB_KC_KP_TIMES, PS2_KEY_KP_TIMES },
                { ADB_KC_KP_EQUAL, PS2_KEY_KP_EQUAL },
                { ADB_KC_0, PS2_KEY_0 },
                { ADB_KC_1, PS2_KEY_1 },
                { ADB_KC_2, PS2_KEY_2 },
                { ADB_KC_3, PS2_KEY_3 },
                { ADB_KC_4, PS2_KEY_4 },
                { ADB_KC_5, PS2_KEY_5 },
                { ADB_KC_6, PS2_KEY_6 },
                { ADB_KC_7, PS2_KEY_7 },
                { ADB_KC_8, PS2_KEY_8 },
                { ADB_KC_9, PS2_KEY_9 },
                { ADB_KC_APOS, PS2_KEY_APOS },
                { ADB_KC_COMMA, PS2_KEY_COMMA },
                { ADB_KC_MINUS, PS2_KEY_MINUS },
                { ADB_KC_DOT, PS2_KEY_DOT },
                { ADB_KC_DIV, PS2_KEY_DIV },
                { ADB_KC_SINGLE, PS2_KEY_SINGLE },
                { ADB_KC_A, PS2_KEY_A },
                { ADB_KC_B, PS2_KEY_B },
                { ADB_KC_C, PS2_KEY_C },
                { ADB_KC_D, PS2_KEY_D },
                { ADB_KC_E, PS2_KEY_E },
                { ADB_KC_F, PS2_KEY_F },
                { ADB_KC_G, PS2_KEY_G },
                { ADB_KC_H, PS2_KEY_H },
                { ADB_KC_I, PS2_KEY_I },
                { ADB_KC_J, PS2_KEY_J },
                { ADB_KC_K, PS2_KEY_K },
                { ADB_KC_L, PS2_KEY_L },
                { ADB_KC_M, PS2_KEY_M },
                { ADB_KC_N, PS2_KEY_N },
                { ADB_KC_O, PS2_KEY_O },
                { ADB_KC_P, PS2_KEY_P },
                { ADB_KC_Q, PS2_KEY_Q },
                { ADB_KC_R, PS2_KEY_R },
                { ADB_KC_S, PS2_KEY_S },
                { ADB_KC_T, PS2_KEY_T },
                { ADB_KC_U, PS2_KEY_U },
                { ADB_KC_V, PS2_KEY_V },
                { ADB_KC_W, PS2_KEY_W },
                { ADB_KC_X, PS2_KEY_X },
                { ADB_KC_Y, PS2_KEY_Y },
                { ADB_KC_Z, PS2_KEY_Z },
                { ADB_KC_SEMI, PS2_KEY_SEMI },
                { ADB_KC_BACK, PS2_KEY_BACK },
                { ADB_KC_OPEN_SQ, PS2_KEY_OPEN_SQ },
                { ADB_KC_CLOSE_SQ, PS2_KEY_CLOSE_SQ },
                { ADB_KC_EQUAL, PS2_KEY_EQUAL },
                { ADB_KC_EUROPE2, PS2_KEY_EUROPE2 },
                { ADB_KC_F1, PS2_KEY_F1 },
                { ADB_KC_F2, PS2_KEY_F2 },
                { ADB_KC_F3, PS2_KEY_F3 },
                { ADB_KC_F4, PS2_KEY_F4 },
                { ADB_KC_F5, PS2_KEY_F5 },
                { ADB_KC_F6, PS2_KEY_F6 },
                { ADB_KC_F7, PS2_KEY_F7 },
                { ADB_KC_F8, PS2_KEY_F8 },
                { ADB_KC_F9, PS2_KEY_F9 },
                { ADB_KC_F10, PS2_KEY_F10 },
                { ADB_KC_F11, PS2_KEY_F11 },
                { ADB_KC_F12, PS2_KEY_F12 },
                { ADB_KC_F13, PS2_KEY_F13 },
                { ADB_KC_F14, PS2_KEY_F14 },
                { ADB_KC_F15, PS2_KEY_F15 },
                { ADB_KC_F16, PS2_KEY_F16 },
                { ADB_KC_F17, PS2_KEY_F17 },
                { ADB_KC_F18, PS2_KEY_F18 },
                { ADB_KC_F19, PS2_KEY_F19 },
                { ADB_KC_F20, PS2_KEY_F20 },
                { ADB_KC_F21, PS2_KEY_F21 },
                { ADB_KC_F22, PS2_KEY_F22 },
                { ADB_KC_F23, PS2_KEY_F23 },
                { ADB_KC_F24, PS2_KEY_F24 },
                { ADB_KC_KP_COMMA, PS2_KEY_KP_COMMA },
                { ADB_KC_INTL1, PS2_KEY_INTL1 },
                { ADB_KC_INTL2, PS2_KEY_INTL2 },
                { ADB_KC_INTL3, PS2_KEY_INTL3 },
                { ADB_KC_INTL4, PS2_KEY_INTL4 },
                { ADB_KC_INTL5, PS2_KEY_INTL5 },
                { ADB_KC_LANG1, PS2_KEY_LANG1 },
                { ADB_KC_LANG2, PS2_KEY_LANG2 },
                { ADB_KC_LANG3, PS2_KEY_LANG3 },
                { ADB_KC_LANG4, PS2_KEY_LANG4 },
                { ADB_KC_LANG5, PS2_KEY_LANG5 },
                { ADB_KC_PRTSCR, PS2_KEY_PRTSCR },
                { ADB_KC_CTRL, PS2_KEY_R_CTRL },
                { ADB_KC_ALT, PS2_KEY_R_ALT },
                { ADB_KC_L_GUI, PS2_KEY_L_GUI },
                { ADB_KC_R_GUI, PS2_KEY_R_GUI },
                { ADB_KC_MENU, PS2_KEY_MENU },
                { ADB_KC_BREAK, PS2_KEY_BREAK },
                { ADB_KC_HOME, PS2_KEY_HOME },
                { ADB_KC_END, PS2_KEY_END },
                { ADB_KC_PGUP, PS2_KEY_PGUP },
                { ADB_KC_PGDN, PS2_KEY_PGDN },
                { ADB_KC_L_ARROW, PS2_KEY_L_ARROW },
                { ADB_KC_R_ARROW, PS2_KEY_R_ARROW },
                { ADB_KC_UP_ARROW, PS2_KEY_UP_ARROW },
                { ADB_KC_DN_ARROW, PS2_KEY_DN_ARROW },
                { ADB_KC_INSERT, PS2_KEY_INSERT },
                { ADB_KC_DELETE, PS2_KEY_DELETE },
                { ADB_KC_KP_ENTER, PS2_KEY_KP_ENTER },
                { ADB_KC_KP_DIV, PS2_KEY_KP_DIV },
                { ADB_KC_NEXT_TR, PS2_KEY_NEXT_TR },
                { ADB_KC_PREV_TR, PS2_KEY_PREV_TR },
                { ADB_KC_STOP, PS2_KEY_STOP },
                { ADB_KC_PLAY, PS2_KEY_PLAY },
                { ADB_KC_MUTE, PS2_KEY_MUTE },
                { ADB_KC_VOL_UP, PS2_KEY_VOL_UP },
                { ADB_KC_VOL_DN, PS2_KEY_VOL_DN },
                { ADB_KC_MEDIA, PS2_KEY_MEDIA },
                { ADB_KC_EMAIL, PS2_KEY_EMAIL },
                { ADB_KC_CALC, PS2_KEY_CALC },
                { ADB_KC_COMPUTER, PS2_KEY_COMPUTER },
                { ADB_KC_WEB_SEARCH, PS2_KEY_WEB_SEARCH },
                { ADB_KC_WEB_HOME, PS2_KEY_WEB_HOME },
                { ADB_KC_WEB_BACK, PS2_KEY_WEB_BACK },
                { ADB_KC_WEB_FORWARD, PS2_KEY_WEB_FORWARD },
                { ADB_KC_WEB_STOP, PS2_KEY_WEB_STOP },
                { ADB_KC_WEB_REFRESH, PS2_KEY_WEB_REFRESH },
                { ADB_KC_WEB_FAVOR, PS2_KEY_WEB_FAVOR },
                { ADB_KC_POWER, PS2_KEY_POWER },
                { ADB_KC_SLEEP, PS2_KEY_SLEEP },
                { ADB_KC_WAKE, PS2_KEY_WAKE }
              };
#endif
