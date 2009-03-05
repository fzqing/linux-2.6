#ifndef __PXA_KBD_H
#define __PXA_KBD_H

#define MAX_KEYS 0xff
#define NO_KEY			0xFF

#define     KP_SC_A                    KEY_A
#define     KP_SC_B                    KEY_B
#define     KP_SC_C                    KEY_C
#define     KP_SC_D                    KEY_D
#define     KP_SC_E                    KEY_E
#define     KP_SC_F                    KEY_F
#define     KP_SC_G                    KEY_G
#define     KP_SC_H                    KEY_H
#define     KP_SC_I                    KEY_I
#define     KP_SC_J                    KEY_J
#define     KP_SC_K                    KEY_K
#define     KP_SC_L                    KEY_L
#define     KP_SC_M                    KEY_M
#define     KP_SC_N                    KEY_N
#define     KP_SC_O                    KEY_O
#define     KP_SC_P                    KEY_P
#define     KP_SC_Q                    KEY_Q
#define     KP_SC_R                    KEY_R
#define     KP_SC_S                    KEY_S
#define     KP_SC_T                    KEY_T
#define     KP_SC_U                    KEY_U
#define     KP_SC_V                    KEY_V
#define     KP_SC_W                    KEY_W
#define     KP_SC_X                    KEY_X
#define     KP_SC_Y                    KEY_Y
#define     KP_SC_Z                    KEY_Z

#define     KP_SC_PERIOD           KEY_PREVIOUS
#define     KP_SC_AT_SYMBOL    KEY_2	/* @ */
#define     KP_SC_SLASH             KEY_SLASH
#define     KP_SC_BACKSLASH     KEY_BACKSLASH

#define     KP_SC_HOME              KEY_HOME
#ifndef CONFIG_KEYPAD_ZYLONITE_SHIFT
#define     KP_SC_SHIFT             KEY_CAPSLOCK
#else
#define     KP_SC_SHIFT             KEY_LEFTSHIFT
#endif
#define     KP_SC_SPACE	            KEY_SPACE
#define     KP_SC_SPACE_RSVD	KEY_F21
#define     KP_SC_BLUEKEY		KEY_BLUE
#define     KP_SC_BACK                 KEY_BACK

#define     KP_SC_1                    KEY_1
#define     KP_SC_2                    KEY_2
#define     KP_SC_3                    KEY_3
#define     KP_SC_4                    KEY_4
#define     KP_SC_5                    KEY_5
#define     KP_SC_6                    KEY_6
#define     KP_SC_7                    KEY_7
#define     KP_SC_8                    KEY_8
#define     KP_SC_9                    KEY_9
#define     KP_SC_0                    KEY_0

#define     KP_SC_SOFT1                KEY_F22
#define     KP_SC_SOFT2                KEY_F23
#define     KP_SC_SEND                 KEY_SENDFILE
#define     KP_SC_END                  KEY_END
/* Navigation center scan codes */
#define     KP_SC_N_UP                 KEY_UP
#define     KP_SC_N_DOWN               KEY_DOWN
#define     KP_SC_N_LEFT               KEY_LEFT
#define     KP_SC_N_RIGHT              KEY_RIGHT
#define     KP_SC_N_ACTION             KEY_ENTER

#define     KP_SC_SCROLL_PUSH          KEY_ENTER

#define     KP_SC_STAR		KEY_8	    /* * */
#define     KP_SC_POUND		KEY_3	    /* # */
#define     KP_SC_BANG		KEY_9	    /* ! */
#define     KP_SC_QMARK		KEY_BACKSLASH	/* ? */
#define     KP_SC_DOLLAR	KEY_4	/* $ */
#define     KP_SC_AMPERSAND	KEY_7	/* & */

#define     KP_SC_COLON                KEY_SEMICOLON
#define     KP_SC_SEMICOLON        KEY_SEMICOLON
/*#define     KP_SC_????		0x83 */
/*#define     KP_SC_????		0xA3 */
#define     KP_SC_TAB                  KEY_TAB
#define     KP_SC_L_PAREN		KEY_KPLEFTPAREN	/* ( */
#define     KP_SC_R_PAREN              KEY_KPRIGHTPAREN
/*#define     KP_SC_L_ARROW  ???	0xA4 */
#define     KP_SC_ENTER		KEY_ENTER	/* Also called "RETURN" */

#define     KP_SC_SCROLL_UP            KEY_UP
#define     KP_SC_SCROLL_DOWN          KEY_DOWN

#define     KP_SC_RECORD               KEY_RECORD
#define     KP_SC_DELETE               KEY_DELETE

#define     KP_SC_AUX1                KEY_AUX
#define     KP_SC_AUX2                 KEY_AUX
#define     KP_SC_AUX3                 KEY_AUX

#define     KP_SC_VOL_UP               KEY_VOLUMEUP
#define     KP_SC_VOL_DN               KEY_VOLUMEDOWN

/* New navigation-station chord scan codes */

#define     KP_SC_N_UP_LEFT              KEY_F1
#define     KP_SC_N_UP_RIGHT             KEY_F2
#define     KP_SC_N_DOWN_LEFT            KEY_F3
#define     KP_SC_N_DOWN_RIGHT           KEY_F4
#define     KP_SC_N_ACTION_UP            KEY_F5
#define     KP_SC_N_ACTION_DOWN          KEY_F6
#define     KP_SC_N_ACTION_LEFT          KEY_F7
#define     KP_SC_N_ACTION_RIGHT         KEY_F8
#define     KP_SC_N_ACTION_UP_LEFT       KEY_F9
#define     KP_SC_N_ACTION_UP_RIGHT      KEY_F10
#define     KP_SC_N_ACTION_DOWN_LEFT     KEY_F11
#define     KP_SC_N_ACTION_DOWN_RIGHT    KEY_F12

/*
 * For the sake of speed and simplicity, this table is configured so that the
 * single-closure matrix scan codes are placed using an MK_INn (row) by
 * MK_OUTm (column) format.  That will permit direct use of the KPAS
 * register's lower 8 bits, which yield that indexing.
 * Multiclosure (chorded) scan codes are processed in-line by the multipress parser
 */

/*
 Note: Zylonite-specific
 Unassigned cells filled with 0xFE (KP_SC_INVALID_KEY)
 */
#define KP_MAX_SINGLE_SCAN_CODES 64

/*
 * we treat
 *   key CONTACT as KP_SC_AUX1;
 *   key CALENDAR as KP_SC_AUX2;
 *   key EMAIL as KP_SC_AUX3
 */
static int pxakbd_keycode[KP_MAX_SINGLE_SCAN_CODES] = {
	/*IN0 */
/*OUT0 */ KP_SC_A,
/*OUT1 */ KP_SC_B,
/*OUT2 */ KP_SC_C,
/*OUT3 */ KP_SC_AUX1,
/*OUT4 */ KP_SC_AUX3,
/*OUT5 */ KP_SC_D,
/*OUT6 */ KP_SC_SOFT1,
/*OUT7 */ KP_SC_N_UP,

	/*IN1 */
/*OUT0 */ KP_SC_E,
/*OUT1 */ KP_SC_F,
/*OUT2 */ KP_SC_G,
/*OUT3 */ KP_SC_1,
/*OUT4 */ KP_SC_3,
/*OUT5 */ KP_SC_H,
/*OUT6 */ KP_SC_SOFT2,
/*OUT7*/ KP_SC_N_DOWN,

	/*IN2 */
/*OUT0 */ KP_SC_I,
/*OUT1 */ KP_SC_J,
/*OUT2 */ KP_SC_K,
/*OUT3 */ KP_SC_4,
/*OUT4 */ KP_SC_6,
/*OUT5 */ KP_SC_L,
/*OUT6 */ KP_SC_HOME,
/*OUT7 */ KP_SC_N_LEFT,

	/*IN3 */
/*OUT0 */ KP_SC_M,
/*OUT1 */ KP_SC_N,
/*OUT2 */ KP_SC_O,
/*OUT3 */ KP_SC_7,
/*OUT4 */ KP_SC_9,
/*OUT5 */ KP_SC_P,
/*OUT6 */ KP_SC_END,
/*OUT7 */ KP_SC_N_RIGHT,

	/*IN4 */
/*OUT0 */ KP_SC_AUX2,
/*OUT1 */ KP_SC_2,
/*OUT2 */ KP_SC_5,
/*OUT3 */ KP_SC_8,
/*OUT4*/ KP_SC_0,
/*OUT5 */ KP_SC_SPACE,
/*OUT6 */ KP_SC_SCROLL_PUSH,
/*OUT7 */ 0xFE,

	/*IN5 */
/*OUT0 */ KP_SC_Q,
/*OUT1 */ KP_SC_R,
/*OUT2 */ KP_SC_S,
/*OUT3 */ KP_SC_STAR,
/*OUT4 */ KP_SC_POUND,
/*OUT5 */ KP_SC_T,
/*OUT6 */ KP_SC_SEND,
/*OUT7 */ KP_SC_N_ACTION,

	/*IN6 */
/*OUT0 */ KP_SC_U,
/*OUT1 */ KP_SC_V,
/*OUT2 */ KP_SC_W,
/*OUT3 */ KP_SC_SHIFT,
/*OUT4 */ KP_SC_DELETE,
/*OUT5 */ KP_SC_X,
/*OUT6 */ KP_SC_BACK,
/*OUT7 */ KP_SC_VOL_UP,

	/*IN7 */
/*OUT0 */ 0xFE,
/*OUT1 */ KP_SC_Y,
/*OUT2 */ KP_SC_Z,
/*OUT3 */ 0xFE,
/*OUT4 */ 0xFE,
/*OUT5 */ 0xFE,
/*OUT6 */ KP_SC_RECORD,
/*OUT7 */ KP_SC_VOL_DN,
};				/*pxakbd_keycodeLower */

#define     KEYPAD_COLS_NUM      8
#define     KEYPAD_ROWS_NUM     8

#endif
