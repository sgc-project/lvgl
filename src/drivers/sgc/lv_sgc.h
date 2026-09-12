/**
 * @file lv_sgc.h
 * Display driver for the simple-graphics-controller daemon (@sgc).
 */

#ifndef LV_SGC_H
#define LV_SGC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../display/lv_display.h"

#if LV_USE_SGC

/**********************
 *      TYPEDEFS
 **********************/

/** State of the @sgc session backing the display. Informational: the driver
 *  handles revokes, re-grants and reconnects by itself. */
typedef enum {
    LV_SGC_STATE_ACTIVE,    /**< Holding the lease, the display is running */
    LV_SGC_STATE_SUSPENDED, /**< Parked: revoked, or waiting for the daemon to come back */
} lv_sgc_state_t;

/**
 * Called whenever the session changes state. Optional - an application does not
 * have to react: the display and its screens survive a suspension, and the
 * driver re-attaches them on its own once the controller grants the card
 * again.
 * @param state     the new state
 * @param user_data the pointer passed to lv_sgc_set_state_cb()
 */
typedef void (*lv_sgc_state_cb_t)(lv_sgc_state_t state, void * user_data);

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a display on a DRM lease acquired from the @sgc daemon.
 *
 * Connects to the daemon's socket, acquires the first advertised DRM card
 * (blocking until the controller answers) and drives the LVGL DRM display on
 * the granted lease fd; with LV_SGC_INPUT the advertised input devices come
 * along as well. There is no fallback: without the daemon there is no display,
 * and nothing is opened directly.
 *
 * The driver owns the session from here on: the lease is revocable, and when
 * the controller takes the card back the display is parked (refreshing stops,
 * the device is released) while the application's screens stay alive; the
 * driver re-attaches them to the lease it is granted next. A daemon that goes
 * away is handled the same way - the driver reconnects and resumes the display
 * by itself.
 *
 * @return pointer to the created display, or NULL on failure
 */
lv_display_t * lv_sgc_create(void);

/**
 * Register a callback for session state changes. Optional, for status displays
 * and diagnostics; the driver does not need the application to do anything.
 * @param cb        the callback, or NULL to unregister
 * @param user_data pointer passed to the callback
 */
void lv_sgc_set_state_cb(lv_sgc_state_cb_t cb, void * user_data);

/**
 * Get the current session state.
 * @return the state of the @sgc session
 */
lv_sgc_state_t lv_sgc_get_state(void);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_SGC*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_SGC_H*/
