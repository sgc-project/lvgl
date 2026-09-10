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

/** State of the @sgc session backing the display. */
typedef enum {
    LV_SGC_STATE_ACTIVE,    /**< We hold the lease and a display is up */
    LV_SGC_STATE_SUSPENDED, /**< The controller revoked the lease; waiting to be re-granted */
    LV_SGC_STATE_LOST,      /**< The connection to the controller is gone: the session is over */
} lv_sgc_state_t;

/**
 * Called whenever the session changes state. A display (and, with it, every
 * screen of that display) only exists while the state is LV_SGC_STATE_ACTIVE,
 * so an application must rebuild its screens when it is called with
 * LV_SGC_STATE_ACTIVE again after a suspension.
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
 * the granted lease fd. There is no fallback: without the daemon there is no
 * display, and nothing is opened directly.
 *
 * The lease is revocable. When the controller takes the card back the display
 * is deleted and the session waits for the re-grant; a re-grant builds a fresh
 * display on the new lease fd. Register lv_sgc_set_state_cb() to follow that
 * cycle.
 *
 * @return pointer to the created display, or NULL on failure
 */
lv_display_t * lv_sgc_create(void);

/**
 * Register a callback for session state changes (main thread, from the LVGL
 * timer handler).
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
