/**
 * @file lv_sgc.c
 * Display driver for the simple-graphics-controller daemon (@sgc).
 *
 * The controller owns the graphics devices; this driver takes a DRM card lease
 * (and, with LV_SGC_INPUT, the input devices) from it instead of opening the
 * devices itself, and drives the LVGL DRM display on the lease fd. The whole
 * driver only exists when LV_USE_SGC is enabled in lv_conf.h - with it off this
 * file compiles to nothing and the direct DRM/GBM drivers are untouched.
 */

#include "lv_sgc.h"

#if LV_USE_SGC

#include <string.h>
#include <unistd.h>

#include "libsgc.h"
#include "../../stdlib/lv_sprintf.h"
#include "../display/drm/lv_linux_drm.h"

/*********************
 *      DEFINES
 *********************/

#define SGC_ERR_LEN 256

/**********************
 *      TYPEDEFS
 **********************/

typedef struct {
    sgc_client * client;         /**< Session handle; NULL before connect and after release */
    sgc_resource drm;            /**< The DRM card resource this display renders on */
    lv_display_t * disp;         /**< Display on the lease; NULL while suspended */
    lv_timer_t * pump;           /**< Pumps the session for revoke/re-grant events */
    lv_sgc_state_cb_t state_cb;  /**< Application callback for state changes */
    void * state_cb_data;
    lv_sgc_state_t state;
} lv_sgc_ctx_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void pump_cb(lv_timer_t * t);
static void disp_delete_cb(lv_event_t * e);
static const char * kind_name(int kind);
static void set_state(lv_sgc_state_t state);
static lv_display_t * build_display(int lease_fd);
static void suspend_display(void);
static void end_session(const char * reason);

/**********************
 *  STATIC VARIABLES
 **********************/

static lv_sgc_ctx_t ctx;

/** Set while this driver deletes the display itself, so that the display's
 *  delete event is not mistaken for the application tearing the display down. */
static bool deleting_display;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_display_t * lv_sgc_create(void)
{
    char err[SGC_ERR_LEN] = {0};
    sgc_resource * advertised = NULL;
    size_t advertised_count = 0;
    bool have_drm = false;

    /* Reset the session fields but keep the state callback the application may
     * have registered with lv_sgc_set_state_cb() before creating the display. */
    ctx.client = NULL;
    ctx.disp = NULL;
    ctx.pump = NULL;
    ctx.state = LV_SGC_STATE_LOST;
    ctx.drm.kind = 0;
    ctx.drm.index = 0;

    ctx.client = sgc_connect(err, sizeof(err));
    if(ctx.client == NULL) {
        LV_LOG_ERROR("sgc: cannot reach the @sgc daemon: %s", err);
        return NULL;
    }
    LV_LOG_INFO("sgc: connected to @sgc");

    if(sgc_advertised(ctx.client, &advertised, &advertised_count) != 0) {
        LV_LOG_ERROR("sgc: the daemon did not list its resources");
        goto fail;
    }

    for(size_t i = 0; i < advertised_count; i++) {
        LV_LOG_INFO("sgc: advertised %s{%d}", kind_name(advertised[i].kind), advertised[i].index);
        if(!have_drm && advertised[i].kind == SGC_RESOURCE_DRM) {
            /* Like the other clients: render on the first advertised card. */
            ctx.drm = advertised[i];
            have_drm = true;
        }
    }
    sgc_free(advertised);
    advertised = NULL;

    if(!have_drm) {
        LV_LOG_ERROR("sgc: the daemon advertises no DRM card to render on");
        goto fail;
    }

    LV_LOG_INFO("sgc: acquiring %s{%d}...", kind_name(ctx.drm.kind), ctx.drm.index);
    if(sgc_acquire(ctx.client, ctx.drm, err, sizeof(err)) != 0) {
        LV_LOG_ERROR("sgc: %s{%d} was not granted: %s", kind_name(ctx.drm.kind), ctx.drm.index, err);
        goto fail;
    }
    LV_LOG_INFO("sgc: %s{%d} lease granted", kind_name(ctx.drm.kind), ctx.drm.index);

    int lease_fd = sgc_fd(ctx.client, ctx.drm, err, sizeof(err));
    if(lease_fd < 0) {
        LV_LOG_ERROR("sgc: cannot borrow the lease fd: %s", err);
        goto fail;
    }

    if(build_display(lease_fd) == NULL) {
        close(lease_fd);
        goto fail;
    }

    ctx.pump = lv_timer_create(pump_cb, LV_SGC_PUMP_PERIOD, NULL);
    set_state(LV_SGC_STATE_ACTIVE);
    return ctx.disp;

fail:
    if(advertised) sgc_free(advertised);
    if(ctx.client) {
        sgc_release(ctx.client);
        ctx.client = NULL;
    }
    return NULL;
}

void lv_sgc_set_state_cb(lv_sgc_state_cb_t cb, void * user_data)
{
    ctx.state_cb = cb;
    ctx.state_cb_data = user_data;
}

lv_sgc_state_t lv_sgc_get_state(void)
{
    return ctx.state;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Build a display on a lease fd. Takes ownership of the fd on success; on
 * failure the caller still owns it.
 */
static lv_display_t * build_display(int lease_fd)
{
    lv_display_t * disp = lv_linux_drm_create();
    if(disp == NULL) {
        LV_LOG_ERROR("sgc: cannot create the DRM display");
        return NULL;
    }

    if(lv_linux_drm_set_fd(disp, lease_fd, -1) != LV_RESULT_OK) {
        LV_LOG_ERROR("sgc: cannot modeset on the lease");
        deleting_display = true;
        lv_display_delete(disp);
        deleting_display = false;
        return NULL;
    }

    lv_display_add_event_cb(disp, disp_delete_cb, LV_EVENT_DELETE, NULL);
    ctx.disp = disp;

    LV_LOG_INFO("sgc: display %" LV_PRId32 "x%" LV_PRId32 " is up on the lease",
                lv_display_get_horizontal_resolution(disp), lv_display_get_vertical_resolution(disp));
    return disp;
}

/** The controller took the card back: the lease (and the display with it) is
 *  gone. The session stays connected - the daemon re-grants the card to this
 *  queued session once it is free again. */
static void suspend_display(void)
{
    if(ctx.disp) {
        deleting_display = true;
        lv_display_delete(ctx.disp);
        deleting_display = false;
        ctx.disp = NULL;
    }
    set_state(LV_SGC_STATE_SUSPENDED);
}

/** The connection is gone: nothing can be rendered anymore. */
static void end_session(const char * reason)
{
    if(reason) LV_LOG_ERROR("sgc: %s", reason);

    if(ctx.pump) {
        lv_timer_delete(ctx.pump);
        ctx.pump = NULL;
    }

    if(ctx.disp) {
        deleting_display = true;
        lv_display_delete(ctx.disp);
        deleting_display = false;
        ctx.disp = NULL;
    }

    if(ctx.client) {
        sgc_release(ctx.client);
        ctx.client = NULL;
    }

    set_state(LV_SGC_STATE_LOST);
}

static void pump_cb(lv_timer_t * t)
{
    LV_UNUSED(t);

    sgc_event ev;
    char err[SGC_ERR_LEN] = {0};

    /* Poll once per LVGL timer period; events that arrived meanwhile are
     * buffered in the session, so nothing is missed. */
    int ret = sgc_pump(ctx.client, 0, &ev, err, sizeof(err));
    if(ret == 0) return;

    if(ret < 0) {
        end_session(err);
        return;
    }

    LV_LOG_INFO("sgc: %s %s{%d}", ev.kind == SGC_EVENT_GRANTED ? "granted" : "revoked",
                kind_name(ev.resource.kind), ev.resource.index);

    if(ev.resource.kind != SGC_RESOURCE_DRM) {
        if(ev.fd >= 0) close(ev.fd); /* input devices are not consumed yet */
        return;
    }

    if(ev.kind == SGC_EVENT_REVOKED) {
        suspend_display();
    }
    else if(ev.kind == SGC_EVENT_GRANTED) {
        if(ev.fd < 0) {
            LV_LOG_ERROR("sgc: re-grant without a lease fd");
            return;
        }
        if(build_display(ev.fd) == NULL) {
            close(ev.fd);
            return;
        }
        set_state(LV_SGC_STATE_ACTIVE);
    }
}

/** The application deleted the display: the session ends with it. */
static void disp_delete_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) != LV_EVENT_DELETE) return;
    if(deleting_display) return;

    ctx.disp = NULL;

    if(ctx.pump) {
        lv_timer_delete(ctx.pump);
        ctx.pump = NULL;
    }
    if(ctx.client) {
        sgc_release(ctx.client);
        ctx.client = NULL;
    }
    ctx.state = LV_SGC_STATE_LOST; /* no callback: the application is tearing down */
}

static void set_state(lv_sgc_state_t state)
{
    ctx.state = state;
    if(ctx.state_cb) ctx.state_cb(state, ctx.state_cb_data);
}

static const char * kind_name(int kind)
{
    switch(kind) {
        case SGC_RESOURCE_FBDEV:    return "Fbdev";
        case SGC_RESOURCE_DRM:      return "Drm";
        case SGC_RESOURCE_MOUSE:    return "Mouse";
        case SGC_RESOURCE_KEYBOARD: return "Keyboard";
        case SGC_RESOURCE_TOUCH:    return "Touch";
        default:                    return "?";
    }
}

#endif /*LV_USE_SGC*/
