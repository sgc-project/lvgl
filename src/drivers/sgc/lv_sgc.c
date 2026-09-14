/**
 * @file lv_sgc.c
 * Display driver for the simple-graphics-controller daemon (@sgc).
 *
 * The controller owns the graphics devices; this driver takes a DRM card lease
 * (and, with LV_SGC_INPUT, the input devices) from it instead of opening the
 * devices itself, and owns the whole session lifecycle: connecting, acquiring,
 * the revocable lease and reconnecting when the daemon goes away. The
 * application only ever gets a display - the lease, the park/resume cycle and
 * recovery all happen here.
 *
 * The display (and every screen on it) is created once and lives for as long as
 * the session does: a revoke parks it - refreshing stops, the device is
 * released, the application's UI stays alive - and a re-grant attaches it to
 * the fresh lease. Reconnecting after the daemon disappeared works the same
 * way, so an application never has to deal with any of this.
 *
 * Input devices (LV_SGC_INPUT) ride with the seat: the daemon revokes them when
 * this client leaves the display, so a re-grant asks for them again, and a
 * device that merely went away is suspended - the daemon re-grants that one on
 * its own, and the LVGL input device reading the dead fd is replaced when it
 * does.
 *
 * The whole driver exists only when LV_USE_SGC is enabled in lv_conf.h: with
 * the option off this file compiles to nothing and the direct DRM/GBM drivers
 * are untouched.
 */

#include "lv_sgc.h"

#if LV_USE_SGC

#include <string.h>
#include <unistd.h>


#include "libsgc.h"
#include "../../stdlib/lv_sprintf.h"
#include "../display/drm/lv_linux_drm.h"

#if LV_SGC_INPUT
    #if !LV_USE_EVDEV
        #error "LV_SGC_INPUT needs LV_USE_EVDEV"
    #endif
    #include "../evdev/lv_evdev.h"
#endif

/*********************
 *      DEFINES
 *********************/

#define SGC_ERR_LEN 256
#define SGC_MAX_INPUTS 8
/** Retry period [ms] for connecting to the daemon and acquiring the lease. */
#define SGC_RETRY_PERIOD 1000

/**********************
 *      TYPEDEFS
 **********************/

/** An input device taken from the daemon and fed to LVGL. */
typedef struct {
    sgc_resource resource; /**< Kind + index as advertised by the daemon; kind < 0 when free */
    lv_indev_t * indev;    /**< LVGL input device reading the granted fd, NULL when not held */
} lv_sgc_input_t;

typedef struct {
    sgc_client * client;         /**< Session handle; NULL while disconnected */
    sgc_resource drm;            /**< The DRM card resource this display renders on */
    lv_display_t * disp;         /**< The display; created once, parked across revokes */
    lv_timer_t * pump;           /**< Pumps the session, and retries connecting */
    lv_sgc_state_cb_t state_cb;  /**< Optional application notification */
    void * state_cb_data;
    lv_sgc_state_t state;
    lv_sgc_input_t inputs[SGC_MAX_INPUTS]; /**< Input devices held from the daemon */
    sgc_resource wanted[SGC_MAX_INPUTS]; /**< Input devices this app wants: the ones the
                                          *   daemon advertised. What a seat handover
                                          *   re-acquires (see inputs_reacquire) */
    size_t wanted_count;         /**< Entries in `wanted` */
    uint32_t retry_at;           /**< Tick of the next connect attempt while disconnected */
} lv_sgc_ctx_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void pump_cb(lv_timer_t * t);
static void disp_delete_cb(lv_event_t * e);
static const char * kind_name(int kind);
static void set_state(lv_sgc_state_t state);
static lv_result_t attach_display(int lease_fd);
static void park_display(void);
static void release_controller(void);
static bool connect_and_acquire(void);
#if LV_SGC_INPUT
    static bool is_input_kind(int kind);
    static void input_attach(sgc_resource r, int fd);
    static void input_release(sgc_resource r);
    static void inputs_acquire(sgc_resource * advertised, size_t count);
    static void inputs_remember(sgc_resource * advertised, size_t count);
    static void inputs_reacquire(void);
    static void inputs_release_all(void);
    static void inputs_attach_to_display(lv_display_t * disp);
#endif

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
    ctx.client = NULL;
    ctx.disp = NULL;
    ctx.pump = NULL;
    ctx.state = LV_SGC_STATE_SUSPENDED;
    ctx.drm.kind = 0;
    ctx.drm.index = 0;
    ctx.retry_at = 0;
    ctx.wanted_count = 0;

    for(int i = 0; i < SGC_MAX_INPUTS; i++) {
        ctx.inputs[i].resource.kind = -1;
        ctx.inputs[i].resource.index = -1;
        ctx.inputs[i].indev = NULL;
    }

    /* Bring the session up once, synchronously: an application that gets a
     * display back expects it to be showing. The pump timer keeps it alive and
     * recovers it from then on. */
    if(!connect_and_acquire()) {
        release_controller();
        return NULL;
    }

    ctx.pump = lv_timer_create(pump_cb, LV_SGC_PUMP_PERIOD, NULL);
    return ctx.disp;
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
 * Connect to the daemon, acquire the advertised DRM card and attach the display
 * to the lease; then take the advertised input devices (best effort). Returns
 * false when there is nothing to render on (no daemon, no card, denied).
 */
static bool connect_and_acquire(void)
{
    char err[SGC_ERR_LEN] = {0};
    sgc_resource * advertised = NULL;
    size_t advertised_count = 0;
    bool have_drm = false;
    int lease_fd;

    ctx.client = sgc_connect(err, sizeof(err));
    if(ctx.client == NULL) {
        LV_LOG_WARN("sgc: @sgc is not reachable: %s", err);
        return false;
    }

    if(sgc_advertised(ctx.client, &advertised, &advertised_count) != 0) {
        LV_LOG_WARN("sgc: the daemon did not list its resources");
        goto fail;
    }

    for(size_t i = 0; i < advertised_count; i++) {
        if(!have_drm && advertised[i].kind == SGC_RESOURCE_DRM) {
            /* Like the other clients: render on the first advertised card. */
            ctx.drm = advertised[i];
            have_drm = true;
        }
    }
    if(!have_drm) {
        LV_LOG_WARN("sgc: the daemon advertises no DRM card to render on");
        goto fail;
    }

    LV_LOG_INFO("sgc: connected to @sgc, acquiring %s{%d}", kind_name(ctx.drm.kind), ctx.drm.index);
    if(sgc_acquire(ctx.client, ctx.drm, err, sizeof(err)) != 0) {
        LV_LOG_WARN("sgc: %s{%d} was not granted: %s", kind_name(ctx.drm.kind), ctx.drm.index, err);
        goto fail;
    }
    LV_LOG_INFO("sgc: %s{%d} lease granted", kind_name(ctx.drm.kind), ctx.drm.index);

    lease_fd = sgc_fd(ctx.client, ctx.drm, err, sizeof(err));
    if(lease_fd < 0) {
        LV_LOG_WARN("sgc: cannot borrow the lease fd: %s", err);
        goto fail;
    }

    if(attach_display(lease_fd) != LV_RESULT_OK) {
        close(lease_fd); /* not taken over on failure */
        goto fail;
    }

#if LV_SGC_INPUT
    /* Remember what the daemon offers before the list is freed: a seat handover
     * has to ask for these again (inputs_reacquire). */
    inputs_remember(advertised, advertised_count);
    inputs_acquire(ctx.wanted, ctx.wanted_count);
#endif

    sgc_free(advertised);
    set_state(LV_SGC_STATE_ACTIVE);
    return true;

fail:
    if(advertised) sgc_free(advertised);
    return false;
}

/**
 * Attach the display to a lease fd, creating the display on the first call.
 * Takes ownership of the fd on success; on failure the caller keeps it.
 */
static lv_result_t attach_display(int lease_fd)
{
    bool created = false;

    if(ctx.disp == NULL) {
        ctx.disp = lv_linux_drm_create();
        if(ctx.disp == NULL) {
            LV_LOG_ERROR("sgc: cannot create the DRM display");
            return LV_RESULT_INVALID;
        }
        created = true;
    }

    if(lv_linux_drm_set_fd(ctx.disp, lease_fd, -1) != LV_RESULT_OK) {
        LV_LOG_ERROR("sgc: cannot modeset on the lease");
        if(created) {
            deleting_display = true;
            lv_display_delete(ctx.disp);
            deleting_display = false;
            ctx.disp = NULL;
        }
        return LV_RESULT_INVALID;
    }

    if(created) {
        /* The display outlives revokes, so it is only deleted with the session. */
        lv_display_add_event_cb(ctx.disp, disp_delete_cb, LV_EVENT_DELETE, NULL);
    }

#if LV_SGC_INPUT
    inputs_attach_to_display(ctx.disp);
#endif

    LV_LOG_INFO("sgc: display %" LV_PRId32 "x%" LV_PRId32 " running on the lease",
                lv_display_get_horizontal_resolution(ctx.disp), lv_display_get_vertical_resolution(ctx.disp));
    return LV_RESULT_OK;
}

/** Park the display: give the device back, stop drawing, keep the UI alive. */
static void park_display(void)
{
    if(ctx.disp) {
        lv_linux_drm_detach(ctx.disp);
    }
}

/** Drop everything that belongs to the controller connection. */
static void release_controller(void)
{
#if LV_SGC_INPUT
    inputs_release_all();
#endif
    if(ctx.client) {
        sgc_release(ctx.client);
        ctx.client = NULL;
    }
}

static void pump_cb(lv_timer_t * t)
{
    LV_UNUSED(t);

    if(ctx.client == NULL) {
        /* Disconnected: the daemon may be back (or may have restarted). */
        if(lv_tick_elaps(ctx.retry_at) < SGC_RETRY_PERIOD) return;
        ctx.retry_at = lv_tick_get();

        LV_LOG_INFO("sgc: trying to reconnect to @sgc");
        if(connect_and_acquire()) {
            LV_LOG_INFO("sgc: session recovered");
        }
        else {
            release_controller();
        }
        return;
    }

    sgc_event ev;
    char err[SGC_ERR_LEN] = {0};

    /* Poll once per LVGL timer period; events that arrived meanwhile are
     * buffered in the session, so nothing is missed. */
    int ret = sgc_pump(ctx.client, 0, &ev, err, sizeof(err));
    if(ret == 0) return;

    if(ret < 0) {
        /* The daemon is gone; the lease died with it. Park and retry. */
        LV_LOG_WARN("sgc: session with @sgc is over: %s", err);
        park_display();
        release_controller();
        ctx.retry_at = lv_tick_get();
        set_state(LV_SGC_STATE_SUSPENDED);
        return;
    }

    LV_LOG_INFO("sgc: %s %s{%d}", ev.kind == SGC_EVENT_GRANTED ? "granted" : "revoked",
                kind_name(ev.resource.kind), ev.resource.index);

    if(ev.resource.kind != SGC_RESOURCE_DRM) {
#if LV_SGC_INPUT
        if(!is_input_kind(ev.resource.kind)) {
            if(ev.fd >= 0) close(ev.fd);
            return;
        }

        if(ev.kind == SGC_EVENT_REVOKED) {
            input_release(ev.resource);
        }
        else if(ev.fd >= 0) {
            input_attach(ev.resource, ev.fd); /* takes ownership of the fd */
        }
#else
        /* Without LV_SGC_INPUT no input device is acquired, so nothing consumes
         * a granted fd: hand it straight back. */
        if(ev.fd >= 0) close(ev.fd);
#endif
        return;
    }

    if(ev.kind == SGC_EVENT_REVOKED) {
        /* The controller lent the card to someone else. The application keeps
         * its UI; we wait for the re-grant and attach the display again. */
        park_display();
        set_state(LV_SGC_STATE_SUSPENDED);
    }
    else if(ev.kind == SGC_EVENT_GRANTED) {
        if(ev.fd < 0) {
            LV_LOG_ERROR("sgc: re-grant without a lease fd");
            return;
        }
        if(attach_display(ev.fd) != LV_RESULT_OK) {
            close(ev.fd);
            return;
        }
        set_state(LV_SGC_STATE_ACTIVE);

#if LV_SGC_INPUT
        /* Back on screen: the lease was revoked with the input devices that
         * went with it (the daemon revokes a seat's devices when it leaves the
         * seat and never re-grants them with the display), so ask again -
         * otherwise the app is visible with no pointer and no keyboard. */
        inputs_reacquire();
#endif
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

    release_controller();
    ctx.state = LV_SGC_STATE_SUSPENDED; /* no callback: the application is tearing down */
}

static void set_state(lv_sgc_state_t state)
{
    if(ctx.state == state) return;

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

#if LV_SGC_INPUT
static bool is_input_kind(int kind)
{
    return kind == SGC_RESOURCE_MOUSE || kind == SGC_RESOURCE_KEYBOARD || kind == SGC_RESOURCE_TOUCH;
}

static lv_indev_type_t indev_type_of(int kind)
{
    /* A mouse or a touch screen moves a pointer, a keyboard is a keypad. */
    return kind == SGC_RESOURCE_KEYBOARD ? LV_INDEV_TYPE_KEYPAD : LV_INDEV_TYPE_POINTER;
}

static lv_sgc_input_t * input_find(sgc_resource r)
{
    for(int i = 0; i < SGC_MAX_INPUTS; i++) {
        if(ctx.inputs[i].resource.kind == r.kind && ctx.inputs[i].resource.index == r.index) {
            return &ctx.inputs[i];
        }
    }
    return NULL;
}

static lv_sgc_input_t * input_slot(sgc_resource r)
{
    lv_sgc_input_t * slot = input_find(r);
    if(slot) return slot;

    for(int i = 0; i < SGC_MAX_INPUTS; i++) {
        if(ctx.inputs[i].resource.kind < 0) {
            ctx.inputs[i].resource = r;
            ctx.inputs[i].indev = NULL;
            return &ctx.inputs[i];
        }
    }
    return NULL;
}

/** Hand a granted input fd to LVGL (takes ownership of the fd). */
static void input_attach(sgc_resource r, int fd)
{
    lv_sgc_input_t * slot = input_slot(r);
    if(slot == NULL) {
        LV_LOG_WARN("sgc: no slot left for %s{%d}", kind_name(r.kind), r.index);
        close(fd);
        return;
    }

    /* A grant for a device this app already holds is the device coming BACK:
     * it was suspended rather than revoked (no revoke arrives, and the daemon
     * re-grants the same resource with a fresh fd). The LVGL device that is
     * still there reads the fd that died with the old one, so it goes. */
    if(slot->indev) {
        LV_LOG_INFO("sgc: %s{%d} came back - replacing the input device that lost its device",
                    kind_name(r.kind), r.index);
        lv_evdev_delete(slot->indev);
        slot->indev = NULL;
    }

    lv_indev_t * indev = lv_evdev_create_fd(indev_type_of(r.kind), fd);
    if(indev == NULL) {
        LV_LOG_WARN("sgc: cannot create an input device for %s{%d}", kind_name(r.kind), r.index);
        slot->resource.kind = -1;
        slot->resource.index = -1;
        return;
    }

    if(ctx.disp) lv_indev_set_display(indev, ctx.disp);
    slot->indev = indev;
    LV_LOG_INFO("sgc: %s{%d} attached as an LVGL input device", kind_name(r.kind), r.index);
}

/** Stop reading an input device; its fd is closed by the evdev driver. */
static void input_release(sgc_resource r)
{
    lv_sgc_input_t * slot = input_find(r);
    if(slot == NULL) return;

    if(slot->indev) {
        lv_evdev_delete(slot->indev);
        slot->indev = NULL;
        LV_LOG_INFO("sgc: %s{%d} released", kind_name(r.kind), r.index);
    }
    slot->resource.kind = -1;
    slot->resource.index = -1;
}

/**
 * Acquire every advertised input device. Best effort: another client may hold
 * one, and a UI without it is fine - an input device must never keep the
 * display from coming up.
 */
static void inputs_acquire(sgc_resource * advertised, size_t count)
{
    char err[SGC_ERR_LEN];

    for(size_t i = 0; i < count; i++) {
        if(!is_input_kind(advertised[i].kind)) continue;
        /* Already ours: a device that went away is suspended, not revoked, so
         * the daemon brings it back on its own - asking again would only earn a
         * "not available while its device is away". */
        if(input_find(advertised[i]) != NULL) continue;

        lv_memzero(err, sizeof(err));
        if(sgc_acquire(ctx.client, advertised[i], err, sizeof(err)) != 0) {
            LV_LOG_WARN("sgc: %s{%d} was not granted - continuing without it: %s",
                        kind_name(advertised[i].kind), advertised[i].index, err);
            continue;
        }
        LV_LOG_INFO("sgc: %s{%d} granted", kind_name(advertised[i].kind), advertised[i].index);

        int fd = sgc_fd(ctx.client, advertised[i], err, sizeof(err));
        if(fd < 0) {
            LV_LOG_WARN("sgc: cannot borrow the %s{%d} fd: %s",
                        kind_name(advertised[i].kind), advertised[i].index, err);
            continue;
        }
        input_attach(advertised[i], fd);
    }
}

/**
 * Remember the input devices the daemon advertises: what this app wants. A seat
 * handover makes the daemon revoke them with the display, and the list is the
 * only record of what to ask for again.
 */
static void inputs_remember(sgc_resource * advertised, size_t count)
{
    ctx.wanted_count = 0;
    for(size_t i = 0; i < count && ctx.wanted_count < SGC_MAX_INPUTS; i++) {
        if(is_input_kind(advertised[i].kind)) {
            ctx.wanted[ctx.wanted_count++] = advertised[i];
        }
    }
}

/**
 * Ask for the wanted devices again after the display came back. Best effort
 * like every input acquire, and a no-op when there is nothing to ask for.
 */
static void inputs_reacquire(void)
{
    if(ctx.wanted_count == 0) return;

    LV_LOG_INFO("sgc: back on screen - re-acquiring the input devices");
    inputs_acquire(ctx.wanted, ctx.wanted_count);
}

static void inputs_release_all(void)
{
    for(int i = 0; i < SGC_MAX_INPUTS; i++) {
        if(ctx.inputs[i].resource.kind >= 0) {
            input_release(ctx.inputs[i].resource);
        }
    }
}

static void inputs_attach_to_display(lv_display_t * disp)
{
    for(int i = 0; i < SGC_MAX_INPUTS; i++) {
        if(ctx.inputs[i].indev) lv_indev_set_display(ctx.inputs[i].indev, disp);
    }
}
#endif /*LV_SGC_INPUT*/

#endif /*LV_USE_SGC*/
