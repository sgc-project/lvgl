/**
 * @file lv_linux_drm.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_linux_drm.h"
#if LV_USE_LINUX_DRM && !LV_LINUX_DRM_USE_EGL

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "../../../stdlib/lv_sprintf.h"
#include "../../../draw/lv_draw_buf.h"
#include "../../../core/lv_obj.h"

#if LV_USE_LINUX_DRM_GBM_BUFFERS
    #include <gbm.h>
    #include <linux/dma-buf.h>
    #include <sys/ioctl.h>
#endif

/*********************
 *      DEFINES
 *********************/
#if LV_COLOR_DEPTH == 32
    #define DRM_FOURCC DRM_FORMAT_XRGB8888
#elif LV_COLOR_DEPTH == 16
    #define DRM_FOURCC DRM_FORMAT_RGB565
#else
    #error LV_COLOR_DEPTH not supported
#endif

#define BUFFER_CNT 2

/** Bound for waiting on a page flip event. A device that is gone - a revoked
 *  DRM lease, a removed card - never delivers the event. */
#define DRM_FLIP_TIMEOUT_MS 1000

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
    uint32_t handle;
    uint32_t pitch;
    uint32_t offset;
    unsigned long int size;
    uint8_t * map;
    uint32_t fb_handle;
#if LV_USE_LINUX_DRM_GBM_BUFFERS
    struct gbm_bo * gbm_bo;   /* buffer object the mapping belongs to */
    void * gbm_map_data;      /* opaque handle required by gbm_bo_unmap() */
#endif
} drm_buffer_t;

typedef struct {
    int fd;
    uint32_t conn_id, enc_id, crtc_id, plane_id, crtc_idx;
    uint32_t width, height;
    uint32_t mmWidth, mmHeight;
    uint32_t fourcc;
    drmModeModeInfo mode;
    uint32_t blob_id;
    drmModeCrtc * saved_crtc;
    drmModeAtomicReq * req;
    bool needs_modeset; /**< The next atomic commit has to set a mode (first commit after attaching) */
    drmEventContext drm_event_ctx;
    drmModePlane * plane;
    drmModeCrtc * crtc;
    drmModeConnector * conn;
    uint32_t count_plane_props;
    uint32_t count_crtc_props;
    uint32_t count_conn_props;
    drmModePropertyPtr plane_props[128];
    drmModePropertyPtr crtc_props[128];
    drmModePropertyPtr conn_props[128];
    drm_buffer_t drm_bufs[BUFFER_CNT];
    drm_buffer_t * act_buf;
#if LV_USE_LINUX_DRM_GBM_BUFFERS
    struct gbm_device * gbm_device;
#endif
} drm_dev_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static uint32_t get_plane_property_id(drm_dev_t * drm_dev, const char * name);
static uint32_t get_crtc_property_id(drm_dev_t * drm_dev, const char * name);
static uint32_t get_conn_property_id(drm_dev_t * drm_dev, const char * name);
static void page_flip_handler(int fd, unsigned int sequence, unsigned int tv_sec, unsigned int tv_usec,
                              void * user_data);
static int drm_get_plane_props(drm_dev_t * drm_dev);
static int drm_get_crtc_props(drm_dev_t * drm_dev);
static int drm_get_conn_props(drm_dev_t * drm_dev);
static int drm_add_plane_property(drm_dev_t * drm_dev, const char * name, uint64_t value);
static int drm_add_crtc_property(drm_dev_t * drm_dev, const char * name, uint64_t value);
static int drm_add_conn_property(drm_dev_t * drm_dev, const char * name, uint64_t value);
static int find_plane(drm_dev_t * drm_dev, unsigned int fourcc, uint32_t * plane_id, uint32_t crtc_id,
                      uint32_t crtc_idx);
static int drm_find_connector(drm_dev_t * drm_dev, int64_t connector_id);
static void drm_get_plane_type_zpos(int fd, uint32_t plane_id, uint64_t * type, uint64_t * zpos);
static int drm_open(const char * path);
static int drm_setup(drm_dev_t * drm_dev, int fd, int64_t connector_id, unsigned int fourcc);

static uint32_t tick_get_cb(void);

#if !LV_USE_LINUX_DRM_GBM_BUFFERS
    static int drm_allocate_dumb(drm_dev_t * drm_dev, drm_buffer_t * buf);
#elif LV_USE_LINUX_DRM_GBM_BUFFERS
    static int create_gbm_buffer(drm_dev_t * drm_dev, drm_buffer_t * buf);
#endif

static int drm_setup_buffers(drm_dev_t * drm_dev);
static int drm_dmabuf_set_plane(drm_dev_t * drm_dev, drm_buffer_t * buf);
static void drm_flush_wait(lv_display_t * drm_dev);
static void drm_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void drm_dmabuf_set_active_buf(lv_event_t * event);
static void drm_del_event_cb(lv_event_t * event);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/
#ifndef DIV_ROUND_UP
    #define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_display_t * lv_linux_drm_create(void)
{
    lv_display_t * disp;

    lv_tick_set_cb(tick_get_cb);

    drm_dev_t * drm_dev = lv_malloc_zeroed(sizeof(drm_dev_t));
    LV_ASSERT_MALLOC(drm_dev);
    if(drm_dev == NULL) return NULL;

    drm_dev->fd = -1;

    disp = lv_display_create(800, 480);
    if(disp == NULL) {
        lv_free(drm_dev);
        return NULL;
    }
    lv_display_set_driver_data(disp, drm_dev);
    lv_display_set_flush_wait_cb(disp, drm_flush_wait);
    lv_display_set_flush_cb(disp, drm_flush);
    lv_display_add_event_cb(disp, drm_del_event_cb, LV_EVENT_DELETE, NULL);

    return disp;
}

/* Called by LVGL when there is something that needs redrawing
 * it sets the active buffer. if GBM buffers are used, it issues a DMA_BUF_SYNC
 * ioctl call to lock the buffer for CPU access, the buffer is unlocked just
 * before the atomic commit */
static void drm_dmabuf_set_active_buf(lv_event_t * event)
{
    drm_dev_t * drm_dev;
    lv_display_t * disp;
    lv_draw_buf_t * act_buf;
    int i;

    disp = (lv_display_t *) lv_event_get_current_target(event);
    drm_dev = (drm_dev_t *) lv_display_get_driver_data(disp);
    act_buf = lv_display_get_buf_active(disp);

    if(drm_dev->act_buf == NULL) {

        for(i = 0; i < BUFFER_CNT; i++) {
            if(act_buf->unaligned_data == drm_dev->drm_bufs[i].map) {
                drm_dev->act_buf = &drm_dev->drm_bufs[i];
                LV_LOG_TRACE("Set active buffer idx: %d", i);
                break;
            }
        }

#if LV_USE_LINUX_DRM_GBM_BUFFERS
        struct dma_buf_sync sync_req;
        sync_req.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
        int res;

        if((res = ioctl(drm_dev->act_buf->handle, DMA_BUF_IOCTL_SYNC, &sync_req)) != 0) {
            LV_LOG_ERROR("Failed to start DMA-BUF R/W SYNC res: %d", res);
        }
#endif

    }
    else {

        LV_LOG_TRACE("active buffer already set");
    }

}

lv_result_t lv_linux_drm_set_file(lv_display_t * disp, const char * file, int64_t connector_id)
{
    int fd = drm_open(file);
    if(fd < 0) return LV_RESULT_INVALID;

    lv_result_t res = lv_linux_drm_set_fd(disp, fd, connector_id);
    if(res != LV_RESULT_OK) {
        close(fd);
    }
    return res;
}

lv_result_t lv_linux_drm_set_fd(lv_display_t * disp, int fd, int64_t connector_id)
{
    int ret;

    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);
    if(drm_dev == NULL || fd < 0) {
        return LV_RESULT_INVALID;
    }

    ret = drm_setup(drm_dev, fd, connector_id, DRM_FOURCC);
    if(ret) {
        return LV_RESULT_INVALID;
    }

    int32_t hor_res = drm_dev->width;
    int32_t ver_res = drm_dev->height;

    ret = drm_setup_buffers(drm_dev);
    if(ret) {
        LV_LOG_ERROR("DRM buffer allocation failed");
        /* The display did not take ownership of the fd. */
        drm_dev->fd = -1;
        return LV_RESULT_INVALID;
    }

    LV_LOG_INFO("DRM subsystem and buffer mapped successfully");

    int32_t width = drm_dev->mmWidth;

    size_t buf_size = LV_MIN(drm_dev->drm_bufs[1].size, drm_dev->drm_bufs[0].size);
    uint32_t stride = drm_dev->drm_bufs[0].pitch;
    /* Resolution must be set first because if the screen is smaller than the size passed
     * to lv_display_create then the buffers aren't big enough for LV_DISPLAY_RENDER_MODE_DIRECT.
     */
    lv_display_set_resolution(disp, hor_res, ver_res);
    lv_display_set_buffers_with_stride(disp, drm_dev->drm_bufs[1].map, drm_dev->drm_bufs[0].map, buf_size,
                                       stride, LV_DISPLAY_RENDER_MODE_DIRECT);


    /* Set the handler that is called before a redraw occurs to set the active buffer/plane
     * when GBM buffers are used the DMA_BUF_SYNC_START is issued there */
    lv_display_add_event_cb(disp, drm_dmabuf_set_active_buf, LV_EVENT_REFR_START, drm_dev);

    if(width) {
        lv_display_set_dpi(disp, DIV_ROUND_UP(hor_res * 25400, width * 1000));
    }

    /* The display is attached again: it gets its flush path and its refresh
     * back, and every buffer has to be drawn again. */
    lv_display_set_flush_cb(disp, drm_flush);
    lv_display_set_flush_wait_cb(disp, drm_flush_wait);
    lv_display_flush_ready(disp);
    lv_display_enable_invalidation(disp, true);
    lv_display_create_refr_timer(disp); /* no-op unless the display was parked */

    lv_timer_t * refr = lv_display_get_refr_timer(disp);
    if(refr) lv_timer_resume(refr);

    lv_obj_t * scr = lv_display_get_screen_active(disp);
    if(scr) lv_obj_invalidate(scr);

    LV_LOG_INFO("Resolution is set to %" LV_PRId32 "x%" LV_PRId32 " at %" LV_PRId32 "dpi",
                hor_res, ver_res, lv_display_get_dpi(disp));
    return LV_RESULT_OK;
}

void lv_linux_drm_set_mode_cb(lv_display_t * disp, lv_linux_drm_select_mode_cb_t callback)
{
    LV_UNUSED(disp);
    LV_UNUSED(callback);
    LV_LOG_WARN("DRM without EGL support doesn't currently support setting a mode selection callback");
}
/**********************
 *   STATIC FUNCTIONS
 **********************/

static uint32_t get_plane_property_id(drm_dev_t * drm_dev, const char * name)
{
    uint32_t i;

    LV_LOG_TRACE("Find plane property: %s", name);

    for(i = 0; i < drm_dev->count_plane_props; ++i)
        if(!lv_strcmp(drm_dev->plane_props[i]->name, name))
            return drm_dev->plane_props[i]->prop_id;

    LV_LOG_TRACE("Unknown plane property: %s", name);

    return 0;
}

static uint32_t get_crtc_property_id(drm_dev_t * drm_dev, const char * name)
{
    uint32_t i;

    LV_LOG_TRACE("Find crtc property: %s", name);

    for(i = 0; i < drm_dev->count_crtc_props; ++i)
        if(!lv_strcmp(drm_dev->crtc_props[i]->name, name))
            return drm_dev->crtc_props[i]->prop_id;

    LV_LOG_TRACE("Unknown crtc property: %s", name);

    return 0;
}

static uint32_t get_conn_property_id(drm_dev_t * drm_dev, const char * name)
{
    uint32_t i;

    LV_LOG_TRACE("Find conn property: %s", name);

    for(i = 0; i < drm_dev->count_conn_props; ++i)
        if(!lv_strcmp(drm_dev->conn_props[i]->name, name))
            return drm_dev->conn_props[i]->prop_id;

    LV_LOG_TRACE("Unknown conn property: %s", name);

    return 0;
}

static void page_flip_handler(int fd, unsigned int sequence, unsigned int tv_sec, unsigned int tv_usec,
                              void * user_data)
{
    LV_UNUSED(fd);
    LV_UNUSED(sequence);
    LV_UNUSED(tv_sec);
    LV_UNUSED(tv_usec);
    LV_LOG_TRACE("flip");
    drm_dev_t * drm_dev = user_data;
    if(drm_dev->req) {
        drmModeAtomicFree(drm_dev->req);
        drm_dev->req = NULL;
    }
}

static int drm_get_plane_props(drm_dev_t * drm_dev)
{
    uint32_t i;

    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(drm_dev->fd, drm_dev->plane_id,
                                                                  DRM_MODE_OBJECT_PLANE);
    if(!props) {
        LV_LOG_ERROR("drmModeObjectGetProperties failed");
        return -1;
    }
    LV_LOG_TRACE("Found %u plane props", props->count_props);
    drm_dev->count_plane_props = props->count_props;
    for(i = 0; i < props->count_props; i++) {
        drm_dev->plane_props[i] = drmModeGetProperty(drm_dev->fd, props->props[i]);
        LV_LOG_TRACE("Added plane prop %u:%s", drm_dev->plane_props[i]->prop_id, drm_dev->plane_props[i]->name);
    }
    drmModeFreeObjectProperties(props);

    return 0;
}

static int drm_get_crtc_props(drm_dev_t * drm_dev)
{
    uint32_t i;

    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(drm_dev->fd, drm_dev->crtc_id,
                                                                  DRM_MODE_OBJECT_CRTC);
    if(!props) {
        LV_LOG_ERROR("drmModeObjectGetProperties failed");
        return -1;
    }
    LV_LOG_TRACE("Found %u crtc props", props->count_props);
    drm_dev->count_crtc_props = props->count_props;
    for(i = 0; i < props->count_props; i++) {
        drm_dev->crtc_props[i] = drmModeGetProperty(drm_dev->fd, props->props[i]);
        LV_LOG_TRACE("Added crtc prop %u:%s", drm_dev->crtc_props[i]->prop_id, drm_dev->crtc_props[i]->name);
    }
    drmModeFreeObjectProperties(props);

    return 0;
}

static int drm_get_conn_props(drm_dev_t * drm_dev)
{
    uint32_t i;

    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(drm_dev->fd, drm_dev->conn_id,
                                                                  DRM_MODE_OBJECT_CONNECTOR);
    if(!props) {
        LV_LOG_ERROR("drmModeObjectGetProperties failed");
        return -1;
    }
    LV_LOG_TRACE("Found %u connector props", props->count_props);
    drm_dev->count_conn_props = props->count_props;
    for(i = 0; i < props->count_props; i++) {
        drm_dev->conn_props[i] = drmModeGetProperty(drm_dev->fd, props->props[i]);
        LV_LOG_TRACE("Added connector prop %u:%s", drm_dev->conn_props[i]->prop_id, drm_dev->conn_props[i]->name);
    }
    drmModeFreeObjectProperties(props);

    return 0;
}

static int drm_add_plane_property(drm_dev_t * drm_dev, const char * name, uint64_t value)
{
    int ret;
    uint32_t prop_id = get_plane_property_id(drm_dev, name);

    if(!prop_id) {
        LV_LOG_ERROR("Couldn't find plane prop %s", name);
        return -1;
    }

    ret = drmModeAtomicAddProperty(drm_dev->req, drm_dev->plane_id, get_plane_property_id(drm_dev, name), value);
    if(ret < 0) {
        LV_LOG_ERROR("drmModeAtomicAddProperty (%s:%" PRIu64 ") failed: %d", name, value, ret);
        return ret;
    }

    return 0;
}

static int drm_add_crtc_property(drm_dev_t * drm_dev, const char * name, uint64_t value)
{
    int ret;
    uint32_t prop_id = get_crtc_property_id(drm_dev, name);

    if(!prop_id) {
        LV_LOG_ERROR("Couldn't find crtc prop %s", name);
        return -1;
    }

    ret = drmModeAtomicAddProperty(drm_dev->req, drm_dev->crtc_id, get_crtc_property_id(drm_dev, name), value);
    if(ret < 0) {
        LV_LOG_ERROR("drmModeAtomicAddProperty (%s:%" PRIu64 ") failed: %d", name, value, ret);
        return ret;
    }

    return 0;
}

static int drm_add_conn_property(drm_dev_t * drm_dev, const char * name, uint64_t value)
{
    int ret;
    uint32_t prop_id = get_conn_property_id(drm_dev, name);

    if(!prop_id) {
        LV_LOG_ERROR("Couldn't find conn prop %s", name);
        return -1;
    }

    ret = drmModeAtomicAddProperty(drm_dev->req, drm_dev->conn_id, get_conn_property_id(drm_dev, name), value);
    if(ret < 0) {
        LV_LOG_ERROR("drmModeAtomicAddProperty (%s:%" PRIu64 ") failed: %d", name, value, ret);
        return ret;
    }

    return 0;
}

static int drm_dmabuf_set_plane(drm_dev_t * drm_dev, drm_buffer_t * buf)
{
    int ret;
    uint32_t flags = DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_ATOMIC_NONBLOCK;

#if LV_USE_LINUX_DRM_GBM_BUFFERS

    struct dma_buf_sync sync_req;

    sync_req.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    if(ioctl(buf->handle, DMA_BUF_IOCTL_SYNC, &sync_req) != 0) {
        LV_LOG_ERROR("Failed to end DMA-BUF R/W SYNC");
    }

#endif

    drm_dev->req = drmModeAtomicAlloc();

    /* On the first Atomic commit after attaching, do a modeset */
    if(drm_dev->needs_modeset) {
        drm_add_conn_property(drm_dev, "CRTC_ID", drm_dev->crtc_id);

        drm_add_crtc_property(drm_dev, "MODE_ID", drm_dev->blob_id);
        drm_add_crtc_property(drm_dev, "ACTIVE", 1);

        flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;

        drm_dev->needs_modeset = false;
    }

    drm_add_plane_property(drm_dev, "FB_ID", buf->fb_handle);
    drm_add_plane_property(drm_dev, "CRTC_ID", drm_dev->crtc_id);
    drm_add_plane_property(drm_dev, "SRC_X", 0);
    drm_add_plane_property(drm_dev, "SRC_Y", 0);
    drm_add_plane_property(drm_dev, "SRC_W", drm_dev->width << 16);
    drm_add_plane_property(drm_dev, "SRC_H", drm_dev->height << 16);
    drm_add_plane_property(drm_dev, "CRTC_X", 0);
    drm_add_plane_property(drm_dev, "CRTC_Y", 0);
    drm_add_plane_property(drm_dev, "CRTC_W", drm_dev->width);
    drm_add_plane_property(drm_dev, "CRTC_H", drm_dev->height);

    ret = drmModeAtomicCommit(drm_dev->fd, drm_dev->req, flags, drm_dev);
    if(ret) {
        LV_LOG_ERROR("drmModeAtomicCommit failed: %s (%d)", strerror(errno), errno);
        drmModeAtomicFree(drm_dev->req);
        drm_dev->req = NULL;
        return ret;
    }

    return 0;
}

static int find_plane(drm_dev_t * drm_dev, unsigned int fourcc, uint32_t * plane_id, uint32_t crtc_id,
                      uint32_t crtc_idx)
{
    LV_UNUSED(crtc_id);
    drmModePlaneResPtr planes;
    drmModePlanePtr plane;
    unsigned int i;
    unsigned int j;
    int ret = 0;
    uint32_t best_id = 0;
    uint64_t best_zpos = 0;

    planes = drmModeGetPlaneResources(drm_dev->fd);
    if(!planes) {
        LV_LOG_ERROR("drmModeGetPlaneResources failed");
        return -1;
    }

    LV_LOG_TRACE("drm: found planes %u", planes->count_planes);

    for(i = 0; i < planes->count_planes; ++i) {
        plane = drmModeGetPlane(drm_dev->fd, planes->planes[i]);
        if(!plane) {
            LV_LOG_ERROR("drmModeGetPlane failed: %s", strerror(errno));
            ret = -1;
            break;
        }

        if(!(plane->possible_crtcs & (1 << crtc_idx))) {
            drmModeFreePlane(plane);
            continue;
        }

        for(j = 0; j < plane->count_formats; ++j) {
            if(plane->formats[j] == fourcc)
                break;
        }

        if(j == plane->count_formats) {
            drmModeFreePlane(plane);
            continue;
        }

        uint64_t type = 0;
        uint64_t zpos = 0;
        drm_get_plane_type_zpos(drm_dev->fd, plane->plane_id, &type, &zpos);

        /* Prefer the CRTC's primary plane: it is the one meant for full screen
         * scanout, and taking it also replaces a framebuffer left on it by the
         * kernel console (fbcon). */
        if(type == DRM_PLANE_TYPE_PRIMARY) {
            *plane_id = plane->plane_id;
            drmModeFreePlane(plane);
            LV_LOG_TRACE("found primary plane %d", *plane_id);
            goto out;
        }

        /* Otherwise remember the plane composited highest (largest zpos) so the
         * app ends up above overlays the kernel may occupy, e.g. fbcon's fb on
         * drivers where the primary plane is not enumerable. */
        if(best_id == 0 || zpos > best_zpos) {
            best_id = plane->plane_id;
            best_zpos = zpos;
        }

        drmModeFreePlane(plane);
    }

    if(best_id) {
        *plane_id = best_id;
        LV_LOG_TRACE("found plane %d (zpos %" LV_PRIu64 ")", *plane_id, best_zpos);
    }
    else {
        ret = -1;
    }

out:
    drmModeFreePlaneResources(planes);
    return ret;
}

/**
 * Read the "type" (primary/overlay/cursor) and "zpos" properties of a plane.
 * Missing properties leave the out values at 0.
 */
static void drm_get_plane_type_zpos(int fd, uint32_t plane_id, uint64_t * type, uint64_t * zpos)
{
    drmModeObjectProperties * props = drmModeObjectGetProperties(fd, plane_id, DRM_MODE_OBJECT_PLANE);
    if(!props) return;

    for(uint32_t i = 0; i < props->count_props; i++) {
        drmModePropertyPtr p = drmModeGetProperty(fd, props->props[i]);
        if(!p) continue;

        if(!strcmp(p->name, "type")) *type = props->prop_values[i];
        else if(!strcmp(p->name, "zpos")) *zpos = props->prop_values[i];

        drmModeFreeProperty(p);
    }

    drmModeFreeObjectProperties(props);
}

static int drm_find_connector(drm_dev_t * drm_dev, int64_t connector_id)
{
    drmModeConnector * conn = NULL;
    drmModeEncoder * enc = NULL;
    drmModeRes * res;
    int i;
    int ret = -1;

    if((res = drmModeGetResources(drm_dev->fd)) == NULL) {
        LV_LOG_ERROR("drmModeGetResources() failed");
        goto free_res;
    }

    if(res->count_crtcs <= 0) {
        LV_LOG_ERROR("no Crtcs");
        goto free_res;
    }

    /* find all available connectors */
    for(i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(drm_dev->fd, res->connectors[i]);
        if(!conn)
            continue;

        if(connector_id >= 0 && conn->connector_id != connector_id) {
            drmModeFreeConnector(conn);
            continue;
        }

        if(conn->connection == DRM_MODE_CONNECTED) {
            LV_LOG_TRACE("drm: connector %d: connected", conn->connector_id);
        }
        else if(conn->connection == DRM_MODE_DISCONNECTED) {
            LV_LOG_TRACE("drm: connector %d: disconnected", conn->connector_id);
        }
        else if(conn->connection == DRM_MODE_UNKNOWNCONNECTION) {
            LV_LOG_TRACE("drm: connector %d: unknownconnection", conn->connector_id);
        }
        else {
            LV_LOG_TRACE("drm: connector %d: unknown", conn->connector_id);
        }

        if(conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            break;

        drmModeFreeConnector(conn);
        conn = NULL;
    };

    if(!conn) {
        LV_LOG_ERROR("suitable connector not found");
        goto free_res;
    }

    drm_dev->conn_id = conn->connector_id;
    LV_LOG_TRACE("conn_id: %d", drm_dev->conn_id);
    drm_dev->mmWidth = conn->mmWidth;
    drm_dev->mmHeight = conn->mmHeight;

    lv_memcpy(&drm_dev->mode, &conn->modes[0], sizeof(drmModeModeInfo));

    if(drmModeCreatePropertyBlob(drm_dev->fd, &drm_dev->mode, sizeof(drm_dev->mode),
                                 &drm_dev->blob_id)) {
        LV_LOG_ERROR("error creating mode blob");
        goto free_res;
    }

    drm_dev->width = conn->modes[0].hdisplay;
    drm_dev->height = conn->modes[0].vdisplay;

    for(i = 0 ; i < res->count_encoders; i++) {
        enc = drmModeGetEncoder(drm_dev->fd, res->encoders[i]);
        if(!enc)
            continue;

        LV_LOG_TRACE("enc%d enc_id %d conn enc_id %d", i, enc->encoder_id, conn->encoder_id);

        if(enc->encoder_id == conn->encoder_id)
            break;

        drmModeFreeEncoder(enc);
        enc = NULL;
    }

    if(enc) {
        drm_dev->enc_id = enc->encoder_id;
        LV_LOG_TRACE("enc_id: %d", drm_dev->enc_id);
        drm_dev->crtc_id = enc->crtc_id;
        LV_LOG_TRACE("crtc_id: %d", drm_dev->crtc_id);
        drmModeFreeEncoder(enc);
        enc = NULL;
    }
    else {
        /* Encoder hasn't been associated yet, look it up */
        bool found = false;
        for(i = 0; i < conn->count_encoders; i++) {
            int crtc, crtc_id = -1;

            enc = drmModeGetEncoder(drm_dev->fd, conn->encoders[i]);
            if(!enc)
                continue;

            for(crtc = 0 ; crtc < res->count_crtcs; crtc++) {
                uint32_t crtc_mask = 1 << crtc;

                crtc_id = res->crtcs[crtc];

                LV_LOG_TRACE("enc_id %d crtc%d id %d mask %x possible %x", enc->encoder_id, crtc, crtc_id, crtc_mask,
                             enc->possible_crtcs);

                if(enc->possible_crtcs & crtc_mask)
                    break;
            }

            if(crtc_id > 0) {
                drm_dev->enc_id = enc->encoder_id;
                LV_LOG_TRACE("enc_id: %d", drm_dev->enc_id);
                drm_dev->crtc_id = crtc_id;
                LV_LOG_TRACE("crtc_id: %d", drm_dev->crtc_id);
                drmModeFreeEncoder(enc);
                enc = NULL;
                found = true;
                break;
            }

            drmModeFreeEncoder(enc);
            enc = NULL;
        }

        if(!found) {
            LV_LOG_ERROR("suitable encoder not found");
            goto free_res;
        }
    }

    drm_dev->crtc_idx = UINT32_MAX;

    for(i = 0; i < res->count_crtcs; ++i) {
        if(drm_dev->crtc_id == res->crtcs[i]) {
            drm_dev->crtc_idx = i;
            break;
        }
    }

    if(drm_dev->crtc_idx == UINT32_MAX) {
        LV_LOG_ERROR("drm: CRTC not found");
        goto free_res;
    }

    LV_LOG_TRACE("crtc_idx: %d", drm_dev->crtc_idx);
    ret = 0;

free_res:
    if(enc) {
        drmModeFreeEncoder(enc);
        enc = NULL;
    }
    if(conn) {
        drmModeFreeConnector(conn);
        conn = NULL;
    }
    if(res) {
        drmModeFreeResources(res);
        res = NULL;
    }
    return ret;
}

static int drm_open(const char * path)
{
    int fd, flags;
    uint64_t has_dumb;
    int ret;

    fd = open(path, O_RDWR);
    if(fd < 0) {
        LV_LOG_ERROR("cannot open \"%s\"", path);
        return -1;
    }

    /* set FD_CLOEXEC flag */
    if((flags = fcntl(fd, F_GETFD)) < 0 ||
       fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0) {
        LV_LOG_ERROR("fcntl FD_CLOEXEC failed");
        goto err;
    }

    /* check capability */
    ret = drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_dumb);
    if(ret < 0 || has_dumb == 0) {
        LV_LOG_ERROR("drmGetCap DRM_CAP_DUMB_BUFFER failed or \"%s\" doesn't have dumb "
                     "buffer", path);
        goto err;
    }

    return fd;
err:
    close(fd);
    return -1;
}

static int drm_setup(drm_dev_t * drm_dev, int fd, int64_t connector_id, unsigned int fourcc)
{
    int ret;

    drm_dev->fd = fd;
    drm_dev->needs_modeset = true;

    ret = drmSetClientCap(drm_dev->fd, DRM_CLIENT_CAP_ATOMIC, 1);
    if(ret) {
        LV_LOG_ERROR("No atomic modesetting support: %s", strerror(errno));
        goto err;
    }

    ret = drm_find_connector(drm_dev, connector_id);
    if(ret) {
        LV_LOG_ERROR("available drm devices not found");
        goto err;
    }

    ret = find_plane(drm_dev, fourcc, &drm_dev->plane_id, drm_dev->crtc_id, drm_dev->crtc_idx);
    if(ret) {
        LV_LOG_ERROR("Cannot find plane");
        goto err;
    }

    drm_dev->plane = drmModeGetPlane(drm_dev->fd, drm_dev->plane_id);
    if(!drm_dev->plane) {
        LV_LOG_ERROR("Cannot get plane");
        goto err;
    }

    drm_dev->crtc = drmModeGetCrtc(drm_dev->fd, drm_dev->crtc_id);
    if(!drm_dev->crtc) {
        LV_LOG_ERROR("Cannot get crtc");
        goto err;
    }

    drm_dev->conn = drmModeGetConnector(drm_dev->fd, drm_dev->conn_id);
    if(!drm_dev->conn) {
        LV_LOG_ERROR("Cannot get connector");
        goto err;
    }

    ret = drm_get_plane_props(drm_dev);
    if(ret) {
        LV_LOG_ERROR("Cannot get plane props");
        goto err;
    }

    ret = drm_get_crtc_props(drm_dev);
    if(ret) {
        LV_LOG_ERROR("Cannot get crtc props");
        goto err;
    }

    ret = drm_get_conn_props(drm_dev);
    if(ret) {
        LV_LOG_ERROR("Cannot get connector props");
        goto err;
    }

    drm_dev->drm_event_ctx.version = DRM_EVENT_CONTEXT_VERSION;
    drm_dev->drm_event_ctx.page_flip_handler = page_flip_handler;
    drm_dev->fourcc = fourcc;

    LV_LOG_INFO("drm: Found plane_id: %u connector_id: %d crtc_id: %d",
                drm_dev->plane_id, drm_dev->conn_id, drm_dev->crtc_id);

    LV_LOG_INFO("drm: %dx%d (%dmm X% dmm) pixel format %c%c%c%c",
                drm_dev->width, drm_dev->height, drm_dev->mmWidth, drm_dev->mmHeight,
                (fourcc >> 0) & 0xff, (fourcc >> 8) & 0xff, (fourcc >> 16) & 0xff, (fourcc >> 24) & 0xff);


#if LV_USE_LINUX_DRM_GBM_BUFFERS

    /* Create GBM device and buffer */
    drm_dev->gbm_device = gbm_create_device(drm_dev->fd);

    if(drm_dev->gbm_device == NULL) {
        LV_LOG_ERROR("Failed to create GBM device");
        goto err;
    }

    LV_LOG_INFO("GBM device backend: %s", gbm_device_get_backend_name(drm_dev->gbm_device));
#endif

    return 0;

err:
#if LV_USE_LINUX_DRM_GBM_BUFFERS
    if(drm_dev->gbm_device) {
        gbm_device_destroy(drm_dev->gbm_device);
        drm_dev->gbm_device = NULL;
    }
#endif

    if(drm_dev->plane) {
        drmModeFreePlane(drm_dev->plane);
        drm_dev->plane = NULL;
    }
    if(drm_dev->crtc) {
        drmModeFreeCrtc(drm_dev->crtc);
        drm_dev->crtc = NULL;
    }
    if(drm_dev->conn) {
        drmModeFreeConnector(drm_dev->conn);
        drm_dev->conn = NULL;
    }
    /* The device fd belongs to the caller: only forget it here. */
    drm_dev->fd = -1;
    return -1;
}

#if !LV_USE_LINUX_DRM_GBM_BUFFERS
static int drm_allocate_dumb(drm_dev_t * drm_dev, drm_buffer_t * buf)
{
    struct drm_mode_create_dumb creq;
    struct drm_mode_map_dumb mreq;
    uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
    int ret;

    /* create dumb buffer */
    lv_memzero(&creq, sizeof(creq));
    creq.width = drm_dev->width;
    creq.height = drm_dev->height;
    creq.bpp = LV_COLOR_DEPTH;
    ret = drmIoctl(drm_dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
    if(ret < 0) {
        LV_LOG_ERROR("DRM_IOCTL_MODE_CREATE_DUMB fail");
        return -1;
    }

    buf->handle = creq.handle;
    buf->pitch = creq.pitch;
    buf->size = creq.size;

    /* prepare buffer for memory mapping */
    lv_memzero(&mreq, sizeof(mreq));
    mreq.handle = creq.handle;
    ret = drmIoctl(drm_dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);
    if(ret) {
        LV_LOG_ERROR("DRM_IOCTL_MODE_MAP_DUMB fail");
        return -1;
    }

    buf->offset = mreq.offset;
    LV_LOG_INFO("size %lu pitch %u offset %u", buf->size, buf->pitch, buf->offset);

    /* perform actual memory mapping */
    buf->map = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_dev->fd, mreq.offset);
    if(buf->map == MAP_FAILED) {
        LV_LOG_ERROR("mmap fail");
        return -1;
    }

    /* clear the framebuffer to 0 (= full transparency in ARGB8888) */
    lv_memzero(buf->map, creq.size);

    /* create framebuffer object for the dumb-buffer */
    handles[0] = creq.handle;
    pitches[0] = creq.pitch;
    offsets[0] = 0;
    ret = drmModeAddFB2(drm_dev->fd, drm_dev->width, drm_dev->height, drm_dev->fourcc,
                        handles, pitches, offsets, &buf->fb_handle, 0);
    if(ret) {
        LV_LOG_ERROR("drmModeAddFB fail");
        return -1;
    }

    return 0;
}
#endif /*!LV_USE_LINUX_DRM_GBM_BUFFERS*/

#if LV_USE_LINUX_DRM_GBM_BUFFERS

static int create_gbm_buffer(drm_dev_t * drm_dev, drm_buffer_t * buf)
{
    struct gbm_bo * gbm_bo;
    int prime_fd;
    uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
    uint32_t n_planes;
    int res;

    /* gbm_bo_format does not define anything other than ARGB8888 or XRGB8888 */
    if(LV_COLOR_DEPTH != 32) {
        LV_LOG_ERROR("Unsupported color format");
        return -1;
    }

    /* Create a linear GBM buffer object - best practice when modifiers are not used */
    if(!(gbm_bo = gbm_bo_create(drm_dev->gbm_device,
                                drm_dev->width, drm_dev->height, GBM_BO_FORMAT_XRGB8888,
                                GBM_BO_USE_SCANOUT | GBM_BO_USE_LINEAR))) {

        LV_LOG_ERROR("Unable to create gbm buffer object");
        return -1;
    }

    /* Currently only, one plane per dma-buf/prime fd is supported - but some GPUs feature
     * several planes (multiple fds or sometimes a single fd for multiple planes).
     * current implementation is kept simple for now */

    n_planes = gbm_bo_get_plane_count(gbm_bo);

    if(n_planes != 1) {
        LV_LOG_ERROR("The current implementation only supports a single plane per fd");
        return -1;
    }

    uint32_t h = gbm_bo_get_height(gbm_bo);
    pitches[0] = buf->pitch = gbm_bo_get_stride_for_plane(gbm_bo, 0);
    offsets[0] = buf->offset = gbm_bo_get_offset(gbm_bo, 0);
    buf->size = h * buf->pitch;

    LV_LOG_INFO("Created GBM BO of size: %lu pitch: %u offset: %u",
                buf->size, buf->pitch, buf->offset);

    prime_fd = gbm_bo_get_fd_for_plane(gbm_bo, 0);

    if(prime_fd < 0) {
        LV_LOG_ERROR("Failed to get prime fd for plane 0");
        return -1;

    }

    /* A dma-buf fd is not required to be mappable by its exporter, so map the
     * buffer through GBM instead of mmap()ing the prime fd directly. */
    uint32_t map_stride = 0;
    void * map_data = NULL;
    void * map = gbm_bo_map(gbm_bo, 0, 0, drm_dev->width, drm_dev->height,
                            GBM_BO_TRANSFER_WRITE, &map_stride, &map_data);

    if(map == NULL) {
        LV_LOG_ERROR("Failed to map gbm buffer object: %s", strerror(errno));
        gbm_bo_destroy(gbm_bo);
        return -1;
    }

    buf->map = map;
    buf->gbm_bo = gbm_bo;
    buf->gbm_map_data = map_data;

    /* Used to perform DMA_BUF_SYNC ioctl calls during the rendering cycle */
    buf->handle = prime_fd;

    /* Convert prime fd to a libdrm buffer handle */
    drmPrimeFDToHandle(drm_dev->fd, buf->handle, &handles[0]);

    /* create libdrm framebuffer */
    res = drmModeAddFB2(drm_dev->fd, drm_dev->width, drm_dev->height, drm_dev->fourcc,
                        handles, pitches, offsets, &buf->fb_handle, 0);

    if(res) {
        LV_LOG_ERROR("drmModeAddFB2 failed");
        return -1;
    }

    if(drmCloseBufferHandle(drm_dev->fd, handles[0]) != 0) {
        LV_LOG_ERROR("drmCloseBufferHandle failed");
        return -1;
    }

    return 0;

}

#endif /* LV_USE_LINUX_DRM_GBM_BUFFERS */

static int drm_setup_buffers(drm_dev_t * drm_dev)
{
    int ret;

#if LV_USE_LINUX_DRM_GBM_BUFFERS
    ret = create_gbm_buffer(drm_dev, &drm_dev->drm_bufs[0]);
    if(ret < 0) {
        return ret;
    }

    ret = create_gbm_buffer(drm_dev, &drm_dev->drm_bufs[1]);
    if(ret < 0) {
        return ret;
    }

#else
    /* Use dumb buffers */
    ret = drm_allocate_dumb(drm_dev, &drm_dev->drm_bufs[0]);
    if(ret)
        return ret;

    ret = drm_allocate_dumb(drm_dev, &drm_dev->drm_bufs[1]);
    if(ret)
        return ret;

#endif

    return 0;
}

static void drm_flush_wait(lv_display_t * disp)
{
    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);

    struct pollfd pfd;
    pfd.fd = drm_dev->fd;
    pfd.events = POLLIN;

    while(drm_dev->req) {
        int ret;
        do {
            ret = poll(&pfd, 1, DRM_FLIP_TIMEOUT_MS);
        } while(ret == -1 && errno == EINTR);

        if(ret > 0) {
            drmHandleEvent(drm_dev->fd, &drm_dev->drm_event_ctx);
        }
        else {
            if(ret == 0) LV_LOG_ERROR("no page flip event within %d ms", DRM_FLIP_TIMEOUT_MS);
            else LV_LOG_ERROR("poll failed: %s", strerror(errno));

            /* Drop the pending request so the refresh loop is not stuck on a
             * flip that will never complete. */
            drmModeAtomicFree(drm_dev->req);
            drm_dev->req = NULL;
        }
    }
}

static void drm_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    if(!lv_display_flush_is_last(disp)) return;

    LV_UNUSED(area);
    LV_UNUSED(px_map);
    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);

    LV_ASSERT(drm_dev->act_buf != NULL);

    if(drm_dmabuf_set_plane(drm_dev, drm_dev->act_buf)) {
        LV_LOG_ERROR("Flush fail");
        return;
    }

    drm_dev->act_buf = NULL;

}

/**
 * Release everything the display holds on the device - buffers, framebuffers,
 * DRM objects and the device fd - and park it. The display object and every
 * screen (the application's UI) stay alive, so the very same display can be
 * attached to a device again with lv_linux_drm_set_fd().
 */
static void drm_detach_device(drm_dev_t * drm_dev, lv_display_t * disp)
{
    /* A flush may still be in flight - its page flip will never complete now.
     * Abandon it, otherwise the next refresh spins in LVGL's wait_for_flushing()
     * (with flush_wait_cb cleared it busy-waits on disp->flushing). */
    lv_display_flush_ready(disp);

    /* The application keeps running while parked. Its invalidations must not
     * queue draws into buffers that are about to be released, and must not wake
     * the refresh timer up again. */
    lv_display_enable_invalidation(disp, false);

    /* Nothing may be drawn into the buffers that are about to go away, so take
     * them away from LVGL: a stray refresh bails out instead of writing into
     * freed memory. lv_linux_drm_set_fd() installs fresh ones. */
    lv_display_set_draw_buffers(disp, NULL, NULL);

    /* Park it for real: with the refresh timer gone nothing can draw into the
     * buffers that are about to be released - and an invalidation cannot wake
     * the refresh up again through LV_EVENT_REFR_REQUEST. */
    lv_display_delete_refr_timer(disp);
    lv_display_set_flush_cb(disp, NULL);
    lv_display_set_flush_wait_cb(disp, NULL);

    /* Restore original CRTC if saved */
    if(drm_dev->fd >= 0 && drm_dev->saved_crtc) {
        drmModeCrtc * s_crtc = drm_dev->saved_crtc;
        drmModeSetCrtc(drm_dev->fd, s_crtc->crtc_id, s_crtc->buffer_id, 0, 0,
                       NULL, 0, &s_crtc->mode);
        drmModeFreeCrtc(s_crtc);
        drm_dev->saved_crtc = NULL;
    }

    /* Drop any pending atomic request */
    if(drm_dev->req) {
        drmModeAtomicFree(drm_dev->req);
        drm_dev->req = NULL;
    }

#if LV_USE_LINUX_DRM_GBM_BUFFERS
    for(int i = 0; i < BUFFER_CNT; ++i) {
        drm_buffer_t * b = &drm_dev->drm_bufs[i];

        if(b->fb_handle) {
            drmModeRmFB(drm_dev->fd, b->fb_handle);
            b->fb_handle = 0;
        }

        if(b->gbm_bo) {
            gbm_bo_unmap(b->gbm_bo, b->gbm_map_data);
            gbm_bo_destroy(b->gbm_bo);
            b->gbm_bo = NULL;
            b->gbm_map_data = NULL;
            b->map = MAP_FAILED;
        }

        if((int)b->handle >= 0) {
            close((int)b->handle);
            b->handle = 0;
        }
    }

    if(drm_dev->gbm_device) {
        gbm_device_destroy(drm_dev->gbm_device);
        drm_dev->gbm_device = NULL;
    }

#else /* dumb buffers */
    for(int i = 0; i < BUFFER_CNT; ++i) {
        drm_buffer_t * b = &drm_dev->drm_bufs[i];

        if(b->fb_handle) {
            drmModeRmFB(drm_dev->fd, b->fb_handle);
            b->fb_handle = 0;
        }
        if(MAP_FAILED != b->map) {
            munmap(b->map, b->size);
            b->map = MAP_FAILED;
        }
        if(b->handle) {
            struct drm_mode_destroy_dumb d = { .handle = b->handle };
            /* Dumb buffers should be destroyed if they are closed, but might as well use the DRM_IOCTL_MODE_DESTROY_DUMB */
            drmIoctl(drm_dev->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d);
            b->handle = 0;
        }
    }
#endif

    /* Free DRM properties */
    for(uint32_t i = 0; i < drm_dev->count_plane_props; ++i) {
        if(drm_dev->plane_props[i]) {
            drmModeFreeProperty(drm_dev->plane_props[i]);
            drm_dev->plane_props[i] = NULL;
        }
    }
    drm_dev->count_plane_props = 0;

    for(uint32_t i = 0; i < drm_dev->count_crtc_props; ++i) {
        if(drm_dev->crtc_props[i]) {
            drmModeFreeProperty(drm_dev->crtc_props[i]);
            drm_dev->crtc_props[i] = NULL;
        }
    }
    drm_dev->count_crtc_props = 0;

    for(uint32_t i = 0; i < drm_dev->count_conn_props; ++i) {
        if(drm_dev->conn_props[i]) {
            drmModeFreeProperty(drm_dev->conn_props[i]);
            drm_dev->conn_props[i] = NULL;
        }
    }
    drm_dev->count_conn_props = 0;

    if(drm_dev->blob_id) {
        drmModeDestroyPropertyBlob(drm_dev->fd, drm_dev->blob_id);
        drm_dev->blob_id = 0;
    }

    if(drm_dev->conn) {
        drmModeFreeConnector(drm_dev->conn);
        drm_dev->conn = NULL;
    }

    if(drm_dev->crtc) {
        drmModeFreeCrtc(drm_dev->crtc);
        drm_dev->crtc = NULL;
    }

    if(drm_dev->plane) {
        drmModeFreePlane(drm_dev->plane);
        drm_dev->plane = NULL;
    }

    if(drm_dev->fd >= 0) {
        close(drm_dev->fd);
        drm_dev->fd = -1;
    }

    /* The next attach sets a mode again. */
    drm_dev->needs_modeset = true;
}

static void drm_del_event_cb(lv_event_t * e)
{
    if(LV_EVENT_DELETE != lv_event_get_code(e))
        return;

    lv_display_t * disp = lv_event_get_current_target(e);
    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);
    if(!drm_dev) return;

    drm_detach_device(drm_dev, disp);

    lv_display_set_driver_data(disp, NULL);
    lv_free(drm_dev);
}

/**
 * @brief Detach the display from its DRM device
 *
 * Releases the device fd, the buffers and every DRM object the display holds,
 * and stops its refreshing: the display object and its screens stay alive.
 * Attach it again with lv_linux_drm_set_fd() to render on another device - for
 * example on a fresh DRM lease fd after a revoke.
 *
 * @param disp pointer to the display object created with lv_linux_drm_create()
 */
void lv_linux_drm_detach(lv_display_t * disp)
{
    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);
    if(drm_dev == NULL) return;

    lv_display_remove_event_cb_with_user_data(disp, drm_dmabuf_set_active_buf, drm_dev);
    drm_detach_device(drm_dev, disp);
}

static uint32_t tick_get_cb(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t time_ms = t.tv_sec * 1000 + (t.tv_nsec / 1000000);
    return time_ms;
}

#endif /*LV_USE_LINUX_DRM && !LV_LINUX_DRM_USE_EGL*/
