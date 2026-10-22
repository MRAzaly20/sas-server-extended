/* ============================================================
 *  goose_handler.c — Per-IED GOOSE subscriber logic
 *  Registers subscriptions from JSON config and dispatches
 *  actions (intertrip, CB status, etc.) on the local IedServer.
 * ============================================================ */
#include "goose_handler.h"
#include <stdio.h>
#include <string.h>

/* ---- Context passed to each subscriber callback ---- */
typedef struct {
    IedServer      server;
    IedModel      *model;
    const char    *ied_name;
    GooseSubEntry  entry;    /* copy of the subscription config */
    int            last_trip_state;
    ConsoleCommandFn cmd_fn;
    void          *ctx_ptr;
} GooseSubCtx;

/* Max number of concurrent contexts */
#define MAX_CTX 16
static GooseSubCtx g_ctx_pool[MAX_CTX];
static int g_ctx_count = 0;

/* ---- Helper: relay trip -> energize remote trip coil (TCI) ----
 * Trip authority rule: the BCU never decides on faults; it only
 * exposes a trip-coil interface. The RELAYS author the trip decision
 * (main: P643/P543, backup: P142, busbar: P746). Receiving a trip
 * GOOSE is the physical equivalent of energizing the breaker's trip
 * coil; the BCU process opens XCBR1 after the coil delay. */
static void action_trip_coil_cb(GooseSubCtx *ctx, MmsValue *values) {
    if (!values) return;

    /* Dataset: [0]=*Tr.general or *Op.general boolean */
    MmsValue *trip_val = MmsValue_getElement(values, 0);
    if (!trip_val) return;

    bool tripped = MmsValue_getBoolean(trip_val);

    if (tripped == ctx->last_trip_state) return;
    ctx->last_trip_state = tripped;

    ctx->cmd_fn(ctx->server, ctx->model, NULL,
                tripped ? "trip_coil_energize" : "trip_coil_release",
                ctx->ctx_ptr);

    if (tripped)
        printf("[GOOSE RX] %s: Trip coil ENERGIZED by %s\n",
               ctx->ied_name, ctx->entry.gocbRef);
}

/* ---- Helper: backup relay tracks main-relay trip status ---- */
static void action_main_trip_status(GooseSubCtx *ctx, MmsValue *values) {
    if (!values) return;

    MmsValue *trip_val = MmsValue_getElement(values, 0);
    if (!trip_val) return;

    bool tripped = MmsValue_getBoolean(trip_val);

    if (tripped == ctx->last_trip_state) return;
    ctx->last_trip_state = tripped;

    ctx->cmd_fn(ctx->server, ctx->model, NULL,
                tripped ? "main_trip_set" : "main_trip_clear",
                ctx->ctx_ptr);

    printf("[GOOSE RX] %s: Main relay trip status %s (from %s)\n",
           ctx->ied_name, tripped ? "SET" : "CLEAR",
           ctx->entry.gocbRef);
}

/* ---- Helper: backup relay tracks CB position (breaker feedback) ---- */
static void action_cb_status_feedback(GooseSubCtx *ctx, MmsValue *values) {
    if (!values) return;

    MmsValue *pos = MmsValue_getElement(values, 0);
    if (!pos) return;

    /* BCU SwitchStatus dataset: [0]=XCBR1.Pos.stVal (Dbpos) */
    int dbpos = MmsValue_toInt32(pos);

    /* Edge-detect closed state to avoid keep-alive console spam */
    bool closed = (dbpos == 2);
    if (closed == ctx->last_trip_state) return;
    ctx->last_trip_state = closed;

    ctx->cmd_fn(ctx->server, ctx->model, NULL,
                closed ? "cb_feedback_closed" : "cb_feedback_open",
                ctx->ctx_ptr);

    printf("[GOOSE RX] %s: CB feedback from %s = %s\n",
           ctx->ied_name, ctx->entry.gocbRef,
           (dbpos == 1) ? "OPEN" :
           (dbpos == 2) ? "CLOSED" : "INTERMEDIATE");
}

/* ---- Helper: log incoming CB status ---- */
static void action_log_cb_status(GooseSubCtx *ctx, MmsValue *values) {
    if (!values) return;
    MmsValue *pos = MmsValue_getElement(values, 0);
    if (!pos) return;
    int dbpos = MmsValue_toInt32(pos);
    const char *str = (dbpos == 1) ? "OPEN" :
                      (dbpos == 2) ? "CLOSED" : "INTERMEDIATE";
    printf("[GOOSE RX] %s: CB status from %s = %s\n",
           ctx->ied_name, ctx->entry.gocbRef, str);
}

/* ---- Helper: read E04 CouplerReady status ---- */
static void action_coupler_status(GooseSubCtx *ctx, MmsValue *values) {
    if (!values) return;
    
    /* E04 SwitchStatus dataset has CouplerReady (GGIO1.Ind1.stVal) at index 5 */
    if (MmsValue_getArraySize(values) <= 5) return;
    MmsValue *ready_val = MmsValue_getElement(values, 5);
    if (!ready_val) return;
    
    bool is_ready = MmsValue_getBoolean(ready_val);
    if (ctx->cmd_fn) {
        ctx->cmd_fn(ctx->server, ctx->model, NULL,
                    is_ready ? "coupler_ready_set" : "coupler_ready_clear",
                    ctx->ctx_ptr);
    }
}

/* ---- Main GOOSE listener callback ---- */
static void goose_listener(GooseSubscriber sub, void *param) {
    GooseSubCtx *ctx = (GooseSubCtx *)param;

    if (!GooseSubscriber_isValid(sub)) return;

    MmsValue *values = GooseSubscriber_getDataSetValues(sub);

    /* Dispatch by action string */
    int matched = 0;
    if (strcmp(ctx->entry.action, "trip_coil_cb") == 0) {
        matched = 1;
        action_trip_coil_cb(ctx, values);
    } else if (strcmp(ctx->entry.action, "main_trip_status") == 0) {
        matched = 1;
        action_main_trip_status(ctx, values);
    } else if (strcmp(ctx->entry.action, "cb_status_feedback") == 0) {
        matched = 1;
        action_cb_status_feedback(ctx, values);
    } else if (strcmp(ctx->entry.action, "log_cb_status") == 0) {
        matched = 1;
        action_log_cb_status(ctx, values);
    } else if (strcmp(ctx->entry.action, "coupler_status") == 0) {
        matched = 1;
        action_coupler_status(ctx, values);
    } else {
        printf("[GOOSE RX] %s: Unhandled action '%s' for %s\n",
               ctx->ied_name, ctx->entry.action, ctx->entry.gocbRef);
    }

    if (!matched) {
        printf("[GOOSE RX DEBUG] %s: Packet received but action unmatched (goCbRef: %s)\n", ctx->ied_name, ctx->entry.gocbRef);
    }
}

/* ---- Initialize ---- */
GooseReceiver goose_handler_init(const IedConfig *cfg,
                                 IedServer server,
                                 IedModel *model,
                                 ConsoleCommandFn cmd_fn,
                                 void *ctx_ptr)
{
    if (cfg->num_goose_subs == 0) return NULL;

    GooseReceiver receiver = GooseReceiver_create();
    if (cfg->goose_interface[0] != '\0')
        GooseReceiver_setInterfaceId(receiver, cfg->goose_interface);

    printf("[GOOSE] Registering %d subscriptions for %s:\n",
           cfg->num_goose_subs, cfg->ied_name);

    for (int i = 0; i < cfg->num_goose_subs && g_ctx_count < MAX_CTX; i++) {
        GooseSubCtx *ctx = &g_ctx_pool[g_ctx_count++];
        ctx->server   = server;
        ctx->model    = model;
        ctx->ied_name = cfg->ied_name;
        ctx->entry    = cfg->goose_subs[i];
        ctx->cmd_fn   = cmd_fn;
        ctx->ctx_ptr  = ctx_ptr;

        GooseSubscriber sub = GooseSubscriber_create(
            (char*)cfg->goose_subs[i].gocbRef, NULL);
        GooseSubscriber_setAppId(sub, cfg->goose_subs[i].appId);
        GooseSubscriber_setListener(sub, goose_listener, ctx);
        GooseReceiver_addSubscriber(receiver, sub);

        printf("  [%d] GoCbRef=%s AppID=%d action=%s\n",
               i, cfg->goose_subs[i].gocbRef,
               cfg->goose_subs[i].appId, cfg->goose_subs[i].action);
    }

    GooseReceiver_start(receiver);
    printf("[GOOSE] Receiver %s (%s)\n",
           GooseReceiver_isRunning(receiver) ? "started" : "FAILED",
           cfg->ied_name);

    return receiver;
}

/* ---- Cleanup ---- */
void goose_handler_cleanup(GooseReceiver receiver) {
    if (receiver) {
        GooseReceiver_stop(receiver);
        GooseReceiver_destroy(receiver);
    }
    g_ctx_count = 0;
}
