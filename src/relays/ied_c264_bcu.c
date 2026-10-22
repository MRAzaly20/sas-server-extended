/* ============================================================
 *  ied_c264_bcu.c — MiCOM C264 Bay Control Unit
 *  GI 150kV — All bays (E01–E06)
 *
 *  Functions: XCBR1 (CB), CSWI1 (switch control), XSWI
 *             (disconnectors), CILO (interlocking), GGIO
 *
 *  TRIP AUTHORITY MODEL:
 *    - The BCU NEVER decides on faults and performs no protection.
 *    - Protection relays (P643/P543/P142/P746) author trip decisions
 *      and publish trip GOOSE to the BCU's trip-coil interface (TCI).
 *    - The TCI is a dumb actuator: coil energize -> physical coil
 *      delay -> XCBR1 opens. The BCU merely reflects physics.
 *    - 86 lockout here is a *state*, not a decision: manual close is
 *      blocked while any relay trip source is still asserted.
 *
 *  GOOSE Publisher:  SwitchStatus (XCBR1.Pos, CSWI1.Pos)
 *  GOOSE Subscriber: relay trip GOOSE -> trip coil interface
 *
 *  Usage: E01_BCU.exe config/E01_BCU.json
 * ============================================================ */
#include "ied_server_base.h"
#include "iec61850_server.h"
#include "hal_thread.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifndef MODEL_EXTERN
#define MODEL_EXTERN model_E01_BCU
#endif
extern IedModel MODEL_EXTERN;

/* ---- BCU state ---- */
typedef struct {
    int  xcbr1_pos;          /* Circuit Breaker */
    int  xswi1_pos;          /* Bus-A disconnector */
    int  xswi2_pos;          /* Bus-B disconnector */
    int  xswi3_pos;          /* Line disconnector */
    int  xswi4_pos;          /* Earthing switch */
    int  active_trip_sources; /* Relay trip coils currently energized (86 state) */
    int  coil_pending;       /* Trip coil energized, CB opens after delay */
    uint64_t coil_energized_ms;
    int  coupler_ready;      /* E04 coupler: CB+DS-A+DS-B all closed */
    int  is_coupler;         /* True if this is the E04 Coupler bay */
    int  has_dsl;            /* True if this bay has a Line Disconnector */
    int  has_es;             /* True if this bay has an Earth Switch */

    /* Simulated analogs */
    float v_a, v_b, v_c;
    float i_a, i_b, i_c;
    float freq;
    float tot_w, tot_var;
} BCUContext;

#define NOMINAL_V   86.6f
#define NOMINAL_I   200.0f
#define NOMINAL_F   50.0f

/* Config cached by the tick so GOOSE-invoked commands (cfg == NULL)
 * can still display real IED name / settings */
static const IedConfig *g_cached_cfg = NULL;

/* ---- Helpers ---- */
static void bcu_set_float(IedServer s, IedModel *m, const char *ref, float v) {
    DataAttribute *da = (DataAttribute*)
        IedModel_getModelNodeByShortObjectReference(m, ref);
    if (da) {
        IedServer_lockDataModel(s);
        IedServer_updateFloatAttributeValue(s, da, v);
        IedServer_unlockDataModel(s);
    }
}

static void bcu_set_dbpos(IedServer s, IedModel *m, const char *ref, int v) {
    DataAttribute *da = (DataAttribute*)
        IedModel_getModelNodeByShortObjectReference(m, ref);
    if (da) {
        IedServer_lockDataModel(s);
        IedServer_updateDbposValue(s, da, v);
        IedServer_unlockDataModel(s);
    }
}

static void bcu_set_bool(IedServer s, IedModel *m, const char *ref, int v) {
    DataAttribute *da = (DataAttribute*)
        IedModel_getModelNodeByShortObjectReference(m, ref);
    if (da) {
        IedServer_lockDataModel(s);
        IedServer_updateBooleanAttributeValue(s, da, v ? true : false);
        IedServer_unlockDataModel(s);
    }
}

static const char *get_ld(IedModel *m) {
    return m->firstChild ? m->firstChild->name : "UNKNOWN";
}

/* ---- Update switch positions in the model ---- */
static void update_all_switch_model(IedServer s, IedModel *m, BCUContext *ctx) {
    const char *ld = get_ld(m);
    char ref[128];
    /* Dbpos: 1=OPEN, 2=CLOSED */
    snprintf(ref, sizeof(ref), "%s/XCBR1.Pos.stVal", ld);
    bcu_set_dbpos(s, m, ref, ctx->xcbr1_pos);
    snprintf(ref, sizeof(ref), "%s/XSWI1.Pos.stVal", ld);
    bcu_set_dbpos(s, m, ref, ctx->xswi1_pos);
    snprintf(ref, sizeof(ref), "%s/XSWI2.Pos.stVal", ld);
    bcu_set_dbpos(s, m, ref, ctx->xswi2_pos);
    snprintf(ref, sizeof(ref), "%s/XSWI3.Pos.stVal", ld);
    bcu_set_dbpos(s, m, ref, ctx->xswi3_pos);
    snprintf(ref, sizeof(ref), "%s/XSWI4.Pos.stVal", ld);
    bcu_set_dbpos(s, m, ref, ctx->xswi4_pos);
    
    /* If this is the coupler (E04), publish CouplerReady as GGIO Ind1 */
    snprintf(ref, sizeof(ref), "%s/GGIO1.Ind1.stVal", ld);
    int is_ready = (ctx->xcbr1_pos == 2 && ctx->xswi1_pos == 2 && ctx->xswi2_pos == 2);
    bcu_set_bool(s, m, ref, is_ready);
}

/* ---- Update analog measurements (3-phase WYE CDC + aggregates) ---- */
static void update_analogs(IedServer s, IedModel *m, BCUContext *ctx) {
    const char *ld = get_ld(m);
    char ref[128];

    /* --- Phase Currents --- */
    snprintf(ref, sizeof(ref), "%s/MMXU1.A.phsA.cVal.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->i_a);
    snprintf(ref, sizeof(ref), "%s/MMXU1.A.phsB.cVal.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->i_b);
    snprintf(ref, sizeof(ref), "%s/MMXU1.A.phsC.cVal.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->i_c);
    snprintf(ref, sizeof(ref), "%s/MMXU1.A.phsA.cVal.ang.f", ld);
    bcu_set_float(s, m, ref, 0.0f);
    snprintf(ref, sizeof(ref), "%s/MMXU1.A.phsB.cVal.ang.f", ld);
    bcu_set_float(s, m, ref, -120.0f);
    snprintf(ref, sizeof(ref), "%s/MMXU1.A.phsC.cVal.ang.f", ld);
    bcu_set_float(s, m, ref, 120.0f);

    /* --- Phase Voltages --- */
    snprintf(ref, sizeof(ref), "%s/MMXU1.PhV.phsA.cVal.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->v_a);
    snprintf(ref, sizeof(ref), "%s/MMXU1.PhV.phsB.cVal.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->v_b);
    snprintf(ref, sizeof(ref), "%s/MMXU1.PhV.phsC.cVal.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->v_c);
    snprintf(ref, sizeof(ref), "%s/MMXU1.PhV.phsA.cVal.ang.f", ld);
    bcu_set_float(s, m, ref, 0.0f);
    snprintf(ref, sizeof(ref), "%s/MMXU1.PhV.phsB.cVal.ang.f", ld);
    bcu_set_float(s, m, ref, -120.0f);
    snprintf(ref, sizeof(ref), "%s/MMXU1.PhV.phsC.cVal.ang.f", ld);
    bcu_set_float(s, m, ref, 120.0f);

    /* --- Aggregates --- */
    snprintf(ref, sizeof(ref), "%s/MMXU1.TotW.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->tot_w);
    snprintf(ref, sizeof(ref), "%s/MMXU1.TotVAr.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->tot_var);
    snprintf(ref, sizeof(ref), "%s/MMXU1.Hz.mag.f", ld);
    bcu_set_float(s, m, ref, ctx->freq);
}

/* ============================================================
 *  PROTECTION TICK — 100ms cycle
 * ============================================================ */
static ControlHandlerResult bcu_control_handler(ControlAction action, void* parameter, MmsValue* value, bool test);

static void bcu_tick(IedServer server, IedModel *model,
                     const IedConfig *cfg, uint64_t now_ms, void *ctx_ptr)
{
    BCUContext *ctx = (BCUContext *)ctx_ptr;
    (void)now_ms;

    /* Cache cfg for the GOOSE-invoked command path (which gets NULL) */
    g_cached_cfg = cfg;

    /* ---- Trip-coil interface (TCI): pure physics, no decisions ----
     * The protection relays author the trip (GOOSE -> trip_coil_energize).
     * This block only models the coil delay until the CB mechanism opens. */
    if (ctx->coil_pending && now_ms - ctx->coil_energized_ms >=
                             (uint64_t)cfg->prot.trip_coil_delay_ms) {
        ctx->coil_pending = 0;
        ctx->xcbr1_pos = 1;  /* CB opens — physical consequence of the coil */
        printf("[TCI %s] Coil delay elapsed -> XCBR1 OPEN (trip executed)\n",
               cfg->ied_name);
    }

    /* Simulate analog noise when CB is closed */
    if (ctx->xcbr1_pos == 2) {
        float noise = (float)(rand() % 100 - 50) / 1000.0f;
        ctx->v_a = NOMINAL_V + noise * 2;
        ctx->v_b = NOMINAL_V - noise;
        ctx->v_c = NOMINAL_V + noise * 0.5f;
        ctx->i_a = NOMINAL_I + noise * 10;
        ctx->i_b = NOMINAL_I - noise * 5;
        ctx->i_c = NOMINAL_I + noise * 3;
        ctx->freq = NOMINAL_F + noise * 0.01f;
        ctx->tot_w  = 25.0f + noise;
        ctx->tot_var = 8.0f + noise * 0.5f;
    } else {
        ctx->i_a = ctx->i_b = ctx->i_c = 0;
        ctx->tot_w = ctx->tot_var = 0;
        ctx->freq = NOMINAL_F;
    }

    update_analogs(server, model, ctx);
    update_all_switch_model(server, model, ctx);

    static int handler_registered = 0;
    if (!handler_registered) {
        handler_registered = 1;
        const char *ld = get_ld(model);
        char ref[128];
        const char *nodes[] = {"XCBR1.Pos", "XSWI1.Pos", "XSWI2.Pos", "XSWI3.Pos", "XSWI4.Pos"};
        for (int i = 0; i < 5; i++) {
            snprintf(ref, sizeof(ref), "%s/%s", ld, nodes[i]);
            DataObject* node = (DataObject*)IedModel_getModelNodeByShortObjectReference(model, ref);
            if (node) {
                IedServer_setControlHandler(server, node, (ControlHandler) bcu_control_handler, ctx);
            }
        }
    }
}

/* ============================================================
 *  INTERLOCKING VALIDATION (CILO Logic)
 *  Unified check for both MMS control and CLI console commands.
 *  Returns 1 if operation is permitted, 0 if blocked.
 *  Populates *reason with the blocking rationale.
 * ============================================================ */
static int bcu_can_operate(const BCUContext *ctx, const char *sw, int is_close, const char **reason)
{
    int is_cb  = (strcmp(sw, "CB") == 0  || strcmp(sw, "XCBR1") == 0);
    int is_dsa = (strcmp(sw, "DSA") == 0 || strcmp(sw, "XSWI1") == 0);
    int is_dsb = (strcmp(sw, "DSB") == 0 || strcmp(sw, "XSWI2") == 0);
    int is_dsl = (strcmp(sw, "DSL") == 0 || strcmp(sw, "XSWI3") == 0);
    int is_es  = (strcmp(sw, "ES") == 0  || strcmp(sw, "XSWI4") == 0);

    /* --- CIRCUIT BREAKER (CB / XCBR1) ---
     * Rule 1: CB can only CLOSE if:
     *   - Earth Switch is OPEN.
     *   - 86 Lockout is RESET.
     *   - For Coupler: both Bus-A and Bus-B disconnectors are CLOSED.
     *   - For Line & Transformer bays: at least one Bus DS (DS-A or DS-B) is CLOSED,
     *     AND Line Disconnector (DS-L) is CLOSED (if bay has DSL).
     * CB OPEN is unconditionally permitted. */
    if (is_cb) {
        if (is_close) {
            if (ctx->has_es && ctx->xswi4_pos == 2) {
                if (reason) *reason = "Earth switch is closed";
                return 0;
            }
            if (ctx->active_trip_sources > 0) {
                if (reason) *reason = "86 lockout active (relay trip asserted)";
                return 0;
            }
            if (ctx->is_coupler) {
                if (ctx->xswi1_pos != 2 || ctx->xswi2_pos != 2) {
                    if (reason) *reason = "Both Bus-A and Bus-B disconnectors must be closed for Coupler CB close";
                    return 0;
                }
            } else {
                if (ctx->xswi1_pos != 2 && ctx->xswi2_pos != 2) {
                    if (reason) *reason = "Neither Bus-A nor Bus-B disconnector is closed";
                    return 0;
                }
                if (ctx->has_dsl && ctx->xswi3_pos != 2) {
                    if (reason) *reason = "Line disconnector (DS-L) must be closed before CB close";
                    return 0;
                }
            }
        }
        return 1;
    }

    /* --- BUS-A DISCONNECTOR (DS-A / XSWI1) ---
     * Rule 2: DS can CLOSE/OPEN only if CB is OPEN,
     * unless busbar transfer is active (coupler_ready == 1). */
    if (is_dsa) {
        if (ctx->xcbr1_pos == 2 && !ctx->coupler_ready && !ctx->is_coupler) {
            if (reason) *reason = "CB is closed and bus coupler is not ready";
            return 0;
        }
        if (is_close) {
            if (ctx->has_es && ctx->xswi4_pos == 2) {
                if (reason) *reason = "Earth switch is closed";
                return 0;
            }
            if (ctx->xswi2_pos == 2 && !ctx->coupler_ready && !ctx->is_coupler) {
                if (reason) *reason = "DS-B already closed and bus coupler is not ready";
                return 0;
            }
        }
        return 1;
    }

    /* --- BUS-B DISCONNECTOR (DS-B / XSWI2) ---
     * Rule 2: DS can CLOSE/OPEN only if CB is OPEN,
     * unless busbar transfer is active (coupler_ready == 1). */
    if (is_dsb) {
        if (ctx->xcbr1_pos == 2 && !ctx->coupler_ready && !ctx->is_coupler) {
            if (reason) *reason = "CB is closed and bus coupler is not ready";
            return 0;
        }
        if (is_close) {
            if (ctx->has_es && ctx->xswi4_pos == 2) {
                if (reason) *reason = "Earth switch is closed";
                return 0;
            }
            if (ctx->xswi1_pos == 2 && !ctx->coupler_ready && !ctx->is_coupler) {
                if (reason) *reason = "DS-A already closed and bus coupler is not ready";
                return 0;
            }
        }
        return 1;
    }

    /* --- LINE DISCONNECTOR (DS-L / XSWI3) ---
     * Rule 3: Line DS can only OPEN/CLOSE if CB is in the OPEN position. */
    if (is_dsl) {
        if (!ctx->has_dsl) {
            if (reason) *reason = "Line disconnector not equipped on this bay";
            return 0;
        }
        if (ctx->xcbr1_pos == 2) {
            if (reason) *reason = "CB is closed (Line DS cannot open/close on-load)";
            return 0;
        }
        if (is_close && ctx->has_es && ctx->xswi4_pos == 2) {
            if (reason) *reason = "Earth switch is closed";
            return 0;
        }
        return 1;
    }

    /* --- EARTH SWITCH (ES / XSWI4) ---
     * ES can only CLOSE if all primary switchgear on this bay is OPEN. */
    if (is_es) {
        if (!ctx->has_es) {
            if (reason) *reason = "Earth switch not equipped on this bay";
            return 0;
        }
        if (is_close && (ctx->xcbr1_pos == 2 || ctx->xswi1_pos == 2 || ctx->xswi2_pos == 2 ||
                         (ctx->has_dsl && ctx->xswi3_pos == 2))) {
            if (reason) *reason = "Primary switchgear is not fully open";
            return 0;
        }
        return 1;
    }

    if (reason) *reason = "Unknown apparatus";
    return 0;
}

static ControlHandlerResult bcu_control_handler(ControlAction action, void* parameter, MmsValue* value, bool test) {
    BCUContext *ctx = (BCUContext *)parameter;
    if (test) return CONTROL_RESULT_FAILED;
    
    int is_close = MmsValue_getBoolean(value) ? 1 : 0;
    DataObject* node = ControlAction_getControlObject(action);
    if (!node) return CONTROL_RESULT_FAILED;
    ModelNode* parentNode = ModelNode_getParent((ModelNode*)node);
    if (!parentNode) return CONTROL_RESULT_FAILED;
    const char *objName = ModelNode_getName(parentNode);
    
    printf("[BCU MMS] Control requested: %s -> %s\n", objName, is_close ? "CLOSE" : "OPEN");

    const char *reason = NULL;
    if (!bcu_can_operate(ctx, objName, is_close, &reason)) {
        printf("[BCU MMS] Interlocking BLOCKED %s %s: %s\n",
               objName, is_close ? "CLOSE" : "OPEN", reason ? reason : "Condition not met");
        return CONTROL_RESULT_FAILED;
    }

    if (strcmp(objName, "XCBR1") == 0) {
        ctx->xcbr1_pos = is_close ? 2 : 1;
        printf("[BCU MMS] XCBR1 -> %s\n", is_close ? "CLOSED" : "OPEN");
        return CONTROL_RESULT_OK;
    } else if (strcmp(objName, "XSWI1") == 0) {
        ctx->xswi1_pos = is_close ? 2 : 1;
        printf("[BCU MMS] XSWI1 -> %s\n", is_close ? "CLOSED" : "OPEN");
        return CONTROL_RESULT_OK;
    } else if (strcmp(objName, "XSWI2") == 0) {
        ctx->xswi2_pos = is_close ? 2 : 1;
        printf("[BCU MMS] XSWI2 -> %s\n", is_close ? "CLOSED" : "OPEN");
        return CONTROL_RESULT_OK;
    } else if (strcmp(objName, "XSWI3") == 0) {
        ctx->xswi3_pos = is_close ? 2 : 1;
        printf("[BCU MMS] XSWI3 -> %s\n", is_close ? "CLOSED" : "OPEN");
        return CONTROL_RESULT_OK;
    } else if (strcmp(objName, "XSWI4") == 0) {
        ctx->xswi4_pos = is_close ? 2 : 1;
        printf("[BCU MMS] XSWI4 -> %s\n", is_close ? "CLOSED" : "OPEN");
        return CONTROL_RESULT_OK;
    }
    
    return CONTROL_RESULT_FAILED;
}

/* ============================================================
 *  CONSOLE COMMANDS — switchgear operations
 * ============================================================ */
static int bcu_command(IedServer server, IedModel *model,
                       const IedConfig *cfg, const char *cmd, void *ctx_ptr)
{
    BCUContext *ctx = (BCUContext *)ctx_ptr;

    /* ---- Trip-coil interface commands (invoked via GOOSE RX by
     *      goose_handler; also usable from console) ----
     *      NOTE: GOOSE path passes cfg == NULL; fall back to the
     *      tick-cached config for display values only. */
    const IedConfig *dcfg = cfg ? cfg : g_cached_cfg;
    const char *tci_name = dcfg ? dcfg->ied_name : "(via GOOSE)";
    int tci_delay = dcfg ? dcfg->prot.trip_coil_delay_ms : 0;
    if (strcmp(cmd, "trip_coil_energize") == 0) {
        if (!ctx->coil_pending) {
            ctx->coil_pending = 1;
            ctx->coil_energized_ms = Hal_getTimeInMs();
            printf("[TCI %s] Trip coil ENERGIZED (opens in %dms)\n",
                   tci_name, tci_delay);
        }
        ctx->active_trip_sources++;
        printf("[TCI %s] Trip sources active: %d\n",
               tci_name, ctx->active_trip_sources);
        return 1;
    }
    if (strcmp(cmd, "trip_coil_release") == 0) {
        if (ctx->active_trip_sources > 0)
            ctx->active_trip_sources--;
        if (ctx->active_trip_sources == 0)
            printf("[TCI %s] All trip sources released — 86 lockout reset\n",
                   tci_name);
        else
            printf("[TCI %s] Trip sources active: %d (lockout still set)\n",
                   tci_name, ctx->active_trip_sources);
        return 1;
    }
    if (strcmp(cmd, "coupler_ready_set") == 0) {
        ctx->coupler_ready = 1;
        return 1;
    }
    if (strcmp(cmd, "coupler_ready_clear") == 0) {
        ctx->coupler_ready = 0;
        return 1;
    }
    
    if (strstr(cmd, "open ") == cmd || strstr(cmd, "close ") == cmd) {
        int is_close = (cmd[0] == 'c');
        const char *obj = cmd + (is_close ? 6 : 5);
        
        if (strcmp(obj, "CB") != 0 && strcmp(obj, "DSA") != 0 &&
            strcmp(obj, "DSB") != 0 && strcmp(obj, "DSL") != 0 &&
            strcmp(obj, "ES") != 0) {
            return 0; // Unknown switch
        }
        
        const char *reason = NULL;
        if (!bcu_can_operate(ctx, obj, is_close, &reason)) {
            printf("[BCU %s] BLOCKED %s %s: %s\n",
                   cfg ? cfg->ied_name : "BCU", obj, is_close ? "CLOSE" : "OPEN",
                   reason ? reason : "Interlocking condition not met");
            return 1;
        }

        if (strcmp(obj, "CB") == 0) {
            ctx->xcbr1_pos = is_close ? 2 : 1;
        } else if (strcmp(obj, "DSA") == 0) {
            ctx->xswi1_pos = is_close ? 2 : 1;
        } else if (strcmp(obj, "DSB") == 0) {
            ctx->xswi2_pos = is_close ? 2 : 1;
        } else if (strcmp(obj, "DSL") == 0) {
            ctx->xswi3_pos = is_close ? 2 : 1;
        } else if (strcmp(obj, "ES") == 0) {
            ctx->xswi4_pos = is_close ? 2 : 1;
        }
        
        update_all_switch_model(server, model, ctx);
        printf("[BCU %s] %s -> %s\n", cfg ? cfg->ied_name : "BCU", obj, is_close ? "CLOSED" : "OPEN");
        return 1;
    }

    if (strcmp(cmd, "status") == 0) {
        printf("  CB (XCBR1): %s\n", ctx->xcbr1_pos == 2 ? "CLOSED" : "OPEN");
        printf("  DS-A (XSWI1): %s\n", ctx->xswi1_pos == 2 ? "CLOSED" : "OPEN");
        printf("  DS-B (XSWI2): %s\n", ctx->xswi2_pos == 2 ? "CLOSED" : "OPEN");
        if (ctx->has_dsl) printf("  DS-L (XSWI3): %s\n", ctx->xswi3_pos == 2 ? "CLOSED" : "OPEN");
        if (ctx->has_es) printf("  ES (XSWI4): %s\n", ctx->xswi4_pos == 2 ? "CLOSED" : "OPEN");
        printf("  Trip sources: %d\n", ctx->active_trip_sources);
        printf("  86 Lockout:   %s\n", ctx->active_trip_sources > 0 ? "ACTIVE" : "RESET");
        printf("  CouplerReady: %s\n", ctx->coupler_ready ? "TRUE" : "FALSE");
        printf("  V_a:    %.1f kV\n", ctx->v_a);
        printf("  I_a:    %.1f A\n", ctx->i_a);
        printf("  P:      %.1f MW\n", ctx->tot_w);
        return 1;
    }
    if (strncmp(cmd, "help", 4) == 0) {
        printf("C264 BCU commands:\n");
        printf("  open <SW>   — Open switch (CB, DSA, DSB, DSL, ES)\n");
        printf("  close <SW>  — Close switch (CB, DSA, DSB, DSL, ES)\n");
        printf("  status      — Show switchgear state\n");
        printf("  (Trip authority: protection relays only. The BCU exposes\n");
        printf("   a trip-coil interface; relays trip via GOOSE -> TCI.)\n");
        return 1;
    }
    return 0;
}

/* ---- MAIN ---- */
int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <config.json>\n", argv[0]);
        return 1;
    }
    BCUContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.xcbr1_pos = 2;  /* Start with CB closed (normal state) */
    ctx.xswi1_pos = 2;  /* Start with DS-A closed */
    ctx.xswi2_pos = 1;  /* Start with DS-B open */
    ctx.xswi3_pos = 2;  /* Line DS closed */
    ctx.xswi4_pos = 1;  /* ES open */
    ctx.coupler_ready = 0; /* Default to 0, updated by GOOSE */
    
    ctx.is_coupler = (strstr(argv[1], "E04_") != NULL);
    ctx.has_dsl = (strstr(argv[1], "E04_") == NULL);
    ctx.has_es = (strstr(argv[1], "E01_") == NULL && strstr(argv[1], "E02_") == NULL && strstr(argv[1], "E03_") == NULL && strstr(argv[1], "E04_") == NULL);
    
    ctx.v_a = ctx.v_b = ctx.v_c = NOMINAL_V;
    ctx.i_a = ctx.i_b = ctx.i_c = NOMINAL_I;
    ctx.freq = NOMINAL_F;
    ctx.tot_w = 25.0f;
    ctx.tot_var = 8.0f;

    return ied_server_run(&MODEL_EXTERN, argv[1], bcu_tick, bcu_command, &ctx);
}
