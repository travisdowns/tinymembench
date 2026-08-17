/*
 * Working-set sweep: dependent-load latency and streaming read bandwidth
 * measured across a ladder of footprints, at a chosen thread count.
 *
 * The point of the sweep is to locate cache boundaries rather than to report a
 * single number for one buffer size, which is what the bandwidth and latency
 * benchmarks in main.c do. On a machine whose last level cache is shared with
 * other tenants, the footprint at which latency leaves its plateau measures how
 * much of that cache this tenant actually has, which can be a small fraction of
 * the size the CPU advertises.
 */

#ifndef __SWEEP_H__
#define __SWEEP_H__

#include <stddef.h>

typedef struct
{
    size_t size;      /* aggregate footprint in bytes, summed over threads */
    int    threads;
    int    trial;     /* 1-based */
    double lat_ns;    /* mean per-thread dependent-load latency */
    double bw_gbs;    /* aggregate streaming read bandwidth, GB/s */
} sweep_result;

typedef struct
{
    int  threads;     /* <= 0 means every online cpu */
    int  pin;         /* pin thread i to cpu i */
    int  trials;      /* repeats per footprint */
    long hops;        /* timed dependent loads per thread per trial */
    long warm_hops;   /* cap on the warm pass; <= 0 means a full traversal */
    int  hugepages;   /* 1 = MADV_HUGEPAGE, -1 = MADV_NOHUGEPAGE, 0 = default */
    size_t min_size;  /* skip footprints below this, 0 for the default ladder */
    size_t max_size;  /* skip footprints above this, 0 for no limit */
} sweep_config;

/*
 * Runs the sweep, calling emit() once per (footprint, trial). emit() returning
 * zero aborts the sweep. Returns the number of results emitted, or -1 on error.
 */
int sweep_run(const sweep_config *cfg,
              int (*emit)(const sweep_result *result, void *ctx), void *ctx);

/* Fills cfg with the defaults documented in the usage text. */
void sweep_defaults(sweep_config *cfg);

#endif
