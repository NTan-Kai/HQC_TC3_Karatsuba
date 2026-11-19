// TC3 + Karatsuba HQC-128 Linux userspace MMIO controller for KR260
// Based on CRGA.v and vect_mul_top.v supplied by the project author.
// Copyright (c) 2026. Example code; use only on the intended KR260 PL image.
// Linux/AArch64: gcc -O2 -std=c11 -Wall -Wextra -o tc3_control tc3_karatsuba_control.c
// NOTE: The custom AXI4 slave RTL still needs handshake/latency verification.
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_BASE UINT64_C(0x80000000)
#define REGION_SIZE  UINT64_C(0x01000000)
#define A_OFF        UINT64_C(0x00000000)
#define B_OFF        UINT64_C(0x00100000)
#define O_OFF        UINT64_C(0x00200000)
#define CTRL_OFF     UINT64_C(0x00400000)
#define CTRL_STATUS  UINT64_C(0x00)
#define CTRL_PROFILE UINT64_C(0x10)
#define CTRL_TOTAL_L UINT64_C(0x20)
#define CTRL_TOTAL_H UINT64_C(0x30)
#define CTRL_CORE_L  UINT64_C(0x40)
#define CTRL_CORE_H  UINT64_C(0x50)
#define CTRL_RED_L   UINT64_C(0x60)
#define CTRL_RED_H   UINT64_C(0x70)
#define STATUS_BUSY  (1u << 0)
#define STATUS_DONE  (1u << 1)
#define WORDS        277u
#define POLY_BITS    17669u
#define LAST_MASK    UINT64_C(0x1F)
#define PRODUCT_WORDS (2u * WORDS)

static uint8_t *g_mmio;
static void *g_map;
static size_t g_map_size;
static int g_fd = -1;
static bool g_use_uio;
static uint64_t g_active_base = DEFAULT_BASE;

static uint64_t monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        perror("clock_gettime");
        exit(1);
    }
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}
static void tiny_pause(void) {
    const struct timespec req = {.tv_sec = 0, .tv_nsec = 1000000};
    (void)nanosleep(&req, NULL);
}
static inline uint32_t reg32(uint64_t offset) {
    uint32_t value = *(volatile uint32_t *)(void *)(g_mmio + offset);
    atomic_thread_fence(memory_order_seq_cst);
    return value;
}
static inline void put32(uint64_t offset, uint32_t value) {
    *(volatile uint32_t *)(void *)(g_mmio + offset) = value;
    atomic_thread_fence(memory_order_seq_cst);
}
static inline uint64_t reg64(uint64_t offset) {
    uint64_t value = *(volatile uint64_t *)(void *)(g_mmio + offset);
    atomic_thread_fence(memory_order_seq_cst);
    return value;
}
static inline void put64(uint64_t offset, uint64_t value) {
    *(volatile uint64_t *)(void *)(g_mmio + offset) = value;
    atomic_thread_fence(memory_order_seq_cst);
}
// Each 128-bit packed RAM entry contains two 64-bit words; offsets differ by 8 bytes.
static uint64_t word_offset(uint64_t bank, unsigned i) {
    return bank + UINT64_C(16) * (i / 2u) + UINT64_C(8) * (i % 2u);
}
static uint64_t counter64(uint64_t low, uint64_t high) {
    uint32_t lo = reg32(CTRL_OFF + low);
    uint32_t hi = reg32(CTRL_OFF + high);
    return ((uint64_t)hi << 32) | lo;
}
static uint64_t prng_next(uint64_t *state) {
    uint64_t x = *state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    return x * UINT64_C(2685821657736338717);
}
static void make_vectors(uint64_t a[WORDS], uint64_t b[WORDS], bool random_case) {
    memset(a, 0, WORDS * sizeof(*a));
    memset(b, 0, WORDS * sizeof(*b));
    if (!random_case) {
        a[0] = 1;
        b[0] = 1;
        b[1] = 1;
        b[WORDS - 1] = 5;
    } else {
        uint64_t seed = UINT64_C(0x98ABCD1234576123);
        for (unsigned i = 0; i < WORDS; ++i) {
            a[i] = prng_next(&seed);
            b[i] = prng_next(&seed);
        }
        a[WORDS - 1] &= LAST_MASK;
        b[WORDS - 1] &= LAST_MASK;
    }
}
// Independent software reference: carry-less polynomial multiplication and reduction x^n = 1.
static void reference_gf2(const uint64_t a[WORDS], const uint64_t b[WORDS], uint64_t out[WORDS]) {
    uint64_t product[PRODUCT_WORDS] = {0};
    memset(out, 0, WORDS * sizeof(*out));
    for (unsigned j = 0; j < WORDS; ++j) {
        uint64_t bits = b[j];
        while (bits != 0) {
            unsigned shift = (unsigned)__builtin_ctzll(bits);
            bits &= bits - 1u;
            for (unsigned i = 0; i < WORDS; ++i) {
                uint64_t x = a[i];
                if (x == 0) continue;
                product[i + j] ^= x << shift;
                if (shift != 0)
                    product[i + j + 1u] ^= x >> (64u - shift);
            }
        }
    }
    // Polynomial degree <= 2*(POLY_BITS-1); fold exponents >= n back by n.
    for (unsigned bit = 0; bit <= 2u * (POLY_BITS - 1u); ++bit) {
        if ((product[bit / 64u] >> (bit % 64u)) & UINT64_C(1)) {
            unsigned dest = (bit >= POLY_BITS) ? bit - POLY_BITS : bit;
            out[dest / 64u] ^= UINT64_C(1) << (dest % 64u);
        }
    }
    out[WORDS - 1] &= LAST_MASK;
}
static int self_test(void) {
    uint64_t a[WORDS], b[WORDS], expected[WORDS];
    make_vectors(a, b, false);
    reference_gf2(a, b, expected);
    for (unsigned i = 0; i < WORDS; i++) {
        if (expected[i] != b[i]) {
            fprintf(stderr, "Self-test mismatch at word %u\n", i);
            return 1;
        }
    }
    make_vectors(a, b, true);
    reference_gf2(a, b, expected);
    if ((expected[WORDS - 1] & ~LAST_MASK) != 0) {
        fprintf(stderr, "Reference reduction upper-bit error\n");
        return 1;
    }
    puts("SELFTEST PASS: GF(2) identity/reference and HQC-128 last-word mask");
    return 0;
}
static void close_mmio(void) {
    if (g_map != NULL && g_map != MAP_FAILED) munmap(g_map, g_map_size);
    if (g_fd >= 0) close(g_fd);
    g_map = NULL;
    g_mmio = NULL;
    g_fd = -1;
}
static int open_mmio(uint64_t base, const char *uio_device) {
    g_active_base = base;
    if (uio_device != NULL) {
        g_fd = open(uio_device, O_RDWR | O_SYNC);
        g_use_uio = true;
        if (g_fd < 0) { perror(uio_device); return -1; }
        g_map_size = REGION_SIZE;
        g_map = mmap(NULL, g_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, g_fd, 0);
        if (g_map == MAP_FAILED) { perror("mmap UIO"); close_mmio(); return -1; }
        g_mmio = (uint8_t *)g_map;
        printf("Mapped %s (UIO offset 0; confirm its map0 address/size matches CRGA)\n", uio_device);
    } else {
        long page = sysconf(_SC_PAGE_SIZE);
        if (page <= 0 || base % (uint64_t)page != 0) {
            fprintf(stderr, "Base must be page-aligned for /dev/mem\n");
            return -1;
        }
        g_fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (g_fd < 0) { perror("open /dev/mem (root usually required)"); return -1; }
        g_map_size = REGION_SIZE;
        g_map = mmap(NULL, g_map_size, PROT_READ | PROT_WRITE, MAP_SHARED, g_fd, (off_t)base);
        if (g_map == MAP_FAILED) { perror("mmap /dev/mem"); close_mmio(); return -1; }
        g_mmio = (uint8_t *)g_map;
        printf("Mapped CRGA: base=0x%" PRIx64 ", bytes=0x%" PRIx64 "\n", base, (uint64_t)REGION_SIZE);
    }
    return 0;
}
static int probe_profile(void) {
    // BEWARE: a broken AXI slave, missing PS clock, or invalid map may hang the read.
    uint32_t size = reg32(CTRL_OFF + CTRL_PROFILE);
    uint32_t rem = reg32(CTRL_OFF + CTRL_PROFILE + 4u);
    uint32_t status = reg32(CTRL_OFF + CTRL_STATUS);
    printf("PROFILE: size_words=%u rem_bits=%u (expected %u and %u)\n",
           size, rem, WORDS, POLY_BITS % 64u);
    printf("STATUS: 0x%08" PRIx32 " [busy=%u done=%u fsm=%u]\n",
           status, !!(status & STATUS_BUSY), !!(status & STATUS_DONE), (status >> 2) & 7u);
    if (size != WORDS || rem != (POLY_BITS % 64u)) {
        fputs("ERROR: CRGA profile mismatch or broken AXI read path; no writes performed.\n", stderr);
        return 1;
    }
    puts("PROBE PASS: profile registers match HQC-128");
    return 0;
}
// Print actual FPGA results next to the input operands, one 64-bit word per row.
// --nonzero-only suppresses lines where all three values are zero.
static void print_abc(const uint64_t a[WORDS], const uint64_t b[WORDS],
                      const uint64_t c[WORDS], bool nonzero_only) {
    printf("\n===== A, B, C (%u words, 64-bit hexadecimal) =====\n", WORDS);
    puts(" Word |                A                |                B                |                C");
    puts("------+---------------------------------+---------------------------------+---------------------------------");
    unsigned shown = 0;
    for (unsigned i = 0; i < WORDS; ++i) {
        if (nonzero_only && a[i] == 0 && b[i] == 0 && c[i] == 0)
            continue;
        printf(" %4u | 0x%016" PRIX64 "              | 0x%016" PRIX64
               "              | 0x%016" PRIX64 "\n", i, a[i], b[i], c[i]);
        ++shown;
    }
    printf("Displayed %u/%u words (%s).\n", shown, WORDS,
           nonzero_only ? "nonzero only" : "all words");
    printf("A(x), B(x), C(x) each contain %u significant bits.\n\n", POLY_BITS);
}

static int run_fpga(bool random_case, unsigned timeout_ms, double clock_hz,
                    bool dump_abc, bool nonzero_only) {
    uint64_t a[WORDS], b[WORDS], expected[WORDS];
    make_vectors(a, b, random_case);
    reference_gf2(a, b, expected);
    printf("TEST PATTERN: %s\n", random_case ? "deterministic pseudorandom" : "A(x)=1, sparse B(x)");
    printf("Writing %u 64-bit words per operand via 128-bit packed address window...\n", WORDS);
    for (unsigned i = 0; i < WORDS; ++i) {
        put64(word_offset(A_OFF, i), a[i]);
        put64(word_offset(B_OFF, i), b[i]);
    }
    atomic_thread_fence(memory_order_seq_cst);
    unsigned errors = 0;
    for (unsigned i = 0; i < WORDS; ++i) {
        uint64_t got_a = reg64(word_offset(A_OFF, i));
        uint64_t got_b = reg64(word_offset(B_OFF, i));
        if (got_a != a[i] || got_b != b[i]) {
            if (errors < 8)
                printf("INPUT ERROR word[%u]: A got=%016" PRIx64 " expected=%016" PRIx64
                       ", B got=%016" PRIx64 " expected=%016" PRIx64 "\n",
                       i, got_a, a[i], got_b, b[i]);
            ++errors;
        }
    }
    if (errors) {
        fprintf(stderr, "ABORT: %u input readback mismatches; START was NOT issued.\n", errors);
        return 2;
    }
    puts("INPUT READBACK PASS");
    uint32_t status_before = reg32(CTRL_OFF + CTRL_STATUS);
    if (status_before & STATUS_BUSY) {
        fprintf(stderr, "ABORT: core already busy (status=%08" PRIx32 ")\n", status_before);
        return 3;
    }
    printf("Issuing START at 0x%08" PRIx64 "...\n", (uint64_t)(g_active_base + CTRL_OFF));
    put32(CTRL_OFF + CTRL_STATUS, 1u);
    bool saw_done_cleared = !(status_before & STATUS_DONE);
    uint64_t begin = monotonic_ms();
    uint32_t status = 0;
    for (;;) {
        status = reg32(CTRL_OFF + CTRL_STATUS);
        if (!(status & STATUS_DONE)) saw_done_cleared = true;
        if (saw_done_cleared && (status & STATUS_DONE)) break;
        if (monotonic_ms() - begin >= timeout_ms) {
            fprintf(stderr, "TIMEOUT: status=0x%08" PRIx32 " after %u ms\n", status, timeout_ms);
            return 4;
        }
        tiny_pause();
    }
    printf("DONE: status=0x%08" PRIx32 ", elapsed_host_ms=%" PRIu64 "\n",
           status, monotonic_ms() - begin);
    errors = 0;
    uint64_t c[WORDS];
    for (unsigned i = 0; i < WORDS; ++i) {
        uint64_t got = reg64(word_offset(O_OFF, i));
        c[i] = got;
        if (got != expected[i]) {
            if (errors < 12)
                printf("OUTPUT ERROR C[%u]: got=%016" PRIx64 " expected=%016" PRIx64 "\n",
                       i, got, expected[i]);
            ++errors;
        }
    }
    if (dump_abc) print_abc(a, b, c, nonzero_only);
    uint64_t total = counter64(CTRL_TOTAL_L, CTRL_TOTAL_H);
    uint64_t core = counter64(CTRL_CORE_L, CTRL_CORE_H);
    uint64_t reduction = counter64(CTRL_RED_L, CTRL_RED_H);
    printf("COUNTERS: total=%" PRIu64 ", core=%" PRIu64 ", reduction=%" PRIu64 " cycles\n",
           total, core, reduction);
    if (clock_hz > 0)
        printf("Core-measured processing time: %.3f ms (clock %.3f MHz)\n",
               (double)total * 1000. / clock_hz, clock_hz / 1e6);
    if (errors) {
        fprintf(stderr, "TEST FAIL: %u output mismatches; inspect CRGA AXI RDATA/latency and datapath.\n", errors);
        return 5;
    }
    puts("TC3 KARATSUBA TEST PASS");
    return 0;
}
static void usage(const char *p) {
    printf("Usage: %s --selftest | --probe | --run [OPTIONS]\n", p);
    puts("  --selftest          Check software GF(2) reference without accessing FPGA");
    puts("  --probe             Read CRGA profile/status only (no writes; may hang on broken AXI)");
    puts("  --run               Write test vectors, START, poll DONE and compare output");
    puts("  --random            With --run, use deterministic random operands instead of A=1");
    puts("  --dump              Show all 277 A/B/C words in hex (default for --run)");
    puts("  --nonzero-only      Show only word indices where A, B, or C is nonzero");
    puts("  --no-dump           Disable A/B/C table to keep terminal output short");
    puts("  --base 0x80000000  CRGA physical base from Vivado Address Editor (/dev/mem only)");
    puts("  --uio /dev/uio0     Use UIO instead of /dev/mem; verify correct UIO device/map size");
    puts("  --timeout-ms 5000  Polling timeout in milliseconds (default 5000)");
    puts("  --clock-hz 96000000 Print time derived from the actual PL clock, if known");
    puts("NOTE: Run only on the KR260 with correct PL bitstream, PS clocks, reset and 16 MiB map.");
    puts("NOTE: This RTL's AXI read and WSTRB handling has not been fully verified.");
}
int main(int argc, char **argv) {
    enum { NONE, SELFTEST, PROBE, RUN } mode = NONE;
    bool random_case = false;
    bool dump_abc = true;
    bool nonzero_only = false;
    unsigned timeout_ms = 5000;
    uint64_t base = DEFAULT_BASE;
    double clock_hz = 0;
    const char *uio = NULL;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--selftest")) mode = SELFTEST;
        else if (!strcmp(argv[i], "--probe")) mode = PROBE;
        else if (!strcmp(argv[i], "--run")) mode = RUN;
        else if (!strcmp(argv[i], "--random")) random_case = true;
        else if (!strcmp(argv[i], "--dump")) dump_abc = true;
        else if (!strcmp(argv[i], "--no-dump")) dump_abc = false;
        else if (!strcmp(argv[i], "--nonzero-only")) { dump_abc = true; nonzero_only = true; }
        else if (!strcmp(argv[i], "--uio") && ++i < argc) uio = argv[i];
        else if (!strcmp(argv[i], "--base") && ++i < argc) {
            char *end = NULL;
            errno = 0;
            base = strtoull(argv[i], &end, 0);
            if (errno || !end || *end) { fprintf(stderr, "Invalid --base\n"); return 2; }
        }
        else if (!strcmp(argv[i], "--timeout-ms") && ++i < argc) {
            char *end = NULL;
            errno = 0;
            unsigned long val = strtoul(argv[i], &end, 0);
            if (errno || !end || *end || val == 0 || val > 3600000UL) {
                fprintf(stderr, "Invalid --timeout-ms\n"); return 2;
            }
            timeout_ms = (unsigned)val;
        }
        else if (!strcmp(argv[i], "--clock-hz") && ++i < argc) {
            char *end = NULL;
            errno = 0;
            clock_hz = strtod(argv[i], &end);
            if (errno || !end || *end || clock_hz <= 0) {
                fprintf(stderr, "Invalid --clock-hz\n"); return 2;
            }
        }
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) { usage(argv[0]); return 0; }
        else { fprintf(stderr, "Unknown or incomplete option: %s\n", argv[i]); usage(argv[0]); return 2; }
    }
    if (mode == NONE || (random_case && mode != RUN)) { usage(argv[0]); return 2; }
    if (mode == SELFTEST) return self_test();
    if (geteuid() != 0 && !uio) {
        fprintf(stderr, "Use sudo for /dev/mem or specify a configured --uio device.\n");
        return 2;
    }
    puts("WARNING: Any MMIO access can hang if the FPGA AXI target is not clocked/responding.");
    if (open_mmio(base, uio) != 0) return 2;
    if (g_use_uio && base != DEFAULT_BASE)
        puts("NOTE: --base is ignored for UIO; ensure the selected UIO map is CRGA.");
    int ret = probe_profile();
    if (ret == 0 && mode == RUN)
        ret = run_fpga(random_case, timeout_ms, clock_hz, dump_abc, nonzero_only);
    close_mmio();
    return ret;
}
