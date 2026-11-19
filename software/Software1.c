// TC3 + Karatsuba HQC-128 Linux userspace MMIO controller for KR260
// Based on CRGA.v and vect_mul_top.v supplied by the project author.
// Copyright (c) 2026. Example code; use only on the intended KR260 PL image.
// Linux/AArch64: gcc -O2 -std=c11 -Wall -Wextra -o main Software1.c
// New CRGA AXI4-Lite staging map only. XSA: 0x80000000..0x80000fff.
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
#define REGION_SIZE  UINT64_C(0x00001000)
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
// AXI-Lite staging registers; native addresses are packed-word indices.
static unsigned g_bridge_timeout_ms = 5000;
static void native_access(bool read_op, uint32_t address, uint32_t data[4]) {
    uint32_t status = reg32(0x28);
    if (status & 1u) {
        fputs("Bridge already busy; use only one controller process.\n", stderr); exit(10);
    }
    put32(0x00, address);
    if (!read_op) {
        for (unsigned j=0;j<4;j++) put32(0x10+4u*j, data[j]);
        put32(0x20, 0xffffu);
    }
    put32(0x24, read_op ? 2u : 1u);
    uint64_t begin=monotonic_ms();
    for (;;) {
        status=reg32(0x28);
        if (!(status & 1u) && (status & 2u)) break;
        if (monotonic_ms()-begin >= g_bridge_timeout_ms) {
            fprintf(stderr,"Bridge timeout native=0x%05" PRIx32 " status=%08" PRIx32 "\n",address,status);
            exit(11);
        }
    }
    if (status & 4u) {
        fprintf(stderr,"Bridge error native=0x%05" PRIx32 " code=%" PRIu32 "\n",address,reg32(0x2c));
        exit(12);
    }
    if (read_op) for(unsigned j=0;j<4;j++) data[j]=reg32(0x30+4u*j);
}
static uint32_t native_control(unsigned index) {
    uint32_t d[4]={0}; native_access(true,0x40000u+index,d); return d[0];
}
static uint64_t counter64(unsigned low, unsigned high) {
    uint32_t lo=native_control(low), hi=native_control(high);
    return ((uint64_t)hi<<32)|lo;
}
static void write_operand(uint32_t bank, const uint64_t x[WORDS]) {
    for(unsigned p=0;p<(WORDS+1u)/2u;p++) {
        uint64_t lo=x[2*p], hi=(2*p+1<WORDS)?x[2*p+1]:0;
        uint32_t d[4]={(uint32_t)lo,(uint32_t)(lo>>32),(uint32_t)hi,(uint32_t)(hi>>32)};
        native_access(false,bank+p,d);
    }
}
static void read_operand(uint32_t bank, uint64_t x[WORDS]) {
    for(unsigned p=0;p<(WORDS+1u)/2u;p++) {
        uint32_t d[4]={0}; native_access(true,bank+p,d);
        x[2*p]=((uint64_t)d[1]<<32)|d[0];
        if(2*p+1<WORDS) x[2*p+1]=((uint64_t)d[3]<<32)|d[2];
    }
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
    // Read-only probe: never issue a native bridge command here.
    uint32_t size=reg32(0x40), n=reg32(0x44), status=reg32(0x28);
    printf("PROFILE: size_words=%" PRIu32 " poly_bits=%" PRIu32 " (expected %u and %u)\n",size,n,WORDS,POLY_BITS);
    printf("BRIDGE: %08" PRIx32 " [busy=%u complete=%u error=%u]\n",status,!!(status&1),!!(status&2),!!(status&4));
    if(size!=WORDS || n!=POLY_BITS) {
        fputs("ERROR: profile mismatch; no writes performed. Check new bitstream/base/PS clock/reset.\n",stderr);
        return 1;
    }
    puts("PROBE PASS: new AXI-Lite register map matches HQC-128");
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
    uint64_t a[WORDS],b[WORDS],expected[WORDS],ra[WORDS],rb[WORDS],c[WORDS];
    g_bridge_timeout_ms=timeout_ms;
    uint32_t status=native_control(0);
    if(status&STATUS_BUSY) { fputs("ABORT: core busy before load\n",stderr); return 3; }
    make_vectors(a,b,random_case); reference_gf2(a,b,expected);
    puts(random_case?"TEST: deterministic random":"TEST: A=1, sparse B");
    write_operand(0,a); write_operand(0x10000,b);
    read_operand(0,ra); read_operand(0x10000,rb);
    unsigned errors=0;
    for(unsigned i=0;i<WORDS;i++) if(ra[i]!=a[i] || rb[i]!=b[i]) {
        if(errors<8) fprintf(stderr,"INPUT mismatch word %u\n",i);
        errors++;
    }
    if(errors) { fprintf(stderr,"ABORT: %u input errors; START not issued\n",errors); return 2; }
    puts("INPUT READBACK PASS");
    uint32_t start_data[4]={1,0,0,0};
    uint64_t begin=monotonic_ms();
    native_access(false,0x40000,start_data);
    // Bridge completion is delayed until native START has been accepted by
    // the core and old DONE cleared. A fast new run may already have finished;
    // do not require Linux polling to observe the transient DONE=0 interval.
    for(;;) {
        status=native_control(0);
        if((status&STATUS_DONE) && !(status&STATUS_BUSY)) break;
        if(monotonic_ms()-begin>=timeout_ms) {
            fprintf(stderr,"Core timeout status=%08" PRIx32 "\n",status); return 4;
        }
        tiny_pause();
    }
    printf("DONE: native_status=%08" PRIx32 " host_elapsed_ms=%" PRIu64 "\n",status,monotonic_ms()-begin);
    read_operand(0x20000,c);
    errors=0;
    for(unsigned i=0;i<WORDS;i++) if(c[i]!=expected[i]) {
        if(errors<12) fprintf(stderr,"C[%u] got=%016" PRIx64 " expected=%016" PRIx64 "\n",i,c[i],expected[i]);
        errors++;
    }
    if(dump_abc) print_abc(a,b,c,nonzero_only);
    uint64_t total=counter64(2,3), core=counter64(4,5), red=counter64(6,7);
    printf("COUNTERS: total=%" PRIu64 " core=%" PRIu64 " reduction=%" PRIu64 " cycles\n",total,core,red);
    if(clock_hz>0) printf("Processing time %.6f ms at supplied clock %.6f MHz\n",(double)total*1000.0/clock_hz,clock_hz/1e6);
    if(errors) { fprintf(stderr,"TEST FAIL: %u output mismatches\n",errors); return 5; }
    puts("TC3 KARATSUBA TEST PASS"); return 0;
}
static void usage(const char *p) {
    printf("Usage: %s --selftest | --probe | --run [OPTIONS]\n", p);
    puts("  --selftest          Check software GF(2) reference without accessing FPGA");
    puts("  --probe             Read new wrapper profile/bridge status only (no writes)");
    puts("  --run               Write test vectors, START, poll DONE and compare output");
    puts("  --random            With --run, use deterministic random operands instead of A=1");
    puts("  --dump              Show all 277 A/B/C words in hex (default for --run)");
    puts("  --nonzero-only      Show only word indices where A, B, or C is nonzero");
    puts("  --no-dump           Disable A/B/C table to keep terminal output short");
    puts("  --base 0x80000000  CRGA physical base from Vivado Address Editor (/dev/mem only)");
    puts("  --uio /dev/uio0     Use UIO instead of /dev/mem; verify correct UIO device/map size");
    puts("  --timeout-ms 5000  Polling timeout in milliseconds (default 5000)");
    puts("  --clock-hz 96000000 Print time derived from the actual PL clock, if known");
    puts("NOTE: Run only on the KR260 with correct PL bitstream, PS clocks, reset and 4 KiB AXI-Lite map.");
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
    atexit(close_mmio);
    int ret = probe_profile();
    if (ret == 0 && mode == RUN)
        ret = run_fpga(random_case, timeout_ms, clock_hz, dump_abc, nonzero_only);
    close_mmio();
    return ret;
}
