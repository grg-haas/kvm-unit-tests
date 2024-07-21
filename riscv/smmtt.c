#include <libcflat.h>
#include <string.h>
#include <asm/sbi.h>
#include <devicetree.h>
#include "asm/smp.h"
#include "vmalloc.h"

/* Shared functionality */

__attribute__((aligned(0x1000)))
static unsigned char shmem[0x1000];

#define RISCV_MSG_ID_SMM_VERSION		0x1
#define RISCV_MSG_ID_SMM_COMMUNICATE	0x2
#define RISCV_MSG_ID_SMM_EVENT_COMPLETE 0x3

static void initialize_comms(void)
{
    sbi_ecall(SBI_EXT_MPXY, SBI_EXT_MPXY_SET_SHMEM, sizeof(shmem),
            (unsigned long) &shmem[0], (unsigned long) &shmem[0x1000 - 1], 0b00, 0, 0);

    sbi_ecall(SBI_EXT_MPXY, SBI_EXT_MPXY_SEND_MSG_NO_RESP, 0x1001,
            RISCV_MSG_ID_SMM_EVENT_COMPLETE, 0, 0, 0, 0);
}

/* Primary domain */

static void handle_primary(void)
{
    /* 1. This sequence transfers control over to the secondary */
    initialize_comms();

    // 5. Pop over to secondary domain again, this time with SMM_COMMUNICATE
    sbi_ecall(SBI_EXT_MPXY, SBI_EXT_MPXY_SEND_MSG_NO_RESP, 0x1001,
              RISCV_MSG_ID_SMM_COMMUNICATE, 0, 0, 0, 0);
}

/* Secondary domain */

static volatile bool can_load;
static void load_handler(struct pt_regs *regs)
{
    can_load = false;
    regs->epc += 2;
}

static volatile bool can_store;
static void store_handler(struct pt_regs *regs)
{
    can_store = false;
    regs->epc += 2;
}

/* If we hit either of the exceptions below, that means that we are not allowed
 * to execute this specific area of memory. Since we probe regions by jumping to
 * them, we eseentially want to execute a ret to get back to where we want to be
 * in the test program. Therefore, we should sret back to the value in the ra
 * register rather than epc + 2 */

static volatile bool can_exec;
static void exec_handler(struct pt_regs *regs)
{
    can_exec = false;
    regs->epc = regs->ra;
}

static void inst_handler(struct pt_regs *regs)
{
    /* If we can pass access checks to get here,
     * we would be able to execute if valid */
    can_exec = true;
    regs->epc = regs->ra;
}

#define NUM_TEST_REGIONS 16

struct test_region {
    const char *name;
    uintptr_t base;
    size_t size;
    uint32_t perms;
    void *addr;
} test_regions[NUM_TEST_REGIONS] = { 0 };

static void init_regions(void)
{
    int len, i, offs, reg;
    uint32_t perms;
    const uint32_t *regions, *prop;
    const char *name;

    const void *fdt = dt_fdt();
    if (!fdt) {
        exit(-1);
    }

    offs = fdt_path_offset(fdt, "/chosen/opensbi-domains/domain");
    regions = fdt_getprop(fdt, offs, "regions", &len);

    len /= sizeof(uint32_t);
    for (i = 0, reg = 0; i < len && reg < NUM_TEST_REGIONS; i += 2) {
        offs = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(regions[i]));
        perms = fdt32_to_cpu(regions[i + 1]);

        name = fdt_get_name(fdt, offs, NULL);
        if (strncmp(name, "test", 4)) {
            continue;
        }

        test_regions[reg].name = name;
        test_regions[reg].perms = perms;

        prop = fdt_getprop(fdt, offs, "base", NULL);
        test_regions[reg].base = ((uint64_t) fdt32_to_cpu(prop[0]) << 32) | fdt32_to_cpu(prop[1]);

        prop = fdt_getprop(fdt, offs, "size", NULL);
        test_regions[reg].size = ((uint64_t) fdt32_to_cpu(prop[0]) << 32) | fdt32_to_cpu(prop[1]);

        test_regions[reg].addr = vmap(test_regions[reg].base, test_regions[reg].size);
        reg++;
    }
}

#define SBI_READABLE	(1UL << 3)
#define SBI_WRITABLE	(1UL << 4)
#define SBI_EXECUTABLE	(1UL << 5)

#define REGION_HAS_PERMS(reg, perm) \
    (((reg)->perms & (perm)) != 0)

#define TEST_SUCCESS(reg, perm, can) \
    (!((can) ^ REGION_HAS_PERMS(reg, perm)))

static void run_region_tests(void)
{
    bool load_okay, store_okay, exec_okay;
    struct test_region *reg;
    volatile uint32_t *curr, *end;
    uint32_t val;
    int i, page;

    for (i = 0; i < NUM_TEST_REGIONS; i++) {
        reg = &test_regions[i];
        if (reg->name) {
            report_prefix_push(reg->name);
            curr = reg->addr;
            end = (void *) curr + reg->size - 1;

            load_okay = true;
            store_okay = true;
            exec_okay = true;

            /* Test every page of this region */
            page = 0;
            while (curr < end) {
                can_load = true;
                can_store = true;
                can_exec = true;

                /* Try loading */
                val = *curr;

                /* Try storing */
                *curr = val;

                /* If we can store, try executing */
                if (can_store) {
                    // Put a valid instruction sequence here
                    curr[0] = 0x00000013; // nop
                    curr[1] = 0x00000013; // nop
                    curr[2] = 0x00000013; // nop
                    curr[3] = 0x00008067; // ret
                    __asm__ __volatile__ ("sfence.vma");
                }
                ((void (*)(void)) curr)();

                load_okay &= TEST_SUCCESS(reg, SBI_READABLE, can_load);
                store_okay &= TEST_SUCCESS(reg, SBI_WRITABLE, can_store);
                exec_okay &= TEST_SUCCESS(reg, SBI_EXECUTABLE, can_exec);

                if (!TEST_SUCCESS(reg, SBI_READABLE, can_load)) {
                    report_fail("Page %x read perms incorrect (got %x expected %x)",
                                page, can_load, REGION_HAS_PERMS(reg, SBI_READABLE));
                    break;
                }

                if (!TEST_SUCCESS(reg, SBI_WRITABLE, can_store)) {
                    report_fail("Page %x store perms incorrect (got %x expected %x)",
                                page, can_store, REGION_HAS_PERMS(reg, SBI_WRITABLE));
                    break;
                }

                if (!TEST_SUCCESS(reg, SBI_EXECUTABLE, can_exec)) {
                    report_fail("Page %x exec perms incorrect (got %x expected %x)",
                                page, can_exec, REGION_HAS_PERMS(reg, SBI_EXECUTABLE));
                    break;
                }

                // Handle 32-bit overflow
                curr = (void *) curr + 0x1000;
                if (!curr )
                    break;

                page++;
#if __riscv_xlen == 32
                if (page == 0x3fffe) {
                    break;
                }
#endif
            }

            if (load_okay && store_okay && exec_okay) {
                report_pass("All pages correct");
            } else {
                report_fail("Some pages wrong");
            }
            report_prefix_pop();
        }
    }
}

static void handle_secondary(void)
{
    /* Parse the device tree and figure out our configuration */
    init_regions();

    /* Install some exception handlers */
    install_exception_handler(EXC_LOAD_ACCESS, load_handler);
    install_exception_handler(EXC_STORE_ACCESS, store_handler);
    install_exception_handler(EXC_INST_ACCESS, exec_handler);
    install_exception_handler(EXC_INST_ILLEGAL, inst_handler);

    /* Run tests */
    run_region_tests();

    /* 2. This sequence transfers control back to the primary */
    initialize_comms();

}

int main(int argc, char **argv)
{
    if (argc != 2) {
        exit(-1);
    }

    if (!strcmp(argv[1], "primary")) {
        handle_primary();
    } else if (!strcmp(argv[1], "secondary")) {
        handle_secondary();
    } else {
        exit(-1);
    }

    return 0;
}
