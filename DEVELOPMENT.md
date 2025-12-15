## Bug Backlog
- Solve issue of failing to print `Avg: %.2f` and `Range: [%.1f, %.1f]`
    
    ```bash
    -- [00:00:00.256,347] <inf> state_machine: State: Monitoring - Avg: , Samples: 2 | Buffer: 2 samples | Range: [%.1f, %.1f]
    -- [00:00:01.256,561] <inf> state_machine: State: Monitoring - Avg: , Samples: 3 | Buffer: 3 samples | Range: [%.1f, %.1f]
    -- [00:00:01.256,683] <inf> state_machine: State: Monitoring - Avg: , Samples: 3 | Buffer: 3 samples | Range: [%.1f, %.1f]
    -- [00:00:01.256,805] <inf> state_machine: State: Monitoring - Avg: , Samples: 3 | Buffer: 3 samples | Range: [%.1f, %.1f]
    -- [00:00:02.255,706] <inf> state_machine: State: Monitoring - Avg: , Samples: 2 | Buffer: 2 samples | Range: [%.1f, %.1f]
    -- [00:00:02.255,859] <inf> state_machine: State: Monitoring - Avg: , Samples: 2 | Buffer: 2 samples | Range: [%.1f, %.1f]
    -- [00:00:02.257,049] <inf> state_machine: State: Monitoring - Avg: , Samples: 4 | Buffer: 4 samples | Range: [%.1f, %.1f]
    -- [00:00:02.257,171] <inf> state_machine: State: Monitoring - Avg: , Samples: 4 | Buffer: 4 samples | Range: [%.1f, %.1f]
    ```
    - Attempts:
        - increase buffer size `static std::array<char, 2048> buffer{};` (no change)
        - changing buffer into non-static, or outside `std::visit()`, instead nothing gets returned (even worse)
        - increase stack size (no change)

            ```
            # prj.conf
            CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=4096
            CONFIG_HEAP_MEM_POOL_SIZE=8192
            CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=4096
            CONFIG_MAIN_STACK_SIZE=8192
            CONFIG_ISR_STACK_SIZE=4096
            ```
    - Potential cause:
        - ???