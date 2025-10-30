#include "pico/stdlib.h"
#include "test_motor_pid.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║              MOTOR PID CONTROL TEST SYSTEM                    ║\n");
    printf("║                    Week 9 Testing                             ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    printf("\nPress any key in serial monitor to start tests...\n");
    getchar();
    
    // Run all tests
    run_all_tests();
    
    printf("\nTesting complete. Robot stopped.\n");
    
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}