#include <iostream>
#include <time.h>
#include <stdio.h>

using namespace std;

uint64_t get_tick_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_nsec / 1000000) + ((uint64_t)ts.tv_sec * 1000ull);
}

int main()
{
    uint64_t y, x;

    while(1)
    {
        x = get_tick_ms();
        for(int i = 0; i<1000000; i++) ;
        y = get_tick_ms();
        uint64_t elapsed = y-x;
        printf("%lu \n",elapsed);
    }
    
    return 0;
}