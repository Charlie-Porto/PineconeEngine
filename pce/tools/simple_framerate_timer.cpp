#ifndef simple_framerate_timer_cpp
#define simple_framerate_timer_cpp

#include <iostream>


/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
purpose: a simple timer for testing frame rate.
-----------------------------------------------------------------*/

class simple_framerate_timer {

public:
    simple_framerate_timer() {
        time_keeper = 1;
        cycle_loops = 0;
        total_loops = 0;
        avg_loops_per_second = 0;
    }

    void update_timer(int t) {
        if (time_keeper > t) {
            ++cycle_loops;
        }
        else if (time_keeper <= t) {
            time_keeper = t + 1;
            total_loops += cycle_loops;

            std::cout << "Time: " << t << "s" << "      Frame Rate: " << cycle_loops << "\n";

            cycle_loops = 0;
        }
    }

private:
    int time_keeper;
    int cycle_loops;
    int total_loops;
    int avg_loops_per_second;
};

#endif /* simple_framerate_timer_cpp */
