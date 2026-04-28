#include <math.h>

// Using atan2f for 32-bit floats
float calc_heading_in_rad(short mx_data, short my_data) {
    float mx = (float)mx_data - 113.0f;
    float my = (float)my_data - (-263.0f);
    
    return atan2f(my, mx);
}
