#include "rainbow.h"

#include "esp_log.h"
#include "pax_gfx.h"

static inline float clamp(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

static inline float fract(float x) {
    return x - (int) x;
}

static inline float mix(float a, float b, float t) {
    return a + (b - a) * t;
}


pax_col_t rainbow(int pos, int max) {
    float h = ((float) (pos % max)) / max;
    return pax_col_hsv(h * 360, 0xff, 0xff);
}