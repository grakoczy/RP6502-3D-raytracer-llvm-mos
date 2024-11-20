#include <rp6502.h>
// #include "qfp16.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "bitmap_graphics.h"

#define COLOR_FROM_RGB8(r,g,b) (((b>>3)<<11)|((g>>3)<<6)|(r>>3))
// XRAM locations
#define KEYBOARD_INPUT 0xFF10 // KEYBOARD_BYTES of bitmask data

#define SCREEN_WIDTH 240 
#define SCREEN_HEIGHT 124 
// Window size
#define WIDTH 120
#define HEIGHT 120

// Structs for basic math and objects
typedef struct {
    float x, y, z;
} Vector3;

typedef struct {
    Vector3 center;
    float radius;
    uint16_t color;
    bool reflects; // New field to indicate if the sphere should reflect light
} Sphere;

typedef struct {
    Vector3 min;
    Vector3 max;
    uint16_t color;
    bool reflects; // Indicate if the shape should reflect light
} Box;

typedef struct {
    Vector3 center;
    float radius;
    float height;
    uint16_t color;
    bool reflects; // Indicate if the shape should reflect light
} Cone;

typedef struct {
    Vector3 origin, direction;
} Ray;

typedef struct {
    float t;
    Vector3 point;
    Vector3 normal;
    Sphere* sphere;
} HitInfo;

// Scene Objects
Sphere spheres[] = {
    {{-1.2f, 0.3f, 2.0f}, 0.6f, COLOR_FROM_RGB8(95, 50, 50), true}, // Red Sphere
    {{0.8f, 0.5f, 2.5f}, 1.0f, COLOR_FROM_RGB8(0, 255, 0), false},  // Green Sphere
    {{0.0f, -1000.5f, 2.0f}, 1000.0f, COLOR_FROM_RGB8(127, 127, 255), false} // Ground plane (Large sphere)
};
const int sphereCount = sizeof(spheres) / sizeof(Sphere);

Box boxes[] = {
    {{0.0f, 0.0f, 0.0f}, {0.5f, 0.5f, 0.5f}, COLOR_FROM_RGB8(200, 100, 100), false}
};
const int boxCount = sizeof(boxes) / sizeof(Box);

// Light position
Vector3 lightPos = {-2.0f, 1.0f, -2.0f};

void extractRGB(uint16_t color, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Extract 5-bit red value
    uint8_t red5 = (color & 0x1F);
    // Extract 6-bit green value
    uint8_t green6 = (color >> 5) & 0x3F;
    // Extract 5-bit blue value
    uint8_t blue5 = (color >> 11) & 0x1F;

    // Convert to 8-bit values (optional)
    *r = (red5 << 3) | (red5 >> 2); // Scale 5-bit to 8-bit
    *g = (green6 << 2) | (green6 >> 4); // Scale 6-bit to 8-bit
    *b = (blue5 << 3) | (blue5 >> 2); // Scale 5-bit to 8-bit
}


float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;                       // Bit-level hack to get integer representation
    i = 0x5f3759df - (i >> 1);             // Magic constant and bit manipulation
    y = *(float *)&i;                      // Convert back to float
    y = y * (threehalfs - (x2 * y * y));   // One iteration of Newton's method
    // y = y * (threehalfs - (x2 * y * y)); // Uncomment for more iterations (improves accuracy)

    return y;
}

float my_sqrtf(float number) {
    return number * Q_rsqrt(number);
}

float fabs(float x) {
    // If x is negative, return its negation, otherwise return x as is.
    return (x < 0.0f) ? -x : x;
}


void WaitForAnyKey(){

    xregn(0, 0, 0, 1, KEYBOARD_INPUT);
    RIA.addr0 = KEYBOARD_INPUT;
    RIA.step0 = 0;
    while (RIA.rw0 & 1)
        ;
}


// Utility functions
Vector3 vector_add(Vector3 a, Vector3 b) { return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 vector_sub(Vector3 a, Vector3 b) { return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 vector_scale(Vector3 v, float s) { return (Vector3){v.x * s, v.y * s, v.z * s}; }
float vector_dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vector3 vector_normalize(Vector3 v) {
    float len = my_sqrtf(vector_dot(v, v));
    return vector_scale(v, 1.0f / len);
}

// Ray-sphere intersection
bool ray_sphere_intersect(Ray* ray, Sphere* sphere, HitInfo* hit) {
    Vector3 oc = vector_sub(ray->origin, sphere->center);
    float a = vector_dot(ray->direction, ray->direction);
    float b = 2.0f * vector_dot(oc, ray->direction);
    float c = vector_dot(oc, oc) - sphere->radius * sphere->radius;
    float discriminant = b * b - 4 * a * c;

    if (discriminant > 0) {
        float t = (-b - my_sqrtf(discriminant)) / (2.0f * a);
        if (t > 0) {
            hit->t = t;
            hit->point = vector_add(ray->origin, vector_scale(ray->direction, t));
            hit->normal = vector_normalize(vector_sub(hit->point, sphere->center));
            hit->sphere = sphere;
            return true;
        }
    }
    return false;
}

Vector3 box_normal(Vector3 point, Box* box) {
    Vector3 normal = {0.0f, 0.0f, 0.0f};
    if ( fabs(point.x - box->min.x) < FLT_EPSILON) normal.x = -1.0f;
    else if (fabs(point.x - box->max.x) < FLT_EPSILON) normal.x = 1.0f;
    else if (fabs(point.y - box->min.y) < FLT_EPSILON) normal.y = -1.0f;
    else if (fabs(point.y - box->max.y) < FLT_EPSILON) normal.y = 1.0f;
    else if (fabs(point.z - box->min.z) < FLT_EPSILON) normal.z = -1.0f;
    else if (fabs(point.z - box->max.z) < FLT_EPSILON) normal.z = 1.0f;
    return vector_normalize(normal);
}

bool ray_box_intersect(Ray* ray, Box* box, HitInfo* hit) {
    Vector3 invDir = vector_scale(ray->direction, -1.0f);
    invDir.x = 1.0f / invDir.x;
    invDir.y = 1.0f / invDir.y;
    invDir.z = 1.0f / invDir.z;

    float tmin = (box->min.x - ray->origin.x) * invDir.x;
    float tmax = (box->max.x - ray->origin.x) * invDir.x;
    if (tmin > tmax) { float temp = tmin; tmin = tmax; tmax = temp; }

    float tymin = (box->min.y - ray->origin.y) * invDir.y;
    float tymax = (box->max.y - ray->origin.y) * invDir.y;
    if (tymin > tymax) { float temp = tymin; tymin = tymax; tymax = temp; }

    if ((tmin > tymax) || (tymin > tmax)) return false;

    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    float tzmin = (box->min.z - ray->origin.z) * invDir.z;
    float tzmax = (box->max.z - ray->origin.z) * invDir.z;
    if (tzmin > tzmax) { float temp = tzmin; tzmin = tzmax; tzmax = temp; }

    if ((tmin > tzmax) || (tzmin > tmax)) return false;

    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    if (tmax < 0.0f) return false; // Box is behind the ray

    hit->t = tmin;
    hit->point = vector_add(ray->origin, vector_scale(ray->direction, tmin));
    hit->normal = box_normal(hit->point, box);
    hit->sphere = box;

    return true;
};



// Scene rendering
// Updated trace_ray function with single reflection
uint16_t trace_ray(Ray* ray, int x, int y) {
    HitInfo hit, closestHit;
    closestHit.t = 1e30f; // Large value for initial check
    bool hitAnything = false;

    // Find closest hit amont spheres
    for (int i = 0; i < sphereCount; i++) {
        if (ray_sphere_intersect(ray, &spheres[i], &hit) && hit.t < closestHit.t) {
            closestHit = hit;
            hitAnything = true;
        }
    }

    // Find closest hit among boxes
    for (int i = 0; i < boxCount; i++) {
        if (ray_box_intersect(ray, &boxes[i], &hit) && hit.t < closestHit.t) {
            closestHit = hit;
            hitAnything = true;
        }
    }

    if (hitAnything) {
        // Basic Phong shading
        Vector3 lightDir = vector_normalize(vector_sub(lightPos, closestHit.point));
        float diffuse = fmaxf(0.0f, vector_dot(closestHit.normal, lightDir));
        
        // Shadow check
        Ray shadowRay = {closestHit.point, lightDir};
        bool inShadow = false;
        for (int i = 0; i < sphereCount; i++) {
            HitInfo shadowHit;
            if (ray_sphere_intersect(&shadowRay, &spheres[i], &shadowHit) && shadowHit.t > 0.001f) {
                inShadow = true;
                break;
            }
        }
        for (int i = 0; i < boxCount; i++) {
            HitInfo shadowHit;
            if (ray_box_intersect(&shadowRay, &boxes[i], &shadowHit) && shadowHit.t > 0.001f) {
                inShadow = true;
                break;
            }
        }

        // Calculate the base color (direct lighting)
        uint16_t sphereColor = closestHit.sphere->color;
        uint8_t r, g, b;
        extractRGB(sphereColor, &r, &g, &b);
        // printf("Extracted RGB: (%u, %u, %u)\n", r, g, b);
        int baseR = r * (inShadow ? 0.1f : diffuse);
        int baseG = g * (inShadow ? 0.1f : diffuse);
        int baseB = b * (inShadow ? 0.1f : diffuse);

        // If the sphere is the red sphere, add reflection
        if (closestHit.sphere->reflects) { // Use the new field to check for reflections
            // Calculate the reflection ray
            Vector3 viewDir = vector_scale(ray->direction, -1.0f);
            float dot = vector_dot(viewDir, closestHit.normal);
            Vector3 reflectionDir = vector_sub(
                vector_scale(closestHit.normal, 2.0f * dot), viewDir);
            reflectionDir = vector_normalize(reflectionDir);

            Ray reflectionRay = {vector_add(closestHit.point, vector_scale(reflectionDir, 0.001f)), reflectionDir};
            HitInfo reflectionHit;
            reflectionHit.t = 1e30f;
            bool reflectionHitAnything = false;

            // Find closest hit for the reflection ray
            for (int i = 0; i < sphereCount; i++) {
                if (ray_sphere_intersect(&reflectionRay, &spheres[i], &hit) && hit.t < reflectionHit.t) {
                    reflectionHit = hit;
                    reflectionHitAnything = true;
                }
            }

            // Calculate reflection color
            int reflectR = 0, reflectG = 0, reflectB = 0;
            if (reflectionHitAnything) {
                uint16_t reflectionColor = reflectionHit.sphere->color;
                extractRGB(reflectionColor, &r, &g, &b);
                float reflectDiffuse = fmaxf(0.0f, vector_dot(vector_normalize(vector_sub(lightPos, reflectionHit.point)), reflectionHit.normal));
                reflectR = r * reflectDiffuse;
                reflectG = g * reflectDiffuse;
                reflectB = b * reflectDiffuse;
            }

            // Blend the base color with the reflection color (50% contribution)
            baseR = (baseR + reflectR) / 2;
            baseG = (baseG + reflectG) / 2;
            baseB = (baseB + reflectB) / 2;
        }

        // Clamp the color values to 0-255
        // baseR = min(max(baseR, 0), 255);
        // baseG = min(max(baseG, 0), 255);
        // baseB = min(max(baseB, 0), 255);

        return COLOR_FROM_RGB8(baseR, baseG, baseB);
    }

    // Background color
    return COLOR_FROM_RGB8(50, 50, 50);
}


// Main drawing function
void render_scene() {
    Vector3 cameraPos = {0.0f, 0.0f, -0.50f};
    float viewportWidth = 2.0f;
    float viewportHeight = 2.0f * HEIGHT / WIDTH;
    float viewportDist = 1.0f;

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
			
            float u = (x - WIDTH / 2.0f) * viewportWidth / WIDTH;
            float v = -(y - HEIGHT / 2.0f) * viewportHeight / HEIGHT;
            Vector3 rayDir = vector_normalize((Vector3){u, v, viewportDist});
            Ray ray = {cameraPos, rayDir};
            uint16_t color = trace_ray(&ray, x, y);
            draw_pixel(color, x, y);
        }
    }
}

// Progressive rendering parameters
#define INITIAL_BLOCK_SIZE 20 // Start with 8x8 blocks, can be adjusted
#define PROGRESS_BAR_WIDTH 2 // Width of the progress bar in pixels

uint8_t blockSizes[] = {20, 8, 4, 1};
uint16_t colors[] = {COLOR_FROM_RGB8(200, 0, 0), 
                         COLOR_FROM_RGB8(252, 143, 0),
                         COLOR_FROM_RGB8(235, 219, 52),
                         COLOR_FROM_RGB8(84, 252, 0)};
int colorCount = sizeof(colors) / sizeof(colors[0]);

// Function to cycle through a set of colors for the progress bar
uint16_t get_progress_bar_color(int step) {
    // Example colors (16-bit RGB565 format): Red, Green, Blue, Yellow, Magenta, Cya
    return colors[step % colorCount]; // Cycle through the colors
}

void draw_progress_bar(int currentProgress, int totalProgress, uint16_t color) {
    float progressLength = (float)((float)(currentProgress * WIDTH) / totalProgress); // Calculate the width of the progress bar
    // printf("len: %i\n", progressLength);
    // Draw the progress bar at the bottom of the screen
    fill_rect(color, WIDTH + 5, 0, PROGRESS_BAR_WIDTH, (uint16_t)progressLength);
}

void render_scene_progressive() {
    Vector3 cameraPos = {0.0f, 0.0f, -0.50f};
    float viewportWidth = 2.0f;
    float viewportHeight = 2.0f * HEIGHT / WIDTH;
    float viewportDist = 1.0f;

    int stepIndex = 0; // To track the current step and change the progress bar color

    // Start with a large block size and reduce until we reach single pixel rendering
    for (int i = 0; i < sizeof(blockSizes); i++) {
        uint8_t blockSize = blockSizes[i];
        // Calculate the total number of ray traces for the current block size
        int totalRays = (WIDTH / blockSize) * (HEIGHT / blockSize);
        int completedRays = 0;

        // Get the progress bar color for the current block size iteration
        uint16_t progressBarColor = get_progress_bar_color(stepIndex);
        stepIndex++; // Move to the next color for the next block size

        // printf("blockSize: %i\n", blockSize);

        // Iterate over the screen in blocks of current blockSize
        for (int y = 0; y < HEIGHT; y += blockSize) {
            for (int x = 0; x < WIDTH; x += blockSize) {

                // Calculate the central position of the block for ray tracing
                float u = ((x + blockSize / 2.0f) - WIDTH / 2.0f) * viewportWidth / WIDTH;
                float v = -((y + blockSize / 2.0f) - HEIGHT / 2.0f) * viewportHeight / HEIGHT;

                draw_rect(progressBarColor, x, y, blockSize, blockSize); // show where we are on the screen

                Vector3 rayDir = vector_normalize((Vector3){u, v, viewportDist});
                Ray ray = {cameraPos, rayDir};
                
                // Trace the ray for the central pixel of the block
                uint16_t color = trace_ray(&ray, x + blockSize / 2, y + blockSize / 2);

                // Fill the current block with the calculated color
                fill_rect(color, x, y, blockSize, blockSize);

                // Update progress after each trace_ray call
                completedRays++;
                // printf("completedRays: %i, totalRays: %i\n", completedRays, totalRays);
                // draw_progress_bar(completedRays, totalRays, progressBarColor);
            }
        }

        // Reset the progress bar when moving to a smaller block size
        // draw_progress_bar(1, 1, progressBarColor); // Clear the progress bar (black color)
    }
}

int main() {
    
    init_bitmap_graphics(0xFF00, 0x0000, 0, 2, SCREEN_WIDTH, SCREEN_HEIGHT, 16);
    erase_canvas();

    long startTime = clock();

    // render_scene();
    render_scene_progressive();

    long endTime = clock();

    printf("render took: %lu", (endTime - startTime) / 100);

    WaitForAnyKey();

    return 0;
}
