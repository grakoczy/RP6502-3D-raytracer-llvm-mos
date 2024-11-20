#include <rp6502.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "colors.h"
#include "usb_hid_keys.h"
#include "bitmap_graphics.h"


// XRAM locations
#define KEYBOARD_INPUT 0xFF10 // KEYBOARD_BYTES of bitmask data

// 256 bytes HID code max, stored in 32 uint8
#define KEYBOARD_BYTES 32
uint8_t keystates[KEYBOARD_BYTES] = {0};

// keystates[code>>3] gets contents from correct byte in array
// 1 << (code&7) moves a 1 into proper position to mask with byte contents
// final & gives 1 if key is pressed, 0 if not
#define key(code) (keystates[code >> 3] & (1 << (code & 7)))

// Simple 3D to 2D projection (scaled)
#define SCALE 64
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 360

// Precompute sine and cosine values for rotation
#define NUM_POINTS 360
int16_t sine_values[NUM_POINTS];
int16_t cosine_values[NUM_POINTS];

// Constants for fixed-point trigonometry
enum {cA1 = 3370945099UL, cB1 = 2746362156UL, cC1 = 292421UL};
enum {n = 13, p = 32, q = 31, r = 3, a = 12};

// Function to calculate fixed-point sine
int16_t fpsin(int16_t i) {
    i <<= 1;
    uint8_t c = i < 0; // Set carry for output pos/neg

    if(i == (i | 0x4000)) // Flip input value to corresponding value in range [0..8192)
        i = (1 << 15) - i;
    i = (i & 0x7FFF) >> 1;

    uint32_t y = (cC1 * ((uint32_t)i)) >> n;
    y = cB1 - (((uint32_t)i * y) >> r);
    y = (uint32_t)i * (y >> n);
    y = (uint32_t)i * (y >> n);
    y = cA1 - (y >> (p - q));
    y = (uint32_t)i * (y >> n);
    y = (y + (1UL << (q - a - 1))) >> (q - a); // Rounding

    return c ? -y : y;
}

// Function to calculate fixed-point cosine
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))

// Cube vertices in 3D space (8 corners of a cube)
int16_t cube_vertices[8][3] = {
    {-4096, -4096, -4096}, {4096, -4096, -4096}, {4096, 4096, -4096}, {-4096, 4096, -4096},  // Back face
    {-4096, -4096,  4096}, {4096, -4096,  4096}, {4096, 4096,  4096}, {-4096, 4096,  4096}   // Front face
};


// Fast integer-based line drawing function
void draw_line_f(uint8_t color, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // Calculate the differences
    int8_t dx = x1 - x0;
    int8_t dy = y1 - y0;

    // Determine if we are stepping in positive or negative directions
    int8_t sx = (dx > 0) ? 1 : -1;
    int8_t sy = (dy > 0) ? 1 : -1;

    // Absolute value of differences
    dx = (dx > 0) ? dx : -dx;
    dy = (dy > 0) ? dy : -dy;

    // Error term, initialized to 0
    int8_t err = (dx > dy ? dx : -dy) / 2;
    int8_t e2;

    while (1) {
        // Draw the pixel at (x0, y0)
        draw_pixel(color, x0, y0);

        // Check if we've reached the endpoint
        if (x0 == x1 && y0 == y1) {
            break;
        }

        e2 = err;

        // Update the error term and coordinates
        if (e2 > -dx) {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dy) {
            err += dx;
            y0 += sy;
        }
    }
}


void precompute_sin_cos() {
    int16_t angle_step = 32768 / NUM_POINTS; // 32768 is 2^15, representing 2*pi

    for (int i = 0; i < NUM_POINTS; i++) {
        sine_values[i] = fpsin(i * angle_step);
        cosine_values[i] = fpcos(i * angle_step);
    }
}


void project_3d_to_2d(int16_t vertex[3], int16_t* x2d, int16_t* y2d) {
    *x2d = (vertex[0] / SCALE) + SCREEN_WIDTH / 2;
    *y2d = (vertex[1] / SCALE) + SCREEN_HEIGHT / 2;
}

// Rotate around X axis
void rotate_x(int16_t* y, int16_t* z, int angle) {
    int16_t tmp_y = *y;
    int16_t tmp_z = *z;
    *y = ((long)tmp_y * (long)cosine_values[angle] - (long)tmp_z * (long)sine_values[angle])>> 12;
    *z = ((long)tmp_y * (long)sine_values[angle] + (long)tmp_z * (long)cosine_values[angle])>> 12;
}

// Rotate around Y axis
void rotate_y(int16_t* x, int16_t* z, int angle) {
    int16_t tmp_x = *x;
    int16_t tmp_z = *z;
    *x = ((long)tmp_x * (long)cosine_values[angle] + (long)tmp_z * (long)sine_values[angle])>> 12;
    *z = ((long)tmp_z * (long)cosine_values[angle] - (long)tmp_x * (long)sine_values[angle])>> 12;
}

// Rotate around Z axis
void rotate_z(int16_t* x, int16_t* y, int angle) {
    int16_t tmp_x = *x;
    int16_t tmp_y = *y;
    *x = (tmp_x * (long)cosine_values[angle] - (long)tmp_y * (long)sine_values[angle])>> 12;
    *y = (tmp_x * (long)sine_values[angle] + (long)tmp_y * (long)cosine_values[angle])>> 12;
}

// Draw the cube by connecting the vertices with lines
void draw_cube(int angleX, int angleY, int angleZ, int16_t color, uint8_t mode) {
    int16_t x2d[8], y2d[8];

    // Rotate and project all vertices
    for (uint8_t i = 0; i < 8; i++) {
        // printf("i: %i\n", i);
        int16_t vertex[3] = {cube_vertices[i][0], cube_vertices[i][1], cube_vertices[i][2]};


        // Apply rotations
        rotate_x(&vertex[1], &vertex[2], angleX);
        rotate_y(&vertex[0], &vertex[2], angleY);
        rotate_z(&vertex[0], &vertex[1], angleZ);


        // Project the 3D vertex to 2D
        project_3d_to_2d(vertex, &x2d[i], &y2d[i]);
        
    }

    erase_canvas();

    // Connect the vertices with lines to draw the cube (front and back faces)
    if (mode == 0) {
    draw_line(color, x2d[0], y2d[0], x2d[1], y2d[1]);
    draw_line(color, x2d[1], y2d[1], x2d[2], y2d[2]);
    draw_line(color, x2d[2], y2d[2], x2d[3], y2d[3]);
    draw_line(color, x2d[3], y2d[3], x2d[0], y2d[0]);
    draw_line(color, x2d[4], y2d[4], x2d[5], y2d[5]);
    draw_line(color, x2d[5], y2d[5], x2d[6], y2d[6]);
    draw_line(color, x2d[6], y2d[6], x2d[7], y2d[7]);
    draw_line(color, x2d[7], y2d[7], x2d[4], y2d[4]);
    draw_line(color, x2d[0], y2d[0], x2d[4], y2d[4]);
    draw_line(color, x2d[1], y2d[1], x2d[5], y2d[5]);
    draw_line(color, x2d[2], y2d[2], x2d[6], y2d[6]);
    draw_line(color, x2d[3], y2d[3], x2d[7], y2d[7]);
    }
    if (mode == 1) {
        draw_pixel(color, x2d[0], y2d[0]);
        draw_pixel(color, x2d[1], y2d[1]);
        draw_pixel(color, x2d[2], y2d[2]);
        draw_pixel(color, x2d[3], y2d[3]);
        draw_pixel(color, x2d[4], y2d[4]);
        draw_pixel(color, x2d[5], y2d[5]);
        draw_pixel(color, x2d[6], y2d[6]);
        draw_pixel(color, x2d[7], y2d[7]);
        draw_pixel(color, x2d[0], y2d[0]);
        draw_pixel(color, x2d[1], y2d[1]);
        draw_pixel(color, x2d[2], y2d[2]);
        draw_pixel(color, x2d[3], y2d[3]);
    }

    // int d = 0;
    // while (d < 1000) {
    //     d++;
    //     draw_pixel(BLACK, 0, 0);
    // }


}


int main() {
    
    bool handled_key = false;
    bool paused = true;
    uint8_t mode = 0;
    uint8_t i = 0;

    init_bitmap_graphics(0xFF00, 0x0000, 0, 1, SCREEN_WIDTH, SCREEN_HEIGHT, 1);

    erase_canvas();
    
    printf("Precomputing sine and cosine values...\n");
    // Precompute sine and cosine values
    precompute_sin_cos();

    int angleX = 10, angleY = 10, angleZ = 10;

    printf("Start drawing\n");

    // Draw the rotating cube
    draw_cube(angleX, angleY, angleZ, WHITE, mode);

    set_cursor(10, 220);
    draw_string("Press SPACE to start/stop");
    set_cursor(10, 230);
    draw_string("Press 1 or 2 to change drawing style");


    while (true) {


       // Update rotation angles

        if (!paused) {

            angleX = (angleX + 1) % NUM_POINTS;
            angleY = (angleY + 1) % NUM_POINTS;
            angleZ = (angleZ + 1) % NUM_POINTS;

            // printf("angleX: %i, angleY: %i, angleZ: %i\n", angleX, angleY, angleZ);

            // Draw the rotating cube
            draw_cube(angleX, angleY, angleZ, WHITE, mode);
        }

        xregn( 0, 0, 0, 1, KEYBOARD_INPUT);
        RIA.addr0 = KEYBOARD_INPUT;
        RIA.step0 = 0;

        // fill the keystates bitmask array
        for (uint8_t i = 0; i < KEYBOARD_BYTES; i++) {
            uint8_t j, new_keys;
            RIA.addr0 = KEYBOARD_INPUT + i;
            new_keys = RIA.rw0;

            // check for change in any and all keys
            for (j = 0; j < 8; j++) {
                uint8_t new_key = (new_keys & (1<<j));
                if ((((i<<3)+j)>3) && (new_key != (keystates[i] & (1<<j)))) {
                    printf( "key %d %s\n", ((i<<3)+j), (new_key ? "pressed" : "released"));
                }
            }

            keystates[i] = new_keys;
        }

        // check for a key down
        if (!(keystates[0] & 1)) {
            if (!handled_key) { // handle only once per single keypress
                // handle the keystrokes
                if (key(KEY_SPACE)) {
                    paused = !paused;
                }
                if (key(KEY_1)) {
                    mode = 0;
                }
                if (key(KEY_2)) {
                    mode = 1;
                }
                if (key(KEY_ESC)) {
                    break;
                }
                handled_key = true;
            }
        } else { // no keys down
            handled_key = false;
        }

    }

    return 0;
    
}
