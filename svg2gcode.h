#ifndef SVG2GCODE_H
#define SVG2GCODE_H

#include <stdio.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

int generateGcode(int argc, char* argv[], int** penColors, int* penColorCount, float* paperDimensions, int* generationConfig, char* fileName);

#ifdef __cplusplus
}
#endif
#endif //SVG2GCODE_H