#ifndef SVG2GCODE_H
#define SVG2GCODE_H

#ifndef NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
int generateGcode(int argc, char* argv[], int** penColors, int* penColorCount, float* paperDimensions, int* generationConfig);
#ifdef __cplusplus
}
#endif
#endif
