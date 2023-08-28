/*
 * svg2gcode (c) Matti Koskinen 2014
 *
 * reorder-function based on StippleGen
 * Copyright (C) 2012 by Windell H. Oskay, www.evilmadscientist.com
 *
 * nanosvg.h (c) 2014 Mikko Mononen
 * some routines based on nanosvg-master example1.c
 *
 * Xgetopt used on VS2010 (or later) by Hans Dietrich,David Smith
 * code public domain
 *
 * No comments :-)
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * http://creativecommons.org/licenses/LGPL/2.1/
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdlib.h>
#include <stdio.h>
#ifdef _MSC_VER
#include "XGetopt.h"
#else
#include <unistd.h>
#endif

#include <float.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>
#include "svg2gcode.h"


#define DEBUG_OUTPUT
//#define DP_DEBUG_OUTPUT
#define BTSVG
#define MAX_BEZ 128 //64;
#define BUFFER_SIZE 8192 //Character buffer size for writing to files.
#define MAXINT(a,b) (((a)>(b))?(a):(b))
#define MININT(a,b) (((a)<(b))?(a):(b))
#define AVG_OPT_WINDOW 10 //Sliding window size for path optimization.
#define MAX_OPT_SECONDS 1200 //20 Minute limit for opt function
#define NUM_TOOLS 6
#define DOUGLAS_PEUCKER_EPSILON 0.1 //in mm

#include <stdio.h>
#include <math.h>
#include <string.h>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

static int sixColorWidth = 306;

static float minf(float a, float b) { return a < b ? a : b; }
static float maxf(float a, float b) { return a > b ? a : b; }
static int numTools = 6;
float bounds[4];
static int pathCount,pointsCount,shapeCount;
static struct NSVGimage* g_image = NULL;
int numCompOut = 0;
int pathCountOut = 0;
int pointCountOut = 0;

typedef struct {
  float x;
  float y;
  float xn;
  float yn;
} SVGPoint;

typedef struct {
  int *colors;
  int count; //Count of shapes to be drawn with this pen.
  int slot;
} Pen;

typedef struct {
  float points[8];
  int shape; //corresponds to a shape id. 
  char closed;
} ToolPath;

typedef struct {
  int id;
  int numToolpaths;
  unsigned int stroke;
} Shape;

typedef struct TransformSettings {
    float scale;
    float pointsToDocumentScale;
    float drawingWidth;
    float drawingHeight;
    float drawSpaceWidth;
    float drawSpaceHeight;
    float loadedFileWidth;
    float loadedFileHeight;
    float xInsetLeft;
    float yInsetTop;
    float xInsetRight;
    float yInsetBottom;
    float shiftX;
    float shiftY;
    float centerX;
    float centerY;
    float originalCenterX;
    float originalCenterY;
    float cosRot;
    float sinRot;
    float xMarginLeft;
    float xMarginRight;
    float yMarginTop;
    float yMarginBottom;
    int fitToMaterial;
    int centerOnMaterial;
    int swapDim;
    int svgRotation;
    int contentsToDrawspace;
    float* pointBounds;
} TransformSettings;

typedef struct GCodeState {
    int npaths;
    int quality;
    float precision;
    int pointsCulledPrec;
    int pointsCulledBounds;
    int feed;
    int feedY;
    int zFeed;
    int tempFeed;
    int slowTravel;
    int shapeStart;
    float zFloor;
    float ztraverse;
    char xy;
    unsigned int currColor;
    unsigned int targetColor;
    int targetTool;
    int currTool;
    int colorMatch;
    float toolChangePos;
    float tol;
    int maxPaths;
    float x;
    float y;
    float firstx;
    float firsty;
    float tempx;
    float tempy;
    float trackedDist; //dist since last pen change or brush refill
    double totalDist;
    float brushDist; //dist in a pen or brush (configurable)
    int countIntermediary;
    float xold;
    float yold;
    float * pathPoints;
    float * pathPointsBuf;
    int pathPointsBufIndex;
    int colorToFile;
    int colorFileOpen;
    int colorCount;
    int useToolOffsets;
} GCodeState;

SVGPoint bezPoints[MAX_BEZ];
static int bezCount = 0;
int collinear = 0;
#ifdef _WIN32

static uint64_t seed;

static int32_t rand31() {
    uint64_t tmp1;
    uint32_t tmp2;

    /* x = (16807 * x) % 0x7FFFFFFF */
    tmp1 = (uint64_t) ((int32_t) seed * (int64_t) 16807);
    tmp2 = (uint32_t) tmp1 & (uint32_t) 0x7FFFFFFF;
    tmp2 += (uint32_t) (tmp1 >> 31);
    if ((int32_t) tmp2 < (int32_t) 0)
      tmp2 = (tmp2 + (uint32_t) 1) & (uint32_t) 0x7FFFFFFF;
    return (int32_t)tmp2;
}

static void seedrand(float seedval) {
  seed = (int32_t) ((double) seedval + 0.5);
  if (seed < 1L) {                   /* seed from current time */
    seed = time(NULL);
    seed = ((seed - 1UL) % 0x7FFFFFFEUL) + 1UL;
  }
  else {
      seed = ((seed - 1L) % 0x7FFFFFFEL) + 1L;
    }
    seed = rand31();
    seed = rand31();
}

static double drnd31() {
  double x;
  seed = rand31();
  x = (double)(seed-0x3FFFFFFFL) * (2.0 / 1073741823.015625);
  if(fabs(x) > 1.0)
    x = drnd31();
  return fabs(x);
}
#endif

double distanceBetweenPoints(double x1, double y1, double x2, double y2){
  //printf("Distance between points, x1: %f, y1: %f, x2: %f, y2: %f\n", x1, y1, x2, y2);
  double result;
  double dx2 = (x2 - x1)*(x2 - x1);
  double dy2 = (y2 - y1)*(y2 - y1);
  result = sqrt(dx2 + dy2);
  //printf("Distance between points: %f\n", result);
  return result;
}

static float distPtSeg(float x, float y, float px, float py, float qx, float qy)
{
  float pqx, pqy, dx, dy, d, t;
  pqx = qx-px;
  pqy = qy-py;
  dx = x-px;
  dy = y-py;
  d = pqx*pqx + pqy*pqy;
  t = pqx*dx + pqy*dy;
  if (d > 0) t /= d;
  if (t < 0) t = 0;
  else if (t > 1) t = 1;
  dx = px + t*pqx - x;
  dy = py + t*pqy - y;
  return dx*dx + dy*dy;
}

float interpFeedrate(int A, int B, float m){ //fr X and fr Y
  if(m == -1){
    return B;
  }
  float scale_factor = (B-A) / (M_PI / 2.0);
  return A + scale_factor * atan(m);
}

float absoluteSlope(float x1, float y1, float x2, float y2){
  if(x2 - x1 == 0){
    return - 1;
  } else {
    return fabs((float)(y2-y1) / (x2 - x1));
  }
}

// bezier smoothing
static void cubicBez(float x1, float y1, float x2, float y2,
             float x3, float y3, float x4, float y4,
             float tol, int level)
{
  float x12,y12,x23,y23,x34,y34,x123,y123,x234,y234,x1234,y1234;
  float d;

  if (level > 12) {
    printf("cubicBez > lvl 12");
    return;
  }
  x12 = (x1+x2)*0.5f;
  y12 = (y1+y2)*0.5f;
  x23 = (x2+x3)*0.5f;
  y23 = (y2+y3)*0.5f;
  x34 = (x3+x4)*0.5f;
  y34 = (y3+y4)*0.5f;
  x123 = (x12+x23)*0.5f;
  y123 = (y12+y23)*0.5f;
  x234 = (x23+x34)*0.5f;
  y234 = (y23+y34)*0.5f;
  x1234 = (x123+x234)*0.5f;
  y1234 = (y123+y234)*0.5f;

  float crossProduct1 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
  float crossProduct2 = (x2 - x1) * (y4 - y1) - (y2 - y1) * (x4 - x1);
  if (fabs(crossProduct1) == 0 && fabs(crossProduct2) == 0) {
    // The curve is a straight line
    collinear = 1;
  }

  d = distPtSeg(x1234, y1234, x1,y1, x4,y4);
  if (d > tol*tol) {
    cubicBez(x1,y1, x12,y12, x123,y123, x1234,y1234, tol, level+1);
    cubicBez(x1234,y1234, x234,y234, x34,y34, x4,y4, tol, level+1);
  } else {
    bezPoints[bezCount].x = x4; //number of points in a given curve will be bezCount.
    bezPoints[bezCount].y = y4;
    bezCount++;
    if(bezCount >= MAX_BEZ) {
      printf("!bez count\n");
      bezCount = MAX_BEZ;
    }
  }
}

#ifdef _WIN32
#define RANDOM() drnd31()
#else //OSX LINUX much faster than win
#define RANDOM() (drand48())
#endif

//This needs to be redone.
static void calcPaths(SVGPoint* points, ToolPath* paths, GCodeState * state, Shape* shapes, FILE* debug) {
  struct NSVGshape* shape;
  struct NSVGpath* path;
  int i, j, k, l, p, b, bezCount;
  bezCount = 0;
  i = 0;
  k = 0;
  j = 0;
  p = 0;
  int shapeCount = 0;
  for (shape = g_image->shapes; shape != NULL; shape = shape->next) {
    for (path = shape->paths; path != NULL; path = path->next) { //For each path in shape.
      shapes[i].id = i;
      shapes[i].stroke = shape->stroke.color;
      for (j = 0; j < path->npts - 1; j += 3) { //iterates through all the paths in a single shape.
        float* pp = &path->pts[j * 2];
        if (j == 0) {
            points[i].x = pp[0];
            points[i].y = pp[1];
        }
        bezCount++;
        for (b = 0; b < 8; b++) {
          paths[k].points[b] = pp[b];
        }
        points[i].xn = pp[6]; //Set last point in svg point to final point iterated thru by previous loop.
        points[i].yn = pp[7];
        paths[k].closed = path->closed;
        paths[k].shape = i;
        shapes[i].numToolpaths++;
        k++;
      }
      
      if (k > pointsCount) {
        printf("Error: k > pointsCount\n");
        state->npaths = 0;
        return;
      }
      if (i > pathCount) {
        printf("Error: i > pathCount\n");
        exit(-1);
      }
      state->maxPaths = MAXINT(shapes[i].numToolpaths, state->maxPaths);
      i++;
    }
    j++;
    shapeCount++;
  }
  state->npaths = k;
}

int colorToPen(int color, int** penColors, int* penColorCounts){ //returns the pen each color is assigned to.
  int pen = 0;

  for(int i = 0; i < 6; i++){
    for(int j = 0; j < penColorCounts[i]; j++){
      if(color == penColors[i][j]){
        pen = i;
        return pen;
      }
    }
  }

  return pen;
}

//submethod for mergeSort
void merge(Shape * arr, int left, int mid, int right, int** penColors, int* penColorCounts) {
    int n1 = mid - left + 1;
    int n2 = right - mid;

    Shape *leftArr = (Shape *) malloc(n1 * sizeof(Shape));
    Shape *rightArr = (Shape *) malloc(n2 * sizeof(Shape));
    memset(leftArr, 0, n1 * sizeof(Shape));
    memset(rightArr, 0, n2 * sizeof(Shape));

    for (int i = 0; i < n1; i++) {
        leftArr[i] = arr[left + i];
    }
    for (int i = 0; i < n2; i++) {
        rightArr[i] = arr[mid + 1 + i];
    }

    int i = 0, j = 0, k = left;
    while (i < n1 && j < n2) {
        if (colorToPen(leftArr[i].stroke, penColors, penColorCounts) <= colorToPen(rightArr[j].stroke, penColors, penColorCounts)) { //I think we want to sort this by the pen int for each color.
            arr[k] = leftArr[i];
            i++;
        } else {
            arr[k] = rightArr[j];
            j++;
        }
        k++;
    }

    while (i < n1) {
        arr[k] = leftArr[i];
        i++;
        k++;
    }

    while (j < n2) {
        arr[k] = rightArr[j];
        j++;
        k++;
    }

    free(leftArr);
    free(rightArr);
}

//sub array implementation of merge sort for sorting shapes by color
void mergeSort(Shape * arr, int left, int right, int level, int* mergeLevel, int** penColors, int* penColorCounts) {
  if(level > *mergeLevel){
    printf("Merge Sort level: %d\n", level);
    *mergeLevel = level;
  }
  if (left < right) {
    int mid = left + (right - left) / 2;
    mergeSort(arr, left, mid, level+1, mergeLevel, penColors, penColorCounts);
    mergeSort(arr, mid + 1, right, level+1, mergeLevel, penColors, penColorCounts);
    merge(arr, left, mid, right, penColors, penColorCounts);
  }
}

int colorInPen(Pen pen, unsigned int color, int colorCount){
  int i = 0;
  while(i < colorCount){
    if(pen.colors[i] == color){ //Each pen holds its associated colors.
      return 1;
    }
    i++;
  }
  return 0;
}

  //This needs to be redone.
//calculate the svg space bounds for the image and create initial shape sized list of colors.
static void calcBounds(struct NSVGimage* image, int numTools, Pen *penList, int penColorCount[6])
{
  struct NSVGshape* shape;
  struct NSVGpath* path;
  int i;
  int colorMatch = 0;
  bounds[0] = FLT_MAX;
  bounds[1] = FLT_MAX;
  bounds[2] = -FLT_MAX;
  bounds[3] = -FLT_MAX;
  pathCount = 0;
  pointsCount = 0;
  shapeCount = 0;
  for (shape = image->shapes; shape != NULL; shape = shape->next) { //for all shapes in an image. Color is at this level
    for (path = shape->paths; path != NULL; path = path->next) { //for all path's in a shape. Path's inherit their shape color.
      for (i = 0; i < path->npts-1; i++) { //for all points in a path.
        float* p = &path->pts[i*2];
        bounds[0] = minf(bounds[0], p[0]);
        bounds[1] = minf(bounds[1], p[1]);
        bounds[2] = maxf(bounds[2], p[0]);
        bounds[3] = maxf(bounds[3], p[1]);
        pointsCount++;
      }
      pathCount++;
    }
    //add to penList[n] here based on color.
    for(int c = 0; c < numTools; c++){
        if(colorInPen(penList[c], shape->stroke.color, penColorCount[c])){
          penList[c].count++;
          colorMatch =1;
          continue;
        }
      }
    if(colorMatch ==0){ //if no color match was found add to tool 1
      penList[0].count++;
    } else {
      colorMatch = 0;
    }
    shapeCount++;
  }
  printf("pathCount = %d\n", pathCount);
  printf("shapeCount = %d\n",shapeCount);
}

float svgPointDistance(SVGPoint * p1, SVGPoint * p2) {
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    return sqrt(dx * dx + dy * dy);
}

float svgPointDistanceSA(SVGPoint * p1, SVGPoint * p2) {
    float midx1 = (p1->x + p1->xn) / 2;
    float midy1 = (p1->y + p1->yn) / 2;
    float midx2 = (p2->x + p2->xn) / 2;
    float midy2 = (p2->y + p2->yn) / 2;

    float dx = midx1 - midx2;
    float dy = midy1 - midy2;
    return sqrt(dx * dx + dy * dy);
}

float tour_distance(Shape* shapes, SVGPoint* points, int pathCount){
  double total = 0.0;
  for(int i = 0; i < pathCount - 1; i++){ //pathCount -1?
    total += svgPointDistance(&points[shapes[i].id], &points[shapes[i+1].id]);
  }
  return total;
}

float randomFloat() {
  return (float)rand() / (float)RAND_MAX ;
}

//Calculate and return the perpendicular distance from a line drawn between (x1, y1) and (x2, y2) and the point (px, py)
float distanceLineToPoint(float px, float py, float x1, float y1, float x2, float y2) {
    float result = fabs((y2-y1)*px - (x2-x1)*py + x2*y1 - y2*x1) / sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));
    return result;
}

//DouglasPeucker path simplification algorithm.
void DouglasPeucker(FILE * gcode, float *points, int startIndex, int endIndex, float epsilon, float *buffer, 
                    int *bufferIndex, int * pathPointsIndex, int *lastWritten, TransformSettings* settings) {
    float dmax = 0;
    int index = -1;

    //First and last point in divided segment.
    float x1 = (points[startIndex]) * (settings->scale);
    float y1 = (points[startIndex + 1]) * (settings->scale);
    float x2 = (points[endIndex]) * (settings->scale);
    float y2 = (points[endIndex + 1]) * (settings->scale);

    //if start and end of shape are in same location, handle this special case.
    if(x1 == x2 && y1 == y2 && startIndex == 0 && endIndex == *pathPointsIndex -2){
#ifdef DP_DEBUG_OUTPUT
      fprintf(gcode, "( Same start and end point )\n");
#endif
      float maxDist = 0;
      int maxIndex = -1;
      for(int i = startIndex + 2; i < endIndex - 2; i += 2){
        float checkX = points[i] * settings->scale;
        float checkY = points[i+1] * settings->scale;
        float dist = distanceBetweenPoints(x1, y1, checkX, checkY);
        if(dist > maxDist){
          maxDist = dist;
          maxIndex = i;
        }
      }
      if(maxIndex == -1) {
#ifdef DP_DEBUG_OUTPUT
        fprintf(gcode, "( Writing Point to Buffer at Index: %d | X: %.4f, Y: %.4f )\n", *bufferIndex, points[startIndex], points[startIndex + 1]);
#endif
        buffer[*bufferIndex] = (points[startIndex]);// - bounds[0]); //x 
        buffer[*bufferIndex + 1] = (points[startIndex + 1]);// - bounds[1]); //y
        *bufferIndex += 2;
        return;
      } else {
        DouglasPeucker(gcode, points, startIndex, maxIndex, epsilon, buffer, bufferIndex, pathPointsIndex, lastWritten, settings);
        DouglasPeucker(gcode, points, maxIndex, endIndex, epsilon, buffer, bufferIndex, pathPointsIndex, lastWritten, settings);
        return;
      }
      //Once we found the furthest away point, call dp recursively.
      
    }

    //Iterate from first point after start, to last point before end. Find the x and y value.
    //Get the perpendicular distance to pN from line between pStart pEnd.
    //Find the maximum index of such point and track the value.
    for(int i = startIndex + 2; i < endIndex; i += 2) {
        float px = points[i] * settings->scale;
        float py = points[i + 1] * settings->scale;
        float d = distanceLineToPoint(px, py, x1, y1, x2, y2);
        if (d >= dmax) {
            index = i;
            dmax = d;
        }
    }
    
    //If the max dist is greater than epsilon, we want to preserve this point, so we can treat it as  the new end/start of
    //the recursive dp call.
    if(dmax > epsilon) {
        DouglasPeucker(gcode, points, startIndex, index, epsilon, buffer, bufferIndex, pathPointsIndex, lastWritten, settings);
        DouglasPeucker(gcode, points, index, endIndex, epsilon, buffer, bufferIndex, pathPointsIndex, lastWritten, settings);
    }
    else {
#ifdef DP_DEBUG_OUTPUT
        fprintf(gcode, "( Writing Point to Buffer at Index: %d | X: %.4f, Y: %.4f )\n", *bufferIndex, points[startIndex], points[startIndex + 1]);
#endif
        buffer[*bufferIndex] = (points[startIndex]);// - bounds[0]);
        buffer[*bufferIndex + 1] = (points[startIndex + 1]);// - bounds[1]);
        *bufferIndex += 2;
    }
    if (endIndex == *pathPointsIndex - 2 && (*lastWritten == 0)) {
#ifdef DP_DEBUG_OUTPUT
        fprintf(gcode, "( Writing Point to Buffer at Index: %d | X: %.4f, Y: %.4f )\n", *bufferIndex, points[endIndex], points[endIndex + 1]);
#endif
        buffer[*bufferIndex] = (points[endIndex]);// - bounds[0]);
        buffer[*bufferIndex + 1] = (points[endIndex + 1]);// - bounds[1]);
        *bufferIndex += 2;
        *lastWritten = 1;
    }
}

//Shape level path optimization algorithm. Check swaps, swap if better distance, repeat.
//Iterates until avg rate of improvement over AVG_OPT_WINDOW iterations is below some threshold.
void simulatedAnnealing(Shape* shapes, SVGPoint * points, int pathCount, double initialTemp, float coolingRate, int quality, int numComp) { //simulated annealing implementation for no test output.
  int count_swaps = 0;
  int count_cycles = 0;
  int tempInd;
  int sa_probability = 0;
  
  double elapsed_time = 0;
  double temp = initialTemp;
  double lastPrintTemp = initialTemp;
  float oldDist, newDist = 0;
  float previousDistance, currentDistance= tour_distance(shapes, points, pathCount);

  Shape tempShape;
  printf("Un-Optimized/Un-Scaled Non-Write Travel: %f\n", currentDistance);
  printf("Initial Temp :%f\n", initialTemp);

  clock_t current_opt_time, opt_start_time = clock();

  //Track improvement over last 10 iterations.
  double dist_imp_tracking[AVG_OPT_WINDOW];
  int dist_it = 0;
  double dist_improvement = 0;
  double dist_avg_improvement = 1;

  while (((dist_avg_improvement < -50) || (dist_it < AVG_OPT_WINDOW)) && ((clock() - opt_start_time) / (double)CLOCKS_PER_SEC < MAX_OPT_SECONDS)) {
    current_opt_time = clock();
    elapsed_time = (current_opt_time - opt_start_time) / (double)CLOCKS_PER_SEC;

    dist_avg_improvement = 0;
    previousDistance = currentDistance;
    for(int i = 0; i < numComp; i++) {
      //Based on stipplegen 2-opt heuristic.
      int pointA = rand() % (pathCount - 2);
      int pointB = rand() % (pathCount - 2);

      if(abs(pointB - pointA) < 2) {
        continue;
      }

      if(pointB > pathCount -2 || pointA > pathCount-2){ //See if this stops random segfaults?
        continue;
      }

      if(pointB < pointA) {
        tempInd = pointB;
        pointB = pointA;
        pointA = tempInd;
      } 

      //update old dist and new dist based on nearest end/start point of shape we are swapping to. Need to have points track endpoints as well.
      oldDist = svgPointDistanceSA(&points[shapes[pointA].id], &points[shapes[pointA+1].id]) + svgPointDistanceSA(&points[shapes[pointB].id], &points[shapes[pointB+1].id]);
      newDist = svgPointDistanceSA(&points[shapes[pointA].id], &points[shapes[pointB].id]) + svgPointDistanceSA(&points[shapes[pointB+1].id], &points[shapes[pointA+1].id]);

      if(newDist < oldDist){
        count_swaps++;
        int indexRight = pointB;  
        int indexLeft = pointA + 1;
        while(indexRight > indexLeft) {
          tempShape = shapes[indexLeft];
          shapes[indexLeft] = shapes[indexRight];
          shapes[indexRight] = tempShape;
          indexRight--;
          indexLeft++;
        }
        currentDistance -= oldDist - newDist;
      }
    }

    dist_improvement = currentDistance - previousDistance; //Current distance *should* be smaller than previous distance. So elements in arr will be negative.
    dist_imp_tracking[dist_it % AVG_OPT_WINDOW] = dist_improvement; //Update array with dist improvement.

    int dist_avg_it = MININT(dist_it++, AVG_OPT_WINDOW); //find number of elements in array, min of dist_it +1 or AVG_OPT_WINDOW

    for(int i = 0; i < dist_avg_it; i++){ //iterate through array and increment the avg accumulator.
      dist_avg_improvement += dist_imp_tracking[i];
    }
    
    dist_avg_improvement = dist_avg_improvement/dist_avg_it; //divide accumulator by the number of elements.

    count_cycles++;
    temp *= 1 - coolingRate;

    if (lastPrintTemp - temp >= 0.1 * lastPrintTemp) {
        // printf("  Distance Improvement %f\n", tour_distance(shapes, points, pathCount) - previousDistance);
        printf("  Avg Improvement over last %d: %f\n", AVG_OPT_WINDOW, dist_avg_improvement);
        printf("  Elapsed Time: %f\n", elapsed_time);
        lastPrintTemp = temp;
        fflush(stdout);
    }
    if (elapsed_time >= MAX_OPT_SECONDS) {
        break;
    }
  }

  printf("Un-Scaled Non-Write Travel: %f\n", tour_distance(shapes, points, pathCount));
  printf("Number of swaps %d\n", count_swaps);
  printf("Number of iterations %d\n", count_cycles);
  fflush(stdout);
}

TransformSettings calculateScaleForContentsToDrawspace(TransformSettings settings, float* bounds) {
    // code specific for contentsToDrawspace
    printf("  Forcing scale to drawSpace\n");
    settings.scale = 1;
    float pointsWidth = bounds[2] - bounds[0];
    float pointsHeight = bounds[3] - bounds[1];
    if (settings.swapDim) {
        printf("Swapping points dim in contents to drawspace\n");
        float temp = pointsWidth;
        pointsWidth = pointsHeight;
        pointsHeight = temp;
    }
    printf("Points wdith:%f, Points height:%f\n", pointsWidth, pointsHeight);
    float pointsRatio = pointsWidth / pointsHeight;
    float drawSpaceRatio = settings.drawSpaceWidth / settings.drawSpaceHeight;
    settings.scale = (drawSpaceRatio > pointsRatio) ? (settings.drawSpaceHeight / pointsHeight) : (settings.drawSpaceWidth / pointsWidth);
    settings.loadedFileWidth = pointsWidth;
    settings.loadedFileHeight = pointsHeight;
    return settings;
}

TransformSettings calculateScaleForFitToMaterial(TransformSettings settings) {
    // code specific for fitToMaterial
    printf("  FitToMat\n");
    float materialRatio = settings.drawSpaceWidth / settings.drawSpaceHeight;
    float svgRatio = settings.loadedFileWidth / settings.loadedFileHeight;
    settings.scale = (materialRatio > svgRatio) ? (settings.drawSpaceHeight / settings.loadedFileHeight) : (settings.drawSpaceWidth / settings.loadedFileWidth);
    settings.shiftX = settings.xMarginLeft;
    settings.shiftY = settings.yMarginTop;
    if (settings.centerOnMaterial) {
      settings.shiftX = settings.xMarginLeft + ((settings.drawSpaceWidth - settings.loadedFileWidth) / 2);
      settings.shiftY = settings.yMarginTop + ((settings.drawSpaceHeight - settings.loadedFileHeight) / 2);
      printf("If centerOnMaterial shiftX:%f, shiftY:%f\n", settings.shiftX, settings.shiftY);
  }


  return settings;
}

TransformSettings calculateScaleForNotFitToMaterial(TransformSettings settings) {
    // code specific for !fitToMaterial
    printf("  Not FitToMat\n");
    settings.scale = 1;
    return settings;
}

TransformSettings calcShiftAndCenter(TransformSettings settings) {
    settings.loadedFileWidth = settings.loadedFileWidth * settings.scale;
    settings.loadedFileHeight = settings.loadedFileHeight * settings.scale;
    printf("Scaled loadedFileWidth: %f Scaled loadedFileHeight: %f\n", settings.loadedFileWidth, settings.loadedFileHeight);
    settings.shiftX = settings.xMarginLeft; 
    settings.shiftY = settings.yMarginTop;  

    // If centering on material, calculate shift
    if (settings.centerOnMaterial) {
        settings.shiftX = settings.xMarginLeft + ((settings.drawSpaceWidth - settings.loadedFileWidth) / 2);
        settings.shiftY = settings.yMarginTop + ((settings.drawSpaceHeight - settings.loadedFileHeight) / 2);
        printf("If centerOnMaterial shiftX:%f, shiftY:%f\n", settings.shiftX, settings.shiftY);
    }

    // Calculate center of scaled and rotated drawing. 
    settings.centerX = settings.shiftX + settings.loadedFileWidth / 2;
    settings.centerY = settings.shiftY + settings.loadedFileHeight / 2;
    settings.originalCenterX = settings.centerX;
    settings.originalCenterY = settings.centerY;

    if(settings.swapDim) {
        settings.originalCenterX = settings.shiftX + settings.loadedFileHeight/2;
        settings.originalCenterY = settings.shiftY + settings.loadedFileWidth/2;
    }

    if(settings.contentsToDrawspace) { // Need to apply a shift when scaling contents to drawspace.
        settings.shiftX -= (settings.xInsetLeft * settings.scale);
        settings.shiftY -= (settings.yInsetTop * settings.scale);
    }

    printf("originalCenterX:%f, originalCenterY:%f\n", settings.originalCenterX, settings.originalCenterY);
    printf("centerX:%f, centerY:%f\n", settings.centerX, settings.centerY);
    fflush(stdout);

    settings.cosRot = cos((90 * settings.svgRotation) * (M_PI / 180)); 
    settings.sinRot = sin((90 * settings.svgRotation) * (M_PI / 180));

    return settings;
}

//All calculations for shift and rotation are handled here.
TransformSettings calcTransform(NSVGimage * g_image, float * paperDimensions, int * generationConfig){
  TransformSettings settings;

  //Width and height of the document in mm.
  settings.contentsToDrawspace = generationConfig[10];
  float width = paperDimensions[9];
  float height = paperDimensions[10];
  settings.pointsToDocumentScale = 1;

  settings.loadedFileWidth = width; //Width and height of svg from frontend.
  settings.loadedFileHeight = height;
  settings.svgRotation = generationConfig[2]; //Rotation amount /90
  settings.xMarginLeft = paperDimensions[2]; //Margins
  settings.yMarginTop = paperDimensions[3];
  settings.xMarginRight = paperDimensions[7];
  settings.yMarginBottom = paperDimensions[8];
  settings.drawSpaceWidth = paperDimensions[0] - settings.xMarginLeft - settings.xMarginRight; //Amount of space for drawing.
  settings.drawSpaceHeight = paperDimensions[1] - settings.yMarginTop - settings.yMarginBottom;
  settings.pointBounds = bounds; //[xmin, ymin, xmax, ymax] bounding box of points in doc.
  //bounds of points
  
  printf("Viewbox info from g_image: viewMinx: %f, viewMiny: %f, viewWidth: %f, viewHeight: %f, alignType: %d\n", g_image->viewMinx, g_image->viewMiny, g_image->viewWidth, g_image->viewHeight, g_image->alignType);
  printf("Image width:%f, Image Height:%f\n", settings.loadedFileWidth, settings.loadedFileHeight);
  printf("PaperDimensions X:%f, PaperDimension Y:%f\n",paperDimensions[0], paperDimensions[1]);
  printf("drawSpaceWidth: %f, drawSpaceHeight:%f\n", settings.drawSpaceWidth, settings.drawSpaceHeight);

  settings.swapDim = (generationConfig[2] == 1 || generationConfig[2] == 3);

  // Swap width and height if necessary
  if (settings.swapDim) {
    float temp = settings.loadedFileWidth;
    settings.loadedFileWidth = settings.loadedFileHeight;
    settings.loadedFileHeight = temp;
    printf("Swapped image width:%f Image Height:%f\n", settings.loadedFileWidth, settings.loadedFileHeight);
  }

  //assume g_image->alignType = 0 for now. Currently inset values are not scaled to paper dimensions.
  settings.xInsetLeft = bounds[0] - g_image->viewMinx; // Left side of point bounding box distance from viewbox
  settings.xInsetRight = (g_image->width) - bounds[2]; // Right side
  settings.yInsetTop = bounds[1] - g_image->viewMiny; // Top side
  settings.yInsetBottom = (g_image->height) - bounds[3]; // Bottom side

  for (int i = 0; i < settings.svgRotation; i++) {
    float tempInset = settings.xInsetLeft;
    settings.xInsetLeft = settings.yInsetBottom;
    settings.yInsetBottom = settings.xInsetRight;
    settings.xInsetRight = settings.yInsetTop;
    settings.yInsetTop = tempInset;
  }

  // Determine if fitting to material is necessary
  // If file is too large, or we selected to fitToMaterial.
  settings.fitToMaterial = ((settings.loadedFileWidth > settings.drawSpaceWidth) || (settings.loadedFileHeight > settings.drawSpaceHeight) || generationConfig[0]); 
  printf("Fit to material: %d\n", settings.fitToMaterial);
  settings.centerOnMaterial = generationConfig[1];
  //SCALE CALCULATIONS

  if (settings.contentsToDrawspace) {
    settings = calculateScaleForContentsToDrawspace(settings, bounds);
    goto finish_calc;
  } else if (settings.fitToMaterial) { //If fit to material is toggled or SVG is larger than draw space.
    settings = calculateScaleForFitToMaterial(settings);
  } else {
    settings = calculateScaleForNotFitToMaterial(settings);
  }

  finish_calc:
  //END SCALE CALCULATIONS
  printf("Scale%f\n", settings.scale);
  settings = calcShiftAndCenter(settings);

  return settings;
}

void printTransformSettings(TransformSettings settings) {
  printf("Point Bounds\n");
  for(int i = 0; i < 4; i++){
    printf("    Bounds[%d] = %f\n", i, bounds[i]);
  }
  printf("\nscale: %f\n", settings.scale);
  printf("pointsToDocumentScale: %f\n", settings.pointsToDocumentScale);
  printf("drawingWidth: %f\n", settings.loadedFileWidth);
  printf("drawingHeight: %f\n", settings.loadedFileHeight);
  printf("drawSpaceWidth: %f\n", settings.drawSpaceWidth);
  printf("drawSpaceHeight: %f\n", settings.drawSpaceHeight);
  printf("loadedFileWidth: %f\n", settings.loadedFileWidth);
  printf("loadedFileHeight: %f\n", settings.loadedFileHeight);
  printf("shiftX: %f\n", settings.shiftX);
  printf("shiftY: %f\n", settings.shiftY);
  printf("centerX: %f\n", settings.centerX);
  printf("centerY: %f\n", settings.centerY);
  printf("originalCenterX: %f\n", settings.originalCenterX);
  printf("originalCenterY: %f\n", settings.originalCenterY);
  printf("cosRot: %f\n", settings.cosRot);
  printf("sinRot: %f\n", settings.sinRot);
  printf("xmarginleft: %f\n", settings.xMarginLeft);
  printf("xmarginright: %f\n", settings.xMarginRight);
  printf("ymargintop: %f\n", settings.yMarginTop);
  printf("ymarginbottom: %f\n", settings.yMarginBottom);
  printf("xInsetLeft: %f\n", settings.xInsetLeft);
  printf("xInsetRight: %f\n", settings.xInsetRight);
  printf("yInsetTop: %f\n", settings.yInsetTop);
  printf("yInsetBottom: %f\n", settings.yInsetBottom);
  printf("fitToMaterial: %d\n", settings.fitToMaterial);
  printf("centerOnMaterial: %d\n", settings.centerOnMaterial);
  printf("swapDim: %d\n", settings.swapDim);
  printf("svgRotation: %d\n", settings.svgRotation);
  printf("contentsToDrawspace: %d\n\n", settings.contentsToDrawspace);
}

float rotateX(TransformSettings* settings, float firstx, float firsty) {
  float rotatedX = (firstx - settings->originalCenterX) * settings->cosRot - (firsty - settings->originalCenterY) * settings->sinRot + settings->centerX;
  return rotatedX;
}

float rotateY(TransformSettings* settings, float firstx, float firsty) {
  float rotatedY = (firstx - settings->originalCenterX) * settings->sinRot + (firsty - settings->originalCenterY) * settings->cosRot + settings->centerY;
  return rotatedY;
}

void printGCodeState(GCodeState* state) {
  printf("\n");  // Start with newline
  printf("npaths: %d\n", state->npaths);
  printf("quality: %d\n", state->quality);
  printf("feed: %d\n", state->feed);
  printf("precision: %f\n", state->precision);
  printf("feedY: %d\n", state->feedY);
  printf("zFeed: %d\n", state->zFeed);
  printf("tempFeed: %d\n", state->tempFeed);
  printf("slowTravel: %d\n", state->slowTravel);
  printf("shapeStart: %d\n", state->shapeStart);
  printf("zFloor: %f\n", state->zFloor);
  printf("ztraverse: %f\n", state->ztraverse);
  printf("xy: %c\n", state->xy);
  printf("currColor: %d\n", state->currColor);
  printf("targetColor: %d\n", state->targetColor);
  printf("targetTool: %d\n", state->targetTool);
  printf("currTool: %d\n", state->currTool);
  printf("colorMatch: %d\n", state->colorMatch);
  printf("toolChangePos: %f\n", state->toolChangePos);
  printf("tol: %f\n", state->tol);
  printf("x: %f\n", state->x);
  printf("y: %f\n", state->y);
  printf("firstx: %f\n", state->firstx);
  printf("firsty: %f\n", state->firsty);
  printf("totalDist: %lf\n", state->totalDist);
  printf("xold: %f\n", state->xold);
  printf("yold: %f\n", state->yold);
  printf("colorCount: %d\n", state->colorCount);
  printf("colorToFile: %d\n", state->colorToFile);
  printf("useToolOffsets: %d\n", state->useToolOffsets);
  printf("\n");  // End with newline
}

GCodeState initializeGCodeState(float* paperDimensions, int* generationConfig, int* penColorCount){
  GCodeState state;
  
  state.quality = generationConfig[8];
  state.precision = paperDimensions[6];
  state.feed= generationConfig[5]; 
  state.feedY = generationConfig[5]; //Set x and y to same FR for now.
  state.zFeed = generationConfig[7];
  state.pointsCulledPrec = 0;
  state.pointsCulledBounds = 0;
  state.tempFeed = 0;
  state.slowTravel = 3500;
  state.shapeStart = 1;
  state.zFloor = paperDimensions[4];
  state.ztraverse = paperDimensions[5];
  state.xy = 1;
  state.currColor = -1;
  state.targetColor = 0;
  state.targetTool = 0;
  state.currTool = -1;
  state.colorMatch = 0;
  state.toolChangePos = -51.5;

  if(state.quality == 2){
    state.tol = 0.25;
  } else if (state.quality == 1){
    state.tol = 0.5;
  } else {
    state.tol = 1;
  }

  state.maxPaths = 0;
  state.xold = 0;
  state.yold = 0;

  state.npaths = 0;
  state.x = 0;
  state.y = 0;
  state.firstx = 0;
  state.firsty = 0;
  state.tempx = 0;
  state.tempy = 0;
  state.trackedDist = 0;
  state.totalDist = 0;
  state.brushDist = 1000000; //for testing right now. 1,000,000 = 1km should be around normal for a bp pen.
  state.countIntermediary = 0;
  state.colorToFile = 1;
  state.pathPointsBufIndex = 0;

  for(int i = 0; i < 6; i++){
    state.colorCount += penColorCount[i];
  }

  state.colorToFile = (state.colorCount > 1) && generationConfig[9];
  state.useToolOffsets = generationConfig[11];
  if(state.useToolOffsets){
    state.zFloor = 0;
  }

  return state;
}

void toolDown(FILE * gcode, GCodeState * gcodeState, int * machineTypePtr){
  fprintf(gcode, "G1 Z%f F%d\n", gcodeState->zFloor, gcodeState->zFeed);
}

void toolUp(FILE * gcode, GCodeState * gcodeState, int * machineTypePtr){
  fprintf(gcode, "G1 Z%f F%d\n", gcodeState->ztraverse, gcodeState->zFeed);
}

char* uint_to_hex_string(unsigned int num) {
    // Determine the number of characters required for the hex representation
    unsigned int temp = num;
    int num_chars = 0;
    do {
        temp >>= 4; // Right-shift by 4 bits (equivalent to division by 16)
        num_chars++;
    } while (temp != 0);

    // Allocate memory for the string representation (+1 for null terminator)
    char* result = (char*)malloc((num_chars + 1) * sizeof(char));

    // Convert to hex characters and store them in the buffer (in reverse order)
    int index = num_chars - 1;
    do {
        int hex_digit = num & 0xF; // Mask the lowest 4 bits to get the hex digit
        result[index] = (hex_digit < 10) ? ('0' + hex_digit) : ('A' + hex_digit - 10);
        num >>= 4; // Right-shift by 4 bits to move to the next hex digit
        index--;
    } while (num != 0);

    // Add null terminator at the end of the string
    result[num_chars] = '\0';

    return result;
}

void writeToolOffset(FILE* gcode, int tool){
  fprintf(gcode, "( TOOL OFFSET CMD )\n");
  int offset = 54 + tool;
  fprintf(gcode, "G%d\n", offset);
}

void writeToolchange(GCodeState* gcodeState, int machineType, FILE* gcode, int numTools, Pen* penList, int* penColorCount, Shape* shapes, int * i) {
  if(machineType == 0 || machineType == 2 || machineType == 1){ //All machines will want to check for tool change eventually.
    gcodeState->targetColor = shapes[*i].stroke;
    //Hopefully this sets target tool without looking at currTool. Target tool shouldnt change as long as shape[*i].color is same as prev.
    gcodeState->targetColor = shapes[*i].stroke;
    if(gcodeState->targetColor != gcodeState->currColor){
      for(int tool = 0; tool < numTools; tool++){ //iterate through all 6 possible pen numbers.
      int target_color = shapes[*i].stroke; // This is the color we are trying to match against.
        for(int col = 0; col < penColorCount[tool]; col++){ //for the number of colors associated with each tool. If target_colorin penList[tool].colors
          if(penList[tool].colors[col] == target_color){
            gcodeState->currColor = target_color;
            gcodeState->targetTool = tool;
            break;
          }
        }
      }
    }

    if(gcodeState->targetTool != gcodeState->currTool){ //This is true if toolchange/pickup is neccesary.

#ifdef DEBUG_OUTPUT
  fprintf(gcode, "    ( Beginning Toolchange )\n");
  fprintf(gcode, "    ( Current Tool:%d, Target Tool:%d )\n", gcodeState->currTool, gcodeState->targetTool);
  printf("    ( Beginning Toolchange )\n");
  printf("    ( Current Tool:%d, Target Tool:%d )\n", gcodeState->currTool, gcodeState->targetTool);
  fflush(stdout);
#endif
      if(gcodeState->useToolOffsets){ //Back to absolute position for toolchanges
        fprintf(gcode, "G53\n");
      }

      if(machineType == 0){ //Tool change routine for 6-Color rotary.
        if(gcodeState->currTool >= 0){
          fprintf(gcode, "G1 A%d\n", gcodeState->currTool*60);
          fprintf(gcode, "G1 Z%i F%i\n", 0, gcodeState->zFeed);
          fprintf(gcode, "G0 X0\n");
          fprintf(gcode, "G1 X%f F%i\n", gcodeState->toolChangePos, gcodeState->slowTravel);
          fprintf(gcode, "G1 X0 F%d\n", gcodeState->slowTravel);
          fprintf(gcode, "G1 A%d\n", gcodeState->targetTool*60);
          fprintf(gcode, "G0 X0\n");
          fprintf(gcode, "G1 X%f F%i\n", gcodeState->toolChangePos, gcodeState->slowTravel);
          fprintf(gcode, "G1 X0 F%i\n", gcodeState->slowTravel);
          gcodeState->currTool = gcodeState->targetTool;
        }
        if(gcodeState->currTool == -1){
          fprintf(gcode, "G1 A%d\n", gcodeState->targetTool*60);
          fprintf(gcode, "G1 Z%i F%i\n", 0, gcodeState->zFeed);
          fprintf(gcode, "G0 X0\n");
          fprintf(gcode, "G1 X%f F%d\n", gcodeState->toolChangePos ,gcodeState->slowTravel);
          fprintf(gcode, "G1 X0 F%d\n", gcodeState->slowTravel);
        }
      } else if (machineType == 2 || machineType == 1){ //Pause command for MVP and LFP. Move to 0,0 and pause.
        fprintf(gcode, "( MVP PAUSE COMMAND TOOL:%d )\n", gcodeState->targetTool);
        fprintf(gcode, "G0 Z%f\nG0 X0\nG0 Y0\n", gcodeState->ztraverse);
        fprintf(gcode, "M0\n");
      }
      
      toolUp(gcode, gcodeState, &machineType);
      gcodeState->x = 0;
      gcodeState->currTool = gcodeState->targetTool;
      if(gcodeState->useToolOffsets){ //Back to new tool offset when using offsets.
        writeToolOffset(gcode, gcodeState->currTool);
      }
      
#ifdef DEBUG_OUTPUT
      fprintf(gcode, "    ( Ending Toolchange )\n");
#endif
    }
  }
  fflush(stdout);
}

void writeFooter(GCodeState* gcodeState, FILE* gcode, int machineType) { //End of job footer + cleanup.
  if (machineType == 0){ //Lift to zero for tool dropoff after job
    fprintf(gcode, "G1 Z%.1f F%i\n", 0.0, gcodeState->zFeed);
  }
  //drop off current tool
  if(machineType == 0){ //6Color
    fprintf(gcode, "G1 A%d\n", gcodeState->currTool*60); //rotate to current color slot
    fprintf(gcode, "G0 X0\n"); //rapid move to close to tool changer
    fprintf(gcode, "G1 X%f\n", gcodeState->toolChangePos); //slow move to dropoff
    fprintf(gcode, "G1 X0\n"); //slow move away from dropoff
  }

  gcodeState->totalDist = gcodeState->totalDist/1000; //conversion to meters
  //send paper to front
  fprintf(gcode, "G0 X0 Y0\n");
  if(machineType == 0){
    fprintf(gcode, "M5\nM30\n");
  } else if(machineType == 1 || machineType == 2){
    fprintf(gcode,"M5\nM2\n");
  }
  fprintf(gcode, "( Total distance traveled = %f m)\n", gcodeState->totalDist);
  fprintf(gcode, "( Intermediary Points: %d )\n", gcodeState->countIntermediary);
  fprintf(gcode, "( PointsCulledPrec: = %d, PointsCulledBounds: = %d)\n", gcodeState->pointsCulledPrec, gcodeState->pointsCulledBounds);
#ifdef DEBUG_OUTPUT
  fprintf(gcode, " (MaxPaths in a shape: %i)\n", gcodeState->maxPaths);
#endif
}

void writeHeader(GCodeState* gcodeState, FILE* gcode, TransformSettings* settings, int machineType, float* paperDimensions) {
#ifdef DEBUG_OUTPUT
  fprintf(gcode, "( Machine Type: %i )\n", machineType);
#endif
  fprintf(gcode, "G90\nG0 M3 S%d\n", 90); //Default header for job
  fprintf(gcode, "G0 Z%f\n", gcodeState->ztraverse);

  if(machineType == 0 || machineType == 2) { //6Color or MVP job paper back and forth.
    fprintf(gcode, "G1 Y0 F%i\n", gcodeState->feedY);
    fprintf(gcode, "G1 Y%f F%d\n", (-1.0*(settings->drawSpaceHeight + settings->yMarginTop - 10)), gcodeState->feedY);
    fprintf(gcode, "G1 Y0 F%i\n", gcodeState->feedY);
  }
}

int firstPoint(int * sp, int * ptIndex, int * pathPointIndex){ //check if current point is first point in shape
  return (*sp == 0 && *ptIndex == 0) || (*sp == 1 && *ptIndex == *pathPointIndex -2 );
}

int lastPoint(int * sp, int * ptIndex, int * pathPointIndex){ //check if current point is last point in shape
  return (*sp == 0 && *ptIndex == *pathPointIndex - 2) || (*sp == 1 && *ptIndex == 0);
}

//Reason == 0 -> outOfBounds, Reason == 1 -> first || last, Reason == 2-> Normal point to write.
int canWritePoint(GCodeState * gcodeState, TransformSettings * settings, int * sp, int  * ptIndex, int * pathPointIndex, float * px, float * py, FILE * gcode, int* writeReason){ //always want to write if it is first or last point in a shape.
  //want a preliminary check that the coordinates are within bounds.
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Checking Point: X:%f Y:%f )\n", *px, *py);
#endif
  if ((*px < 0 || *px > settings->drawSpaceWidth + settings->xMarginLeft + settings->xMarginRight) || (*py > 0 || *py < -1*(settings->drawSpaceHeight + settings->yMarginTop + settings->yMarginBottom))){
    gcodeState->pointsCulledBounds++;
     *writeReason = 0;
    return 1;
  } else if(firstPoint(sp, ptIndex, pathPointIndex) || lastPoint(sp, ptIndex, pathPointIndex)){ //Always write first and last point in a shape.
    *writeReason = 1;
    return 1;
  } 
  *writeReason = 2;
  return 1;
}

int toolRefresh(int * sp, int * ptIndex, int * pathPointIndex, GCodeState * gcodeState, int * dist){
  return (!firstPoint(sp, ptIndex, pathPointIndex)) && (gcodeState->trackedDist + *dist > gcodeState->brushDist);
}

//Rotation/shifting application, and writing is handled in this function.
void writePoint(FILE * gcode, FILE* color_gcode, GCodeState * gcodeState, TransformSettings * settings, int * ptIndex, char * isClosed, int * machineType, int * sp, int * pathPointIndex, int* pointsWritten) {
    float rotatedX, rotatedY, feedRate;
    float dist = 0.0;
    
    // Get the unscaled and unrotated coordinates from pathPoints
    float x = gcodeState->pathPoints[*ptIndex];
    float y = gcodeState->pathPoints[(*ptIndex)+1];
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Un-Rotated/Scaled X: %f, Y: %f )\n", x, y);
#endif
    
    // Scale and shift the coordinates
    float scaledX = x*settings->scale + settings->shiftX;
    float scaledY = y*settings->scale + settings->shiftY;
    
    // Rotate the coordinates if needed
    if(settings->svgRotation > 0){
        rotatedX = rotateX(settings, scaledX, scaledY);
        rotatedY = rotateY(settings, scaledX, scaledY);
    } else {
        rotatedX = scaledX;
        rotatedY = scaledY;
    }
    rotatedY = -rotatedY;
    int writeReason = -1;
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Scaled and Rotated X:%f Y:%f )\n", rotatedX, rotatedY);
#endif

    if(canWritePoint(gcodeState, settings, sp, ptIndex, pathPointIndex, &rotatedX, &rotatedY, gcode, &writeReason)){ //Can write, if first or last in shape, or if dist is large enough.
      gcodeState->xold = gcodeState->x; //Update state tracking if we are writing.
      gcodeState->yold = gcodeState->y;
      gcodeState->x = rotatedX;
      gcodeState->y = rotatedY;
      dist = distanceBetweenPoints(gcodeState->xold, gcodeState->yold, rotatedX, rotatedY);

      if(!firstPoint(sp, ptIndex, pathPointIndex)){ //Intermediary Point check logic.
        gcodeState->trackedDist += dist;
        if(gcodeState->trackedDist >= gcodeState->brushDist){ 
          gcodeState->tempx = gcodeState->xold;
          gcodeState->tempy = gcodeState->yold;
          int numIntermediary = (int)(gcodeState->trackedDist/gcodeState->brushDist); //Cast to int rounds down to floor.
          float dirX, dirY, px, py, mag = 0; //variables for calculating intermediary points.
          float distToPoint;
          for(int i = 0; i < numIntermediary; i++){
            if(i == 0){
              distToPoint = gcodeState->trackedDist-gcodeState->brushDist;
            } else {
              distToPoint = gcodeState->brushDist;
            }
            dirX = rotatedX - gcodeState->tempx;
            dirY = rotatedY - gcodeState->tempy;
            mag = sqrt(dirX*dirX + dirY*dirY);
            dirX = dirX / mag;
            dirY = dirY / mag;
            px = gcodeState->tempx + (distToPoint * dirX);
            py =  gcodeState->tempy + (distToPoint * dirY);
            gcodeState->tempx = px;
            gcodeState->tempy = py;
            //write out to point.
            fprintf(gcode, "( Intermediary point X:%.4f Y:%.4f)\n", px, py);
          }
          gcodeState->countIntermediary += numIntermediary;
          //set tracked dist back to dist from last intermediary point to rotatedX and rotatedY.
          gcodeState->trackedDist = distanceBetweenPoints(gcodeState->tempx, gcodeState->tempy, rotatedX, rotatedY);
        }

        gcodeState->totalDist += dist;
      }

      feedRate = interpFeedrate(gcodeState->feed, gcodeState->feedY, absoluteSlope(gcodeState->xold, gcodeState->yold, gcodeState->x, gcodeState->y));

      if (firstPoint(sp, ptIndex, pathPointIndex) == 0) { //If not first point
        *pointsWritten += 1;
#ifdef DEBUG_OUTPUT
        //fprintf(gcode, "( Writing point with ptIndex: %d, pathPointIndex: %d, X: %.4f, Y: %.4f )\n", *ptIndex, *pathPointIndex, gcodeState->x, gcodeState->y);
#endif
        fprintf(gcode,"G1 X%.4f Y%.4f F%d\n", gcodeState->x, gcodeState->y, (int)feedRate);
        if(gcodeState->colorToFile && gcodeState->colorFileOpen){
          fprintf(color_gcode,"G1 X%.4f Y%.4f F%d\n", gcodeState->x, gcodeState->y, (int)feedRate);
        }
      } else { //if is first point
        *pointsWritten += 1;
#ifdef DEBUG_OUTPUT
        //fprintf(gcode, "( Writing first point with ptIndex: %d, pathPointIndex: %d, X: %.4f, Y: %.4f )\n", *ptIndex, *pathPointIndex, gcodeState->x, gcodeState->y);
#endif
        fprintf(gcode,"G0 X%.4f Y%.4f\n", gcodeState->x, gcodeState->y);
        if(gcodeState->colorToFile && gcodeState->colorFileOpen){
          fprintf(color_gcode, "G0 X%.4f Y%.4f\n", gcodeState->x, gcodeState->y);
        }
      }    
    } else {
#ifdef DEBUG_OUTPUT
      fprintf(gcode, "( Cant write point. Reason: %d )\n", writeReason);
#endif
    }

    if(firstPoint(sp, ptIndex, pathPointIndex)){ //if first point written in path
      gcodeState->firstx = rotatedX;
      gcodeState->firsty = rotatedY;
      toolDown(gcode, gcodeState, machineType);
      if(gcodeState->colorToFile && gcodeState->colorFileOpen){
        toolDown(color_gcode, gcodeState, machineType);
      }
    }
  }

int nearestStartPoint(FILE *gcode, GCodeState *gcodeState, TransformSettings *settings, int pathPointsIndex) {
    int res = 0; //Set to 1 if end is closer to last x and y points in gcodestate, 0 if start is closer.
    float rx1, rx2, ry1, ry2;
    float x1 = (gcodeState->pathPoints[0]) *settings->scale + settings->shiftX;
    float y1 = (gcodeState->pathPoints[1]) *settings->scale + settings->shiftY;;
    float x2 = (gcodeState->pathPoints[pathPointsIndex-2]) *settings->scale + settings->shiftX;
    float y2 = (gcodeState->pathPoints[pathPointsIndex-1]) *settings->scale + settings->shiftY;;

    if(settings->svgRotation > 0) {
        rx1 = rotateX(settings, x1, y1);
        ry1 = rotateY(settings, x1, y1);
        rx2 = rotateX(settings, x2, y2);
        ry2 = rotateY(settings, x2, y2);
    } else {
        rx1 = x1;
        ry1 = y1;
        rx2 = x2;
        ry2 = y2;
    }
    
    ry1 = -ry1;
    ry2 = -ry2;
    
    float distFromStart = distanceBetweenPoints(rx1, ry1, gcodeState->x, gcodeState->y);
    float distFromEnd = distanceBetweenPoints(rx2, ry2, gcodeState->x, gcodeState->y);
    if (distFromEnd < distFromStart){
      res = 1;
    }

    return res;
}

void writeShape(FILE * gcode, FILE* color_gcode, GCodeState * gcodeState, TransformSettings * settings, Shape * shapes, ToolPath * toolPaths, int * machineTypePtr, int * k, int * i) { //k is index in toolPaths. i is index i shapes.
    float rotatedX, rotatedY, rotatedBX, rotatedBY, tempRot;
    int j, l; //local iterators with k <= j, l < npaths;

    // printf("Shape %d at i:%d. Color: %i\n", shapes[*i].id, *i, shapes[*i].stroke);
    // printf("Scale: %f, Rotation: %d, ShiftX: %f, ShiftY: %f\n", settings->scale, settings->svgRotation, settings->shiftX, settings->shiftY);
    
    gcodeState->pathPoints[0] = toolPaths[*k].points[0]; //first points into pathPoints. Not yet scaled or rotated.
    gcodeState->pathPoints[1] = toolPaths[*k].points[1]; //0, 1 are startpoint of path. (x, y)

    int pathPointsIndex = 2;
    for(j = *k; j < gcodeState->npaths; j++) {
        int level;
        if(toolPaths[j].shape == shapes[*i].id) {
            bezCount = 0;
            level = 0;
            
            cubicBez(toolPaths[j].points[0], toolPaths[j].points[1], toolPaths[j].points[2], toolPaths[j].points[3], toolPaths[j].points[4], toolPaths[j].points[5], toolPaths[j].points[6], toolPaths[j].points[7], gcodeState->tol, level);
            for(l = 0; l < bezCount; l++) {
              //unscaled and un-rotated bez points into pathPoints.
              gcodeState->pathPoints[pathPointsIndex] = bezPoints[l].x;
              gcodeState->pathPoints[pathPointsIndex + 1] = bezPoints[l].y;
              pathPointsIndex += 2; //pathPointsIndex-2 is x of endpoint
            }

            toolPaths[j].shape = -1; // This path has been written
        } else {
            break;
        }
    }

    int bufferIndex = 0;
    int lastWritten = 0;
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Starting Shape! )\n");
#endif
#ifdef DP_DEBUG_OUTPUT
    fprintf(gcode, "( Points before DouglasPeucker algorithm: )\n");
    for(int z = 0; z < pathPointsIndex; z += 2) {
        fprintf(gcode, "( X: %.4f, Y: %.4f )\n", gcodeState->pathPoints[z], gcodeState->pathPoints[z + 1]);
    }
#endif

    //Optimize points into gcodeState->pathPointsBuf. Write points from pathPointsBuf, not pathPoints.
    DouglasPeucker(gcode, gcodeState->pathPoints, 0, pathPointsIndex - 2, DOUGLAS_PEUCKER_EPSILON, gcodeState->pathPointsBuf, &bufferIndex, &pathPointsIndex, &lastWritten, settings);
    memcpy(gcodeState->pathPoints, gcodeState->pathPointsBuf, bufferIndex * sizeof(float));
    //track number of culled points.
    gcodeState->pointsCulledPrec += (int)(pathPointsIndex - bufferIndex)*(1/2);
    pathPointsIndex = bufferIndex; //set pathPointsIndex to optimized points buffer size.

#ifdef DEBUG_OUTPUTs
    fprintf(gcode, "( Points after DouglasPeucker algorithm: )\n");
    for(int z = 0; z < pathPointsIndex; z += 2) {
        fprintf(gcode, "( X: %.4f, Y: %.4f )\n", gcodeState->pathPoints[z], gcodeState->pathPoints[z + 1]);
    }
#endif

    char isClosed = toolPaths[j].closed;
    int sp = nearestStartPoint(gcode, gcodeState, settings, pathPointsIndex);
    int pointsWritten = 0;
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Shape %d at i:%d. Color: %i. Scale: %f, ShiftX: %f, ShiftY: %f, Rotation: %d )\n", shapes[*i].id, *i, shapes[*i].stroke, settings->scale, settings->shiftX, settings->shiftY, settings->svgRotation*90);
    fprintf(gcode, "( NumPoints: %d, PathPointsIndex: %d, SP: %d )\n", pathPointsIndex/2, pathPointsIndex, sp);
#endif
    if(sp){
      for(int z = pathPointsIndex-2; z >= 0; z-=2){ //write backwards if sp, forwards if else.
        // Print out point before writing
        writePoint(gcode, color_gcode, gcodeState, settings, &z, &isClosed, machineTypePtr, &sp, &pathPointsIndex, &pointsWritten);
      }
    } else {
      for(int z = 0; z < pathPointsIndex; z += 2){
        // Print out point before writing
        writePoint(gcode, color_gcode, gcodeState, settings, &z, &isClosed, machineTypePtr, &sp, &pathPointsIndex, &pointsWritten);
      }
    }
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Points Written: %d )\n", pointsWritten);
#endif

    toolUp(gcode, gcodeState, machineTypePtr);
    if(gcodeState->colorFileOpen && gcodeState->colorToFile){
      toolUp(color_gcode, gcodeState, machineTypePtr);
    }
}

void printArgs(int argc, char* argv[], int** penColors, int penColorCount[6], float paperDimensions[7], int generationConfig[9]) {
    int i, j;

    printf("argc:\n\t%d\n", argc);

    printf("argv:\n");
    for(i = 0; i < argc; i++) {
        printf("\t%d: %s\n", i, argv[i]);
    }

    printf("penColors:\n");
    for(i = 0; i < numTools; i++) {
        printf("\t%d: ", i+1);
        for(j = 0; j < penColorCount[i]; j++) {
            printf("%d ", penColors[i][j]);
        }
        printf("\n");
    }

    printf("penColorCount:\n");
    for(i = 0; i < numTools; i++) {
        printf("\t%d: %d\n", i, penColorCount[i]);
    }

    printf("paperDimensions:\n");
    for(i = 0; i < 6; i++) {
        printf("\t%d: %.2f\n", i, paperDimensions[i]);
    }

    printf("generationConfig:\n");
    for(i = 0; i < 9; i++) {
        printf("\t%d: %d\n", i, generationConfig[i]);
    }
}


int compareShapes(const void* a, const void* b) {
    Shape* shapeA = (Shape*) a;
    Shape* shapeB = (Shape*) b;
    return (shapeA->id - shapeB->id);
}

//Paper Dimensions: {s.paperX(), s.paperY(), s.xMargin(), s.yMargin(), s.zEngage(), s.penLift(), s.precision(), s.xMarginRight(), s.yMarginBottom()}
//Generation Config: {scaleToMaterialInt, centerOnMaterialInt, s.svgRotation(), s.machineSelection(), s.quality(), s.xFeedrate(), s.yFeedrate(), s.zFeedrate(), s.quality()}

int generateGcode(int argc, char* argv[], int** penColors, int penColorCount[6], float paperDimensions[9], int generationConfig[12], char* fileName) {
  printf("In Generate GCode\n");
#ifdef DEBUG_OUTPUT
  printArgs(argc, argv, penColors, penColorCount, paperDimensions, generationConfig);
#endif
  int i, j, k, l = 1;
  SVGPoint* points;
  ToolPath* toolPaths;
  Shape *shapes; //Corresponds to an NSVGPath
  //all 6 tools will have their color assigned manually. If a path has a color not set in p1-6, assign to p1 by default.
  Pen *penList; //counts each color occurrence + the int assigned. (currently, assign any unknown/unsupported to p1. sum of set of pX should == nPaths;)
  GCodeState gcodeState = initializeGCodeState(paperDimensions, generationConfig, penColorCount);
  printGCodeState(&gcodeState);

  int machineType = generationConfig[3]; //machineType
  printf("Machine Type: %d\n", machineType);
  int ch;
 
  FILE *gcode;
  FILE *debug;
  FILE *color_gcode;

  //CLI argument BS may want to remove.
  printf("Argc:%d\n", argc);
  if(argc < 3) {
    return -1;
  }
  while((ch=getopt(argc,argv,"D:ABhf:n:s:Fz:Z:S:w:t:m:cTV1aLP:CY:X:")) != EOF) { //I think handoff between argc and argv is happening here so can't remove for sake of opening and outputting to a file.
    switch(ch) {
    case 'f': gcodeState.feed = atoi(optarg);
      break;
    case 't': gcodeState.tol = atof(optarg);
      break;
    default:
      return(1);
      break;
    }
  }
  //end cli parsing.

  clock_t start_parse, stop_parse;
  double parse_time;
  start_parse = clock();

  printf("File open string: %s\n", argv[optind]);
  printf("File output string: %s\n", argv[optind+1]);
  g_image = nsvgParseFromFile(argv[optind], "px", 96);
  if(g_image == NULL) {
    printf("error: Can't open input %s\n",argv[optind]);
    return -1;
  }

  stop_parse = clock();
  parse_time = ((double) (stop_parse - start_parse)) / CLOCKS_PER_SEC;

  //Bank of pens, their slot and their color. Pens also track count of shapes to be drawn with their color (for debug purposes)
  penList = (Pen*)malloc(numTools*sizeof(Pen));
  memset(penList, 0, numTools*sizeof(Pen));
  //assign pen colors for penColors input
  for(int i = 0; i<numTools;i++){
    //printf("Tool %d in penColors color: %d\n", i, penColors[i]);
    penList[i].colors = penColors[i]; //assign penList[i].colors to the pointer passed in from penColors (there are numtools poiners to assign.)
  }

  //
  calcBounds(g_image, numTools, penList, penColorCount);
  //Settings and calculations for rotation + transformation.
  TransformSettings settings = calcTransform(g_image, paperDimensions, generationConfig);
  printTransformSettings(settings);
  fflush(stdout);

#ifdef _WIN32
  seedrand((float)time(0));
#endif

 gcode=fopen(argv[optind+1],"w");
 if(gcode == NULL) {
   printf("can't open output %s\n",argv[optind+1]);
   return -1;
 }

 char *writeBuffer = malloc(BUFFER_SIZE);
 if(writeBuffer == NULL){
  printf("Write Buffer Alloc Failed\n");
  return -1;
 }

  if (setvbuf(gcode, writeBuffer, _IOFBF, BUFFER_SIZE) != 0) {
      printf("Failed to set buffer\n");
      return -1;
  }

  points = (SVGPoint*)malloc(pathCount*sizeof(SVGPoint));
  toolPaths = (ToolPath*)malloc(pointsCount*sizeof(ToolPath));
  shapes = (Shape*)malloc(pathCount*sizeof(Shape));
  memset(points, 0, pathCount*sizeof(SVGPoint));
  memset(toolPaths, 0, pointsCount*sizeof(ToolPath));
  memset(shapes, 0, pathCount*sizeof(Shape));
  gcodeState.npaths = 0;

  calcPaths(points, toolPaths, &gcodeState, shapes, debug);
  //alloc a worst case array for storing calculated draw points
  //malloc for (maxPathsinShape * maxNumberofPointsperBez * xandy * sizeofInt)
  gcodeState.pathPoints = malloc(gcodeState.maxPaths * MAX_BEZ * 2 * sizeof(float)); //points are stored as x on even y on odd, eg point p1 = (pathPoints[0],pathPoints[1]) = (x1,y1)
  gcodeState.pathPointsBuf = malloc(gcodeState.maxPaths * MAX_BEZ * 2 * sizeof(float));
  if (gcodeState.pathPoints == NULL) {
    printf("Memory allocation failed!\n");
    exit(1);
  }

  //Simulated annealing implementation for path optimization.
  srand(time(0));

  double initialTempLog = 13000 * log10(pow((sqrt(pointsCount) * sqrt(pathCount) * 1/20), 1.9) + 20000) - 50000;
  double initialTempExp = 0.03*(pow((sqrt(pointsCount) * sqrt(pathCount)), 1.25) +5000) + 4750;
  float coolingRate = 0.035;
  int saNumComp = floor(sqrt(pointsCount)*sqrt(pathCount))*((gcodeState.quality+1)*(gcodeState.quality));

  //Simulated Annealing call
  clock_t start_sa, stop_sa;
  double reorder_time;

  printf("Paths %d Points %d\n",pathCount, pointsCount);
  start_sa = clock();
  simulatedAnnealing(shapes, points, pathCount, initialTempExp, coolingRate, gcodeState.quality, saNumComp);

  stop_sa = clock();
  reorder_time = ((double) (stop_sa - start_sa)) / CLOCKS_PER_SEC;

  printf("Finished Path-Opt\n");
  fflush(stdout);

  printf("Sorting shapes by color\n");
  int mergeCount = 0;
  mergeSort(shapes, 0, pathCount-1, 0, &mergeCount, penColors, penColorCount); //this is stable and can be called on subarrays.
  printf("Completed mergeSort\n");
  fflush(stdout);

  //create char buffer for shapes

  clock_t start_write, stop_write;
  double write_time;
  //Break into writeHeader method.

  start_write = clock();
  writeHeader(&gcodeState, gcode, &settings, machineType, paperDimensions);
  printf("Wrote header\n");
  fflush(stdout);

  // for(int i = 0; i < numTools; i++){
  //   printf("PenColorCount[%d]: %d\n", i, penColorCount[i]);
  // }
  // fflush(stdout);

  //WRITING PATHS BEGINS HERE. 
  for(i=0;i<pathCount;i++) { //equal to the number of shapes, which is the number of NSVGPaths.
    gcodeState.shapeStart=1;
    for(k=0; k < gcodeState.npaths; k++){ //npaths == number of points/ToolPaths in path. Looks at the shape for each toolpath, and if it is equal to the shape in this position's id
                                          //in shapes, then it beigs the print logic. This can almost certainly be optimized because each shape does not have npaths paths associated.
      if(toolPaths[k].shape == -1){ //means already written. Go back to start of above for loop and check next.
          continue;
      } else if(toolPaths[k].shape == shapes[i].id) { //Condition found for writing a shape.
        break;
      }
    }
    if(k > gcodeState.npaths - 1) {
      printf("Continue hit\n");
      continue;
    }
    
    //Handling for color file pointer at start of new file. May want to refactor this.
    if(gcodeState.colorToFile){ //If we are writing colors to files.
      if(gcodeState.currColor != shapes[i].stroke){ //If current color is not the new shapes color. Currently, currColor is not being set properly.
        if(gcodeState.colorFileOpen == 1){ //If there is already a color file open.
          writeFooter(&gcodeState, color_gcode, machineType);

          fclose(color_gcode);
          gcodeState.colorFileOpen = 0;
        }

        char filename[256];

        int len = strlen(argv[optind + 1]);
        char fileStart[len - 5]; //This will also be the folder.
        strncpy(fileStart, argv[optind + 1], len - 6);
        fileStart[len - 6] = '\0';

        int nameLen = strlen(fileName);
        char colFileName[nameLen-5];
        strncpy(colFileName, fileName, nameLen-6);
        colFileName[nameLen-6] = '\0';
        
        char *fileEnd = ".gcode";
        sprintf(filename, "%s/%s-%s%s", fileStart, colFileName, uint_to_hex_string(shapes[i].stroke), fileEnd);
        printf("File name to open: %s\n", filename);
        fflush(stdout);

        color_gcode = fopen(filename, "w");
        if(color_gcode == NULL){
          printf("Failed to open color file: %s\n", filename);
          fflush(stdout);
        }

        gcodeState.colorFileOpen = 1;

        printf("Writing header to color file\n");
        fflush(stdout);
        writeHeader(&gcodeState, color_gcode, &settings, machineType, paperDimensions);
        printf("Finish writing header to color file\n");
        fflush(stdout);
      }
      gcodeState.currColor = shapes[i].stroke; //always want to update currColor.
    }
    

    //Method for writing toolchanges. Checks for toolchange, and writes if neccesary.
    writeToolchange(&gcodeState, machineType, gcode, numTools, penList, penColorCount, shapes, &i);

    //WRITING MOVES FOR DRAWING 
    writeShape(gcode, color_gcode, &gcodeState, &settings, shapes, toolPaths, &machineType, &k, &i);
  }
  
  writeFooter(&gcodeState, gcode, machineType);
  if(gcodeState.colorToFile){
    writeFooter(&gcodeState, color_gcode, machineType);
    fclose(color_gcode);
  }
  
  stop_write = clock();
  write_time = ((double) stop_write - start_write) / CLOCKS_PER_SEC;

  printf("( Total distance traveled = %f m, PointsCulledPrec: = %d)\n", gcodeState.totalDist, gcodeState.pointsCulledPrec);
  printf("Timing Details:\n");
  printf("  ParseTime: %f\n", parse_time);
  printf("  SA Time: %f\n", reorder_time);
  printf("  Write Time: %f\n", write_time);

  fflush(stdout);
  fclose(gcode);
  free(gcodeState.pathPoints);
  free(writeBuffer);
  free(points);
  free(toolPaths);
  free(shapes);
  free(penList);
  nsvgDelete(g_image);

  return 0;
}

#ifndef BTSVG
int main(int argc, char* argv[]){
  printf("Argc:%d\n", argc);
  int penColorCount[6] = {1, 0, 0, 0, 0, 0}; //count of colors per pen needs to be passed into generateGcode. penColorCount[i] corresponds to pen tool i-1.
  int penOneColorArr[] = {0}; //Integer values of colors for each pen. -1 in an arr is placeholder for no colors to this arr.
  int penTwoColorArr[] = {-1};
  int penThreeColorArr[] = {-1};
  int penFourColorArr[] = {-1};
  int penFiveColorArr[] = {-1};
  int penSixColorArr[] = {-1};

  int *penColors[6]; //Init arr of pointers for pen colors.
  int *penOneColors = (int*)malloc(sizeof(int)*penColorCount[0]); //Malloc number of colors per pen to each pointer
  int *penTwoColors = (int*)malloc(sizeof(int)*penColorCount[1]);
  int *penThreeColors = (int*)malloc(sizeof(int)*penColorCount[2]);
  int *penFourColors = (int*)malloc(sizeof(int)*penColorCount[3]);
  int *penFiveColors = (int*)malloc(sizeof(int)*penColorCount[4]);
  int *penSixColors = (int*)malloc(sizeof(int)*penColorCount[5]);
  memset(penOneColors, 0, sizeof(int)*penColorCount[0]);
  memset(penTwoColors, 0, sizeof(int)*penColorCount[1]);
  memset(penThreeColors, 0, sizeof(int)*penColorCount[2]);
  memset(penFourColors, 0, sizeof(int)*penColorCount[3]);
  memset(penFiveColors, 0, sizeof(int)*penColorCount[4]);
  memset(penSixColors, 0, sizeof(int)*penColorCount[5]);
  //assign colors to malloc'd mem
  for(int i = 0; i<numTools; i++){
    for(int j = 0; j < penColorCount[i]; j++){
      switch(i) {
        case 0:
          printf("Assigning penOneColorArr[%d] for tool %d\n", j, i);
          penOneColors[j] = penOneColorArr[j];
          break;
        case 1:
          printf("Assigning penTwoColorArr[%d] for tool %d\n", j, i);
          penTwoColors[j] = penTwoColorArr[j];
          break;
        case 2:
          printf("Assigning penThreeColorArr[%d] for tool %d\n", j, i);
          penThreeColors[j] = penThreeColorArr[j];
          break;
        case 3:
          printf("Assigning penFourColorArr[%d] for tool %d\n", j, i);
          penFourColors[j] = penFourColorArr[j];
          break;
        case 4:
          printf("Assigning penFiveColorArr[%d] for tool %d\n", j, i);
          penFiveColors[j] = penFiveColorArr[j];
          break;
        case 5:
          printf("Assigning penSixColorArr[%d] for tool %d\n", j, i);
          penSixColors[j] = penSixColorArr[j];
          break;
      }
    }
  }

  penColors[0] = penOneColors; //Set arr pointers to malloc'd pointers
  penColors[1] = penTwoColors;
  penColors[2] = penThreeColors;
  penColors[3] = penFourColors;
  penColors[4] = penFiveColors;
  penColors[5] = penSixColors;

  float paperDimensions[2] = {1.0, 1.0};
  int res = generateGcode(argc, argv, penColors, penColorCount, paperDimensions, 1, 1, 25.4, 50.8, -3);
  //Free malloc'd memory
  free(penOneColors);
  free(penTwoColors);
  free(penThreeColors);
  free(penFourColors);
  free(penFiveColors);
  free(penSixColors);
  return res;
}
#endif
