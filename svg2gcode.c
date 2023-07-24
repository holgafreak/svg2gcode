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
#include <string.h>
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include "svg2gcode.h"
#include <math.h>

#define DEBUG_OUTPUT
#define BTSVG
#define maxBez 128 //64;
#define MAXINT(a,b) (((a)>(b))?(a):(b))


#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

static int sixColorWidth = 306;

static float minf(float a, float b) { return a < b ? a : b; }
static float maxf(float a, float b) { return a > b ? a : b; }
static int numTools = 6;
static float bounds[4];
static int pathCount,pointsCount,shapeCount;
static struct NSVGimage* g_image = NULL;
int numCompOut = 0;
int pathCountOut = 0;
int pointCountOut = 0;

typedef struct {
  float x;
  float y;
} SVGPoint;

typedef struct {
  int *colors;
  int count;
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
    float drawingWidth;
    float drawingHeight;
    float drawSpaceWidth;
    float drawSpaceHeight;
    float shiftX;
    float shiftY;
    float centerX;
    float centerY;
    float originalCenterX;
    float originalCenterY;
    float cosRot;
    float sinRot;
    float xmargin;
    float ymargin;
    int fitToMaterial;
    int centerOnMaterial;
    int swapDim;
    int svgRotation;
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
    int numReord;
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
} GCodeState;

SVGPoint bezPoints[maxBez];
static SVGPoint first,last;
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
    if(bezCount >= maxBez) {
      printf("!bez count\n");
      bezCount = maxBez;
    }
  }
}

#ifdef _WIN32 //win doesn't have good RNG
#define RANDOM() drnd31() //((double)rand()/(double)RAND_MAX)
#else //OSX LINUX much faster than win
#define RANDOM() (drand48())
#endif

static int pcomp(const void* a, const void* b) {
  SVGPoint* ap = (SVGPoint*)a;
  SVGPoint* bp = (SVGPoint*)b;
  if(sqrt(ap->x*ap->x + ap->y*ap->y) > sqrt(bp->x*bp->x+bp->y*bp->y)) {
    return 1;
  }
  return -1;
}

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
    for (path = shape->paths; path != NULL; path = path->next) {
      shapes[i].id = i;
      shapes[i].stroke = shape->stroke.color;
      for (j = 0; j < path->npts - 1; j += 3) {
        float* pp = &path->pts[j * 2];
        if (j == 0) {
            points[i].x = pp[0];
            points[i].y = pp[1];
        }
        bezCount++;
        for (b = 0; b < 8; b++) {
          paths[k].points[b] = pp[b];
        }
        paths[k].closed = path->closed;
        paths[k].shape = i;
        shapes[i].numToolpaths++;
        k++;
      }
      cont:
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


//submethod for mergeSort
void merge(Shape * arr, int left, int mid, int right) {
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
        if (leftArr[i].stroke <= rightArr[j].stroke) {
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
void mergeSort(Shape * arr, int left, int right, int level, int* mergeLevel) {
  if(level > *mergeLevel){
    printf("Merge Sort level: %d\n", level);
    *mergeLevel = level;
  }
  if (left < right) {
    int mid = left + (right - left) / 2;
    mergeSort(arr, left, mid, level+1, mergeLevel);
    mergeSort(arr, mid + 1, right, level+1, mergeLevel);
    merge(arr, left, mid, right);
  }
}

int colorInPen(Pen pen, unsigned int color, int colorCount){
  for(int i = 0; i < colorCount; i++){
    if(pen.colors[i] == color){
      return 1;
    }
  }
  return 0;
}

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


// static void threeOptReorder(SVGPoint* pts, int pathCount, char xy, Shape* shapes, Pen* penList){
//   int gain;
//   do {
//     gain = 0;
//     for(int i=0; i<pathCount-3; i++) {
//       for(int j=i+1; j<pathCount-2; j++) {
//         for(int k=j+1; k<pathCount-1; k++) {
//           // calculate the total distance of the three edges (i,i+1), (j,j+1) and (k,k+1)
//           //float old_dist = dist(shapes[i], shapes[i+1]) + dist(shapes[j], shapes[j+1]) + dist(shapes[k], shapes[k+1]);
          
//           // calculate the total distance if we were to replace the three edges with (i,j), (j+1,k) and (k+1,i+1)
//           //float new_dist = dist(shapes[i], shapes[j]) + dist(shapes[j+1], shapes[k]) + dist(shapes[k+1], shapes[i+1]);

//           //if(new_dist < old_dist) {
//             // Perform the 3-opt swap, i.e., reverse the shapes in the range (i+1, k)
//             //std::reverse(shapes + i + 1, shapes + k + 1); reimplement for c.
//             //gain = 1;
//           //}
//         }
//       }
//     }
//   } while(gain);
// }

//need to set up indicies for each color to reorder between, as opposed to reordering the entire list.
//reorder the paths to minimize cutter movement. //default is xy = 1
static void reorder(SVGPoint* pts, int pathCount, char xy, Shape* shapes, Pen* penList, int quality) {
  int i,j,k,temp1,temp2,indexA,indexB, indexH, indexL;
  Shape temp;
  float dx,dy,dist,dist2, dnx, dny, ndist, ndist2;
  SVGPoint p1,p2,p3,p4;
  SVGPoint pn1,pn2,pn3,pn4;
  int numComp = floor(sqrt(pointsCount) * (quality+1));
  for(i=0;i<numComp*pathCount;i++) {
    indexA = (int)(RANDOM()*(pathCount-2));
    indexB = (int)(RANDOM()*(pathCount-2));
    if(abs(indexB-indexA) < 2){
      continue;
    }
    if(indexB < indexA) { //work from left index a and right index b.
      temp1 = indexB;
      indexB = indexA;
      indexA = temp1;
    }
    pn1 = pts[shapes[indexA].id];
    pn2 = pts[shapes[indexA+1].id];
    pn3 = pts[shapes[indexB].id];
    pn4 = pts[shapes[indexB+1].id];
    dnx = pn1.x-pn2.x;
    dny = pn1.y-pn2.y;
    if(xy) {
      ndist = dnx * dnx + dny * dny;
    } else {
      ndist = dny * dny;
    }
    dnx = pn3.x-pn4.x;
    dny = pn3.y-pn4.y;

    if(xy) {
      ndist += (dnx * dnx + dny * dny);
    } else {
      ndist += dny * dny;
    }
    dnx = pn1.x - pn3.x;
    dny = pn1.y - pn3.y;
    if(xy){
      ndist2 = dnx * dnx + dny * dny;
    } else {
      ndist2 = dny * dny;
    }
    dnx = pn2.x - pn4.x;
    dny = pn2.y - pn4.y;
    if(xy) {
      ndist2 += dnx * dnx + dny * dny;
    } else {
      ndist2 += dny * dny;
    }
    if(ndist2 < ndist) {
      indexH = indexB;
      indexL = indexA+1;
      while(indexH > indexL) {
        temp = shapes[indexL];
        shapes[indexL]=shapes[indexH];
        shapes[indexH] = temp;
        indexH--;
        indexL++;
      }
    }
  }
  pathCountOut = pathCount;
  pointCountOut = pointsCount;
  numCompOut = numComp;
}

TransformSettings calcTransform(NSVGimage * g_image, float * paperDimensions, int * generationConfig){
  TransformSettings settings;
  float width = g_image->width;
  float height = g_image->height;
  printf("Image width:%f Image Height:%f\n", width, height);

  settings.svgRotation = generationConfig[2];
  settings.xmargin = paperDimensions[2];
  settings.ymargin = paperDimensions[3];
  //scaling + fitting operations.
  settings.drawSpaceWidth = paperDimensions[0] - (2*settings.xmargin);
  settings.drawSpaceHeight = paperDimensions[1] - (2*settings.ymargin);
  printf("drawSpaceWidth: %f, drawSpaceHeight:%f\n", settings.drawSpaceWidth, settings.drawSpaceHeight);
  settings.swapDim = (generationConfig[2] == 1 || generationConfig[2] == 3);

  // Swap width and height if necessary
  if (settings.swapDim) {
    float temp = width;
    width = height;
    height = temp;
    printf("Swapped image width:%f Image Height:%f\n", width, height);
  }
  settings.drawingWidth = width;
  settings.drawingHeight = height;
  printf("DrawingWidth:%f, DrawingHeight:%f\n", settings.drawingWidth, settings.drawingHeight);
  fflush(stdout);

#ifdef DEBUG_OUTPUT
  printf("Fit To Mat from Config = %i\n", generationConfig[0]);
#endif
  // Determine if fitting to material is necessary
  settings.fitToMaterial = ((settings.drawingWidth > settings.drawSpaceHeight) || (settings.drawingHeight > settings.drawSpaceHeight) || generationConfig[0]);

  // If fitting to material, calculate scale and new drawing dimensions
  if (settings.fitToMaterial) {
    float materialRatio = settings.drawSpaceWidth / settings.drawSpaceHeight;
    float svgRatio = width / height;
    settings.scale = (materialRatio > svgRatio) ? (settings.drawSpaceHeight / height) : (settings.drawSpaceWidth / width);
    printf("Scale%f\n", settings.scale);
    settings.drawingWidth = width * settings.scale;
    settings.drawingHeight = height * settings.scale;
    printf("Scaled drawingWidth:%f drawingHeight:%f\n", settings.drawingWidth, settings.drawingHeight);
    settings.shiftX = settings.xmargin;
    settings.shiftY = settings.ymargin;
  } else {
    settings.scale = 1;
  }

  settings.centerOnMaterial = generationConfig[1];

  // If centering on material, calculate shift
  if (settings.centerOnMaterial) {
      settings.shiftX = settings.xmargin + ((settings.drawSpaceWidth - settings.drawingWidth) / 2);
      settings.shiftY = settings.ymargin + ((settings.drawSpaceHeight - settings.drawingHeight) / 2);
      printf("If centerOnMaterial shiftX:%f, shiftY:%f\n", settings.shiftX, settings.shiftY);
  }

  // Calculate center of scaled and rotated drawing. 
  settings.centerX = settings.shiftX + settings.drawingWidth / 2;
  settings.centerY = settings.shiftY + settings.drawingHeight / 2;
  settings.originalCenterX = settings.centerX;
  settings.originalCenterY = settings.centerY;
  if(settings.swapDim){
    settings.originalCenterX = settings.shiftX + settings.drawingHeight/2;
    settings.originalCenterY = settings.shiftY + settings.drawingWidth/2;
  }

  printf("originalCenterX:%f, originalCenterY:%f\n", settings.originalCenterX, settings.originalCenterY);
  printf("centerX:%f, centerY:%f\n", settings.centerX, settings.centerY);
  fflush(stdout);

  settings.cosRot = cos((90*settings.svgRotation)*(M_PI/180)); 
  settings.sinRot = sin((90*settings.svgRotation)*(M_PI/180));

  return settings;
}

void printTransformSettings(TransformSettings settings) {
  printf("\n\nscale: %f\n", settings.scale);
  printf("drawingWidth: %f\n", settings.drawingWidth);
  printf("drawingHeight: %f\n", settings.drawingHeight);
  printf("drawSpaceWidth: %f\n", settings.drawSpaceWidth);
  printf("drawSpaceHeight: %f\n", settings.drawSpaceHeight);
  printf("shiftX: %f\n", settings.shiftX);
  printf("shiftY: %f\n", settings.shiftY);
  printf("centerX: %f\n", settings.centerX);
  printf("centerY: %f\n", settings.centerY);
  printf("originalCenterX: %f\n", settings.originalCenterX);
  printf("originalCenterY: %f\n", settings.originalCenterY);
  printf("cosRot: %f\n", settings.cosRot);
  printf("sinRot: %f\n", settings.sinRot);
  printf("xmargin: %f\n", settings.xmargin);
  printf("ymargin: %f\n", settings.ymargin);
  printf("fitToMaterial: %d\n", settings.fitToMaterial);
  printf("centerOnMaterial: %d\n", settings.centerOnMaterial);
  printf("swapDim: %d\n", settings.swapDim);
  printf("svgRotation: %d\n\n", settings.svgRotation);
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
  printf("numReord: %d\n", state->numReord);
  printf("x: %f\n", state->x);
  printf("y: %f\n", state->y);
  printf("firstx: %f\n", state->firstx);
  printf("firsty: %f\n", state->firsty);
  printf("totalDist: %lf\n", state->totalDist);
  printf("xold: %f\n", state->xold);
  printf("yold: %f\n", state->yold);
  printf("\n");  // End with newline
}

GCodeState initializeGCodeState(float * paperDimensions, int * generationConfig){
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
  state.currColor = 0;
  state.targetColor = 0;
  state.targetTool = 0;
  state.currTool = -1;
  state.colorMatch = 0;
  state.toolChangePos = -51.5;

  if(state.quality == 2){
    state.tol = 0.25;
    state.numReord = 20;
  } else if (state.quality == 1){
    state.tol = 0.5;
    state.numReord = 10;
  } else {
    state.tol = 1;
    state.numReord = 10;
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

  return state;
}

void toolDown(FILE * gcode, GCodeState * gcodeState, int * machineTypePtr){
  fprintf(gcode, "G1 Z%f F%d\n", gcodeState->zFloor, gcodeState->zFeed);
}

void toolUp(FILE * gcode, GCodeState * gcodeState, int * machineTypePtr){
  fprintf(gcode, "G1 Z%f F%d\n", gcodeState->ztraverse, gcodeState->zFeed);
}

void writeToolchange(GCodeState* gcodeState, int machineType, FILE* gcode, int numTools, Pen* penList, int* penColorCount, Shape * shapes, int * i) {
  if(machineType == 0 || machineType == 2){ //All machines will want to check for tool change eventually.
    gcodeState->targetColor = shapes[*i].stroke;
    if(colorInPen(penList[gcodeState->currTool], shapes[*i].stroke, penColorCount[gcodeState->currTool]) == 0){ //this checks if new shape's color is assigned to current tool
#ifdef DEBUG_OUTPUT
      fprintf(gcode, "( Shape stroke:%i currTool:%i )\n", shapes[*i].stroke, gcodeState->currTool);
#endif
      for(int p = 0; p < numTools; p++){ //iterate through tools numbers (0 -> numTools-1). 
        if(colorInPen(penList[p], shapes[*i].stroke, penColorCount[p])){ //If tool p contains the new shape's color,
          gcodeState->targetTool = p; //Set the target tool to tool p.
          break;
        }
        gcodeState->targetTool = 0;
      }
    }
    if(gcodeState->targetTool != gcodeState->currTool){ //This is true if toolchange is neccesary.
#ifdef DEBUG_OUTPUT
  fprintf(gcode, "    ( Beginning Toolchange )\n");
  fprintf(gcode, "    ( Current Tool:%d, Target Tool:%d )\n", gcodeState->currTool, gcodeState->targetTool);
#endif
      if(machineType == 0){ //Actual tool change code per machine type. LFP and MVP will want to have Pause command for fluidncc
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
      } else if (machineType == 2 && (gcodeState->targetTool != 0)){
        fprintf(gcode, "( MVP PAUSE COMMAND TOOL:%d)\n", gcodeState->targetTool);
      }
      toolUp(gcode, gcodeState, &machineType);
      gcodeState->x = 0;
      gcodeState->currColor = gcodeState->targetColor;
      gcodeState->currTool = gcodeState->targetTool;
#ifdef DEBUG_OUTPUT
      fprintf(gcode, "    ( Ending Toolchange )\n");
#endif
    }
  }
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
  //fprintf(gcode, " (MaxPaths in a shape: %i)\n", gcodeState->maxPaths);
#endif
}

void writeHeader(GCodeState* gcodeState, FILE* gcode, int machineType, float* paperDimensions) {
#ifdef DEBUG_OUTPUT
  //fprintf(gcode, "( Machine Type: %i )\n", machineType);
#endif
  fprintf(gcode, "G90\nG0 M3 S%d\n", 90);
  fprintf(gcode, "G0 Z%f\n", gcodeState->ztraverse);

  if(machineType == 0 || machineType == 2) { //6Color or MVP
    fprintf(gcode, "G1 Y0 F%i\n", gcodeState->feedY);
    fprintf(gcode, "G1 Y%f F%d\n", (-1.0*(paperDimensions[1]-100.0)), gcodeState->feedY);
    fprintf(gcode, "G1 Y0 F%i\n", gcodeState->feedY);
  }
}

int firstPoint(int * sp, int * ptIndex, int * pathPointIndex){ //check if current point is first point in shape
  return (*sp == 0 && *ptIndex == 0) || (*sp == 1 && *ptIndex == *pathPointIndex -2 );
}

int lastPoint(int * sp, int * ptIndex, int * pathPointIndex){ //check if current point is last point in shape
  return (*sp == 0 && *ptIndex == *pathPointIndex - 2) || (*sp == 1 && *ptIndex == 0);
}

int canWritePoint(GCodeState * gcodeState, TransformSettings * settings, int * sp, int  * ptIndex, int * pathPointIndex, float * px, float * py, FILE * gcode){ //always want to write if it is first or last point in a shape.
  //want a preliminary check that the coordinates are within bounds.
  if ((*px < 0 || *px > settings->drawSpaceWidth + settings->xmargin) || (*py > 0 || *py < -1*(settings->drawSpaceHeight + settings->ymargin))){
    gcodeState->pointsCulledBounds++;
    return 0;
  } else if(firstPoint(sp, ptIndex, pathPointIndex) || lastPoint(sp, ptIndex, pathPointIndex)){ //Always write first and last point in a shape.
    return 1;
  } 
  //return 1 if the distance is greater than the precision value and if it is a new point.
  if((distanceBetweenPoints(gcodeState->xold, gcodeState->yold, *px, *py) >= gcodeState->precision) && ((gcodeState->xold != *px) || (gcodeState->yold != *py))){ //can write
    return 1;
  }
  gcodeState->pointsCulledPrec++;
  return 0;
}

int toolRefresh(int * sp, int * ptIndex, int * pathPointIndex, GCodeState * gcodeState, int * dist){
  return (!firstPoint(sp, ptIndex, pathPointIndex)) && (gcodeState->trackedDist + *dist > gcodeState->brushDist);
}

void writePoint(FILE * gcode, GCodeState * gcodeState, TransformSettings * settings, int * ptIndex, char * isClosed, int * machineType, int * sp, int * pathPointIndex) {
    float rotatedX, rotatedY, feedRate;
    float dist = 0.0;
    
    // Get the unscaled and unrotated coordinates from pathPoints
    float x = gcodeState->pathPoints[*ptIndex];
    float y = gcodeState->pathPoints[(*ptIndex)+1];
    
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

    if(canWritePoint(gcodeState, settings, sp, ptIndex, pathPointIndex, &rotatedX, &rotatedY, gcode)){ //Can write, if first or last in shape, or if dist is large enough.
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
      fprintf(gcode,"G1 X%.4f Y%.4f F%d\n", gcodeState->x, gcodeState->y, (int)feedRate);
    }

    if(firstPoint(sp, ptIndex, pathPointIndex)){ //if first point written in path
      gcodeState->firstx = rotatedX;
      gcodeState->firsty = rotatedY;
      toolDown(gcode, gcodeState, machineType);
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

//Now work on refactoring writeShape.
void writeShape(FILE * gcode, GCodeState * gcodeState, TransformSettings * settings, Shape * shapes, ToolPath * toolPaths, int * machineTypePtr, int * k, int * i) { //k is index in toolPaths. i is index i shapes.
    float rotatedX, rotatedY, rotatedBX, rotatedBY, tempRot;
    int writeShape = 1;
    int j, l; //local iterators with k <= j, l < npaths;
    
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
    char isClosed = toolPaths[j].closed;
    int sp = nearestStartPoint(gcode, gcodeState, settings, pathPointsIndex);
    if(sp){
      for(int z = pathPointsIndex-2; z >= 0; z-=2){ //write backwards if sp, forwards if else.
        writePoint(gcode, gcodeState, settings, &z, &isClosed, machineTypePtr, &sp, &pathPointsIndex);
      }
    } else {
      for(int z = 0; z < pathPointsIndex; z += 2){
        writePoint(gcode, gcodeState, settings, &z, &isClosed, machineTypePtr, &sp, &pathPointsIndex);
      }
    }

    toolUp(gcode, gcodeState, machineTypePtr);
}

void printArgs(int argc, char* argv[], int** penColors, int penColorCount[6], float paperDimensions[7], int generationConfig[9]) {
    int i, j;

    printf("argc:\n\t%d\n", argc);

    printf("argv:\n");
    for(i = 0; i < argc; i++) {
        printf("\t%d: %s\n", i, argv[i]);
    }

    printf("penColors:\n");
    for(i = 0; i < 6; i++) {
        printf("\t%d: ", i+1);
        for(j = 0; j < penColorCount[i]; j++) {
            printf("%d ", penColors[i][j]);
        }
        printf("\n");
    }

    printf("penColorCount:\n");
    for(i = 0; i < 6; i++) {
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

void help() {
  printf("usage: svg2gcode [options] svg-file gcode-file\n");
  printf("options:\n");
  printf("\t-Y shift Y-ax\n");
  printf("\t-X shift X-ax\n");
  printf("\t-f feed rate (3500)\n");
  printf("\t-n # number of reorders (30)\n");
  printf("\t-s scale (1.0)\n");
  printf("\t-S 1 scale to material size\n");
  printf("\t-C center on drawing space\n");
  printf("\t-w final width in mm\n");
  printf("\t-t Bezier tolerance (0.5)\n");
  printf("\t-Z z-engage (-1.0)\n");
  printf("\t-B do Bezier curve smoothing\n");
  printf("\t-h this help\n");
}

int generateGcode(int argc, char* argv[], int** penColors, int penColorCount[6], float paperDimensions[7], int generationConfig[9]) {
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
  GCodeState gcodeState = initializeGCodeState(paperDimensions, generationConfig);
  printGCodeState(&gcodeState);

  int machineType = generationConfig[3]; //machineType
  int ch;
 
  FILE *gcode;
  FILE *debug;

  //CLI argument BS may want to remove.
  printf("Argc:%d\n", argc);
  if(argc < 3) {
    help();
    return -1;
  }
  while((ch=getopt(argc,argv,"D:ABhf:n:s:Fz:Z:S:w:t:m:cTV1aLP:CY:X:")) != EOF) { //I think handoff between argc and argv is happening here so can't remove for sake of opening and outputting to a file.
    switch(ch) {
    case 'h': help();
      break;
    case 'f': gcodeState.feed = atoi(optarg);
      break;
    case 'n': gcodeState.numReord = atoi(optarg);
      break;
    case 't': gcodeState.tol = atof(optarg);
      break;
    default: help();
      return(1);
      break;
    }
  }
  //end cli parsing.

  printf("File open string: %s\n", argv[optind]);
  printf("File output string: %s\n", argv[optind+1]);
  g_image = nsvgParseFromFile(argv[optind],"px", 128);
  if(g_image == NULL) {
    printf("error: Can't open input %s\n",argv[optind]);
    return -1;
  }

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

#ifdef _WIN32
  seedrand((float)time(0));
#endif

 gcode=fopen(argv[optind+1],"w");
 if(gcode == NULL) {
   printf("can't open output %s\n",argv[optind+1]);
   return -1;
 }
  //fprintf(gcode, "w x h: %f x %f\n", w, h);
  printf("paths %d points %d\n",pathCount, pointsCount);
  //fprintf(gcode, "( centerX:%f, centerY:%f )\n", centerX, centerY);
  // allocate memory
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
  gcodeState.pathPoints = malloc(gcodeState.maxPaths * maxBez * 2 * sizeof(float)); //points are stored as x on even y on odd, eg point p1 = (pathPoints[0],pathPoints[1]) = (x1,y1)
  if (gcodeState.pathPoints == NULL) {
    printf("Memory allocation failed!\n");
    exit(1);
  }

  //Sorting shapes for path optimization
  printf("Reorder with numShapes: %d\n",pathCount);
  for(k=0;k < gcodeState.numReord; k++) {
    reorder(points, pathCount, gcodeState.xy, shapes, penList, gcodeState.quality);
    printf("%d... ",k);
    fflush(stdout);
  }
  printf("\n");

  //If shapes are reordered by distances first, using a stable sort after for color should maintain the sort order obtained by distances, but organized by colors.
  printf("Sorting shapes by color\n");
  int mergeCount = 0;
  mergeSort(shapes, 0, pathCount-1, 0, &mergeCount); //this is stable and can be called on subarrays. So we want to reorder, then call on subarrays indexed by our mapped colors.
  //End sorting.

  //Break into writeHeader method.
  writeHeader(&gcodeState, gcode, machineType, paperDimensions);

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
#ifdef DEBUG_OUTPUT
    fprintf(gcode, "( Shape %d at i:%d. Color: %i )\n", shapes[i].id, i, shapes[i].stroke);
#endif
    //Method for writing toolchanges. Checks for toolchange, and writes if neccesary.
    writeToolchange(&gcodeState, machineType, gcode, numTools, penList, penColorCount, shapes, &i);

    //WRITING MOVES FOR DRAWING 
    writeShape(gcode, &gcodeState, &settings, shapes, toolPaths, &machineType, &k, &i);
  }
  
  writeFooter(&gcodeState, gcode, machineType);

  printf("( Total distance traveled = %f m, PointsCulledPrec: = %d)\n", gcodeState.totalDist, gcodeState.pointsCulledPrec);
  free(gcodeState.pathPoints);
  fclose(gcode);
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
  int penColorCount[6] = {2, 1, 1, 1, 0, 0}; //count of colors per pen needs to be passed into generateGcode. penColorCount[i] corresponds to pen tool i-1.
  int penOneColorArr[] = {65280, 16711680}; //Integer values of colors for each pen. -1 in an arr is placeholder for no colors to this arr.
  int penTwoColorArr[] = {1710618};
  int penThreeColorArr[] = {2763519};
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
