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
#include <sys/time.h>
#include <sys/types.h>
//#define TESTRNG // remove if on linux or osx
//#define DO_HPGL //remove comment if you want to get a HPGL-code
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#define GHEADER "G90\nG92 X0 Y0\n" //add here your specific G-codes
#define GHEADER_NEW "nG90\nG92 X0 Y0\n" //add here your specific G-codes
                                  //separated with newline \n
#define G32 1
#ifdef G32
#define CUTTERON "G0 M3 S%d\n"
#else
#define CUTTERON "M3 S%d\n" //I chose this, change to yours or add comment
                      // or add newline "\n" if not needed
#endif
#define CUTTEROFF "M5\n" // same for this
#define GFOOTER "M5\nG0 X0 Y0\n" //end G-code here
#define GMODE "M4\n"
//#define DO_HPGL //uncomment to get hpgl-file named test.hpgl on current folder
static float minf(float a, float b) { return a < b ? a : b; }
static float maxf(float a, float b) { return a > b ? a : b; }
static float bounds[4];
static int pathCount,pointsCount,shapeCount;
static int doBez = 0;
static int simplify = 0;
static struct NSVGimage* g_image = NULL;

typedef struct {
  float x;
  float y;
} SVGPoint;

typedef struct {
  float points[8];
  int city;
  char closed;
} ToolPath;

static SVGPoint bezPoints[64];
static SVGPoint first,last;
static int bezCount = 0;
#ifdef _WIN32
//typedef unsigned long int uint64_t;
//typedef unsigned int  uint32_t;


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
    seed = rand31(seed);
    seed = rand31(seed);
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
// bezier smoothing
static void cubicBez(float x1, float y1, float x2, float y2,
		     float x3, float y3, float x4, float y4,
		     float tol, int level)
{
  float x12,y12,x23,y23,x34,y34,x123,y123,x234,y234,x1234,y1234;
  float d;
	
  if (level > 12) return;

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

  d = distPtSeg(x1234, y1234, x1,y1, x4,y4);
  if (d > tol*tol) {
    cubicBez(x1,y1, x12,y12, x123,y123, x1234,y1234, tol, level+1); 
    cubicBez(x1234,y1234, x234,y234, x34,y34, x4,y4, tol, level+1); 
  } else {
    bezPoints[bezCount].x = x4;
    bezPoints[bezCount].y = y4;
    bezCount++;
    if(bezCount > 63) {
      printf("!bez count\n");
      bezCount = 63;
    }
  }
}
//#define TESTRNG
#ifdef _WIN32 //win doesn't have good RNG
#define RANDOM() drnd31() //((double)rand()/(double)RAND_MAX)
#else //OSX LINUX much faster than win
#define RANDOM() (drand48())
#endif


static int pcomp(const void* a, const void* b) {
  SVGPoint* ap = (SVGPoint*)a;
  SVGPoint* bp = (SVGPoint*)b;
  if(sqrt(ap->x*ap->x + ap->y*ap->y) > sqrt(bp->x*bp->x+bp->y*bp->y))
    return 1;
  return -1;
}

// get all paths and add a city for each path		  
static void calcPaths(SVGPoint* points, ToolPath* paths,int *cities, int *npaths) {
struct NSVGshape* shape;
  struct NSVGpath* path;
  FILE *f;
  int i,j,k,l,p,b;
  SVGPoint* pts;
#ifdef DO_HPGL 
  f=fopen("test.hpgl","w");
  fprintf(f,"IN;SP1;");
#endif
  i=0;
  k=0;
  j=0;
  p=0;
  for(shape = g_image->shapes; shape != NULL; shape=shape->next) {
     for(path = shape->paths; path != NULL; path=path->next) {
       if(path->closed && simplify) {
	 pts = (SVGPoint*)malloc(path->npts*sizeof(SVGPoint));
	
	 for(l=0;l<path->npts-1;l++) {
	   float *pp=&path->pts[l*2];
	   if(l==0) {
	     points[i].x = pp[0];
	     points[i].y = pp[1];
	   }
	   pts[l].x = pp[0];
	   pts[l].y = pp[1];
	 }
	 qsort((void*)pts,path->npts-1,sizeof(SVGPoint),pcomp);
	 paths[k].points[0] = pts[path->npts-2].x;
	 paths[k].points[1] = pts[path->npts-2].y;
	 paths[k].points[2] = pts[0].x;
	 paths[k].points[3] = pts[0].y;
	 paths[k].closed = path->closed;
	 paths[k].city = i; 
	 k++;
	 //fprintf(stderr,"i %d pts %f %f %f %f\n",i,pts[path->npts-2].x,pts[path->npts-2].y,pts[0].x,pts[0].y);
	 free(pts);
	 
	 goto cont;
       }

       for(j=0;j<path->npts-1;(doBez ? j+=3 : j++)) {
	 float *pp = &path->pts[j*2];
	 if(j==0) {
	
	   points[i].x = pp[0];
	   points[i].y = pp[1];
#ifdef DO_HPGL
	   fprintf(f,"PU%d,%d;",(int)pp[0],(int)pp[1]);
	   fflush(f);
	 
	 } else {
	   fprintf(f,"PD%d,%d;",(int)pp[0],(int)pp[1]);
	   fflush(f);
#endif
	 }
	 if(doBez) {
	   for(b=0;b<8;b++)
	     paths[k].points[b]=pp[b];
	 } else {
	   paths[k].points[0] = pp[0];
	   paths[k].points[1] = pp[1];
	   paths[k].points[2] = pp[0];
	   paths[k].points[3] = pp[1];
	 }
	 paths[k].closed = path->closed;
	 paths[k].city = i; 
	 k++;

       }
     cont:       
       if(k>pointsCount) {
	 printf("error k > \n");
#ifdef DO_HPGL
	 fprintf(f,"PU0,0;\n");
	 fclose(f);
#endif
	 *npaths = 0;
	 return;
	 
       }
       if(i>pathCount) {
	 printf("error i > \n");
#ifdef DO_HPGL
	 fprintf(f,"PU0,0;\n");
	 fclose(f);
#endif
	 exit(-1);
	 /*
	  *npaths=k;
	  return;
	 */
       }
       cities[i] = i;
       i++;
     }
  
     j++;
  }
  printf("total paths %d, total points %d\n",i,k);
  *npaths = k;
#ifdef DO_HPGL
  fprintf(f,"PU0,0;\n");
  fclose(f);
#endif
}

static void calcBounds(struct NSVGimage* image)
{
  struct NSVGshape* shape;
  struct NSVGpath* path;
  int i;
  bounds[0] = FLT_MAX;
  bounds[1] = FLT_MAX;
  bounds[2] = -FLT_MAX;
  bounds[3] = -FLT_MAX;
  pathCount = 0;
  pointsCount = 0;
  shapeCount = 0;
  for (shape = image->shapes; shape != NULL; shape = shape->next) {
    for (path = shape->paths; path != NULL; path = path->next) {
      for (i = 0; i < path->npts-1; i++) {
	float* p = &path->pts[i*2];
	bounds[0] = minf(bounds[0], p[0]);
	bounds[1] = minf(bounds[1], p[1]);
	bounds[2] = maxf(bounds[2], p[0]);
	bounds[3] = maxf(bounds[3], p[1]);
	pointsCount++;
      }
      pathCount++;
    }
    shapeCount++;
  }
  printf("shapes %d\n",shapeCount);
}
//reorder  the paths to minimize cutter movement
static void reorder(SVGPoint* pts, int* cities, int ncity,char xy) {
  int i,j,k,temp1,temp2,indexA,indexB, indexH, indexL;
  float dx,dy,dist,dist2;
  SVGPoint p1,p2,p3,p4;
  for(i=0;i<800*ncity;i++) {
    indexA = (int)(RANDOM()*(ncity-2));
    indexB = (int)(RANDOM()*(ncity-2));
    if(abs(indexB-indexA) < 2)
      continue;
    if(indexB < indexA) {
      temp1 = indexB;
      indexB = indexA;
      indexA = temp1;
    }
    p1 = pts[cities[indexA]];
    p2 = pts[cities[indexA+1]];
    p3 = pts[cities[indexB]];
    p4 = pts[cities[indexB+1]];
    dx = p1.x-p2.x;
    dy = p1.y-p2.y;
    if(xy)
      dist = dx*dx+dy*dy;
    else
      dist = dy*dy;
    dx = p3.x-p4.x;
    dy = p3.y-p4.y;
    if(xy)
      dist += (dx*dx+dy*dy);
    else
      dist += dy*dy;
    dx = p1.x-p3.x;
    dy = p1.y-p3.y;
    if(xy)
      dist2 = dx*dx+dy*dy;
    else
      dist2 = dy*dy;
    dx = p2.x-p4.x;
    dy = p2.y-p4.y;
    if(xy)
      dist2 += (dx*dx+dy*dy);
    else
      dist2 += dy*dy;
    if(dist2 < dist) {
      indexH = indexB;
      indexL = indexA+1;
      while(indexH > indexL) {
	temp1 = cities[indexL];
	cities[indexL]=cities[indexH];
	cities[indexH] = temp1;
	indexH--;
	indexL++;
      }
    }
  }
}

void help() {
  printf("usage: svg2gcode [options] svg-file gcode-file\n");
  printf("options:\n");
  printf("\t-Y shift Y-ax\n");
  printf("\t-X sfit X-ax\n");
  printf("\t-c use z-axis instead of laser\n");
  printf("\t-f feed rate (3500)\n");
  printf("\t-n # number of reorders (30)\n");
  printf("\t-s scale (1.0)\n");
  printf("\t-F flip Y-axis\n");
  printf("\t-w final width in mm\n");
  printf("\t-t Bezier tolerance (0.5)\n");
  printf("\t-m machine accuracy (0.1)\n");
  printf("\t-z z-traverse (1.0)\n");
  printf("\t-Z z-engage (-1.0)\n");
  printf("\t-B do Bezier curve smoothing\n");
  printf("\t-T engrave only TSP-path\n");
  printf("\t-V optmize for Voronoi Stipples\n");
  printf("\t-h this help\n");
}

int main(int argc, char* argv[]) {

  int i,j,k,l,first = 1;
  //struct NSVGimage* image;
  struct NSVGshape *shape1,*shape2;
  struct NSVGpath *path1,*path2;
  SVGPoint* points;
  ToolPath* paths;
  int *cities,npaths;
  int feed = 3500;
  //int shiftY = 30;
  int fullspeed=4800;
  float ztraverse = 1.;
  float zengage = -1.;
  float width = -1;
  char xy = 1;
  float w,widthInmm = -1.;
  int numReord = 30;
  float scale = 1.;
  float tol = 0.5;
  float size = 100;
  float accuracy = 0.1;
  float x,y,bx,by,bxold,byold,d,firstx,firsty;
  float xold,yold;
  int flip = 0;
  int skip = 0;
  int units = 0;
  int printed=0;
  int cncMode = 0;
  int tsp = 0;
  int tspFirst = 1;
  int autoshift = 0;
  float ashift = 0.;
  int firstandonly = 2; // first svg
  int append = 0;
  int last = 0; // last svg
  int waitTime = 25;
  float maxx = -1000.,minx=1000.,maxy = -1000.,miny=1000.,zmax = -1000.,zmin = 1000;
  float shiftX = 0.;
  float shiftY = 0.;
  float zeroX = 0.;
  float zeroY = 0.;
  FILE *gcode;
  int pwr = 90;
  int ch;
  int dwell = -1;
  char gbuff[128];
  int center = 0;
  printf("v0.0001 8.11.2020\n");
  //seed48(NULL);
  if(argc < 3) {
    help();
    return -1;
  }
  simplify = 0;
  while((ch=getopt(argc,argv,"D:ABhf:n:s:Fz:Z:S:w:t:m:cTV1aLP:CY:X:")) != EOF) {
    switch(ch) {
    case 'C': center = 1;
      break;
    case 'P': pwr = atoi(optarg);
      break;
    case 'D': waitTime=atoi(optarg);
      dwell = atoi(optarg);
      break;
    case 'a': append = 1; firstandonly = 0;
      break;
    case 'V': xy = 0;
      break;
    case 'T': tsp = 1;
      if(xy == 0)
	xy = 1;
      break;
    case 'c': cncMode = 1;
      break;
     
    case 'Y': shiftY = atof(optarg); // shift
      break;
      
     
    case 'X': shiftX = atof(optarg); // shift
      break;
      
    case 'A':autoshift = 1;
      break;
    case 'm': accuracy=atof(optarg);
      break;
    case 'B': doBez = 1;
      break;
    case 'h': help();
      break;
    case 'f': feed = atoi(optarg);
      break;
    case 'n': numReord = atoi(optarg);
      break;
    case 's': scale = atof(optarg);
      break;
    case 't': tol = atof(optarg);
      break;
    case 'F': flip = 1;
      break;
    case 'z': ztraverse = atof(optarg);
      break;
    case 'Z': zengage = atof(optarg);
      break;
    case 'w': widthInmm = atof(optarg);
      break;
    default: help();
      return(1);
      break;
    }
  }
  if(shiftY != 30. && flip == 1)
    shiftY = -shiftY;
  g_image = nsvgParseFromFile(argv[optind],"px",96);
  if(g_image == NULL) {
    printf("error: Can't open input %s\n",argv[optind]);
    return -1;
  }
  calcBounds(g_image);
  fprintf(stderr,"bounds %f %f X %f %f\n",bounds[0],bounds[1],bounds[2],bounds[3]);
  width = g_image->width;
  fprintf(stderr,"width %f\n",width);
  w = fabs(bounds[0]-bounds[2]);
  if(widthInmm != -1.0) 
    scale = widthInmm/w;
  fprintf(stderr,"width  %f w %f scale %f width in mm %f\n",width,w,scale,widthInmm);
  zeroX = -bounds[0];
  zeroY = -bounds[1];
  /* old code
  if(bounds[0] < 0)
    zeroX = fabs(bounds[0]);
  if(bounds[2] < 0)
    zeroY = fabs(bounds[1]);
  //shiftY = shiftY+zeroY+scale*fabs(bounds[1]-bounds[3])/2.;
  //shiftX = shiftX+zeroX+scale*fabs(bounds[0]-bounds[2])/2.;
  // if(center == 0.) {
  //  shiftX = 0.;
  //  shiftY = 0.;
  //}
  */

#ifdef _WIN32
seedrand((float)time(0));
#endif
 if(append)
   gcode = fopen(argv[optind+1],"a");
 else
  gcode=fopen(argv[optind+1],"w");
 if(gcode == NULL) {
   printf("can't open output %s\n",argv[optind+1]);
   return -1;
 }
  printf("paths %d points %d\n",pathCount, pointsCount);
  // allocate memory
  points = (SVGPoint*)malloc(pathCount*2*sizeof(SVGPoint));
  cities = (int*)malloc(pathCount*2*sizeof(int));
  paths = (ToolPath*)malloc(pointsCount*2*sizeof(ToolPath));
  
  npaths = 0;
  calcPaths(points,paths,cities,&npaths);
  /*  if(doBez) {
    Bez = fopen("bez.lst","w");
    if(Bez == NULL) {
      printf("bez file!\n");
      return -1;
    }
    fprintf(Bez,"paths = %d\n",npaths);
  }
  */
  printf("reorder ");
  for(k=0;k<numReord;k++) {
    reorder(points,cities,pathCount,xy);
    printf("%d... ",k);
    fflush(stdout);
  }
  printf("\n");
  //  fprintf(gcode,"( bounds %f %f %f %f paths %d)\n",((zeroX+shiftX+ bounds[0])*scale),((zeroY+shiftY+bounds[1])*scale),((zeroX+shiftX+bounds[2])*scale),((zeroY+shiftY+bounds[3])*scale),npaths);
  fprintf(gcode,"( bounds %f %f %f %f paths %d)\n",shiftX+ (zeroX + bounds[0])*scale,shiftY+(zeroY+bounds[1])*scale,shiftX+(zeroX+bounds[2])*scale,shiftY+(zeroY+bounds[3])*scale,npaths);
  if(first) {
    fprintf(gcode,GHEADER);
    //fprintf(gcode,"G05 P%d\n",pwr);
  }
  if(cncMode) //cnc-mode can use z-axis
    fprintf(gcode,GMODE);
#ifdef G32
  fprintf(gcode,CUTTERON,pwr);
#endif  
  k=0;
  i=0;
  for(i=0;i<pathCount;i++) {
    for(k=0;k<npaths;k++) {
      if(paths[k].city == -1)
	continue;
      if(paths[k].city == cities[i])
	break;
    }
    if(k >= npaths-1) {
      //printf("k > \n");
      continue;
    }
    firstx = x = (paths[k].points[0]+zeroX)*scale+shiftX;
    firsty = y =  (paths[k].points[1]+zeroY)*scale+shiftY; // changed
    if(flip) {
      firsty = -firsty;
      y = -y;
    }
    if(x > maxx)
      maxx = x;
    if(x < minx)
      minx = x;
    if(y > maxy)
      maxy = y;
    if(y < miny)
      miny = y;
    if(tsp) {
      if(tspFirst) {
	fprintf(gcode,"G1 X%.4f Y%.4f F4800 \n",x,y);
	fprintf(gcode,"G4 P0\n");
	tspFirst = 0;
	fprintf(gcode,CUTTERON,pwr);
      }
      else {
	fprintf(gcode,"G1 X%.1f Y%.1f  F%d\n",x,y,feed);
	fprintf(gcode,"G4 P0\n");
      }
      continue;
    }
    if(!cncMode)
      fprintf(gcode,"G0 X%.4f Y%.4f\n",x,y);
#ifndef G32
    else
      fprintf(gcode,"G0 X%.1f Y%.1f\n",x,y);
    fprintf(gcode,"G4 P0\n");
#endif    
    fprintf(gcode,"( city %d )\n",paths[k].city);
#ifndef G32    
    if(!cncMode)
      fprintf(gcode,CUTTERON,pwr);
    fprintf(gcode,"G4 P0\n");
#endif
    printed=0;
    if(tsp)
      continue;
    for(j=k;j<npaths;j++) {
      xold = x;
      yold = y;
      //printf("bezC %d\n",bezCount);
      skip = 0;
      first = 1;

      if(paths[j].city == cities[i]) {
	if(doBez) {
	  bezCount = 0;
	  if(paths[j].points[0] == paths[j].points[2] && paths[j].points[1]==paths[j].points[3])
	    ;//continue;
	  cubicBez(paths[j].points[0],paths[j].points[1],paths[j].points[2],paths[j].points[3],paths[j].points[4],paths[j].points[5],paths[j].points[6],paths[j].points[7],tol,0);
	  bxold=x;
	  byold=y;
	  for(l=0;l<bezCount;l++) {
	    if(bezPoints[l].x > bounds[2] || bezPoints[l].x < bounds[0] || isnan(bezPoints[l].x)) {
	      printf("bezPoints %f %f\n",bezPoints[l].x,bounds[0]);
	      continue;
	    }
	    if(bezPoints[l].y > bounds[3]) {
	      printf("bezPoints y %d\n",l);
	      continue;
	    }
	    bx = (bezPoints[l].x+zeroX)*scale+shiftX;
	    by = (bezPoints[l].y+zeroY)*scale+shiftY;
	    if(flip)
	      by = -by;
	    if(bx > maxx)
	      maxx = bx;
	    if(bx < minx)
	      minx = x;
	    if(by > maxy)
	      maxy = by;
	    if(y < miny)
	      miny = by;
	    
	    d = sqrt((bx-bxold)*(bx-bxold)+(by-byold)*(by-byold));
	    if(1) {
	      printed = 1;
	      fprintf(gcode,"G1 X%.4f Y%.4f  F%d\n",bx,by,feed);
#ifndef	  G32    
	      fprintf(gcode,"G4 P0\n");
#endif	      
	    } else {
	      fprintf(gcode,"( acc )\n");;//continue;
	      //fprintf(gcode,"G05 P%d\n",(int)(pwr*0.33));
	      fprintf(gcode,"G1 X%.4f Y%.4f F%d\n",bx,by,feed);
	    }
	    bxold = bx;
	    byold = by;
	  }
	} else {
	  x = (paths[j].points[0]-fabs(bounds[0]))*scale+shiftX;
	  y = (paths[j].points[1]-fabs(bounds[1]))*scale+shiftY;
	  if(flip)
	    y = -y;
	  if(x > maxx)
	    maxx = x;
	  if(x < minx)
	    minx = x;
	  if(y > maxy)
	    maxy = y;
	  if(y < miny)
	    miny = y;

//	  if(paths[j].points[0]==paths[j].points[2] && paths[j].points[1]==paths[j].points[3]) {
	  if(1) {
	    //d = sqrt((x-xold)*(x-xold)+(y-yold)*(y-yold));
	    if(1) {
	      printed = 1;
	      fprintf(gcode,"G1 X%.4f Y%.4f  F%d\n",x,y,feed);
#ifndef	 G32 
	      fprintf(gcode,"G4 P0\n");
#endif
	    } else {
	      ;//continue;
	      //fprintf(gcode,"G05 P%d\n",(int)(pwr*0.33));
	      //fprintf(gcode,"G01 X%.4f Y%.4f  F%d\n",x,y,feed);
	    }
	    first = 0;
	    xold = x;
	    yold = y;
	  } else {
	    x = (paths[j].points[0]-fabs(bounds[0]))*scale+shiftX;
	    y = (paths[j].points[1]-fabs(bounds[1]))*scale+shiftY;
	    if(flip)
	      y = -y;
	    fprintf(gcode,CUTTEROFF);
	    fprintf(gcode,"( simplified )\n");
	    fprintf(gcode,"G0 X%.4f Y%.4f\n",x,y);
#ifndef G32	    
	    fprintf(gcode,"G4 P0\n");
#endif	    
	    x = (paths[j].points[2]-fabs(bounds[0]))*scale+shiftX;
	    y = (paths[j].points[3]-fabs(bounds[1]))*scale+shiftY;
	    if(flip)
	      y = -y;
#ifndef	 G32  
	    fprintf(gcode,CUTTERON,pwr);
#endif	    
	    //fprintf(gcode,"G01 X%.4f Y%.4f  F%d\n",x,y,feed);
	    xold = x;
	    yold = y;
	  }
	}
	paths[j].city = -1;
	/*
	if(printed == 0) {
	  if(doBez)
	    fprintf(gcode,"G01 X%.4f Y%.4f  F%d\n",bx,by,feed);
	  else
	    fprintf(gcode,"G01 X%.4f Y%.4f  F%d\n",x,y,feed);
	}
	*/
      } else
	break;
    }
    if(tsp)
      continue;
    if(paths[j].closed) {
      fprintf(gcode, "( end )\n");
      //bx = (bezPoints[0].x-bounds[0])*scale;
      //by = (flip ? (bounds[3]-bezPoints[0].y)*scale : (bezPoints[0].y-bounds[1])*scale);
      
      fprintf(gcode,"G1 X%.4f Y%.4f  F%d\n",firstx,firsty,feed);
#ifndef G32      
      fprintf(gcode,"G4 P0\n");
#endif      
      printed = 1;
    }
    if(!cncMode) {
      if(!printed) {
#ifndef G32	
	if(dwell != -1) {
	  fprintf(gcode,"( lowpwr )\n");
	  fprintf(gcode,"M3 S10\n");
	  sprintf(gbuff,"G4 P%d\n",dwell);
	  fprintf(gcode,"%s",gbuff);
	}
	fprintf(gcode,"M5\n");
#else
	fprintf(gcode,"( new dwell? )\n");
#endif	
	//fprintf(gcode,"G05 P%d\n",pwr);
      }
#ifndef G32      
      fprintf(gcode,CUTTEROFF);
#endif      
    } else {
      fprintf(gcode,"G1 ZF%d\n",feed);
      fprintf(gcode,"G4 P0\n");
      printed = 0;
    } 
  }
#ifndef G32  
  if(tsp)
    fprintf(gcode,CUTTEROFF);
#else
  fprintf(gcode,"M5\n");
#endif  
  fprintf(gcode,GFOOTER);
  printf("( size X%.4f Y%.4f x X%.4f Y%.4f )\n",minx,miny,maxx,maxy);
  fprintf(gcode,"( size X%.4f Y%.4f x X%.4f Y%.4f )\n",minx,miny,maxx,maxy);
  fclose(gcode);
  free(points);
  free(cities); 
  free(paths);
  nsvgDelete(g_image);
  return 0;
}
