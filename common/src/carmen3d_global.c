#include "carmen3d_global.h"
#include <carmen_utils/global.h>
#include <pthread.h>

/* void carmen3d_angle_conversion_unit_test() */
/* { */
/*   //some random axis angles: */
/*   erlcm_axis_angle_t a, r; */
/*   double err; */

/*   a.ax = 0; */
/*   a.ay = 0; */
/*   a.az = 0; */
/*   r = carmen3d_euler_to_axis_angle(carmen3d_axis_angle_to_euler(a)); */
/*   err = fabs(a.ax - r.ax) + fabs(a.ay - r.ay) + fabs(a.az - r.az); */
/*   if (err > 1e-8) { */
/*     printf("ERROR, bad angle conversion!\n"); */
/*     exit(1); */
/*   } */

/*   a.ax = 0.9853; */
/*   a.ay = -1.0553; */
/*   a.az = 0.4083; */
/*   r = carmen3d_euler_to_axis_angle(carmen3d_axis_angle_to_euler(a)); */
/*   err = fabs(a.ax - r.ax) + fabs(a.ay - r.ay) + fabs(a.az - r.az); */
/*   if (err > 1e-8) { */
/*     printf("ERROR, bad angle conversion!\n"); */
/*     exit(1); */
/*   } */

/*   a.ax = -1.0389; */
/*   a.ay = -0.7771; */
/*   a.az = 0.0921; */
/*   r = carmen3d_euler_to_axis_angle(carmen3d_axis_angle_to_euler(a)); */
/*   err = fabs(a.ax - r.ax) + fabs(a.ay - r.ay) + fabs(a.az - r.az); */
/*   if (err > 1e-8) { */
/*     printf("ERROR, bad angle conversion!\n"); */
/*     exit(1); */
/*   } */

/*   a.ax = 0.2220; */
/*   a.ay = 1.1334; */
/*   a.az = -0.6655; */
/*   r = carmen3d_euler_to_axis_angle(carmen3d_axis_angle_to_euler(a)); */
/*   err = fabs(a.ax - r.ax) + fabs(a.ay - r.ay) + fabs(a.az - r.az); */
/*   if (err > 1e-8) { */
/*     printf("ERROR, bad angle conversion!\n"); */
/*     exit(1); */
/*   } */

/*   a.ax = 0.2491; */
/*   a.ay = -0.1377; */
/*   a.az = -0.5860; */
/*   r = carmen3d_euler_to_axis_angle(carmen3d_axis_angle_to_euler(a)); */
/*   err = fabs(a.ax - r.ax) + fabs(a.ay - r.ay) + fabs(a.az - r.az); */
/*   if (err > 1e-8) { */
/*     printf("ERROR, bad angle conversion!\n"); */
/*     exit(1); */
/*   } */
/*   printf("Angle Conversion Unit Test Passed!\n"); */
/* } */

// Returns the hash value of the null terminated C string 'string' using the
// SDBM hash algorithm. The number of significant characters for which the
// hash value will be calculated is limited to MAXIMUM_LENGTH_FOR_STRINGS.
unsigned int carmen3d_hash(const char *string)
{
  register unsigned int len, index, hash = 0;
  register char ch;

  len = strlen(string);
  if (len > 1000) {
    len = 1000;
  } // end if

  for (index = 0; index < len; index++) {
    ch = string[index];
    hash = ch + (hash << 6) + (hash << 16) - hash;
  } // end for

  return (hash & 0x7FFFFFFF);
} // end calc_hash

#define NUM_TICTOCS 1000
typedef struct {
  double t;
  double totalT;
  double ema;
  unsigned int numCalls;
  char flag;
  char * description;
} tic_toc_t;

//#define DISABLE_TICTOC

int tt_aveTimeCompare(const void *a, const void *b)
{
  tic_toc_t *t1 = (tic_toc_t *) a;
  tic_toc_t *t2 = (tic_toc_t *) b;
  if (t1->numCalls<1)
      return 1;
  else
      return (t1->totalT / t1->numCalls) < (t2->totalT / t2->numCalls);
}
int tt_totalTimeCompare(const void *a, const void *b)
{
  tic_toc_t *t1 = (tic_toc_t *) a;
  tic_toc_t *t2 = (tic_toc_t *) b;
  if (t1->numCalls<1)
      return 1;
  else
      return t1->totalT < t2->totalT;
}
int tt_aveAlphCompare(const void *a, const void *b)
{
  tic_toc_t *t1 = (tic_toc_t *) a;
  tic_toc_t *t2 = (tic_toc_t *) b;
  if (t1->numCalls<1)
      return 1;
  else
      return strcmp(t1->description, t2->description);
}

static pthread_mutex_t tictoc_lock;
static int tictoc_zeroed = 0;
static int tictoc_initialized = 0;
static tic_toc_t tic_tocs[NUM_TICTOCS];
void carmen3d_tictoc_init() //this MUST be called if you want tictoc to be thread safe
{
#ifdef DISABLE_TICTOC
  return;
#endif
  pthread_mutex_init(&tictoc_lock, NULL);
  tictoc_initialized = 1;
}

double carmen3d_tictoc(const char *description)
{
  return carmen3d_tictoc_ema2(description, .01, NULL);
}

double carmen3d_tictoc_ema(const char *description, double ema_alpha)
{
  return carmen3d_tictoc_ema2(description, ema_alpha, NULL);
}
double carmen3d_tictoc_ema2(const char *description, double ema_alpha, double * ema)
{

#ifdef DISABLE_TICTOC
  return;
#endif
  //profiling tool... call the first time to set clock going, call again to stop clock.
  //call with NULL to print results
  //need to call init before use to setup lock...

  if (!tictoc_zeroed){
      tictoc_zeroed=1;
      memset(tic_tocs,0,sizeof(tic_toc_t)*NUM_TICTOCS);
  }

  if (tictoc_initialized)
    pthread_mutex_lock(&tictoc_lock); //aquire the lock

  double tictoctime = -1;
  if (description != NULL) {
    unsigned idx = carmen3d_hash(description) % NUM_TICTOCS;
    if (tic_tocs[idx].flag == 0) {
      tic_tocs[idx].t = carmen_get_time();
      tic_tocs[idx].flag = 1;
    }
    else if (tic_tocs[idx].flag == 1) {
      tictoctime = carmen_get_time() - tic_tocs[idx].t;
      tic_tocs[idx].totalT = tic_tocs[idx].totalT + tictoctime;
      tic_tocs[idx].ema = tic_tocs[idx].ema * (1 - ema_alpha) + tictoctime * ema_alpha;
      if (tic_tocs[idx].numCalls == 0) {
        //first time... store description
        tic_tocs[idx].description = (char *) malloc((strlen(description) + 1) * sizeof(char));
        strcpy(tic_tocs[idx].description, description);
      }
      if (strcmp(tic_tocs[idx].description, description) != 0) {
        printf("ERROR! HASH COLLISION in tictoc!!!\n");
        printf("%s collided with %s, both had has=%d\n", tic_tocs[idx].description, description, idx);
        exit(1);
      }
      tic_tocs[idx].numCalls++;
      tic_tocs[idx].flag = 0;
      if (ema!=NULL)
        *ema = tic_tocs[idx].ema;
    }
  }
  if (tictoc_initialized)
    pthread_mutex_unlock(&tictoc_lock);
  return tictoctime;
}

double carmen3d_tictoc_get_ave(char * description)
{
  unsigned idx = carmen3d_hash(description) % NUM_TICTOCS;
  return tic_tocs[idx].totalT / tic_tocs[idx].numCalls;
}
double carmen3d_tictoc_get_ema(char * description)
{
  unsigned idx = carmen3d_hash(description) % NUM_TICTOCS;
  return tic_tocs[idx].ema;

}

void carmen3d_tictoc_print()
{
  if (tictoc_initialized)
    pthread_mutex_lock(&tictoc_lock);

#ifdef PRINT_TO_FILE
  FILE * f = fopen("tictoc.txt", "w");
#endif

  printf("\n\n\n");
  printf("timer results (by average time):\n\n");

  qsort(tic_tocs, NUM_TICTOCS, sizeof(tic_toc_t), tt_aveTimeCompare);

  for (int i = 0; i < NUM_TICTOCS; i++) {
      if (tic_tocs[i].numCalls<1)
	  continue;

    int numSpaces = 50 - strlen(tic_tocs[i].description);
    char spaces[50] = { 0 };
    memset(spaces, ' ', sizeof(spaces));
    spaces[numSpaces] = '\0';


    printf("%s %s\t called %d times,\t took %10.5f Average, \t %10.5f total\n", tic_tocs[i].description, spaces,
        tic_tocs[i].numCalls, tic_tocs[i].totalT / tic_tocs[i].numCalls, tic_tocs[i].totalT);
#ifdef PRINT_TO_FILE
    fprintf(f,"%s %s\t called %d times,\t took %10.5f Average, \t %10.5f total\n", tic_tocs[i].description, spaces,
        tic_tocs[i].numCalls, tic_tocs[i].totalT / tic_tocs[i].numCalls, tic_tocs[i].totalT);
#endif

  }
#ifdef PRINT_TO_FILE
  fclose(f);
#endif

  if (tictoc_initialized)
    pthread_mutex_unlock(&tictoc_lock);
}

/*
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code by Nicolas Devillard - 1998. Public domain.
 */

#define ELEM_SWAP(a,b) { register float t=(a);(a)=(b);(b)=t; }

float carmen3d_median(float arr[], int n)
{
  int low, high;
  int median;
  int middle, ll, hh;

  low = 0;
  high = n - 1;
  median = (low + high) / 2;
  for (;;) {
    if (high <= low) /* One element only */
      return arr[median];

    if (high == low + 1) { /* Two elements only */
      if (arr[low] > arr[high])
        ELEM_SWAP(arr[low], arr[high]);
      return arr[median];
    }

    /* Find median of low, middle and high items; swap into position low */
    middle = (low + high) / 2;
    if (arr[middle] > arr[high])
      ELEM_SWAP(arr[middle], arr[high]);
    if (arr[low] > arr[high])
      ELEM_SWAP(arr[low], arr[high]);
    if (arr[middle] > arr[low])
      ELEM_SWAP(arr[middle], arr[low]);

    /* Swap low item (now in position middle) into position (low+1) */
    ELEM_SWAP(arr[middle], arr[low+1]);

    /* Nibble from each end towards middle, swapping items when stuck */
    ll = low + 1;
    hh = high;
    for (;;) {
      do
        ll++;
      while (arr[low] > arr[ll]);
      do
        hh--;
      while (arr[hh] > arr[low]);

      if (hh < ll)
        break;

      ELEM_SWAP(arr[ll], arr[hh]);
    }

    /* Swap middle item (in position low) back into correct position */
    ELEM_SWAP(arr[low], arr[hh]);

    /* Re-set active partition */
    if (hh <= median)
      low = ll;
    if (hh >= median)
      high = hh - 1;
  }
}

#undef ELEM_SWAP
