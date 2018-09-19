#ifndef CARMEN3D_GLOBAL_H
#define CARMEN3D_GLOBAL_H

#ifdef __cplusplus
extern "C" {
#endif
#include <math.h>
#include <carmen_utils/global.h>
#include <lcmtypes/hr_lcmtypes.h>

  static inline double carmen3d_fsign(double num)
  {
    return (((num)>0)-((num)<0));
  }

  static inline int carmen3d_isign(int num)
  {
    if (num > 0)
      return 1;
    else if (num == 0)
      return 0;
    else
      return -1;
  }

  static inline double carmen3d_ema(double ema, double meas, double emaAlpha)
  {
    return ema * (1 - emaAlpha) + meas * emaAlpha;
  }
  static inline double carmen3d_angle_subtract(double theta1, double theta2)
  {
    return carmen_normalize_theta(carmen_normalize_theta(theta1) - carmen_normalize_theta(theta2));

  }

  static inline double carmen3d_dist(ripl_point_t *p1, ripl_point_t *p2)
  {
    return sqrt(carmen_square(p1->x - p2->x) + carmen_square(p1->y - p2->y) + carmen_square(p1->z - p2->z));
  }

  static inline void carmen3d_point_to_point3d(carmen_point_t* point, ripl_point_t* point3d)
  {
    point3d->x = point->x;
    point3d->y = point->y;
    point3d->yaw = point->theta;
  }

  static inline void carmen3d_point3d_to_point(ripl_point_t* point3d, carmen_point_t* point)
  {
    point->x = point3d->x;
    point->y = point3d->y;
    point->theta = point3d->yaw;
  }

  static inline double carmen3d_mm_to_meter(double milimeter_reading)
  {
    return milimeter_reading / 1000.0;
  }


  static inline carmen_point_t carmen3d_body2D_difference(const carmen_point_t *p1, const carmen_point_t *p2)
  {
    carmen_point_t delta;
    double dx = p1->x - p2->x;
    double dy = p1->y - p2->y;

    double s = sin(p2->theta), c = cos(p2->theta);
    delta.x = c * dx + s * dy;
    delta.y = -s * dx + c * dy;
    delta.theta = carmen3d_angle_subtract(p1->theta, p2->theta);
    return delta;
  }

  static inline void carmen3d_rotate_cov_mat_2D(double sigmaX, double sigmaXY, double sigmaY, double theta,
      double *sxxR, double *sxyR, double *syyR)
  {
    //do this via R^-1*S*R
    double c = cos(theta), s = sin(theta);
    //do S*R
    double xx = c * sigmaX - s * sigmaXY;
    double xy1 = s * sigmaX + c * sigmaXY;
    double xy2 = c * sigmaXY - s * sigmaY;
    double yy = s * sigmaXY + c * sigmaY;

    //do R' aka (R^-1) times  [xx xy1;xy2 yy]
    *sxxR = c * xx - s * xy2;
    *sxyR = c * xy1 - s * yy;
    //  double sxy2=s*xx+c*xy2;
    *syyR = s * xy1 + c * yy;
  }

  static inline carmen_point_t carmen3d_body2D_sum(const carmen_point_t *p1,const carmen_point_t *p2)
  {
    //sum taking into account yaw, ignoring z,pitch, and roll (set to p1)!
    carmen_point_t sum;
    double s = sin(p1->theta), c = cos(p1->theta);

    sum.x = p1->x + c * p2->x - s * p2->y;
    sum.y = p1->y + s * p2->x + c * p2->y;
    sum.theta = p1->theta + p2->theta;

    return sum;
  }

  /* static inline ripl_quaternion_t carmen3d_axis_angle_to_quaternion(ripl_axis_angle_t axis_angle) */
  /* { */
  /*   double theta = sqrt(axis_angle.ax * axis_angle.ax + axis_angle.ay * axis_angle.ay + axis_angle.az * axis_angle.az); */
  /*   ripl_quaternion_t quarternion; */
  /*   quarternion.q0 = cos(theta / 2); */
  /*   quarternion.q1 = axis_angle.ax / theta * sin(theta / 2); */
  /*   quarternion.q2 = axis_angle.ay / theta * sin(theta / 2); */
  /*   quarternion.q3 = axis_angle.az / theta * sin(theta / 2); */
  /*   //  printf("theta %f q0 %f q1 %f q2 %f q3 %f\n", theta, quarternion.q0, quarternion.q1, quarternion.q2, quarternion.q3); */
  /*   return quarternion; */
  /* } */

  /* static inline ripl_euler_t carmen3d_quaternion_to_euler(ripl_quaternion_t quaternion) */
  /* { */
  /*   ripl_euler_t euler; */
  /*   euler.yaw = atan2(2 * (quaternion.q0 * quaternion.q3 + quaternion.q1 * quaternion.q2), 1 - 2 * (quaternion.q2 */
  /*       * quaternion.q2 + quaternion.q3 * quaternion.q3)); */
  /*   euler.pitch = asin(2 * (quaternion.q0 * quaternion.q2 - quaternion.q3 * quaternion.q1)); */

  /*   euler.roll = atan2(2 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3), 1 - 2 * (quaternion.q1 */
  /*       * quaternion.q1 + quaternion.q2 * quaternion.q2)); */
  /*   return euler; */
  /* } */

  /* static inline ripl_euler_t carmen3d_axis_angle_to_euler(ripl_axis_angle_t axis_angle) */
  /* { */
  /*   ripl_euler_t euler; */
  /*   euler = carmen3d_quaternion_to_euler(carmen3d_axis_angle_to_quaternion(axis_angle)); */
  /*   return euler; */
  /* } */

  /* static inline ripl_axis_angle_t carmen3d_quaternion_to_axis_angle(ripl_quaternion_t quaternion) */
  /* { */
  /*   ripl_axis_angle_t axis_angle; */

  /*   double theta = 2 * acos(quaternion.q0); */
  /*   if (theta != 0) { */
  /*     axis_angle.ax = quaternion.q1 / sin(theta / 2) * theta; */
  /*     axis_angle.ay = quaternion.q2 / sin(theta / 2) * theta; */
  /*     axis_angle.az = quaternion.q3 / sin(theta / 2) * theta; */
  /*   } */
  /*   else { */
  /*     axis_angle.ax = 0; */
  /*     axis_angle.ay = 0; */
  /*     axis_angle.az = 0; */
  /*   } */
  /*   return axis_angle; */
  /* } */

  /* static inline ripl_rot_matrix_t carmen3d_quaternion_to_rot_matrix(ripl_quaternion_t quaternion) */
  /* { */
  /*   ripl_rot_matrix_t rot_mat; */

  /*   double * R = rot_mat.R; */

  /*   double q0 = quaternion.q0; */
  /*   double q1 = quaternion.q1; */
  /*   double q2 = quaternion.q2; */
  /*   double q3 = quaternion.q3; */

  /*   //  double q0_sq = carmen_square(q0); */
  /*   double q1_sq = carmen_square(q1); */
  /*   double q2_sq = carmen_square(q2); */
  /*   double q3_sq = carmen_square(q3); */

  /*   R[0] = 1 - 2 * (q2_sq + q3_sq); */
  /*   R[1] = 2 * (q1 * q2 - q0 * q3); */
  /*   R[2] = 2 * (q0 * q2 + q1 * q3); */
  /*   R[3] = 2 * (q1 * q2 + q0 * q3); */
  /*   R[4] = 1 - 2 * (q1_sq + q3_sq); */
  /*   R[5] = 2 * (q2 * q3 - q0 * q1); */
  /*   R[6] = 2 * (q1 * q3 - q0 * q2); */
  /*   R[7] = 2 * (q0 * q1 + q2 * q3); */
  /*   R[8] = 1 - 2 * (q1_sq + q2_sq); */

  /*   return rot_mat; */
  /* } */

  /* static inline ripl_quaternion_t carmen3d_euler_to_quaternion(ripl_euler_t euler) */
  /* { */
  /*   double phi = euler.roll; */
  /*   double theta = euler.pitch; */
  /*   double psi = euler.yaw; */
  /*   ripl_quaternion_t quaternion; */
  /*   quaternion.q0 = cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(theta / 2) * sin(psi / 2); */
  /*   quaternion.q1 = sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(theta / 2) * sin(psi / 2); */
  /*   quaternion.q2 = cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(theta / 2) * sin(psi / 2); */
  /*   quaternion.q3 = cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * sin(theta / 2) * cos(psi / 2); */
  /*   return quaternion; */
  /* } */

  /* static inline ripl_rot_matrix_t carmen3d_euler_to_rot_matrix(ripl_euler_t euler) */
  /* { */
  /*   return carmen3d_quaternion_to_rot_matrix(carmen3d_euler_to_quaternion(euler)); */
  /* } */

  /* static inline ripl_axis_angle_t carmen3d_euler_to_axis_angle(ripl_euler_t euler) */
  /* { */
  /*   return carmen3d_quaternion_to_axis_angle(carmen3d_euler_to_quaternion(euler)); */
  /* } */

  //void carmen3d_angle_conversion_unit_test();

#define C3D_EPS 0.0001
#define GRAVITY 9.81
  typedef struct {
    double x, y, z;
  } carmen3d_pos_t;

  typedef struct {
    double xx;
    double xy;
    double xz;
    double yy;
    double yz;
    double zz;
  } carmen3d_covariance_t;


  unsigned int carmen3d_hash(const char *string);
  void carmen3d_tictoc_init();
  void carmen3d_tictoc_print();
  double carmen3d_tictoc_ema(const char *description, double ema_alpha);
  double carmen3d_tictoc_ema2(const char *description, double ema_alpha, double * ema);
  double carmen3d_tictoc(const char *description);

  float carmen3d_median(float arr[], int n);

  static inline void carmen3d_safeRead(int __fd, void * __buf, size_t __nbytes)
  {
    size_t numRead = read(__fd, __buf, __nbytes);
    if (numRead != __nbytes) {
      fprintf(stderr,"ERROR!!!, could not read the requested number of bytes!\n");
    }
  }
  static inline void carmen3d_safeWrite(int __fd, const void * __buf, size_t __nbytes)
  {
    size_t numWritten = write(__fd, __buf, __nbytes);
    if (numWritten != __nbytes) {
    	fprintf(stderr,"ERROR!!!, could not write the requested number of bytes!\n");
    }
  }

  static inline void carmen3d_dumpDoubleArrayToMatlab(const char * fname, const char * varname, double * array,
      int size, int rowStride)
  {
    FILE * f = fopen(fname, "w");
    fprintf(f, "%s = [\n", varname);
    int i = 0;
    for (i = 0; i < size; i++) {
      if (i % rowStride == 0)
        fprintf(f, "\n");
      fprintf(f, "%f,", array[i]);
    }
    fprintf(f, "\n];\n");
    fclose(f);
  }

  static inline void carmen3d_dumpFloatArrayToMatlab(const char * fname, const char * varname, float * array, int size,
      int rowStride)
  {
    FILE * f = fopen(fname, "w");
    fprintf(f, "%s = [\n", varname);
    int i = 0;
    for (i = 0; i < size; i++) {
      if (i % rowStride == 0)
        fprintf(f, "\n");
      fprintf(f, "%f,", array[i]);
    }
    fprintf(f, "\n];\n");
    fclose(f);
  }

  static inline void carmen3d_dumpIntArrayToMatlab(const char * fname, const char * varname, int * array, int size,
      int rowStride)
  {
    FILE * f = fopen(fname, "w");
    fprintf(f, "%s = [\n", varname);
    int i = 0;
    for (i = 0; i < size; i++) {
      if (i % rowStride == 0)
        fprintf(f, "\n");
      fprintf(f, "%d,", array[i]);
    }
    fprintf(f, "\n];\n");
    fclose(f);
  }
  static inline void carmen3d_dumpCharArrayToMatlab(const char * fname, const char * varname, char * array, int size,
      int rowStride)
  {
    FILE * f = fopen(fname, "w");
    fprintf(f, "%s = [\n", varname);
    int i = 0;
    for (i = 0; i < size; i++) {
      if (i % rowStride == 0)
        fprintf(f, "\n");
      fprintf(f, "%d,", (int) array[i]);
    }
    fprintf(f, "\n];\n");
    fclose(f);
  }

  static inline void carmen3d_dumpUCharArrayToMatlab(const char * fname, const char * varname, unsigned char * array,
      int size, int rowStride)
  {
    FILE * f = fopen(fname, "w");
    fprintf(f, "%s = [\n", varname);
    int i = 0;
    for (i = 0; i < size; i++) {
      if (i % rowStride == 0)
        fprintf(f, "\n");
      fprintf(f, "%d,", (int) array[i]);
    }
    fprintf(f, "\n];\n");
    fclose(f);
  }

  static inline double carmen3d_dist_to_line(carmen_point_p pt, carmen_point_p v1, carmen_point_p v2)
  { //compute distance to line that passes through these two points
    //expanded ugliness from symbolic matlab:
    //    a = v1 - v2;
    //    b = pt - v2;
    //    d = abs(det([a;b])) / sqrt(dot(a,a));
    return fabs(-v1->x * pt->y + v1->x * v2->y + v2->x * pt->y + v1->y * pt->x - v1->y * v2->x - v2->y * pt->x) / sqrt(
        v1->x * v1->x - 2 * v1->x * v2->x + v2->x * v2->x + v1->y * v1->y - 2 * v1->y * v2->y + v2->y * v2->y);
  }

  static inline double carmen3d_dist_to_segment(carmen_point_p pt, carmen_point_p v1, carmen_point_p v2,
      int * vertexNum)
  { //compute the distance to the line segment, or the closest endpoint if the point isn't inside
    double dot1 = (v2->x - v1->x) * (pt->x - v1->x) + (v2->y - v1->y) * (pt->y - v1->y);
    double dot2 = (v1->x - v2->x) * (pt->x - v2->x) + (v1->y - v2->y) * (pt->y - v2->y);
    int insideSegment = dot1 > 0 && dot2 > 0;
    if (insideSegment) //we're inside the segment, so return the perpendicular dist
    {
      *vertexNum = 0;
      return carmen3d_dist_to_line(pt, v1, v2);
    }
    else {
      double dv1 = carmen_distance(pt, v1);
      double dv2 = carmen_distance(pt, v2);
      if (dv1 < dv2) {
        *vertexNum = 1;
        return dv1;
      }
      else {
        *vertexNum = 2;
        return dv2;
      }
    }

  }

#ifdef __cplusplus
}
#endif

#endif
