#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
  double* list = new double[new_size];
  for (int i=0; i<length; i++) {
      list[i] = array[i];
  }
  delete[] array;
  for (int y=length; y<new_size; y++) {
      list[y] = 0;
  }
  return list;
}

double* shrink_array(double* array, int length, int new_size) {
    double* list = new double[new_size];
    for (int i=0; i<new_size; i++) {
        list[i] = array[i];
    }
    delete[] array;
    return list;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if (current_size == max_size) {
        double* list = extend_array(array, current_size, current_size+5);
        array = list;
        max_size += 5;
    }
    array[current_size] = element;
    current_size += 1;
  return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
  array[current_size-1] = 0;
  current_size -= 1;
  if (max_size - current_size >= 5) {
      double* list = shrink_array(array, current_size, max_size-5);
      max_size -= 5;
      array = list;
  }
  return array; // YOU CAN CHANGE THIS
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double* list = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = list;
    list = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = list;
    list = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = list;
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
    }
  }

  return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
  global_telemetry_current_size = 0;
  global_telemetry_max_size = 0;
  for (int i=0; i<tot_telemetries; i++) {
      global_telemetry_max_size += telemetries_sizes[i];
  }
  global_telemetry = new double[global_telemetry_max_size];
  for (int i=0; i<tot_telemetries; i++) {
      int y=telemetries_sizes[i];
      while (telemetries[i][y-1] == 0) {
          y -= 1;
      }
      for (int num=0; num<y; num++) {
          global_telemetry = append_to_array(telemetries[i][num], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
      }
  }
  for (int i=0; i<global_telemetry_current_size-3; i+=3) {
      int j=1;
      while (i+3*j+2 < global_telemetry_current_size) {
          if (global_telemetry[i] > global_telemetry[i+3*j]) {
              double a = global_telemetry[i];
              double b = global_telemetry[i+1];
              double c = global_telemetry[i+2];
              global_telemetry[i] = global_telemetry[i+3*j];
              global_telemetry[i+1] = global_telemetry[i+3*j+1];
              global_telemetry[i+2] = global_telemetry[i+3*j+2];
              global_telemetry[i+3*j] = a;
              global_telemetry[i+3*j+1] = b;
              global_telemetry[i+3*j+2] = c;
          }
          j += 1;
      }
  }
}
