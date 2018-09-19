/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/**********************************************
 * library of function for mapserver clients  *
 **********************************************/
#include <carmen_utils/carmen.h>
#ifndef NO_ZLIB
#include <zlib.h>
#endif

carmen_map_t **map_update;
char ***zone_update;

carmen_inline int 
carmen_map_to_world(carmen_map_point_p carmen_map_point, 
		    carmen_world_point_p world_point) 
{
  double x = carmen_map_point->x * carmen_map_point->map->config.resolution;
  double y = carmen_map_point->y * carmen_map_point->map->config.resolution;
  
  world_point->pose.x = x;
  world_point->pose.y = y;
  world_point->pose.theta = 0;
  world_point->map = carmen_map_point->map;

  return 0;
}

carmen_inline int 
carmen_world_to_map(carmen_world_point_p world_point, 
		    carmen_map_point_p carmen_map_point) 
{
  int x = 
    carmen_round(world_point->pose.x / world_point->map->config.resolution);
  int y = 
    carmen_round(world_point->pose.y / world_point->map->config.resolution);

  carmen_map_point->x = x;
  carmen_map_point->y = y;

  carmen_map_point->map = world_point->map;

  return 0;
}

carmen_inline int 
carmen_world_to_map_real(carmen_world_point_p world_point, 
		    carmen_point_p point) 
{
  double x = world_point->pose.x / world_point->map->config.resolution;
  double y = world_point->pose.y / world_point->map->config.resolution;

  point->x = x;
  point->y = y;
  point->theta = world_point->pose.theta;

  return 0;
}

carmen_inline int 
carmen_map_to_trajectory(carmen_map_point_p carmen_map_point, 
			 carmen_traj_point_p traj_point) 
{
  double x = carmen_map_point->x * carmen_map_point->map->config.resolution;
  double y = carmen_map_point->y * carmen_map_point->map->config.resolution;
  
  traj_point->x = x;
  traj_point->y = y;
  traj_point->theta = 0.0;
  traj_point->t_vel = 0.0;
  traj_point->r_vel = 0.0;

  return 0;
}

carmen_inline int
carmen_point_to_map(carmen_point_p point, carmen_map_point_p map_point, 
		    carmen_map_p map)
{
  int x = carmen_round(point->x / map->config.resolution);
  int y = carmen_round(point->y / map->config.resolution);

  map_point->x = x;
  map_point->y = y;
  map_point->map = map;

  return 0;
}

carmen_inline int 
carmen_trajectory_to_map(carmen_traj_point_p traj_point, 
			 carmen_map_point_p carmen_map_point,
			 carmen_map_p map) 
{
  int x = carmen_round(traj_point->x / map->config.resolution);
  int y = carmen_round(traj_point->y / map->config.resolution);

  carmen_map_point->x = x;
  carmen_map_point->y = y;

  carmen_map_point->map = map;

  return 0;
}

/* distance between two carmen_map_points in grid cells */
carmen_inline double 
carmen_distance_map(carmen_map_point_p p1, carmen_map_point_p p2) 
{
  return hypot(p1->x-p2->x, p1->y-p2->y);
}

carmen_inline double 
carmen_distance_world(carmen_world_point_p p1, carmen_world_point_p p2) 
{
  return hypot(p1->pose.x-p2->pose.x, p1->pose.y-p2->pose.y);
}

carmen_inline int
carmen_map_cmp(carmen_map_point_p p1, carmen_map_point_p p2)
{
  if (p1->x == p2->x && p1->y == p2->y)
    return 0;

  return 1;
}

carmen_inline int
carmen_world_cmp(carmen_world_point_p p1, carmen_world_point_p p2)
{
  if (fabs(p1->pose.x - p2->pose.x) < 1.0 &&
      fabs(p1->pose.y - p2->pose.y) < 1.0 && 
      fabs(carmen_normalize_theta(p1->pose.theta - p2->pose.theta)) < .017)
    return 0;

  return 1;
}

carmen_inline carmen_map_p
carmen_map_copy(carmen_map_p map) 
{
  carmen_map_p new_map;
  int i;

  new_map=(carmen_map_p)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(new_map);

  *new_map = *map;
  new_map->complete_map=(float *)
    calloc(map->config.x_size*map->config.y_size, sizeof(float));
  carmen_test_alloc(new_map->complete_map);
  memcpy(new_map->complete_map, map->complete_map, 
	 sizeof(float)*map->config.x_size*map->config.y_size);

  new_map->map=(float **)calloc(map->config.x_size, sizeof(float *));
  carmen_test_alloc(new_map->map);

  for (i = 0; i < new_map->config.x_size; i++)
    new_map->map[i] = new_map->complete_map+i*new_map->config.y_size;
  
  return new_map;
}

int 
carmen_map_apply_offlimits_chunk_to_map(carmen_offlimits_p offlimits_list,
					int list_length, carmen_map_p map)
{
  int current_x, current_y, x, y;
  carmen_bresenham_param_t b_params;
  int i;
  carmen_offlimits_t cur_seg;

  /* This function requires a map to markup */
  if(offlimits_list == NULL || list_length == 0 ||
     map == NULL || map->complete_map == NULL)
    return -1;

  for(i = 0; i < list_length; i++) 
    {
      cur_seg = offlimits_list[i];
      switch(cur_seg.type) 
	{
	case CARMEN_OFFLIMITS_POINT_ID:
	  if(cur_seg.x1 >= 0 && cur_seg.y1 >= 0 && 
	     cur_seg.x1 < map->config.x_size && 
	     cur_seg.y1 < map->config.y_size)
	    map->map[cur_seg.x1][cur_seg.y1] += 2.0;
	  break;
	  
	case CARMEN_OFFLIMITS_LINE_ID:
	  carmen_get_bresenham_parameters(cur_seg.x1, cur_seg.y1, cur_seg.x2, 
					  cur_seg.y2, &b_params);
	  do 
	    {
	      carmen_get_current_point(&b_params, &current_x, &current_y);
	      if(current_x >= 0 && current_y >= 0 &&
		 current_x < map->config.x_size && 
		 current_y < map->config.y_size)
		map->map[current_x][current_y] += 2.0;
	    } 
	  while(carmen_get_next_point(&b_params));
	  break;
	case CARMEN_OFFLIMITS_RECT_ID:
	  for(x = cur_seg.x1; x <= cur_seg.x2; x++)
	    for(y = cur_seg.y1; y <= cur_seg.y2; y++)
	      if(x >= 0 && y >= 0 && 
		 x < map->config.x_size && y < map->config.y_size)
		map->map[x][y] += 2.0;
	  break;
	default:
	  break;
	}
    }
  return 0;
}

carmen_inline void
carmen_map_destroy(carmen_map_p *map)
{
  free((*map)->complete_map);
  free((*map)->map);
  free((*map));
  *map = NULL;
}

