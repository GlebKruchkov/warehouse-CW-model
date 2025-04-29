//The C driver file for a ROSS model
//This file includes:
// - an initialization function for each LP type
// - a forward event function for each LP type
// - a reverse event function for each LP type
// - a finalization function for each LP type

//Includes

#include "model.h"
#include <time.h>
#include "sqlite3.h"

//Helper Functions
void SWAP (double *a, double *b) {
  double tmp = *a;
  *a = *b;
  *b = tmp;
}

void model_init (state *s, tw_lp *lp) {
  int self = lp->gid;
  tw_event *e = tw_event_new(0, 1, lp);
  message *msg = tw_event_data(e);
  msg->type = TAKE_OUT;
  msg->contents = tw_rand_unif(lp->rng);
  msg->sender = self;
  tw_event_send(e);
  if (self == 0) {
    printf("%s\n", "COMMAND_CENTER is initialized");
  } else {
    printf("%s ", "CONVEYOR");
    printf("%d ", self);
    printf("%s\n", " is initialized");
  }
}

void model_event (state *s, tw_bf *bf, message *in_msg, tw_lp *lp) {
  int self = lp->gid;
  *(int *) bf = (int) 0;
  SWAP(&(s->value), &(in_msg->contents));
  if (Store.cnt < MAX_ROBOTS) {
    ++Store.cnt;
    return;
  }
  if (self == 0) { // we are at the main process (head)
    if (Store.cur_robot == 0) {
      glb_time += 1; // one tick done
    }
    int cur_robot = Store.cur_robot;
    if (Store.robots[cur_robot].is_free == true) { // if the robot can take the new task
      bool set = false; // if next task was set
      for (int i = 0; i < CNT_OF_SKU ; ++i) {
        if (
          Store.cnt_boxes_type[i] < Store.thrs_for_each_sku[i] || // if SKU is lower than threshold
          (Store.sku_is_adding[i] == true && Store.cnt_boxes_type[i] < Store.max_boxes_for_each_sku[i])) { // if we started to add box of type "I" we need to add it till the border
        // if (Store.cnt_boxes_type[i] <= CONST_THRESHOLD) { // uncomment this statement and comment upper if you want constant threshold
          if (Store.sku_is_adding[i] == false) { // if we just started adding
            fprintf(f, "%*d %*d     startDepalletize %d\n", 6, event_id, 6, glb_time, i + 1);
            event_id += 1;
            Store.sku_is_adding[i] = true;
          }

          Store.cnt_boxes_type[i] += 1;
          int path_length;
          int* path = dijkstra(Store.robots[cur_robot].current_waypoint, 46 + (int)(rand() % 2), &path_length, self - 1);

          if (path_length <= 0) { // in case we can't find the route
            exit(0);
          }

          set_robot_path(&Store.robots[cur_robot], path, path_length);
          Store.robots[cur_robot].box_to_add = i;

          Store.robots[cur_robot].cur_task = TAKE_IN;
          Send_Event(cur_robot + 1, Store.robots[cur_robot].cur_task, lp, &(lp->gid));
          set = true;
          Store.robots[cur_robot].is_free = false; // robot has the task
          break;
        } else {
          if (Store.sku_is_adding[i] == true) {
            fprintf(f, "%*d %*d    finishDepalletize %d\n", 6, event_id, 6, glb_time, i + 1);
            event_id += 1;
            Store.sku_is_adding[i] = false;
          }
        }
      }

      if (!set) { // if robot still doesn't have the task
        if (Check(cur_robot + 1)) { // if we have something to deliver
          find_data(&(Store.db), Store.box_data[cur_robot + 1][0]); // find in the warehouse free box of needed type

          Store.robots[cur_robot].col = best_box.column;
          Store.robots[cur_robot].row = best_box.row;

          int path_length;
          int* path = dijkstra(Store.robots[cur_robot].current_waypoint, (int)(best_box.column / 10) + 7 + (int)(best_box.column / 50) * 7, &path_length, self - 1);

          if (path_length <= 0) { // in case we can't find the route
            exit(0);
          }

          Store.robots[cur_robot].reserved_channel = best_box.column; // other robots can't use this column
          set_robot_path(&Store.robots[cur_robot], path, path_length);

          Store.robots[cur_robot].cur_task = TAKE_OUT;
          Send_Event(cur_robot + 1, Store.robots[cur_robot].cur_task, lp, &(lp->gid));
          set = true;
          Store.robots[cur_robot].is_free = false; // robot has the task
        } else {
          if (Store.cur_file > MAX_FILES) {
            Store.kill_prog = true;
          } else {
            if (Store.cur_file != 0) {
              rec_id++;
              fprintf(f, "%*d %*d      finishPalletize %s", 6, event_id, 6, glb_time, Store.cur_order);
              event_id += 1;
            }
            Init_Commands(&(event_id), &(rec_id), &(glb_time), Store.files[Store.cur_file]);
            Store.cur_file += 1;
          }
        }
      }

      if (!set && !Store.kill_prog) { // if robot still doesn't have the task
        Send_Event(0, TAKE_IN, lp, &(lp->gid));
      }

    } else { // robot need to finish task
      Send_Event(cur_robot + 1, Store.robots[cur_robot].cur_task, lp, &(lp->gid));
    }

  } else {

    switch (in_msg->type)
    {
      case TAKE_IN:
        if (Store.robots[self - 1].has_box == false && Store.robots[self - 1].reached) { // if robot is ready to take the box
          best_box.row = -1;
          best_box.column = -1;
          // get_best_box_without_sql_random();
          // find_lower_data_by_width(&(Store.db), Store.robots[self - 1].box_to_add);
          // get_best_box_without_sql();
          // find_data_without_SKU_in_col(&(Store.db), Store.robots[self - 1].box_to_add);
          find_lower_data_by_width_different_type(&(Store.db), Store.robots[self - 1].box_to_add);

          if (best_box.row == -1 || best_box.column == -1) { // we did not find where to put the box (there is no free space or channels)
            Store.robots[self - 1].is_free = true; // robot is free
            Store.cur_robot = (Store.cur_robot + 1) % MAX_ROBOTS;
            Store.cnt_boxes_type[Store.robots[self - 1].box_to_add] -= 1;
            Send_Event(0, TAKE_IN, lp, &(lp->gid));
            break;
          }

          Store.robots[self - 1].col = best_box.column; // column where we need to put the box
          Store.robots[self - 1].row = best_box.row; // row where we need to put the box

          fprintf(
            f,
            "%*d %*d %*d     movebox2bot       %*s     %*s     %*d    %*d   %*d\n",
            6, event_id,
            6, glb_time,
            4, self,
            4, Store.vertexes[Store.robots[self - 1].current_waypoint],
            4, Store.vertexes[Store.robots[self - 1].goal_waypoint],
            4, Store.robots[self - 1].box_to_add + 1,
            4, 0,
            2, 0);
          event_id += 1;

          int path_length;
          int* path = dijkstra(
            Store.robots[self - 1].current_waypoint,
            (((Store.robots[self - 1].col / 50) + 1) * 12) + (4 - ((Store.robots[self - 1].col - 50 * (Store.robots[self - 1].col / 50)) / 10)) + 1,
            &path_length,
            self - 1);

          if (path_length <= 0) { // in case we can't find the route
            exit(0);
          }

          set_robot_path(&Store.robots[self - 1], path, path_length);

          Store.robots[self - 1].reserved_channel = best_box.column; // other robots can't use this column

          Store.robots[self - 1].has_box = true; // robot take the box
          Store.robots[self - 1].cur_box = Store.robots[self - 1].box_to_add; // box the robot took
          
        } else if (Store.robots[self - 1].has_box == true && Store.robots[self - 1].reached) { // if robot is ready to put the box
          fprintf(
            f,
            "%*d %*d %*d movebox2channel       %*s     %*s     %*d    %*d   %*d\n",
            6, event_id,
            6, glb_time,
            4, self,
            4, Store.vertexes[Store.robots[self - 1].current_waypoint],
            4, Store.vertexes[Store.robots[self - 1].goal_waypoint],
            4, Store.robots[self - 1].box_to_add + 1,
            4, Store.robots[self - 1].col,
            2, 0);
          event_id += 1;

          Add_Box(&(Store.db), Store.robots[self - 1].box_to_add, self);

          Store.robots[self - 1].reserved_channel = -1; // other robots can use this column
          Store.robots[self - 1].has_box = false; // robot put the box
          Store.robots[self - 1].col = -1;
          Store.robots[self - 1].row = -1;
          Store.robots[self - 1].is_free = true; // robot is ready to take the next task

        } else { // robot continue moving
          if (can_move(&Store.robots[self - 1])) { // if robot is first in queue
            move_robot(&Store.robots[self - 1], 1);
          }

        }
        
        Store.cur_robot = (Store.cur_robot + 1) % MAX_ROBOTS;
        Send_Event(0, TAKE_IN, lp, &(lp->gid));
        break;

      case TAKE_OUT:

        if (Store.robots[self - 1].has_box == false && Store.robots[self - 1].reached) { // if robot is ready to take the box

          if (Store.robots[self - 1].row != 7) { // if robot can't take the box (it is not at the end of the channel)

            Store.robots[self - 1].low_SKU = Store.conveyor[Store.robots[self - 1].col].boxes[7].SKU;

            fprintf(
              f,
              "%*d %*d %*d     movebox2bot       %*s     %*s     %*d    %*d   %*d\n",
              6, event_id,
              6, glb_time,
              4, self,
              4, Store.vertexes[Store.robots[self - 1].current_waypoint],
              4, Store.vertexes[Store.robots[self - 1].goal_waypoint],
              4, Store.robots[self - 1].low_SKU + 1,
              4, 0,
              2, 0);
            event_id += 1;

            Remove_Boxes(&(Store.db), Store.robots[self - 1].low_SKU, &(glb_time), &(event_id), self);

            Store.robots[self - 1].has_box = true;
            Store.robots[self - 1].cur_box = Store.robots[self - 1].low_SKU; // box the robot took

            CheckCurBoxInNextOrders(Store.robots[self - 1].low_SKU, self - 1); // we check if we have the box in the next 50 orders

            int cur_goal_cell_id;
            if (Store.robots[self - 1].temp_reverse_goal_cell.id != -1) {
              // printf("REVERSE UTILIZATION\n");
              cur_goal_cell_id = Store.robots[self - 1].temp_reverse_goal_cell.id; // if we have the box in the next 50 orders, robot can move it to palletize zone
            } else {
              cur_goal_cell_id = (((Store.robots[self - 1].col / 50) + 1) * 12) + (4 - ((Store.robots[self - 1].col - 50 * (Store.robots[self - 1].col / 50)) / 10)) + 1;
            }

            int path_length;
            int* path = dijkstra(
              Store.robots[self - 1].current_waypoint,
              cur_goal_cell_id,
              &path_length,
              self - 1);

            if (path_length <= 0) { // in case we can't find the route
              exit(0);
            }
            set_robot_path(&Store.robots[self - 1], path, path_length);
          } else { // robot can take the box
            fprintf(
              f,
              "%*d %*d %*d     movebox2bot       %*s     %*s     %*d    %*d   %*d\n",
              6, event_id,
              6, glb_time,
              4, self, 
              4, Store.vertexes[Store.robots[self - 1].current_waypoint], 
              4, Store.vertexes[Store.robots[self - 1].goal_waypoint], 
              4, Store.box_data[self][0] + 1, 
              4, 0, 
              2, 0);
            event_id += 1;
            Remove_Boxes(&(Store.db), Store.box_data[self][0], &(glb_time), &(event_id), self);

            Store.robots[self - 1].has_box = true; // robot take the box
            Store.robots[self - 1].cur_box = Store.box_data[self][0]; // box the robot took

            Store.robots[self - 1].reserved_channel = -1; // other robots can use this column

            Store.cnt_boxes_type[Store.box_data[self][0]] -= 1;

            int path_length;
            int* path = dijkstra(
              Store.robots[self - 1].current_waypoint,
              (int)(rand() % 3),
              &path_length,
              self - 1);

            if (path_length <= 0) { // in case we can't find the route
              exit(0);
            }

            set_robot_path(&Store.robots[self - 1], path, path_length);

          }

        } else if (Store.robots[self - 1].has_box == true && Store.robots[self - 1].reached) { // robot is ready to put the box to pallet

          if (Store.robots[self - 1].row != 7 && Store.robots[self - 1].temp_reverse_goal_cell.id == -1) { // if robot need to finish reverse
            rev_quantity += 1;
            fprintf(
              f, 
              "%*d %*d %*d movebox2channel       %*s     %*s     %*d    %*d   %*d\n", 
              6, event_id, 
              6, glb_time, 
              4, self, 
              4, Store.vertexes[Store.robots[self - 1].current_waypoint], 
              4, Store.vertexes[Store.robots[self - 1].goal_waypoint], 
              4, Store.robots[self - 1].low_SKU + 1,
              4, Store.robots[self - 1].col, 
              2, 0);
            event_id += 1;

            int row_to_add = 0;
            while (Store.conveyor[Store.robots[self - 1].col].boxes[row_to_add].SKU == -1) {
              row_to_add += 1;
            }
            row_to_add -= 1;

            int save_row = Store.robots[self - 1].row;

            Store.robots[self - 1].row = row_to_add;

            Add_Box(&(Store.db), Store.robots[self - 1].low_SKU, self);

            int path_length;
            int* path = dijkstra(
              Store.robots[self - 1].current_waypoint,
              (int)(Store.robots[self - 1].col / 10) + 7 + (int)(Store.robots[self - 1].col / 50) * 7,
              &path_length,
              self - 1);

            if (path_length <= 0) { // in case we can't find the route
              exit(0);
            }

            set_robot_path(&Store.robots[self - 1], path, path_length);

            Store.robots[self - 1].has_box = false; // robot put the box to channel
            Store.robots[self - 1].row = save_row + 1;

          } else if (Store.robots[self - 1].row != 7 && Store.robots[self - 1].temp_reverse_goal_cell.id != -1) {
            // printf("REVERSE UTILIZATION\n");
            fprintf(
              f,
              "%*d %*d %*d      movebox2tr       %*s     %*s     %*d    %*d   %*d\n",
              6, event_id,
              6, glb_time,
              4, self,
              4, Store.vertexes[Store.robots[self - 1].current_waypoint],
              4, Store.vertexes[Store.robots[self - 1].goal_waypoint],
              4, Store.robots[self - 1].temp_reverse_SKU + 1,
              4, 0,
              2, 0);
            event_id += 1;
            Store.cnt_boxes_type[Store.robots[self - 1].temp_reverse_SKU] -= 1;

            Store.robots[self - 1].temp_reverse_SKU = -1;
            Store.robots[self - 1].temp_reverse_goal_cell.id = -1;

            int path_length;
            int* path = dijkstra(
              Store.robots[self - 1].current_waypoint,
              (int)(Store.robots[self - 1].col / 10) + 7 + (int)(Store.robots[self - 1].col / 50) * 7,
              &path_length,
              self - 1);

            if (path_length <= 0) { // in case we can't find the route
              exit(0);
            }

            set_robot_path(&Store.robots[self - 1], path, path_length);

            Store.robots[self - 1].has_box = false; // robot put the box to palletizer
            Store.robots[self - 1].row += 1;
          } else { // robot put the box to pallet
            fprintf(
              f,
              "%*d %*d %*d      movebox2tr       %*s     %*s     %*d    %*d   %*d\n",
              6, event_id,
              6, glb_time,
              4, self,
              4, Store.vertexes[Store.robots[self - 1].current_waypoint],
              4, Store.vertexes[Store.robots[self - 1].goal_waypoint],
              4, Store.box_data[self][0] + 1,
              4, 0,
              2, 0);
            event_id += 1;

            Store.robots[self - 1].has_box = false; // robot put the box to channel
            Store.robots[self - 1].row = -1;
            Store.robots[self - 1].col = -1;
            Store.robots[self - 1].is_free = true; // robot is ready to take the next task
          }
        } else { // robot continue moving

          if (can_move(&Store.robots[self - 1])) { // if robot is first in queue
            move_robot(&Store.robots[self - 1], 1);
          }

        }
        
        Store.cur_robot = (Store.cur_robot + 1) % MAX_ROBOTS;
        Send_Event(0, TAKE_IN, lp, &(lp->gid));
        break;

      default:
        printf("\n%s\n", "No message");
        break;
    }
  }
}

//Reverse Event Handler
void model_event_reverse (state *s, tw_bf *bf, message *in_msg, tw_lp *lp) {
  return;
}

//report any final statistics for this LP
void model_final (state *s, tw_lp *lp) {
  if (lp->gid == 0) {
    printf("reverse quatity: %d", rev_quantity);
    rec_id++;
    fprintf(f, "%*d %*d      finishPalletize %s", 6, event_id, 6, glb_time, Store.cur_order);
    int dist_with_sku = 0;
    for (int sku = 0; sku < CNT_OF_SKU; ++sku) {
      fprintf(sku_mileage, "%*d %*d\n", 3, sku + 1, 13, Store.SKU_mileage[sku]);
      dist_with_sku += Store.SKU_mileage[sku];
    }
    fprintf(sku_mileage, "%*s %*d\n", 8, "with_box", 10, dist_with_sku);
    fprintf(sku_mileage, "%*s %*d\n", 5, "empty", 13, Store.dist_without_SKU);
    fprintf(sku_mileage, "%d\n", rev_quantity);
  }
  return;
}
