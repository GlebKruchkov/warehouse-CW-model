#include "model.h"

double calculate_distance(vertex a, vertex b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

int* dijkstra(int start, int end, int* path_length, int cur_robot) {
  int num_vertices = Store.graph->num_vertices;
  double distances[num_vertices];
  bool visited[num_vertices];
  int previous[num_vertices];
  
  for (int i = 0; i < num_vertices; i++) {
      distances[i] = INT_MAX;
      visited[i] = false;
      previous[i] = -1;
  }
  distances[start] = 0;
  
  for (int count = 0; count < num_vertices - 1; count++) {
      int min_index = -1;
      double min_dist = INT_MAX;
      for (int v = 0; v < num_vertices; v++) {
          if (!visited[v] && distances[v] < min_dist) {
              min_dist = distances[v];
              min_index = v;
          }
      }
      
      if (min_index == -1) break;
      
      visited[min_index] = true;
      
      for (int v = 0; v < num_vertices; v++) {
          if (!visited[v] && Store.graph->adj_matrix[min_index][v] > 0 && 
              distances[min_index] + Store.graph->adj_matrix[min_index][v] < distances[v]) {
              distances[v] = distances[min_index] + Store.graph->adj_matrix[min_index][v];
              previous[v] = min_index;
          }
      }
  }
  
  if (distances[end] == INT_MAX) {
      *path_length = 0;
      return NULL;
  }
  
  int path[num_vertices], temp_path_length = 0;
  for (int at = end; at != -1; at = previous[at]) {
      path[temp_path_length++] = at;
  }
  
  int* result_path = (int*)malloc(temp_path_length * sizeof(int));
  for (int i = 0; i < temp_path_length; i++) {
      result_path[i] = path[temp_path_length - 1 - i];
  }
  
  *path_length = temp_path_length;

  return result_path;
}

void set_robot_path(robot* r, int* path, int path_length) {
  if (r->path) free(r->path);
  r->path = (int*)malloc(path_length * sizeof(int));
  for (int i = 0; i < path_length; i++) {
      r->path[i] = path[i];
  }
  r->path_length = path_length;
  Store.robots[r->id].goal_waypoint = r->path[1];
  r->current_waypoint_on_path = 1;
  r->reached = false;
}

bool enqueue_robot(int vertex_idx, int robot_id) {
  if (Store.graph->queue_sizes[vertex_idx] >= MAX_QUEUE_SIZE) {
      return false;
  }
  
  Store.graph->vertex_queues[vertex_idx][Store.graph->queue_sizes[vertex_idx]++] = robot_id;
  return true;
}

int dequeue_robot(int vertex_idx) {
  if (Store.graph->queue_sizes[vertex_idx] == 0) {
      return -1;
  }
  
  int robot_id = Store.graph->vertex_queues[vertex_idx][0];
  
  for (int i = 0; i < Store.graph->queue_sizes[vertex_idx] - 1; i++) {
    Store.graph->vertex_queues[vertex_idx][i] = Store.graph->vertex_queues[vertex_idx][i+1];
  }
  
  Store.graph->queue_sizes[vertex_idx]--;
  Store.graph->vertex_queues[vertex_idx][Store.graph->queue_sizes[vertex_idx]] = -1;
  
  return robot_id;
}

bool is_first_in_queue( int vertex_idx, int robot_id) {
  if (Store.graph->queue_sizes[vertex_idx] == 0) {
      return false;
  }
  return Store.graph->vertex_queues[vertex_idx][0] == robot_id;
}

bool can_move(robot* r) {
  if (r->reached) {
      return false;
  }
  
  int current_vertex = r->current_waypoint;
  int next_vertex = r->path[r->current_waypoint_on_path];
  
  if (!r->waiting) {
      if (!enqueue_robot(next_vertex, r->id)) {
          return false;
      }
      r->waiting = true;
  }
  
  return is_first_in_queue(next_vertex, r->id);
}

void move_robot(robot* r, double time) {
  if (r->reached) {
      return;
  }

  int next_vertex = r->path[r->current_waypoint_on_path];

  vertex end = Store.graph->vertices[next_vertex];
  
  double remaining_distance = calculate_distance(r->cur_pos, end);
  double distance_to_move = r->speed * time;

  double dx = end.x - r->cur_pos.x;
  double dy = end.y - r->cur_pos.y;
  dx /= remaining_distance;
  dy /= remaining_distance;

  int box_type_for_log = 0;
  if (r->has_box) {
    box_type_for_log = r->cur_box + 1;
  }

  if (distance_to_move >= remaining_distance) {
      if (r->has_box) { // if robot with box
        Store.SKU_mileage[r->cur_box] += remaining_distance; // increase mileage of the SKU
      } else { // if robot without box
        Store.dist_without_SKU += remaining_distance;
      }
      if (Store.vertex_robots[next_vertex] >= MAX_ROBOTS_ON_VERTEX) { // if the next vertex is full of robots
        return;
      }
      Store.vertex_robots[r->current_waypoint] -= 1;
      Store.vertex_robots[next_vertex] += 1;
      Store.robots[r->id].goal_waypoint = next_vertex;

      fprintf(
        f,
        "%*d %*d %*d    finishMotion       %*s     %*s     %*d    %*d   %*d\n",
        6, event_id,
        6, glb_time,
        4, r->id + 1,
        4, Store.vertexes[r->current_waypoint],
        4, Store.vertexes[r->path[r->current_waypoint_on_path]],
        4, box_type_for_log,
        4, 0,
        2, 0);
      event_id += 1;
      r->started = false;
      r->cur_pos = end;
      r->current_waypoint = r->path[r->current_waypoint_on_path];
      r->current_waypoint_on_path++;
      r->waiting = false;
      dequeue_robot(next_vertex);
      
      if (r->current_waypoint_on_path >= r->path_length) {
          r->reached = true;
      } else {
          int new_next_vertex = r->path[r->current_waypoint_on_path];
          if (enqueue_robot(new_next_vertex, r->id)) {
              r->waiting = true;
          }
      }
  } else {
      if (!r->started) {
        fprintf(
          f,
          "%*d %*d %*d     startMotion       %*s     %*s     %*d    %*d   %*d\n",
          6, event_id,
          6, glb_time,
          4, r->id + 1,
          4, Store.vertexes[r->current_waypoint],
          4, Store.vertexes[r->path[r->current_waypoint_on_path]],
          4, box_type_for_log,
          4, 0,
          2, 0);
        event_id += 1;
        r->started = true;
      }
      r->cur_pos.x += (dx * distance_to_move);
      r->cur_pos.y += (dy * distance_to_move);
      if (r->has_box) { // if robot with box
        Store.SKU_mileage[r->cur_box] += distance_to_move; // increase mileage of the SKU
      } else { // if robot without box
        Store.dist_without_SKU += distance_to_move;
      }
  }
}

int callback(void *NotUsed, int argc, char **argv, char **azColName) {
    if (atoi(argv[1]) > best_box.row) {
        int flag = 1;
        for (int i = 0; i < MAX_ROBOTS; ++i) {
            if (Store.robots[i].reserved_channel == atoi(argv[2])) {
                flag = 0;
            }
        }
        if (flag == 1) {
            best_box.row = atoi(argv[1]);
            best_box.column = atoi(argv[2]);
        }
    }
    return 0;
}

int find_data(sqlite3 **db1, int type) {
    struct sqlite3 * db = (struct sqlite3 *) *db1;
    best_box.row = -1;
    best_box.column = -1;
    char *err_msg = 0;
    char sql[100];
    sprintf(sql, "SELECT * FROM Warehouse WHERE Type = %d", type);
    sqlite3_exec(db, sql, callback, 0, &err_msg);
    return 0;
}

int lower_callback_by_width(void *NotUsed, int argc, char **argv, char **azColName) {
  if (atoi(argv[1]) > best_box.row) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  return 0;
}

int lower_callback_by_width_same_type(void *user_data, int argc, char **argv, char **azColName) {
  int type = *(int*)user_data;
  if (atoi(argv[1]) > best_box.row) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  if (atoi(argv[1]) == best_box.row && Store.top_sku_in_channel[best_box.column] == type) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  return 0;
}

int lower_callback_by_width_different_type(void *user_data, int argc, char **argv, char **azColName) {
  int type = *(int*)user_data;
  if (atoi(argv[1]) > best_box.row) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  if (atoi(argv[1]) == best_box.row && Store.top_sku_in_channel[best_box.column] != type) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  return 0;
}

int callback_by_width_SKU_in_col(void *user_data, int argc, char **argv, char **azColName) {
  int type = *(int*)user_data;
  if (atoi(argv[1]) > best_box.row) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  if (atoi(argv[1]) == best_box.row) {
    int flag = 1;
    for (int i = 0; i < MAX_ROBOTS; ++i) {
      if (Store.robots[i].reserved_channel == atoi(argv[2])) {
          flag = 0;
      }
    }
    for (int box = 0; box < MAX_BOXES; ++box) {
      if (Store.conveyor[atoi(argv[2])].boxes[box].SKU == type) {
        flag = 0;
      }
    }
    if (flag == 1) {
      best_box.row = atoi(argv[1]);
      best_box.column = atoi(argv[2]);
    }
  }
  return 0;
}

int find_data_without_SKU_in_col(sqlite3 **db1, int type) {
  struct sqlite3 * db = (struct sqlite3 *) *db1;
  char *err_msg = 0;
  char sql[100];
  sprintf(sql, "SELECT * FROM Warehouse WHERE Type = %d", -1);
  sqlite3_exec(db, sql, callback_by_width_SKU_in_col, &type, &err_msg);
  return 0;
}

int find_lower_data_by_width_different_type(sqlite3 **db1, int type) {
  struct sqlite3 * db = (struct sqlite3 *) *db1;
  char *err_msg = 0;
  char sql[100];
  sprintf(sql, "SELECT * FROM Warehouse WHERE Channel_Width >= %d AND Type = %d", -1, -1);
  sqlite3_exec(db, sql, lower_callback_by_width_different_type, &type, &err_msg);
  return 0;
}

int find_lower_data_by_width_same_type(sqlite3 **db1, int type) {
  struct sqlite3 * db = (struct sqlite3 *) *db1;
  char *err_msg = 0;
  char sql[100];
  sprintf(sql, "SELECT * FROM Warehouse WHERE Channel_Width >= %d AND Type = %d", -1, -1);
  sqlite3_exec(db, sql, lower_callback_by_width_same_type, &type, &err_msg);
  return 0;
}

int find_lower_data_by_width(sqlite3 **db1, int type) {
    struct sqlite3 * db = (struct sqlite3 *) *db1;
    char *err_msg = 0;
    char sql[100];
    sprintf(sql, "SELECT * FROM Warehouse WHERE Channel_Width >= %d AND Type = %d", -1, -1);
    sqlite3_exec(db, sql, lower_callback_by_width, 0, &err_msg);
    return 0;
}

void get_best_box_without_sql() {
  int col = -1;
  int row = -1;
  int max_box_quantity = -1;
  for (int i = 0; i < MAX_CONVEYORS; ++i) {
    int flag = 0;
    for (int j = 0; j < MAX_ROBOTS; ++j) {
      if (Store.robots[j].reserved_channel == i) {
        flag = 1;
        break;
      }
    }
    if (flag == 1) {
        continue;
    }
    if (Store.conveyor[i].box_quantity > max_box_quantity && Store.conveyor[i].box_quantity < MAX_BOXES) {
      max_box_quantity = Store.conveyor[i].box_quantity;
      col = i;
      row = MAX_BOXES - 1 - Store.conveyor[i].box_quantity;
    }

  }
  best_box.column = col;
  best_box.row = row;
}

void get_best_box_without_sql_random() {
  for (int i = 0; i < MAX_CONVEYORS; ++i) {
    int flag = 0;
    for (int j = 0; j < MAX_ROBOTS; ++j) {
      if (Store.robots[j].reserved_channel == i) {
        flag = 1;
        break;
      }
    }
    if (flag == 1) {
      continue;
    }
    if (Store.conveyor[i].box_quantity < 8) {
      best_box.column = i;
      best_box.row = 7 - Store.conveyor[i].box_quantity;
      return;
    }

  }
}

int Add_Box(sqlite3 **db1, int type, int process) {
    struct sqlite3 * db = (struct sqlite3 *) *db1;
    int col = Store.robots[process - 1].col;
    int r = Store.robots[process - 1].row;
    char *err_msg = 0;
    char sql[100];
    sprintf(sql, "DELETE FROM Warehouse WHERE Type = %d AND Row = %d AND Column = %d", -1, r, col);
    sqlite3_exec(db, sql, 0, 0, &err_msg);

    Store.conveyor[col].boxes[r].SKU = type;
    Store.conveyor[col].boxes[r].empty = 0;
    Store.conveyor[col].boxes[r].width = Store.b_w[type];
    Store.conveyor[col].box_quantity += 1;
    Store.top_sku_in_channel[col] = type;

    insert_data(db1, type, r, col, Store.b_w[type], Store.conveyor[col].box_quantity);
}

int insert_data(sqlite3 **db1, int type, int row, int col, int width, int box_quantity) {
  struct sqlite3 * db = (struct sqlite3 *) *db1;
  char *err_msg = 0;
  char sql[200];
  sprintf(sql, "INSERT INTO Warehouse(Type, Row, Column, Width, Channel_Width, Box_Quantity) VALUES (%d, %d, %d, %d, %d, %d)", type, row, col, width, Store.conveyor_width[col], box_quantity);
  sqlite3_exec(db, sql, 0, 0, &err_msg);
  return 0;
}

void Swap_Boxes(sqlite3 **db1, int col, int row1, int row2) {
    struct sqlite3 * db = (struct sqlite3 *) *db1;
    bool empty_tmp = Store.conveyor[col].boxes[row1].empty;
    int SKU_tmp = Store.conveyor[col].boxes[row1].SKU;
    int width_tmp =  Store.conveyor[col].boxes[row1].width;
    char *err_msg = 0;
    char sql[100];
    
    sprintf(sql, "DELETE FROM Warehouse WHERE Type = %d AND Row = %d AND Column = %d", Store.conveyor[col].boxes[row1].SKU, row1, col);
    sqlite3_exec(db, sql, 0, 0, &err_msg);
    char sql2[100];
    sprintf(sql2, "DELETE FROM Warehouse WHERE Type = %d AND Row = %d AND Column = %d", Store.conveyor[col].boxes[row2].SKU, row2, col);
    sqlite3_exec(db, sql2, 0, 0, &err_msg);
    Store.conveyor[col].boxes[row1].empty = Store.conveyor[col].boxes[row2].empty;
    Store.conveyor[col].boxes[row1].SKU = Store.conveyor[col].boxes[row2].SKU;
    Store.conveyor[col].boxes[row1].width = Store.conveyor[col].boxes[row2].width;
    Store.conveyor[col].boxes[row2].empty = empty_tmp;
    Store.conveyor[col].boxes[row2].SKU = SKU_tmp;
    Store.conveyor[col].boxes[row2].width = width_tmp;

    insert_data(db1, Store.conveyor[col].boxes[row2].SKU, row2, col, Store.conveyor[col].boxes[row2].width, Store.conveyor[col].box_quantity);
    insert_data(db1, Store.conveyor[col].boxes[row1].SKU, row1, col, Store.conveyor[col].boxes[row1].width, Store.conveyor[col].box_quantity);
}

int Remove_Boxes(sqlite3 **db1, int type, int *time, int *l_id, int process) {
    int col = Store.robots[process - 1].col;
    struct sqlite3 * db = (struct sqlite3 *) *db1;

    char *err_msg = 0;
    char sql[100];
    sprintf(sql, "DELETE FROM Warehouse WHERE Type = %d AND Row = %d AND Column = %d", type, MAX_BOXES - 1, col);
    sqlite3_exec(db, sql, 0, 0, &err_msg);

    Store.conveyor[col].box_quantity -= 1;

    insert_data(db1, -1, MAX_BOXES - 1, col, -1, 1);

    Store.conveyor[col].boxes[MAX_BOXES - 1].SKU = -1;
    Store.conveyor[col].boxes[MAX_BOXES - 1].empty = 1;
    Store.conveyor[col].boxes[MAX_BOXES - 1].width = -1;

    for (int i = MAX_BOXES - 1; i >= 1; --i) {
        Swap_Boxes(db1, col, i, i - 1);
    }

    return col;
}

bool CheckCurBoxInNextOrders(int SKU, int cur_robot) {
  for (int i = Store.request.curr; i < Store.request.curr + 50; ++i) {
    if (i == Store.request.total) {
      break;
    }
    if (Store.request.requests[i][0] == SKU + 1 && Store.request.requests[i][1] > 0) {
      Store.request.requests[i][1] = 0;
      Store.robots[cur_robot].temp_reverse_goal_cell.id = (int)(rand() % 3);
      Store.robots[cur_robot].temp_reverse_SKU = SKU;
      return true;
    }
  }
  return false; 
}

bool Check(int process) {
    if (Store.request.curr == Store.request.total) {
        return false;
    } else {
        int SKU = Store.request.requests[Store.request.curr][0];
        int quantity = Store.request.requests[Store.request.curr][1];
        if (quantity == 0) {
          Store.request.curr++;
          return Check(process);
        }
        Store.box_data[process][0] = SKU - 1;
        Store.request.curr++;
        return true;
    }
}

void Send_Event(int process, message_type command, tw_lp *lp, tw_lpid *self) {
    tw_event *e = tw_event_new(process, 0.001, lp);
    message *msg = tw_event_data(e);
    msg->type = command;
    msg->contents = tw_rand_unif(lp->rng);
    msg->sender = *self;
    tw_event_send(e);
}

int CellIdFromName(char* name) {
  int code = (int)(name[5] - 'A');
  if (code == 0) {
    return (int)(name[7] - '0') - 1;
  }
  if (code % 2 == 0) {
    return (int)(name[7] - '0') + 6 + (code - 1) * 6;
  }
  return 7 + code * 6 - (int)(name[7] - '0');
}

void Init_Commands(int *event_id, int *rec_id, int *glb_time, const char *filename) {
    FILE *file1 = fopen(filename, "r");
    Store.request.req_num = 0;
    Store.request.total = 0;
    Store.request.curr = 0;

    char line[1024];
    char *fields[10];

    fgets(line, sizeof(line), file1);

    char *temp_line = strtok(line, ",");

    strncpy(Store.cur_order, temp_line, sizeof(Store.cur_order) - 1);

    (*rec_id) += 1;

    fprintf(f, "%*d %*d       startPalletize %s", 6, *event_id, 6, *glb_time, strtok(line, ","));
    (*event_id) += 1;
    fgets(line, sizeof(line), file1);

    while (fgets(line, sizeof(line), file1)) {
        fields[0] = strtok(line, ",");
        for (int i = 1; i < 10; i++) {
          fields[i] = strtok(NULL, ",");
        }
        int SKU =  atoi(fields[0]) - 1000;
        int quantity = atoi(fields[1]);
        while (quantity != 0) {
          Store.request.requests[Store.request.req_num][0] = SKU;
          Store.request.requests[Store.request.req_num][1] = 1;
          quantity--;
          Store.request.total++;
          Store.request.req_num++;
        }
    }
    printf("%s %d\n", filename, Store.request.total);
}
