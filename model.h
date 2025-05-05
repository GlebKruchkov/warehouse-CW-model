#ifndef _model_h
#define _model_h

#include "ross.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <dirent.h>
#include "sqlite3.h"

#define MAX_BOXES 8
#define MAX_ROBOTS 15
#define MAX_ROBOTS_ON_VERTEX 2 // value should be from ceil((MAX_ROBOTS + 1) / 12) to MAX_ROBOTS
#define MAX_QUEUE_SIZE MAX_ROBOTS
#define MAX_CONVEYORS 150
#define MAX_FILES 14
#define MAX_RACKS 5
#define MAX_VERTEXES 49
#define CNT_OF_SKU 50
#define CONST_THRESHOLD 17

static const int low_border = 1;
static const int high_border = 51;

static int is_reverse = 0;
static int control_id = 1;
static int cur_boxes = 0;
static int palet_type = 1;
static int rec_id = 1;
static int reverse_count = 0;

FILE *file;
FILE *f;
FILE *sku_mileage;
FILE *f_dep;
FILE *temp_txt;
FILE *bots_starting_positions;
FILE *paleta;
FILE *csv_file;

FILE *test;
FILE *robots_positions;
FILE *control_system_log;

const static double g_robot_calc_time = 0.001;
static const int threshold = (int)((MAX_BOXES * MAX_CONVEYORS) / (high_border - low_border));

static int rev_quantity = 0;

typedef struct
{
    bool reserved;
    bool empty;
    int SKU;
    int width;
} box;

struct _best_box
{
    int row;
    int column;
};

typedef enum
{
    TAKE_IN,
    TAKE_OUT,
    DEFAULT
} message_type;

typedef struct
{
    message_type type;
    double contents;
    tw_lpid sender;
} message;

struct _conveyor
{
    int max_length;
    int current_length;
    box boxes[MAX_BOXES];
    int box_quantity;
};

typedef struct
{
    int width;
    int SKU;
} box_pair;

typedef struct
{
    int id;
    bool reserved;
    int queue[MAX_ROBOTS];
} cell;

typedef struct {
    double x;
    double y;
} vertex;

typedef struct {
    int num_vertices;
    vertex* vertices;
    double** adj_matrix;
    int** vertex_queues;
    int* queue_sizes;
} Graph;

typedef struct
{

    message_type cur_task;

    int cur_box;

    int box_to_add;

    int id;
    vertex cur_pos;
    double speed; // m/s
    int* path;
    int path_length;
    int prev_waypoint;
    int current_waypoint;
    int goal_waypoint;
    bool reached; // robot reached finish_pos
    bool waiting; // robot is waiting in queue

    int current_waypoint_on_path;

    int row;
    int col;
    bool has_box;
    int reserved_channel;
    int low_SKU;

    cell cur_cell;
    cell goal_cell;
    cell temp_reverse_goal_cell;
    int temp_reverse_SKU;

    bool is_free;

    bool started;
} robot;

typedef struct {
    int requests[3000][2];
    int total;
    int curr;
    int req_num;
} file_requests;

struct _Store
{
    int dist_without_SKU;
    int SKU_mileage[CNT_OF_SKU];
    bool first_paletize;
    int cnt;
    int added_cnt;
    int cur_robot;

    Graph* graph;

    file_requests request;
    sqlite3 *db;

    message messages[MAX_ROBOTS];

    int max_boxes_for_each_sku[CNT_OF_SKU];
    int thrs_for_each_sku[CNT_OF_SKU];
    bool sku_is_adding[CNT_OF_SKU];

    int distribution[50];

    int vertex_robots[MAX_VERTEXES];

    char cur_order[50];

    char files[15][1024];
    int cur_file;

    char vertexes[MAX_VERTEXES + 1][5];
    int box_data[MAX_ROBOTS + 1][2];
    int kill_prog;

    robot robots[MAX_ROBOTS];
    cell cells[MAX_VERTEXES];
    
    int b_w[50];
    box_pair box_width[50];
    int conveyor_width[MAX_CONVEYORS];

    int top_sku_in_channel[MAX_CONVEYORS];

    int cnt_boxes_type[50];
    int cnt_boxes_type_const[50];
    struct _conveyor conveyor[MAX_CONVEYORS];

    int glb_time;
    int event_id;
};

struct _Store Store;
struct _best_box best_box;

typedef struct {
  int got_msgs_TAKE_IN;
  int got_msgs_TAKE_OUT;
  int got_msgs_REVERSE;
  double value;
} state;

extern unsigned int setting_1;

extern tw_lptype model_lps[];

extern double calculate_distance(vertex a, vertex b);
extern bool CheckCurBoxInNextOrders(int SKU, int cur_robot);
extern void move_robot(robot* r, double time);
extern bool can_move(robot* r);
extern int find_lower_data_by_width_different_type(sqlite3 **db1, int type);
extern void set_robot_path(robot* r, int* path, int path_length);
extern int* dijkstra(int start, int end, int* path_length, int cur_robot);

extern void model_init(state *s, tw_lp *lp);
extern void model_event(state *s, tw_bf *bf, message *in_msg, tw_lp *lp);
extern void model_event_reverse(state *s, tw_bf *bf, message *in_msg, tw_lp *lp);
extern void model_final(state *s, tw_lp *lp);

extern int callback(void *NotUsed, int argc, char **argv, char **azColName);
extern int insert_data(sqlite3 **db1, int type, int row, int col, int width, int box_quantity);
extern int find_data(sqlite3 **db1, int type);
extern int find_data_by_width(sqlite3 **db1, int type);
extern int Add_Box(sqlite3 **db1, int type, int process);
extern void Swap_Boxes(sqlite3 **db1, int row, int col1, int col2);
extern int Reverse(sqlite3 **db1, int row, int col, int *l_id, int process);
extern int Remove_Boxes(sqlite3 **db, int type, int *l_id, int process);
extern void Init_Commands(int *rec_id, const char *filename);
extern bool Check(int process);
extern int compare(const void *a, const void *b);
extern void Send_Event(int process, message_type command, tw_lp *lp, tw_lpid *self);
extern void Print_Channel(int col, FILE *log_file);
extern void write_csv(const char *filename, sqlite3 *db);
extern void add_to_queue(int robot_id, int next_vert);
extern void del_from_queue(int robot_id);
extern int next_vertex(int cur_vertex, int cur_goal);
extern int GeTrIdFromBot(int curr_cell);
#endif
