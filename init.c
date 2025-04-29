#include "model.h"

void init_robot(robot* r, int id, vertex start, vertex finish, double speed, char* robot_cur_cell) {
    r->id = id;
    r->cur_pos = start;
    r->finish_pos = finish;
    r->speed = speed;
    r->path = NULL;
    r->path_length = 0;
    r->prev_waypoint = 0;
    r->current_waypoint = 0;
    r->current_waypoint_on_path = 0;
    r->reached = false;
    r->waiting = false;
    r->tmp_fl = 1;
    r->cur_task = DEFAULT;
    r->cur_box = -1;
    r->pre_reserved = -1;
    r->low_SKU = -1;
    r->col = -1;
    r->row = -1;
    r->kill = 0;
    r->has_box = false;
    r->reserved_channel = -1;
    r->cur_time = 0;
    r->goal_time = 0;
    r->cur_cell = Store.cells[0];
    r->prev_vertex = -1;
    r->prev_box_type = -1;
    r->prev_channel = -1;
    r->prev_tr_id = -1;
    r->box_to_add = -1;
    r->is_free = true;
}

void add_edge(Graph* graph, int u, int v) {
    double distance = calculate_distance(graph->vertices[u], graph->vertices[v]);
    graph->adj_matrix[u][v] = distance; // one way ->
}

Graph* create_graph(int num_vertices) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    graph->num_vertices = num_vertices;
    graph->vertices = (vertex*)malloc(num_vertices * sizeof(vertex));
    graph->adj_matrix = (double**)malloc(num_vertices * sizeof(double*));
    graph->vertex_queues = (int**)malloc(num_vertices * sizeof(int*));
    graph->queue_sizes = (int*)calloc(num_vertices, sizeof(int));

    for (int i = 0; i < num_vertices; i++) {
        graph->adj_matrix[i] = (double*)malloc(num_vertices * sizeof(double));
        graph->vertex_queues[i] = (int*)malloc(MAX_QUEUE_SIZE * sizeof(int));
        memset(graph->vertex_queues[i], -1, MAX_QUEUE_SIZE * sizeof(int));
        
        for (int j = 0; j < num_vertices; j++) {
            graph->adj_matrix[i][j] = 0;
        }
    }

    return graph;
} 

void initialize_graph() {
    Store.graph = create_graph(MAX_VERTEXES);
    Store.graph->vertices[0] = (vertex){0, 0};    // A1
    Store.graph->vertices[1] = (vertex){2, 0};     // A2
    Store.graph->vertices[2] = (vertex){5, 0};     // A3
    Store.graph->vertices[3] = (vertex){10, 0};    // A4
    Store.graph->vertices[4] = (vertex){15, 0};    // A5
    Store.graph->vertices[5] = (vertex){20, 0};    // A6
    Store.graph->vertices[6] = (vertex){25, 0};    // A7

    Store.graph->vertices[12] = (vertex){0, 2};     // B1
    Store.graph->vertices[11] = (vertex){5, 2};     // B2
    Store.graph->vertices[10] = (vertex){10, 2};    // B3
    Store.graph->vertices[9] = (vertex){15, 2};   // B4
    Store.graph->vertices[8] = (vertex){20, 2};   // B5
    Store.graph->vertices[7] = (vertex){25, 2};   // B6

    Store.graph->vertices[13] = (vertex){0, 7};    // C1
    Store.graph->vertices[14] = (vertex){5, 7};    // C2
    Store.graph->vertices[15] = (vertex){10, 7};   // C3
    Store.graph->vertices[16] = (vertex){15, 7};   // C4
    Store.graph->vertices[17] = (vertex){20, 7};   // C5
    Store.graph->vertices[18] = (vertex){25, 7};   // C6

    Store.graph->vertices[24] = (vertex){0, 9};    // D1
    Store.graph->vertices[23] = (vertex){5, 9};    // D2
    Store.graph->vertices[22] = (vertex){10, 9};   // D3
    Store.graph->vertices[21] = (vertex){15, 9};   // D4
    Store.graph->vertices[20] = (vertex){20, 9};   // D5
    Store.graph->vertices[19] = (vertex){25, 9};   // D6

    Store.graph->vertices[25] = (vertex){0, 14};    // E1
    Store.graph->vertices[26] = (vertex){5, 14};    // E2
    Store.graph->vertices[27] = (vertex){10, 14};   // E3
    Store.graph->vertices[28] = (vertex){15, 14};   // E4
    Store.graph->vertices[29] = (vertex){20, 14};   // E5
    Store.graph->vertices[30] = (vertex){25, 14};   // E6

    Store.graph->vertices[36] = (vertex){0, 16};   // F1
    Store.graph->vertices[35] = (vertex){5, 16};   // F2
    Store.graph->vertices[34] = (vertex){10, 16};  // F3
    Store.graph->vertices[33] = (vertex){15, 16};  // F4
    Store.graph->vertices[32] = (vertex){20, 16};  // F5
    Store.graph->vertices[31] = (vertex){25, 16};  // F6

    Store.graph->vertices[37] = (vertex){0, 21};   // G1
    Store.graph->vertices[38] = (vertex){5, 21};   // G2
    Store.graph->vertices[39] = (vertex){10, 21};  // G3
    Store.graph->vertices[40] = (vertex){15, 21};  // G4
    Store.graph->vertices[41] = (vertex){20, 21};  // G5
    Store.graph->vertices[42] = (vertex){25, 21};  // G6

    Store.graph->vertices[48] = (vertex){0, 23};   // H1
    Store.graph->vertices[47] = (vertex){5, 23};   // H2
    Store.graph->vertices[46] = (vertex){10, 23};  // H3
    Store.graph->vertices[45] = (vertex){15, 23};  // H4
    Store.graph->vertices[44] = (vertex){20, 23};  // H5
    Store.graph->vertices[43] = (vertex){25, 23};  // H6


    add_edge(Store.graph, 0, 1); // A1 -> A2 (3m)
    add_edge(Store.graph, 1, 2); // A2 -> A3 (3m)
    add_edge(Store.graph, 2, 3); // A3 -> A4 (5m)
    add_edge(Store.graph, 3, 4); // A4 -> A5 (5m)
    add_edge(Store.graph, 4, 5); // A5 -> A6 (5m)
    add_edge(Store.graph, 5, 6); // A6 -> A7 (5m)

    add_edge(Store.graph, 7, 8); // B6 -> B5 (5m)
    add_edge(Store.graph, 8, 9); // B5 -> B4 (5m)
    add_edge(Store.graph, 9, 10); // B4 -> B3 (5m)
    add_edge(Store.graph, 10, 11); // B3 -> B2 (5m)
    add_edge(Store.graph, 11, 12); // B2 -> B1 (5m)

    add_edge(Store.graph, 13, 14); // C1 -> C2 (5m)
    add_edge(Store.graph, 14, 15); // C2 -> C3 (5m)
    add_edge(Store.graph, 15, 16); // C3 -> C4 (5m)
    add_edge(Store.graph, 16, 17); // C4 -> C5 (5m)
    add_edge(Store.graph, 17, 18); // C5 -> C6 (5m)

    add_edge(Store.graph, 19, 20); // D6 -> D5 (5m)
    add_edge(Store.graph, 20, 21); // D5 -> D4 (5m)
    add_edge(Store.graph, 21, 22); // D4 -> D3 (5m)
    add_edge(Store.graph, 22, 23); // D3 -> D2 (5m)
    add_edge(Store.graph, 23, 24); // D2 -> D1 (5m)

    add_edge(Store.graph, 25, 26); // E1 -> E2 (5m)
    add_edge(Store.graph, 26, 27); // E2 -> E3 (5m)
    add_edge(Store.graph, 27, 28); // E3 -> E4 (5m)
    add_edge(Store.graph, 28, 29); // E4 -> E5 (5m)
    add_edge(Store.graph, 29, 30); // E5 -> E6 (5m)

    add_edge(Store.graph, 31, 32); // F6 -> F5 (5m)
    add_edge(Store.graph, 32, 33); // F5 -> F4 (5m)
    add_edge(Store.graph, 33, 34); // F4 -> F3 (5m)
    add_edge(Store.graph, 34, 35); // F3 -> F2 (5m)
    add_edge(Store.graph, 35, 36); // F2 -> F1 (5m)

    add_edge(Store.graph, 37, 38); // G1 -> G2 (5m)
    add_edge(Store.graph, 38, 39); // G2 -> G3 (5m)
    add_edge(Store.graph, 39, 40); // G3 -> G4 (5m)
    add_edge(Store.graph, 40, 41); // G4 -> G5 (5m)
    add_edge(Store.graph, 41, 42); // G5 -> G6 (5m)

    add_edge(Store.graph, 43, 44); // H6 -> H5 (5m)
    add_edge(Store.graph, 44, 45); // H5 -> H4 (5m)
    add_edge(Store.graph, 45, 46); // H4 -> H3 (5m)
    add_edge(Store.graph, 46, 47); // H3 -> H2 (5m)
    add_edge(Store.graph, 47, 48); // H2 -> H1 (5m)

    add_edge(Store.graph, 12, 0); // B1 -> A1
    add_edge(Store.graph, 13, 12); // C1 -> B1
    add_edge(Store.graph, 24, 13); // D1 -> C1
    add_edge(Store.graph, 25, 24); // E1 -> D1
    add_edge(Store.graph, 36, 25); // F1 -> E1
    add_edge(Store.graph, 37, 36); // G1 -> F1
    add_edge(Store.graph, 48, 37); // H1 -> G1

    add_edge(Store.graph, 6, 7); // A7 -> B6
    add_edge(Store.graph, 7, 18); // B6 -> C6
    add_edge(Store.graph, 18, 19); // C6 -> D6
    add_edge(Store.graph, 19, 30); // D6 -> E6
    add_edge(Store.graph, 30, 31); // E6 -> F6
    add_edge(Store.graph, 31, 42); // F6 -> G6
    add_edge(Store.graph, 42, 43); // G6 -> H6
}

void InitVertexNames(char ch, int from, int to) {
    if (from < to) {
        int ind = 1;
        while (from - 1 <= to) {
            char str1[5];
            str1[0] = ch;
            str1[1] = '\0';
            char str2[3];
            sprintf(str2, "%d", ind); 
            strcat(str1, str2);
            strcpy(Store.vertexes[from - 1], str1);
            from++;
            ind++;
        }
    } else {
        int ind = 1;
        while (from > to) {
            char str1[5]; 
            str1[0] = ch;
            str1[1] = '\0';
            char str2[3];
            sprintf(str2, "%d", ind); 
            strcat(str1, str2);
            strcpy(Store.vertexes[from - 1], str1); 
            from--;
            ind++;
        }
    }
}

void init_graph() {
    for (int i = 0; i < 49; ++i) {
        Store.direction_graph[i] = -1;
    }

    Store.direction_graph[6] = 7;

    Store.direction_graph[7] = 18;

    Store.direction_graph[18] = 19;

    Store.direction_graph[19] = 30;

    Store.direction_graph[30] = 31;

    Store.direction_graph[31] = 42;

    Store.direction_graph[42] = 43;

    Store.direction_graph[48] = 37;

    Store.direction_graph[37] = 36;

    Store.direction_graph[36] = 25;

    Store.direction_graph[25] = 24;

    Store.direction_graph[24] = 13;

    Store.direction_graph[13] = 12;

    Store.direction_graph[12] = 0;
}


void ConveyorsInit()
{
    char *err_msg = 0;
    char *sql_del = "DROP TABLE IF EXISTS Warehouse";
    char *sql = "CREATE TABLE Warehouse(Type INTEGER, Row INTEGER, Column INTEGER, Width INTEGER, Channel_Width INTEGER, Box_Quantity INTEGER)";
    sqlite3_exec(Store.db, sql_del, 0, 0, &err_msg);
    sqlite3_exec(Store.db, sql, 0, 0, &err_msg);

    init_graph();

    InitVertexNames('A', 1, MAX_RACKS + 1);
    InitVertexNames('B', MAX_RACKS * 2 + 3, MAX_RACKS + 2);
    InitVertexNames('C', MAX_RACKS * 2 + 4, MAX_RACKS * 3 + 3);
    InitVertexNames('D', MAX_RACKS * 5, MAX_RACKS * 3 + 4);
    InitVertexNames('E', MAX_RACKS * 5 + 1, MAX_RACKS * 6);
    InitVertexNames('F', MAX_RACKS * 7 + 2, MAX_RACKS * 6 + 1);
    InitVertexNames('G', MAX_RACKS * 7 + 3, MAX_RACKS * 8 + 2);
    InitVertexNames('H', MAX_RACKS * 9 + 4, MAX_RACKS * 8 + 3);
    Store.vertexes[MAX_VERTEXES][0] = '#';

    Store.kill_prog = false;
    int values[] = {48, 43, 46, 46, 43, 46, 46, 38, 33, 41, 38, 33, 33, 30, 57, 30, 23, 20, 17, 17, 15, 10, 7, 7, 7, 7, 20, 7, 12, 7, 23, 10, 7, 10, 7, 7, 7, 7, 7, 7, 7, 12, 7, 7, 10, 7, 7, 10, 7, 7};
    int initial_distribution[] = {20, 17, 18, 18, 17, 18, 18, 15, 13, 16, 15, 13, 13, 12, 25, 12, 9, 8, 7, 7, 6, 4, 3, 3, 3, 3, 8, 3, 5, 3, 9, 4, 3, 4, 3, 3, 3, 3, 3, 3, 3, 5, 3, 3, 4, 3, 3, 4, 2, 1};
    for (int i = 0; i < 50; i++) {
        Store.distribution[i] = initial_distribution[i];
    }

    Store.cur_robot = 0;

    double all_boxes = 0;
  
    for (int i = 0; i < CNT_OF_SKU; ++i) {
      Store.sku_is_adding[i] = false;
      Store.thrs_for_each_sku[i] = Store.distribution[i] + Store.distribution[i] / 2 + 3; // 735.000000 min boxes on the warehouse
      Store.max_boxes_for_each_sku[i] = values[i]; // 1000.000000 max boxes on the warehouse
    } 
   
    for (int i = 0; i < MAX_CONVEYORS; ++i) {
        Store.top_sku_in_channel[i] = -1;
        Store.conveyor_width[i] = 5;
        Store.conveyor[i].box_quantity = 0;
    }
 

    for (int i = 0; i < CNT_OF_SKU; ++i) {
        Store.b_w[i] = i % 5;
        box_pair bp;
        bp.SKU = i;
        bp.width = i % 5;
        Store.box_width[i] = bp;
    }

    // Initialization of cells
    for (int i = 0; i < MAX_VERTEXES; ++i) {
        Store.vertex_robots[i] = 0;
        cell c_cell;
        for (int j = 0; j < MAX_ROBOTS; ++j) {
            c_cell.queue[j] = -1;
        }
        c_cell.id = i;
        c_cell.reserved = 0;
        Store.cells[i] = c_cell;
    }


    char line[256];
    char *fields[3];
    fgets(line, sizeof(line), bots_starting_positions);
    
    for (int i = 0; i < MAX_ROBOTS; ++i) {

        Store.robots[i].temp_reverse_SKU = -1;
        Store.robots[i].temp_reverse_goal_cell.id = -1;

        fgets(line, sizeof(line), bots_starting_positions);
        fields[0] = strtok(line, ",");
        fields[1] = strtok(NULL, ",");
        init_robot(&Store.robots[i], i, Store.graph->vertices[0], Store.graph->vertices[0], 1.0, fields[1]);
    }

    int change_tmp = 0;
    for (int i = 0; i < CNT_OF_SKU; ++i) {
        Store.cnt_boxes_type[i] = 0;
        Store.cnt_boxes_type_const[i] = 0;
    }

    for (int row = 0; row < MAX_BOXES; ++row) {
        for (int col = 0; col < MAX_CONVEYORS; ++col) {
            box b;
            b.empty = 1;
            b.SKU = -1;
            b.width = -1;
            Store.conveyor[col].boxes[row] = b;
        }
    }
    int id = 1;
    for (int row = MAX_BOXES - 1; row >= 0; --row) {
        for (int col = 0; col < MAX_CONVEYORS; ++col) {
            int current_SKU = (low_border + (change_tmp) % (high_border - low_border));
            Store.conveyor[col].boxes[row].SKU = -1;
            Store.conveyor[col].boxes[row].width = -1;
            Store.conveyor[col].boxes[row].empty = 1;
            change_tmp++;
            insert_data(&(Store.db), -1, row, col, -1, 0);
        }
    }
    
    for (int i = 0; i < MAX_ROBOTS + 1; ++i) {
        Store.used[i] = 0;
    }
    Store.boxes_to_deliver = 0;
    Store.cur_file = 0;

    for (int i = 0; i < MAX_ROBOTS + 1; ++i) {
        Store.box_data[i][0] = -1;
        Store.box_data[i][1] = 0;
    }
}

void InitROSS() {
    initialize_graph();
    ConveyorsInit();
}