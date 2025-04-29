//The C main file for a ROSS model
//This file includes:
// - definition of the LP types
// - command line argument setup
// - a main function

//includes
#include "model.h"
#include "init.c"

//Given an LP's GID (global ID)
//return the PE (aka node, MPI Rank)
tw_peid model_map(tw_lpid gid){
  return (tw_peid) gid / g_tw_nlp;
}

// Define LP types
//   these are the functions called by ROSS for each LP
//   multiple sets can be defined (for multiple LP types)
tw_lptype model_lps[] = {
  {
    (init_f) model_init,
    (pre_run_f) NULL,
    (event_f) model_event,
    (revent_f) model_event_reverse,
    (commit_f) NULL,
    (final_f) model_final,
    (map_f) model_map,
    sizeof(state)
  },
  { 0 },
};

//Define command line arguments default values
unsigned int setting_1 = 0;

//add your command line opts
const tw_optdef model_opts[] = {
	TWOPT_GROUP("ROSS Model"),
	TWOPT_UINT("setting_1", setting_1, "first setting for this model"),
	TWOPT_END(),
};

void displayModelSettings()
{
  if (g_tw_mynode == 0)
  {
    for (int i = 0; i < 30; i++)
      printf("*");
    
    printf("\n");
    printf("Model Configuration:\n");
    printf("\t nnodes: %i\n", tw_nnodes());
    printf("\t g_tw_nlp: %llu\n", g_tw_nlp);

    for (int i = 0; i < 30; i++)
      printf("*");
    
    printf("\n");
  }
}

//for doxygen
#define model_main main


int model_main (int argc, char* argv[]) {
  bots_starting_positions = fopen("../../../../WarehouseModel/bots_starting_positions.csv", "r");
  f = fopen("../../../../WarehouseModel/full_actions_log.txt", "w");
  sku_mileage = fopen("../../../../WarehouseModel/sku_mileage.txt", "w");
  const char *directory_path = "../../../../WarehouseModel/TEST3-SIMSIM";
  struct dirent *entry;
  DIR *dp = opendir(directory_path);
  sqlite3_open("../../../../WarehouseModel/ross-sqlite.db", &Store.db);

  while ((entry = readdir(dp))) {
    if (strstr(entry->d_name, ".csv") != NULL) {
      snprintf(Store.files[Store.cur_file], sizeof(Store.files[Store.cur_file]), "%s/%s", directory_path, entry->d_name);
      Store.cur_file += 1;
    }
  }
  Store.cur_file = 0;

  fprintf(f, "EventID  Time BotID        Command StartPoint EndPoint  BoxType Channel TrID\n");
  fprintf(sku_mileage, "SKU       mileage\n");

	InitROSS();
	int i, num_lps_per_pe;
  tw_opt_add(model_opts);
  tw_init(&argc, &argv);
  num_lps_per_pe = MAX_ROBOTS + 1;
  tw_define_lps(num_lps_per_pe, sizeof(message));
  g_tw_lp_typemap = (tw_typemap_f)&model_map;
  for (int i = 0; i < g_tw_nlp; ++i)
    tw_lp_settype(i, &model_lps[0]);

	tw_run();

	tw_end();
  fprintf(f, "------------------------------------------------------------------------------------\n");
  fprintf(f, "------------------------------------------------------------------------------------\n");
  fprintf(f, "------------------------------------------------------------------------------------\n");

	return 0;
}
