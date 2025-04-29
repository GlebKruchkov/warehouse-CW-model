//The C mapping for a ROSS model
//This file includes:
// - the required LP GID -> PE mapping function
// - Commented out example of LPType map (when there multiple LP types)
// - Commented out example of one set of custom mapping functions:
//   - setup function to place LPs and KPs on PEs
//   - local map function to find LP in local PE's array

#include "model.h"

//Given an LP's GID (global ID)
//return the PE (aka node, MPI Rank)
static tw_peid model_map(tw_lpid gid) {
  return (tw_peid) gid / g_tw_nlp;
}
