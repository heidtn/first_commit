#include "pathgen_node.h"

int main(int argc, char **argv) {  
  ros::init(argc, argv, "pathgen");

  // this will block
  PathGen pathgen;

  return 0;
}
