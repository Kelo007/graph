#include "checker.h"

#include <cassert>
#include <fstream>

using namespace std;

long long check(const string &input, const string &output) {
  ifstream ifs(input);
  ifstream ofs(output);

  int n, m, machine_num, machine_id, node_cost, edge_cost, mem, comm_cost, src,
      dst;

  ifs >> n >> m;
  Checker::Checker checker{n, m};

  for (int i = 0; i < m; ++i) {
    ifs >> src >> dst;
    checker.add_edge(src, dst);
  }

  ifs >> machine_num;
  for (int i = 0; i < machine_num; ++i) {
    ifs >> machine_id >> node_cost >> edge_cost >> mem >> comm_cost;
    checker.add_machine(machine_id, node_cost, edge_cost, mem, comm_cost);
  }

  vector<Checker::Partition> partitions;
  int size;

  while (ofs >> machine_id >> size) {
    partitions.push_back(Checker::Partition(machine_id));
    auto &partition = partitions.back();
    for (int i = 0; i < size; ++i) {
      ofs >> src >> dst;
      partition.edges.emplace_back(src, dst);
    }
  }

  long long result = checker.check(partitions);
  cerr << result << endl;

  ifs.close();
  ofs.close();

  return result;
}

int main(int argc, char *argv[]) {
  assert(argc == 3);
  long long result = check(argv[1], argv[2]);
  if (result < 0) {
    return -1;
  }
  return 0;
}
