#include "solution.h"

#include <fstream>
#include <iostream>
#include <unordered_set>
#include <vector>

using namespace std;

struct Partition {
  int machine_id;
  vector<pair<int, int>> edges;
};

struct Runner {
  Graph graph;
  Machines *machines;
  vector<Partition> partitions;

  // used in one assigning task
  vector<bool> core_set, boundart_set;
  vector<int> remaining_node;
  Heap heap;
  int now_machine_idx;
  int now_mem;
  int now_core_node_size;
  ll now_calc_cost;
  Machine *now_machine;
  Partition *now_partition;
  bool no_extra_remaining_node{false};

  Runner(Graph graph, Machines *machines)
      : graph{std::move(graph)}, machines{machines},
        partitions(machines->machine_cnt), heap{graph.n} {
    core_set.resize(graph.n, false);
    boundart_set.resize(graph.n, false);
    remaining_node.reserve(graph.n);
    for (int i = 0; i < graph.n; ++i) {
      remaining_node.push_back(i);
    }
  }

  int choose_core_node() {
    int core_node_id = -1;
    static bool no_extra_remaining_node = false;

    if (no_extra_remaining_node) {
      return core_node_id;
    }

    while (heap.size() > 0) {
      auto [nid, _] = heap.top();
      heap.pop();
      if (!core_set[nid] && boundart_set[nid]) {
        core_node_id = nid;
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
        cerr << string_format("found core node %d in heap",
                              graph.get_node_id(core_node_id))
             << endl;
#endif
#endif
        break;
      }
    }

    if (core_node_id == -1) {
      const int size = remaining_node.size();
      if (size == 0) {
        no_extra_remaining_node = true;
        return core_node_id;
      }
      int ch = rand() % size;
      for (int i = 0; i < size; ++i) {
        int nid = remaining_node[ch];
        if (graph.g[nid].size() == 0) {
          remaining_node[ch] = remaining_node.back();
          remaining_node.pop_back();
          if (remaining_node.size() == 0) {
            break;
          }
          ch = ch % remaining_node.size();
          continue;
        }
        if (graph.g[nid].size() > graph.frac || core_set[nid]) {
          ch = (ch + 1) % remaining_node.size();
          continue;
        }
        core_node_id = nid;
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
        cerr << string_format("found core node %d in remaining nodes",
                              graph.get_node_id(core_node_id))
             << endl;
#endif
#endif
        break;
      }
    }

    if (core_node_id == -1) {
      const int size = remaining_node.size();
      if (size == 0) {
        no_extra_remaining_node = true;
        return core_node_id;
      }
      int ch = rand() % size;
      for (int i = 0; i < size; ++i) {
        int nid = remaining_node[ch];
        if (core_set[nid]) {
          ch = (ch + 1) % size;
          continue;
        }
        core_node_id = nid;
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
        cerr << string_format(
                    "found core node %d in remaining nodes (large degree)",
                    graph.get_node_id(core_node_id))
             << endl;
#endif
#endif
        break;
      }
    }

    if (core_node_id == -1) {
      no_extra_remaining_node = true;
    }
    return core_node_id;
  }

  bool try_add_mem_and_cost(int mem, ll cost) {
    if (now_mem + mem <= now_machine->mem &&
        now_calc_cost + cost <= machines->cost_bound) {
      now_mem += mem;
      now_calc_cost += cost;
      return true;
    }
    return false;
  }

  void assign_edge(int eid) {
    auto &e = graph.edges[eid];
    assert(!e.alloc);
    now_partition->edges.emplace_back(e.u, e.v);
    e.alloc = true;
  }

  bool expand_boundary_node(int u) {
    if (boundart_set[u]) {
      return true;
    }
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
    cerr << "expanding boundary node " << graph.get_node_id(u) << endl;
#endif
#endif
    boundart_set[u] = true;
    ll cnt = 0;

    for (int i = 0; i < (int)graph.g[u].size();) {
      int eid = graph.g[u][i];
      const auto &e = graph.edges[eid];
      if (e.alloc) {
        graph.remove_edge(u, i);
        continue;
      }
      int v = e.V(u);
      if (boundart_set[v]) {
        if (!try_add_mem_and_cost(EDGE_MEM, now_machine->edge_cost)) {
          return false;
        } else {
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
          cerr << string_format("add edge cost %d %d", graph.get_node_id(u),
                                graph.get_node_id(v))
               << endl;
#endif
#endif
        }
        assign_edge(eid);
        graph.remove_edge(u, i);
        if (!core_set[v]) {
          heap.minus_size(v, 1);
        }
      } else {
        ++i;
        cnt += 1;
      }
    }
    if (!core_set[u]) {
      heap.insert(u, cnt);
    }
    return true;
  }

  bool expand_core_node(int u) {
    assert(!core_set[u]);
    core_set[u] = true;
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
    cerr << "expanding core node " << graph.get_node_id(u) << endl;
#endif
#endif
    if (!expand_boundary_node(u))
      return false;
    while (graph.g[u].size() > 0) {
      int eid = graph.g[u].front();
      auto &e = graph.edges[eid];
      if (e.alloc) {
        graph.remove_edge(u, 0);
        continue;
      }
      int v = e.V(u);
      assert(!boundart_set[v]);
      if (!try_add_mem_and_cost(NODE_MEM + EDGE_MEM,
                                now_machine->node_cost +
                                    now_machine->edge_cost)) {
        return false;
      } else {
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
        cerr << string_format("core(%d) add edge cost %d %d",
                              graph.get_node_id(u), graph.get_node_id(u),
                              graph.get_node_id(v))
             << endl;
        cerr << string_format("core(%d) add node cost %d", graph.get_node_id(u),
                              graph.get_node_id(v))
             << endl;
#endif
#endif
      }
      assign_edge(eid);
      graph.remove_edge(u, 0);
      if (!expand_boundary_node(v))
        return false;
    }
    return true;
  }

  void assign_one_partition(int index) {
    cerr << string_format(
                "assigning one partition to machine (idx=%d), mem=%d, bound=%d",
                index, machines->m[index].mem,
                machines->m[index].calc_node_bound(machines->cost_bound,
                                                   graph.frac))
         << endl;
    fill(core_set.begin(), core_set.end(), false);
    fill(boundart_set.begin(), boundart_set.end(), false);
    heap.clear();
    now_machine_idx = index;
    now_machine = &machines->m[index];
    now_mem = 0;
    now_calc_cost = 0;
    now_partition = &partitions[index];
    now_partition->machine_id = now_machine->machine_id;

    while (now_mem + EDGE_MEM <= now_machine->mem &&
           now_calc_cost + now_machine->edge_cost <= machines->cost_bound) {
      int core_node_id = choose_core_node();
      if (core_node_id == -1) {
        break;
      }
      assert(!core_set[core_node_id]);
      int now_size = now_partition->edges.size();
      bool add_mem_and_cost = false;
      if (!boundart_set[core_node_id]) {
        add_mem_and_cost =
            try_add_mem_and_cost(NODE_MEM, now_machine->node_cost);
        if (!add_mem_and_cost) {
          break;
        } else {
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
          cerr << string_format("add core node %d",
                                graph.get_node_id(core_node_id))
               << endl;
#endif
#endif
        }
      }
      now_core_node_size = graph.g[core_node_id].size();
      bool flag = expand_core_node(core_node_id);
      if (add_mem_and_cost && now_size == (int)now_partition->edges.size()) {
        now_mem -= NODE_MEM;
        now_calc_cost -= now_machine->node_cost;
        core_set[core_node_id] = false;
        boundart_set[core_node_id] = false;
        if (graph.g[core_node_id].size() > 0) {
          assert(!flag);
          remaining_node.push_back(core_node_id);
          no_extra_remaining_node = false;
          flag = rand() % 10 <= 8;
        }
#ifndef NDEBUG
#ifndef ONLINE_JUDGE
        cerr << "remove core node cost " << graph.get_node_id(core_node_id)
             << endl;
#endif
#endif
      }
      if (!flag) {
        break;
      }
    }

    cerr << string_format("finished to assign machine (idx=%d), mem_cost=%d, "
                          "calc_cost=%lld",
                          index, now_mem, now_calc_cost)
         << endl;
  }

  void run() {
    for (int i = 0; i < machines->machine_cnt; ++i) {
      assign_one_partition(i);
    }
  }
};

void Solution::func(const string &input_file, const string &result_file) {
  ifstream ifs(input_file);
  int n, m, machine_num, machine_id, node_cost, edge_cost, mem, comm_cost, src,
      dst;

  ifs >> n >> m;
  cerr << string_format("n = %d, m = %d", n, m) << endl;
  Graph graph{n, m};

  Timer timer;

  timer.start("Adding edges");
  for (int i = 0; i < m; ++i) {
    ifs >> src >> dst;
    graph.add_edge(src, dst);
  }
  timer.stop();

  ifs >> machine_num;
  cerr << "machine_num = " << machine_num << endl;

  timer.start("Adding machines");
  Machines machines{machine_num, &graph};
  for (int i = 0; i < machine_num; ++i) {
    ifs >> machine_id >> node_cost >> edge_cost >> mem >> comm_cost;
    machines.add_machine(machine_id, node_cost, edge_cost, mem, comm_cost);
  }
  timer.stop();

  ifs.close();

  timer.start("Calcing bound");
  machines.calc_cost_bound();
  machines.sort_by_bound();
  timer.stop();

  cerr << "machines calc_bound = " << machines.cost_bound << endl;

  timer.start("Running");
  Runner runner{graph, &machines};
  runner.run();
  timer.stop();

  timer.start("Outputing");
  ofstream ofs(result_file);
  for (int index = 0; index < machine_num; ++index) {
    const auto &partition = runner.partitions[index];
    if (partition.edges.size() > 0) {
      ofs << partition.machine_id << " " << partition.edges.size() << "\n";
      for (const auto &edge : partition.edges) {
        ofs << graph.get_node_id(edge.first) << " "
            << graph.get_node_id(edge.second) << "\n";
      }
    }
  }
  ofs.close();
  timer.stop();
}
