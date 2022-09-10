#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

template <typename T>
inline void hash_combine(std::size_t &seed, const T &val) {
  seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
// auxiliary generic functions to create a hash value using a seed
template <typename T> inline void hash_val(std::size_t &seed, const T &val) {
  hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t &seed, const T &val, const Types &...args) {
  hash_combine(seed, val);
  hash_val(seed, args...);
}

template <typename... Types> inline std::size_t hash_val(const Types &...args) {
  std::size_t seed = 0;
  hash_val(seed, args...);
  return seed;
}

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const {
    return hash_val(p.first, p.second);
  }
};

template <typename... Args>
inline std::string string_format(const std::string &format, Args... args) {
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
               1; // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(),
                     buf.get() + size - 1); // We don't want the '\0' inside
}

namespace Checker {

using Edge = std::pair<int, int>;

struct Machine {
  int machine_id;
  int node_cost;
  int edge_cost;
  int mem;
  int comm_cost;
};

struct Partition {
  int machine_id;
  Partition(int machine_id) : machine_id(machine_id) {}
  std::vector<Edge> edges;
};

struct Checker {
  std::vector<Edge> edges;
  std::unordered_map<int, Machine> machines;
  std::unordered_set<Edge, pair_hash> edge_set;
  std::unordered_map<int, std::vector<int>> node_map;
  int n, m;

  Checker(int n, int m) : n{n}, m{m} {
    edges.reserve(m);
    edge_set.reserve(m * 2);
    node_map.reserve(n * 2);
  }

  void add_edge(int u, int v) { edges.emplace_back(u, v); }
  void add_machine(int machine_id, int node_cost, int edge_cost, int mem,
                   int comm_cost) {
    machines.emplace(machine_id,
                     Machine{machine_id, node_cost, edge_cost, mem, comm_cost});
  }

  bool try_insert_node_to_map(int nid, int idx) {
    auto &v = node_map[nid];
    bool in_partition = v.size() > 0 && v.back() == idx;
    if (!in_partition) {
      v.push_back(idx);
    }
    return in_partition;
  }

  long long check(const std::vector<Partition> &partitions) {
    edge_set.clear();
    node_map.clear();

    for (const auto &e : edges) {
      edge_set.insert(e);
    }
    for (const auto &p : partitions) {
      for (const auto &e : p.edges) {
        int cnt = edge_set.erase(e);
        if (cnt == 0) {
          std::cerr << string_format("Wrong edge : %d -> %d", e.first, e.second)
                    << std::endl;
          return -1;
        }
      }
    }
    if (edge_set.size() > 0) {
      const auto &e = edge_set.begin();
      std::cerr << string_format("Wrong edge : %d -> %d", e->first, e->second)
                << std::endl;
      return -1;
    }

    long long max_calc_cost = 0;
    long long max_comm_cost = 0;

    for (const auto &p : partitions) {
      const auto &m = machines[p.machine_id];
      int mem = 0;
      long long calc_cost = 0;
      for (const auto &e : p.edges) {
        mem += 2;
        calc_cost += m.edge_cost;
        if (!try_insert_node_to_map(e.first, p.machine_id)) {
          mem += 1;
          calc_cost += m.node_cost;
        }
        if (!try_insert_node_to_map(e.second, p.machine_id)) {
          mem += 1;
          calc_cost += m.node_cost;
        }
      }
      if (mem > m.mem) {
        std::cerr << "OOM! machine_id = " << m.machine_id << std::endl;
        return -1;
      }
      std::cerr << string_format("machine id=%d, mem(%d/%d), calc_cost(%lld)",
                                 m.machine_id, mem, m.mem, calc_cost)
                << std::endl;
      max_calc_cost = std::max(max_calc_cost, calc_cost);
    }

    std::unordered_map<int, long long> comm_cost_map;

    for (const auto &it : node_map) {
      const auto &m = it.second;
      const int size = m.size();
      long long comm_cost_sum = 0;
      for (int machine_id : m) {
        const auto &machine = machines[machine_id];
        comm_cost_sum += machine.comm_cost;
      }
      for (int machine_id : m) {
        const auto &machine = machines[machine_id];
        comm_cost_map[machine_id] += (long long)machine.comm_cost * (size - 1) +
                                     (comm_cost_sum - machine.comm_cost);
      }
    }

    for (const auto &it : comm_cost_map) {
      std::cerr << string_format("machine id=%d, comm_cost(%d)", it.first,
                                 it.second)
                << std::endl;
      max_comm_cost = std::max(max_comm_cost, it.second);
    }

    std::cerr << string_format("max_calc_cost=%lld, max_comm_cost=%lld",
                               max_calc_cost, max_comm_cost)
              << std::endl;

    return max_calc_cost + max_comm_cost;
  }
};
} // namespace Checker