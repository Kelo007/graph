#ifndef SOLUTION_H
#define SOLUTION_H

#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

using namespace std;
using ll = long long;

constexpr int NODE_MEM = 1;
constexpr int EDGE_MEM = 2;

// the following are UBUNTU/LINUX, and MacOS ONLY terminal color codes.
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

struct Edge {
  int u{0}, v{0};
  bool alloc{false};
  Edge(int u, int v) : u{u}, v{v} {}
  int V(int x) const { return x == u ? v : u; }
};

struct Graph {
  vector<Edge> edges;
  vector<vector<int>> g;
  unordered_map<int, int> node_id_to_inner_id;
  vector<int> inner_id_to_node_id;
  int inner_id_cnt{0};
  int n, m;
  double frac;

  Graph(int n, int m) : g(n), n{n}, m{m}, frac{(double)m / n} {
    edges.reserve(m);
    node_id_to_inner_id.reserve(n * 2);
    inner_id_to_node_id.reserve(n);
    for (int i = 0; i < n; ++i) {
      g[i].reserve(frac * 2);
    }
  }

  int get_inner_id(int u) {
    auto [it, inserted] = node_id_to_inner_id.try_emplace(u, inner_id_cnt);
    if (inserted) {
      inner_id_to_node_id.push_back(u);
      return inner_id_cnt++;
    }
    return it->second;
  }

  int get_node_id(int u) { return inner_id_to_node_id[u]; }

  void add_edge(int u, int v) {
    u = get_inner_id(u);
    v = get_inner_id(v);
    int eid = edges.size();
    edges.push_back(Edge(u, v));
    g[u].push_back(eid);
    g[v].push_back(eid);
  }

  void remove_edge(int u, int idx) {
    assert(u < n);
    assert(idx < (int)g[u].size());
    g[u][idx] = g[u].back();
    g[u].pop_back();
  }
};

struct Machine {
  int machine_id;
  int node_cost;
  int edge_cost;
  int mem;
  int comm_cost;
  int calc_node_bound(ll cost_bound, double frac) const {
    return min(int(cost_bound / (node_cost + edge_cost * frac)),
               int(mem / (1 + 2 * frac)));
  }
};

struct Machines {
  vector<Machine> m;
  Graph *graph;
  ll cost_bound{0};
  int machine_cnt;
  double W = 1.6;
  Machines(int n, Graph *graph) : graph{graph}, machine_cnt{n} { m.reserve(n); }

  void add_machine(int machine_id, int node_cost, int edge_cost, int mem,
                   int comm_cost) {
    m.push_back(Machine{machine_id, node_cost, edge_cost, mem, comm_cost});
    cost_bound =
        max(cost_bound, (ll)graph->n * node_cost + (ll)graph->m * edge_cost);
  }

  void calc_cost_bound() {
    ll l = 0, r = cost_bound;
    while (l < r) {
      ll mid = (l + r) >> 1;
      int covered_node_cnt = 0;
      for (const auto &machine : m) {
        covered_node_cnt += machine.calc_node_bound(mid, graph->frac);
      }
      if (covered_node_cnt >= graph->n * W) {
        r = mid;
      } else {
        l = mid + 1;
      }
    }
    cost_bound = l;
  }

  void sort_by_bound() {
    sort(m.begin(), m.end(), [=](const Machine &lhs, const Machine &rhs) {
      return lhs.calc_node_bound(cost_bound, graph->frac) >
             rhs.calc_node_bound(cost_bound, graph->frac);
    });
  }
};

struct Heap {
  int n;
  vector<int> vid_to_idx;
  vector<pair<int, ll>> data;
  Heap(int max_n) : n{0}, vid_to_idx(max_n), data(1) {}

  void clear() {
    data.resize(1);
    n = 0;
  }

  void insert(int vid, ll cost) {
    vid_to_idx[vid] = ++n;
    data.emplace_back(vid, cost);
    shift_up(n);
  }

  void shift_up(int idx) {
    while (idx > 1) {
      int p_idx = idx >> 1;
      if (data[idx].second < data[p_idx].second) {
        swap(data[idx], data[p_idx]);
        swap(vid_to_idx[data[idx].first], vid_to_idx[data[p_idx].first]);
        idx = p_idx;
      } else {
        break;
      }
    }
  }

  void shift_down(int idx) {
    while (true) {
      int ch;
      int l = idx << 1;
      int r = l | 1;

      if (r > n) {
        return;
      } else {
        ch = data[l].second < data[r].second ? l : r;
      }

      if (data[idx].second > data[ch].second) {
        swap(data[idx], data[ch]);
        swap(vid_to_idx[data[idx].first], vid_to_idx[data[ch].first]);
        idx = ch;
      } else {
        return;
      }
    }
  }

  int size() { return n; }

  auto top() {
    assert(n >= 1);
    return data[1];
  }

  void pop() {
    assert(n >= 1);
    data[1] = data[n--];
    vid_to_idx[data[1].first] = 1;
    shift_down(1);
  }

  void minus_one(int vid) {
    int idx = vid_to_idx[vid];
    data[idx].second--;
    shift_up(idx);
  }
};

template <typename... Args>
std::string string_format(const std::string &format, Args... args) {
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

struct Timer {
  string id;
  chrono::system_clock::time_point start_time;
  bool valid{false};

  void start(const string &x) {
    cerr << YELLOW << string_format("Timer(%15s) start", x.data()) << RESET
         << endl;
    start_time = chrono::system_clock::now();
    id = x;
    valid = true;
  }

  void stop() {
    if (!valid)
      return;
    auto stop_time = chrono::system_clock::now();
    auto duration =
        chrono::duration_cast<chrono::milliseconds>(stop_time - start_time);
    cerr << YELLOW
         << string_format("Timer(%15s) stop: cost %d ms", id.data(),
                          duration.count())
         << RESET << endl;
    valid = false;
  }

  ~Timer() { stop(); }
};

class Solution {
public:
  static void func(const string &input_file, const string &result_file);
};

#endif
