#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <utility>
using namespace std;

template <typename T>
inline void hash_combine(std::size_t &seed, const T &val) {
  seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
// auxiliary generic functions to create a hash value using a seed
template <typename T> inline void hash_val(std::size_t &seed, const T &val) {
  hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t &seed, const T &val, const Types &... args) {
  hash_combine(seed, val);
  hash_val(seed, args...);
}

template <typename... Types>
inline std::size_t hash_val(const Types &... args) {
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

void gen(const string &filename, int n, int m, int k, int p) {
  ofstream ofs(filename);
  ofs << n << " " << m << "\n";
  unordered_set<pair<int, int>, pair_hash> edges;
  edges.reserve(m * 2);
  for (int i = 0; i < m; ++i) {
    int u = 0, v = 0;
    while (u == v || edges.count(make_pair(u, v)) == 1 ||
           edges.count(make_pair(v, u)) == 1) {
      if (rand() % 100 <= p) {
        u = rand() % (n / 100);
        v = rand() % n;
      } else {
        u = rand() % n;
        v = rand() % n;
      }
    }
    edges.emplace(u, v);
    ofs << u << " " << v << "\n";
  }
  ofs << k << "\n";
  for (int i = 0; i < k; ++i) {
    // ofs << i << " 1 1 " << (rand() & 1 ? (n * k + m * 2) / k + 500 : 2546457)
    //     << " 1"
    //     << "\n";
    ofs << i << " 10 15 " << (rand() % 5 == 0 ? 10000000 : 3000000) << " 15"
        << "\n";
  }
}

int main(int argc, char *argv[]) {
  std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::high_resolution_clock::now();
  auto duration = now.time_since_epoch();
  srand(duration.count());
  int n = 300000;
  int m = 3500000;
  int k = 100;
  int p = 80;
  assert(argc >= 2);
  if (argc >= 3) {
    n = atoi(argv[2]);
  }
  if (argc >= 4) {
    m = atoi(argv[3]);
  }
  if (argc >= 5) {
    k = atoi(argv[4]);
  }
  if (argc >= 6) {
    p = atoi(argv[5]);
  }
  gen(argv[1], n, m, k, p);

  return 0;
}
