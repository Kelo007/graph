#undef NDEBUG

#include "solution.h"
#include <utility>
using namespace std;

void test_heap() {
  Heap heap(100);
  for (int i = 0; i < 100; ++i) {
    heap.insert(i, i);
    assert(heap.top().first == 0);
    assert(heap.top().second == 0);
  }
  for (int i = 0; i < 99; ++i) {
    heap.pop();
    assert(heap.top().first == i + 1);
    assert(heap.top().second == i + 1);
  }
}

int main() { test_heap(); }
