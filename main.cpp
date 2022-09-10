#include <fstream>
#include "solution.h"

int main(int argc, char *argv[]){
    ifstream ifs(argv[1]);
    Solution::func(ifs, argv[2]);
    ifs.close();
    return 0;
}