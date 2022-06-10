#include "common.hpp"
#include<bits/stdc++.h>
using namespace std;
using namespace Eigen;

struct a {
    int k;
    struct b* another_b = NULL;
    a(b* new_b){
        this->another_b = new_b;
    }
};

struct b {
    int l;
    struct a* another_a = NULL;
    b(a* new_a){
        this->another_a = new_a;
    }
};

int main(){
    a* A;
    b* B = new b(A);
    A->another_b = B;
    cout << (B->another_a) << "\n";
    return 0;
}