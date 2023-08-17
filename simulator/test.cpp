#include <iostream>


using namespace std;

bool f1() {
  bool a = true;
  return a;
}

int main() {
  if(f1()) {
    cout << "true" << endl;
  } else {
    cout << "false" << endl;
  }
  return 0;
}