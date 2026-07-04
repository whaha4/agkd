#include <iostream>
#include <string>
using namespace std;

// 递归基案：无参数时结束递归
void unpack() {
    cout << endl;
}

// 可变参数模板：打印第一个，递归处理剩余
template<typename T, typename... Args>
void unpack(T first, Args... rest) {
    cout << first << " ";
    unpack(rest...);
}

int main() {
    string s;
    getline(cin, s);
    unpack(s);

    unpack(1,2.5, "hello", 'a');
    return 0;
}
