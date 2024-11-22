

#include <iostream>
#include <thread>
#include<string>
#include<barrier>
#include<chrono>
using namespace std;
barrier bar(2);
bool now = true;
int h_num = 0;
int c_num = 0;
void ReleaseHydrogen(int& a) {
	while (now || (a >= 4)) {
		if (a >= 4) {
			a -= 4;
			bar.arrive_and_wait();
			cout << "HHHH";
		}
	}
}
void ReleaseCarbon(int& b) {
	while (now || (b >= 1)) {
		if (b >= 1) {
			b -= 1;
			bar.arrive_and_wait();
			cout << "C";
		}
	}
}
int main() {
	string c;
	cin >> c;
	thread t1(ReleaseCarbon, ref(c_num));
	thread t2(ReleaseHydrogen, ref(h_num));
	for (int i = 0;i < c.size();i++) {
		if (c[i] == 'C') {
			c_num++;
		}
		else if (c[i] == 'H') {
			h_num++;
		}
	}
	now = false;
	this_thread::sleep_for(chrono::milliseconds(100));
	if (t1.joinable() || t2.joinable()) {
		t1.detach();
		t2.detach();
		return 0;
	}
}


