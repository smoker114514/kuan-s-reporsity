#include"control.h"
#include"check.h"
void control0(bool& a) {
	if (a) {
		a = false;
		cout << "已关机~(￣▽￣)~*" << endl;
	}
	else {
		cout << "已经关了＞﹏＜" << endl;
	}
}
void control1(bool& a) {
	if (a) {
		cout << "已经开了＞﹏＜" << endl;
	}
	else {
		cout << "\033[45m输完了吗，输完那我开转了┗|｀O′|┛\033[0m" << endl;
		cout << "注意：默认转速为2000r/s顺时针，最高为10000r/s，每开一次，转速回到默认值" << endl;
		a = true;
	}
}
void control2(const bool& a,int &n) {
	if (a) {
		cout << "请输入修改后的转速:" << endl;
		string c;
		cin >> c;
		while (check(c)) {
			cout << "别闹，输数字，返回菜单输入back" << endl;
			cin >> c;
			if (c == "back") {
				return;
			}
		}
		int b = stoi(c);
		if (b <= 10000 && b >= 0) {
			cout << "修改成功" << endl;
			n = b;
		}
		else {
			cout << "修改失败，不在转速阈值内，即将返回菜单界面" << endl;
			return;
		}
	}
	else {
		cout << "电机已关闭，无法修改" << endl;
	}
}
void control3(const bool& a, const int& n) {
	if (a) {
		cout << "当前正在转动，转速为" << n << "r/s" << endl;
	}
	else {
		cout << "当前电机关闭，无法查看" << endl;
	}
}
void control4(bool& a, const bool& b) {
	if (b) {
		if (a) {
			a = false;
			cout << "已修改为逆时针转动" << endl;
		}
		else {
			a = true;
			cout << "已修改为顺时针转动" << endl;
		}
	}
	else {
		cout << "电机已关闭，无法修改" << endl;
	}
}
void control5(const bool& a,const bool& b) {
	if (b) {
		if (a) {
			cout << "当前为顺时针转动" << endl;
		}
		else {
			cout << "当前为逆时针转动" << endl;
		}
	}
	else {
		cout << "电机已关闭，无法查看" << endl;
	}
}