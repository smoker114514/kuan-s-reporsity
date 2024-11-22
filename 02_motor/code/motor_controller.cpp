#include<iostream>
#include"menu.h"
#include"control.h"
#include<string>
#include"check.h"
using namespace std;
bool now = false;
int n = 2000;
bool dirt = true;
int main() {
	cout << "欢迎使用电机控制平台，让我们开始控制电机吧" << endl;
	system("pause");
	system("cls");
	while (1) {
		if (now) {
			cout << "已经开转，画面自行脑补（嗡~~~~~~~~）" << endl;
		}
		show();
		string a;
		cin >> a;
		int b = 0;
		while (check(a)||stoi(a)>6||stoi(a)<0) {
			if (b > 2) {
				cout << "错太多，死机了X﹏X" << endl;
				return 0;
			}
			cout << "别闹，输数字0~6" << endl;
			b++;
			cin >> a;
		}
		int choice = stoi(a);
		b = 0;
		switch (choice)
		{
		case 0:
			control0(now);
			n = 2000;
			system("pause");
			system("cls");
			break;
		case 1:
			control1(now);
			system("pause");
			system("cls");
			break;
		case 2:
			control2(now,n);
			system("pause");
			system("cls");
			break;
		case 3:
			control3(now,n);
			system("pause");
			system("cls");
			break;
		case 4:
			control4(dirt,now);
			system("pause");
			system("cls");
			break;
		case 5:
			control5(dirt,now);
			system("pause");
			system("cls");
			break;
		default:
			system("cls");
			cout << "感谢你的使用，再见（￣︶￣）" << endl;
			return 0;
			break;
		}
	}
}
