#include"control.h"
#include"check.h"
void control0(bool& a) {
	if (a) {
		a = false;
		cout << "�ѹػ�~(������)~*" << endl;
	}
	else {
		cout << "�Ѿ����ˣ��n��" << endl;
	}
}
void control1(bool& a) {
	if (a) {
		cout << "�Ѿ����ˣ��n��" << endl;
	}
	else {
		cout << "\033[45m���������������ҿ�ת�˩�|��O��|��\033[0m" << endl;
		cout << "ע�⣺Ĭ��ת��Ϊ2000r/s˳ʱ�룬���Ϊ10000r/s��ÿ��һ�Σ�ת�ٻص�Ĭ��ֵ" << endl;
		a = true;
	}
}
void control2(const bool& a,int &n) {
	if (a) {
		cout << "�������޸ĺ��ת��:" << endl;
		string c;
		cin >> c;
		while (check(c)) {
			cout << "���֣������֣����ز˵�����back" << endl;
			cin >> c;
			if (c == "back") {
				return;
			}
		}
		int b = stoi(c);
		if (b <= 10000 && b >= 0) {
			cout << "�޸ĳɹ�" << endl;
			n = b;
		}
		else {
			cout << "�޸�ʧ�ܣ�����ת����ֵ�ڣ��������ز˵�����" << endl;
			return;
		}
	}
	else {
		cout << "����ѹرգ��޷��޸�" << endl;
	}
}
void control3(const bool& a, const int& n) {
	if (a) {
		cout << "��ǰ����ת����ת��Ϊ" << n << "r/s" << endl;
	}
	else {
		cout << "��ǰ����رգ��޷��鿴" << endl;
	}
}
void control4(bool& a, const bool& b) {
	if (b) {
		if (a) {
			a = false;
			cout << "���޸�Ϊ��ʱ��ת��" << endl;
		}
		else {
			a = true;
			cout << "���޸�Ϊ˳ʱ��ת��" << endl;
		}
	}
	else {
		cout << "����ѹرգ��޷��޸�" << endl;
	}
}
void control5(const bool& a,const bool& b) {
	if (b) {
		if (a) {
			cout << "��ǰΪ˳ʱ��ת��" << endl;
		}
		else {
			cout << "��ǰΪ��ʱ��ת��" << endl;
		}
	}
	else {
		cout << "����ѹرգ��޷��鿴" << endl;
	}
}