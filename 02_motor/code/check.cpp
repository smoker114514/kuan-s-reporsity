#include"check.h"
bool check(string a) {//���������Ƿ�Ϲ�
	for (int i = 0;i < a.length();i++) {
		if (!isdigit(a[i])) {
			return true;
		}
	}
	return false;
}