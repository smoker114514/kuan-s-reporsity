#include"check.h"
bool check(string a) {//检查输入的是否合规
	for (int i = 0;i < a.length();i++) {
		if (!isdigit(a[i])) {
			return true;
		}
	}
	return false;
}