#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace std;


const int STRIDE = 256;
const int MAX_SIZE = 100;
const int NO_GOAL = 5;

bool m[STRIDE*(MAX_SIZE+2)] = {false};
int goal[STRIDE*(MAX_SIZE+2)];


class Block {
	int offset;
	int w, h;

public:
	Block(int offset, int w, int h) : offset(offset), w(w), h(h) {}

	inline int coords_to_index(int x, int y) {
		return offset + x + STRIDE*y;
	}

	void show() {
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < h; x++) {
				int pt = coords_to_index(x, y);
				cerr << (m[pt] ? "* " : ". ");
			}
			cerr << "  ";
			for (int x = 0; x < h; x++) {
				int pt = coords_to_index(x, y);
				if (goal[pt] == NO_GOAL)
					cerr << '-';
				else
					cerr << goal[pt];
				cerr << ' ';
			}
			cerr << endl;
		}
	}
};


void clear() {
	for (int i = 0; i < sizeof m / sizeof m[0]; i++) {
		m[i] = false;
		goal[i] = NO_GOAL;
	}
}

class FixTheFence {
public:
	string findLoop(vector<string> data) {
		clear();
		Block whole = Block(STRIDE+1, data.size(), data.size());
		for (int y = 0; y < data.size(); y++)
			for (int x = 0; x < data.size(); x++) {
				int pt = whole.coords_to_index(x, y);
				char c = data[y][x];
				if (c == '-')
					goal[pt] = NO_GOAL;
				else
					goal[pt] = c - '0';
			}
		whole.show();
		return "0 0 RDLU";
	}
};


int main(int argc, char* argv[])
{
	int n;

	cin >> n;
	char s[256];
	cin.getline(s, sizeof s);
	assert(s == string(""));

	vector<string> data;
	for (int y = 0; y < n; y++) {
		cin.getline(s, sizeof s);
		data.push_back(s);
	}

	string result = FixTheFence().findLoop(data);
	cout << result << endl;
	return 0;
}
