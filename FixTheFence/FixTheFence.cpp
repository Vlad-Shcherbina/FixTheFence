#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <sstream>
#include <map>
#include <algorithm>
#include <iterator>

#include "consts.h"

using namespace std;


const int STRIDE = 256;
const int MAX_SIZE = 100;
const int NO_GOAL = 5;

const int UP = -STRIDE;
const int DOWN = STRIDE;
const int LEFT = -1;
const int RIGHT = 1;


bool m[STRIDE*(MAX_SIZE+2)] = {false};
int goal[STRIDE*(MAX_SIZE+2)];


typedef pair<int, int> Coords;


class Block {
	int offset;

public:
	int w, h;

	Block(int offset, int w, int h) : offset(offset), w(w), h(h) {}

	inline int coords_to_index(int x, int y) {
		return offset + x + STRIDE*y;
	}

	inline Coords index_to_coords(int pt) {
		return Coords(pt % STRIDE - offset % STRIDE, pt / STRIDE - offset / STRIDE);
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

	string trace_path(int pt, int prev_d, Coords &finish) {
		int start = pt;

		string path;
		int d = prev_d;

		Coords xy;

		while (true) {
			bool ul = m[pt-STRIDE-1];
			bool ur = m[pt-STRIDE];
			bool dl = m[pt-1];
			bool dr = m[pt];

			assert(ul == ur || ur == dr || dr == dl);

			if (ul != ur && d != DOWN) {
				d = UP;
				path.push_back('U');
			} else if (dl != dr && d != UP) {
				d = DOWN;
				path.push_back('D');
			} else if (ul != dl && d != RIGHT) {
				d = LEFT;
				path.push_back('L');
			} else if (ur != dr && d != LEFT) {
				d = RIGHT;
				path.push_back('R');
			} else
				assert(false);

			pt += d;
			xy = index_to_coords(pt);
			if (pt == start)
				break;

			if (xy.first < 0 || xy.first > w || xy.second < 0 || xy.second > h)
				break;
		}
		finish = xy;
		return path;
	}
};


typedef int Gate;
typedef pair<Gate, Gate> GatePair;

const Gate NONE = 0;
const Gate IN = 1;
const Gate OUT = 2;
const Gate BARRIER = 3;

typedef int Topo;
typedef unsigned int Bits;

GatePair GATE_PAIRS[] = {
    GatePair(NONE, NONE), GatePair(NONE, IN), GatePair(NONE, OUT), GatePair(IN, NONE), GatePair(OUT, NONE),
    GatePair(IN, IN), GatePair(IN, OUT), GatePair(OUT, IN), GatePair(OUT, OUT),
	GatePair(BARRIER, BARRIER)};
const int NUM_GATE_PAIRS = sizeof GATE_PAIRS / sizeof GATE_PAIRS[0];

class Propagator {
public:
	int w;
	int num_topos;
	vector<map<GatePair, vector<pair<Bits, Topo> > > > transitions;
	Propagator() {}
	Propagator(int w, int num_topos, const unsigned transition_data[], const int transitions_starts[]) {
		this->w = w;
		this->num_topos = num_topos;
		transitions.resize(num_topos);
		const int *start = transitions_starts;
		for (int i = 0; i < num_topos; i++) {
			for (int j = 0; j < NUM_GATE_PAIRS; j++) {
				vector<pair<Bits, Topo> > &ts = transitions[i][GATE_PAIRS[j]];
				for (int k = start[0]; k < start[1]; k++) {
					Bits bits = transition_data[k] & 255;
					Topo new_topo = transition_data[k] >> 8;
					ts.push_back(make_pair(bits, new_topo));
				}
				start++;
			}
		}
		assert(transition_data[start[0]] == 42);
	}
};

map<int, Propagator> propagators;

void init() {
	for (int i = 0; i < sizeof m / sizeof m[0]; i++) {
		m[i] = false;
		goal[i] = NO_GOAL;
	}
	if (propagators.empty()) {
		propagators[4] = Propagator(4, NUM_TOPOS_4, transition_data_4, transition_starts_4);
		propagators[5] = Propagator(5, NUM_TOPOS_5, transition_data_5, transition_starts_5);
	}
}

class FixTheFence {
public:
	string findLoop(vector<string> data) {
		init();
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

		for (int y = 0; y < whole.h; y++) {
			int pt = whole.coords_to_index(whole.w/2, y);
			m[pt] = true;
		}

		//whole.show();

		for (int pt = 0; ; pt++)
			if (m[pt]) {
				Coords start = whole.index_to_coords(pt+1);
				Coords finish;
				string path = whole.trace_path(pt+1, RIGHT, finish);
				assert(finish == start);

				ostringstream out;
				out << start.second << " " << start.first << " " << path;
				return out.str();
			}

		cerr << "FAIL" << endl;
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
