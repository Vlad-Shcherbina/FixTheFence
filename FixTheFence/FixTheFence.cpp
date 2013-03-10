#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <sstream>
#include <map>
#include <algorithm>
#include <iterator>
#include <ctime>

#include "consts.h"

using namespace std;


const float TIME_LIMIT = 9.5;


typedef int Goal;

const int STRIDE = 256;
const int MAX_SIZE = 100;
const Goal NO_GOAL = 5;

const int UP = -STRIDE;
const int DOWN = STRIDE;
const int LEFT = -1;
const int RIGHT = 1;


bool m[STRIDE*(MAX_SIZE+2)] = {false};
Goal goal[STRIDE*(MAX_SIZE+2)];


typedef pair<int, int> Coords;


class Block {
	int offset;

public:
	int w, h;

	Block(int offset, int w, int h) : offset(offset), w(w), h(h) {}

	Block get_subblock(int x1, int y1, int x2, int y2) const {
		assert(0 <= x1 && x1 <= x2 && x2 <= w);
		assert(0 <= y1 && y1 <= y2 && y2 <= h);
		return Block(offset + x1 + y1 * STRIDE, x2-x1, y2-y1);
	}

	inline int coords_to_index(int x, int y) const {
		return offset + x + STRIDE*y;
	}

	inline Coords index_to_coords(int pt) const {
		return Coords(pt % STRIDE - offset % STRIDE, pt / STRIDE - offset / STRIDE);
	}

	void show() const {
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

	string trace_path(int pt, int prev_d, Coords &finish) const {
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

	void transpose() {
		assert(w == h);
		for (int y = -1; y <= h; y++)
			for (int x = -1; x < y; x++) {
				int pt1 = coords_to_index(x, y);
				int pt2 = coords_to_index(y, x);
				bool tm = m[pt1]; m[pt1] = m[pt2]; m[pt2] = tm;
				Goal tg = goal[pt1]; goal[pt1] = goal[pt2]; goal[pt2] = tg;
			}
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
	const Bits *topo_bits;
	vector<map<GatePair, vector<pair<Bits, Topo> > > > transitions;
	vector<map<GatePair, vector<Topo> > > back_transitions;
	Propagator() {}
	Propagator(
			int w, int num_topos,
			const unsigned topo_bits[],
			const unsigned transition_data[],
			const int transitions_starts[]) {
		this->w = w;
		this->num_topos = num_topos;
		this->topo_bits = topo_bits;
		transitions.resize(num_topos);
		back_transitions.resize(num_topos);
		const int *start = transitions_starts;
		for (int i = 0; i < num_topos; i++) {
			for (int j = 0; j < NUM_GATE_PAIRS; j++) {
				vector<pair<Bits, Topo> > &ts = transitions[i][GATE_PAIRS[j]];
				for (int k = start[0]; k < start[1]; k++) {
					Bits bits = transition_data[k] & 255;
					Topo new_topo = transition_data[k] >> 8;
					ts.push_back(make_pair(bits, new_topo));
					back_transitions[new_topo][GATE_PAIRS[j]].push_back(i);
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
		propagators[4] = Propagator(4, NUM_TOPOS_4, topo_bits_4, transition_data_4, transition_starts_4);
		propagators[5] = Propagator(5, NUM_TOPOS_5, topo_bits_5, transition_data_5, transition_starts_5);
	}
}

map<Coords, Coords> compute_paths(const Block &block) {
	map<Coords, Coords> result;

	for (int y = 0; y <= block.h; y++) {
		int x = -1;
		int pt = block.coords_to_index(x, y);
		if (m[pt] != m[pt-STRIDE]) {
			Coords endpoint;
			block.trace_path(pt+1, RIGHT, endpoint);
			result[Coords(x, y)] = endpoint;
		}
		x = block.w+1;
		pt = block.coords_to_index(x, y);
		if (m[pt-1] != m[pt-STRIDE-1]) {
			Coords endpoint;
			block.trace_path(pt-1, LEFT, endpoint);
			result[Coords(x, y)] = endpoint;
		}
	}
	for (int x = 0; x <= block.w; x++) {
		int y = -1;
		int pt = block.coords_to_index(x, y);
		if (m[pt] != m[pt-1]) {
			Coords endpoint;
			block.trace_path(pt+STRIDE, DOWN, endpoint);
			result[Coords(x, y)] = endpoint;
		}
		y = block.h+1;
		pt = block.coords_to_index(x, y);
		if (m[pt-STRIDE] != m[pt-STRIDE-1]) {
			Coords endpoint;
			block.trace_path(pt-STRIDE, UP, endpoint);
			result[Coords(x, y)] = endpoint;
		}
	}
	return result;
}


struct Node {
	Bits data;
	int refcount;
	Node *next;
};

Node *free_node = NULL;

void incref(Node *node) {
	node->refcount++;
}

void decref(Node *node) {
	while (--node->refcount == 0) {
		Node *t = node->next;
		node->next = free_node;
		free_node = node;
		node = t;
		if (node == NULL)
			return;
	}
}

Node *new_node() {
	if (free_node == NULL) {
		cerr << "allocating another chunk of nodes" << endl;
		const int n = 4096;
		Node *nodes = new Node[n];
		for (int i = 0; i < n-1; i++)
			nodes[i].next = &nodes[i+1];
		nodes[n-1].next = NULL;
		free_node = &nodes[0];
	}
	Node *node = free_node;
	free_node = free_node->next;
	node->refcount = 1;
	node->next = NULL;
	return node;
}


int num_bits(Bits x) {
	int result = 0;
	while (x) {
		x &= x-1;
		result++;
	}
	return result;
}


typedef unsigned State;
typedef int Cost;

struct Entry {
	State state;
	Cost cost;
	Node *solution;
};

typedef vector<Entry> StatesCut;


void dynamic(const Block &block) {
	map<Coords, Coords> paths = compute_paths(block);
	if (paths.empty()) {
		cerr << "block with no ins and outs" << endl;
		return;
	}
	Topo start_topo = 0;
	Topo finish_topo = 0;
	Bits start_bonus = 0;
	Bits start_penalty = 0;

	assert(propagators.find(block.w) != propagators.end());
	Propagator &prop = propagators[block.w];

	vector<GatePair> gate_pairs;
	for (int y = 0; y <= block.h; y++) {
		Gate left_gate = NONE, right_gate = NONE;
		if (paths.find(Coords(-1, y)) != paths.end()) {
			Coords end = paths[Coords(-1, y)];
			if (end.second < y)
				left_gate = OUT;
			else if (end.second > y)
				left_gate = IN;
			else
				left_gate = BARRIER;
		}
		if (paths.find(Coords(block.w+1, y)) != paths.end()) {
			Coords end = paths[Coords(block.w+1, y)];
			if (end.second < y)
				right_gate = OUT;
			else if (end.second > y)
				right_gate = IN;
			else
				right_gate = BARRIER;
		}
		gate_pairs.push_back(make_pair(left_gate, right_gate));
	}

	static bool reachable[MAX_SIZE+1][NUM_TOPOS_5];
	for (int i = 1; i < prop.num_topos; i++)
		reachable[block.h][i] = false;
	reachable[block.h][0] = true;
	for (int y = block.h-1; y >= 0; y--) {
		for (int i = 0; i < prop.num_topos; i++)
			reachable[y][i] = false;
		for (int i = 0; i < prop.num_topos; i++) {
			if (!reachable[y+1][i]) continue;
			vector<Topo> &back = prop.back_transitions[i][gate_pairs[y+1]];
			for (int j = 0; j < back.size(); j++)
				reachable[y][back[j]] = true;
		}
	}

	Entry empty_entry;
	empty_entry.solution = NULL;
	empty_entry.cost = -1;

	vector<StatesCut> states(2, StatesCut(2001, empty_entry));
	Node *empty_sol = new_node();
	empty_sol->data = 0;
	states[0][0].cost = 0;
	states[0][0].solution = empty_sol;
	states[0][0].state = 0;

	for (int y = 0; y <= block.h; y++) {
		StatesCut &cur_states = states[y%2];
		StatesCut &next_states = states[(y+1)%2];

		bool left_bonus = false;
		bool left_penalty = false;
		int pt = block.coords_to_index(-1, y-1);
		Goal g = goal[pt];
		if (g != NO_GOAL) {
			int num_neighbors = 0;
			if (m[pt] != m[pt-STRIDE]) num_neighbors++;
			if (m[pt] != m[pt+STRIDE]) num_neighbors++;
			if (m[pt] != m[pt-1]) num_neighbors++;
			if (num_neighbors == g)
				left_penalty = true;
			else if (num_neighbors+1 == g)
				left_bonus = true;
		}

		bool right_bonus = false;
		bool right_penalty = false;
		pt = block.coords_to_index(block.w, y-1);
		g = goal[pt];
		if (g != NO_GOAL) {
			int num_neighbors = 0;
			if (m[pt] != m[pt-STRIDE]) num_neighbors++;
			if (m[pt] != m[pt+STRIDE]) num_neighbors++;
			if (m[pt] != m[pt+1]) num_neighbors++;
			if (num_neighbors == g)
				right_penalty = true;
			else if (num_neighbors+1 == g)
				right_bonus = true;
		}

		Bits goal0 = 0, goal1 = 0, goal2 = 0, goal3 = 0;
		for (int x = 0; x < block.w; x++) {
			Goal g = goal[block.coords_to_index(x, y)];
			if (g == 0)
				goal0 |= 2 << x;
			else if (g == 1)
				goal1 |= 2 << x;
			else if (g == 2)
				goal2 |= 2 << x;
			else if (g == 3)
				goal3 |= 2 << x;
		}
		
		for (StatesCut::iterator it = cur_states.begin(); it != cur_states.end(); ++it) {
			Cost cost = it->cost;
			if (cost == -1)
				continue;
			Node *sol = it->solution;
			State state = it->state;
			Topo topo = state >> 16;
			Bits bonus = (state >> 8) & 255;
			Bits penalty = state & 255;
			Bits bits = prop.topo_bits[topo];


			if (left_bonus)
				if ((bits ^ (bits >> 1)) & 1)
					cost += 1;
			if (left_penalty)
				if (!((bits ^ (bits >> 1)) & 1))
					cost += 1;
			if (right_bonus)
				if ((bits ^ (bits >> 1)) & (1 << block.w))
					cost += 1;
			if (right_penalty)
				if (!((bits ^ (bits >> 1)) & (1 << block.w)))
					cost += 1;
			
			vector<pair<Bits, Topo> > &zzz = prop.transitions[topo][gate_pairs[y]];
			for (vector<pair<Bits, Topo> >::iterator new_it = zzz.begin(); new_it != zzz.end(); new_it++) {
				Topo new_topo = new_it->second;
				if (!reachable[y][new_topo])
					continue;
				Bits xor_bits = new_it->first;
				Cost new_cost = cost;
				new_cost += num_bits(xor_bits & bonus);
				new_cost += num_bits((~xor_bits) & penalty);
				
				Bits new_bits = bits ^ xor_bits;

				Bits neib_up = new_bits ^ (new_bits << 1);
                Bits neib_down = neib_up >> 1;
                Bits neib_left = xor_bits;

                Bits n0 = ~neib_left;
                Bits n1 = neib_left;

                Bits n2 = n1 & neib_up;
                n1 = (n1 & ~neib_up) | (n0 & neib_up);
                n0 &= ~neib_up;

                Bits n3 = n2 & neib_down;
                n2 = (n2 & ~neib_down) | (n1 & neib_down);
                n1 = (n1 & ~neib_down) | (n0 & neib_down);
                n0 &= ~neib_down;
				
				Bits new_bonus = (n0 & goal1) | (n1 & goal2) | (n2 & goal3);
                Bits new_penalty = (n0 & goal0) | (n1 & goal1) | (n2 & goal2) | (n3 & goal3);

				State new_state = (new_topo << 16) | (new_bonus << 8) | new_penalty;
				
				Entry &new_entry = next_states[(new_state * 119) % next_states.size()];

				if (new_cost > new_entry.cost) {
					Node *node = new_node();
					node->data = xor_bits;
					node->next = sol;
					incref(sol);
					Node *old_solution = new_entry.solution;
					if (old_solution != NULL)
						decref(old_solution);
					new_entry.cost = new_cost;
					new_entry.state = new_state;
					new_entry.solution = node;
				}
			}
		}
		for (StatesCut::iterator it = cur_states.begin(); it != cur_states.end(); ++it) {
			Node *sol = it->solution;
			if (sol != NULL)
				decref(sol);
			it->solution = NULL;
			it->cost = -1;
		}
		//cur_states.clear();
	}

	StatesCut &final_states = states[(block.h+1)%2];
	for (StatesCut::iterator it = final_states.begin(); it != final_states.end(); ++it) {
		State state = it->state;
		Topo topo = state >> 16;
		if (topo != 0) continue;

		Bits bonus = (state >> 8) & 255;
		Bits penalty = state & 255;

		assert(bonus == 0);
		assert(penalty == 0);

		Node *sol = it->solution;

		Bits bits = 0;

		Cost cost = it->cost;
		//cerr << "found solution of cost " << cost << endl;

		vector<Bits> reversed_solution;
		sol = sol->next;
		while (sol->next != NULL) {
			reversed_solution.push_back(sol->data);
			sol = sol->next;
		}
		vector<Bits> solution(reversed_solution.rbegin(), reversed_solution.rend());
		assert(solution.size() == block.h);

		for (int y = 0; y < block.h; y++) {
			bits ^= solution[y];
			for (int x = 0; x < block.w; x++) {
				bool desired = (bits & (2 << x)) ? true : false;
				int pt = block.coords_to_index(x, y);
				if (m[pt] != desired)
					m[pt] = desired;
			}
		}
		break;
	}

	for (StatesCut::iterator it = final_states.begin(); it != final_states.end(); ++it) {
		Node *sol = it->solution;
		if (sol != NULL)
			decref(sol);
	}
}


class FixTheFence {
public:
	string findLoop(vector<string> data) {
		clock_t end_time = clock() + CLOCKS_PER_SEC * TIME_LIMIT;
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

		for (int x = 0; x < whole.w; x++) {
			int pt = whole.coords_to_index(x, whole.h/2);
			m[pt] = true;
		}

		const int strip_width = 5;


		bool transposed = false;

		int cnt = 0;

		while (true) {
			for (int i = 0; i+strip_width <= whole.w; i += 2) {
				if (clock() > end_time)
					break;
				Block block = whole.get_subblock(i, 0, i+strip_width, whole.h);
				//cerr << "*** " << i << endl;
				dynamic(block);
				cnt++;
			}
			if (clock() > end_time)
				break;
			whole.transpose();
			transposed = !transposed;
		}

		if (transposed)
			whole.transpose();

		cerr << cnt << " dynamics were performed" << endl;
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
