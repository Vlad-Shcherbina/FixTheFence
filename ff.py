import sys
import random
import time
import StringIO

#sys.stderr = StringIO.StringIO()

STRIP_WIDTH = 4

TIME_LIMIT = 9.5

STATE_CUTOFF = 10000

PRINT_GRAPH = False



random.seed(1)
num_changes = 0

dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]


def trace_path(block, pt, prev_d):
    start = pt

    m = block.m
    stride = block.stride

    up = -stride
    down = stride
    left = -1
    right = 1

    path = []
    d = prev_d

    while True:
        ul = m[pt-stride-1]
        ur = m[pt-stride]
        dl = m[pt-1]
        dr = m[pt]

        assert not ul != ur != dr != dl

        if ul != ur and d != down:
            d = up
            path.append('U')
        elif dl != dr and d != up:
            d = down
            path.append('D')
        elif ul != dl and d != right:
            d = left
            path.append('L')
        elif ur != dr and d != left:
            d = right
            path.append('R')
        else:
            assert False

        pt += d
        if pt == start:
            break

        x, y = block.index_to_coords(pt)
        if x < 0 or x > block.w or y < 0 or y > block.h:
            break

    return ''.join(path), pt


class Block(object):
    def __init__(self, m, goal, stride, offset, w, h):
        self.m = m
        self.goal = goal
        self.stride = stride
        self.offset = offset
        self.w = w
        self.h = h

        self.undo = []
        self.best_score = self.get_score()

    @staticmethod
    def make_empty(w, h):
        m = [False] * (w+2)*(h+2)
        m[w+3] = True
        goal = [None] * (w+2)*(h+2)
        return Block(m, goal, w+2, w+3, w, h)

    def update_best(self, score):
        if self.best_score <= score:
            self.best_score = score
            self.undo = []

    def undo_to_best(self):
        m = self.m
        for pt in self.undo:
            m[pt] = not m[pt]
        self.undo = []

    def get_subblock(self, x1, y1, x2, y2):
        assert 0 <= x1 <= x2 <= self.w
        assert 0 <= y1 <= y2 <= self.h

        return Block(
            self.m, self.goal,
            stride=self.stride,
            offset=self.offset+x1+y1*self.stride,
            w=x2-x1, h=y2-y1)

    def coords_to_index(self, x, y):
        return self.offset + self.stride*y + x

    def index_to_coords(self, pt):
        # plus one and minus one in order to get negative coords for points
        # slightly to the left of the block
        y, x = divmod(pt-self.offset+1, self.stride)
        x -= 1
        return x, y

    def can_change(self, pt):
        m = self.m
        stride = self.stride
        if m[pt+1+stride] != m[pt+stride] == m[pt+1]: return False
        if m[pt-1+stride] != m[pt+stride] == m[pt-1]: return False
        if m[pt+1-stride] != m[pt-stride] == m[pt+1]: return False
        if m[pt-1-stride] != m[pt-stride] == m[pt-1]: return False
        return m[pt-1] != m[pt+1] or m[pt-stride] != m[pt+stride]

    def enum_points(self):
        stride = self.stride
        for y in range(self.h):
            for pt in xrange(self.offset+stride*y, self.offset+stride*y+self.w):
                yield pt

    def get_score(self):
        stride = self.stride
        m = self.m
        goal = self.goal
        result = 0
        for x in range(-1, self.w+1):
            for y in range(-1, self.h+1):
                pt = self.coords_to_index(x, y)
                g = goal[pt]
                if g is not None:
                    c = m[pt]
                    count = (
                        (c != m[pt-1]) +
                        (c != m[pt+1]) +
                        (c != m[pt+stride]) +
                        (c != m[pt-stride]))
                    result += count == g
        return result

    def score_diff(self, pt):
        m = self.m
        goal = self.goal
        stride = self.stride

        result = 0
        c = m[pt]
        g = goal[pt]
        if g is not None:
            count = (
                (c != m[pt-1]) +
                (c != m[pt+1]) +
                (c != m[pt+stride]) +
                (c != m[pt-stride]))
            result += (4-count) == g
            result -= count == g
        for fwd, side in [(1, stride), (-1, stride), (stride, 1), (-stride, 1)]:
            g = goal[pt+fwd]
            if g is not None:
                cc = m[pt+fwd]
                count = (
                    (cc != m[pt+fwd+fwd]) +
                    (cc != m[pt+fwd+side]) +
                    (cc != m[pt+fwd-side]))
                result += count + (c == cc) == g
                result -= count + (c != cc) == g
        return result

    def change(self, pt):
        global num_changes
        num_changes += 1
        m = self.m
        undo = self.undo

        m[pt] = not m[pt]

        if undo and undo[-1] == pt:
            undo.pop()
            return

        undo.append(pt)
        if len(undo) > self.w*self.h*3:
            undo_set = set()
            for pt in undo:
                if pt in undo_set:
                    undo_set.remove(pt)
                else:
                    undo_set.add(pt)
            undo[:] = list(undo_set)

    def show(self):
        for y in range(self.h):
            for x in range(self.w):
                if self.m[self.coords_to_index(x, y)]:
                    print>>sys.stderr, '*',
                else:
                    print>>sys.stderr, '.',
            print>>sys.stderr, '   ',
            for x in range(self.w):
                g = self.goal[self.coords_to_index(x, y)]
                if g is None:
                    g = '-'
                print>>sys.stderr, g,
            print>>sys.stderr

    def transpose(self):
        assert self.w == self.h
        m = self.m
        goal = self.goal
        for x in range(-1, self.w+1):
            for y in range(-1, x):
                pt1 = self.coords_to_index(x, y)
                pt2 = self.coords_to_index(y, x)
                m[pt1], m[pt2] = m[pt2], m[pt1]
                goal[pt1], goal[pt2] = goal[pt2], goal[pt1]


def compute_paths(block):
    pairs = []
    for y in range(block.h+1):
        x = -1
        pt = block.coords_to_index(x, y)
        if block.m[pt] != block.m[pt-block.stride]:
            _, endpoint = trace_path(block, pt+1, 1)
            end_x, end_y = block.index_to_coords(endpoint)
            pairs.append(((x, y), (end_x, end_y)))
    for x in range(block.w+1):
        y = -1
        pt = block.coords_to_index(x, y)
        if block.m[pt] != block.m[pt-1]:
            _, endpoint = trace_path(block, pt+block.stride, block.stride)
            end_x, end_y = block.index_to_coords(endpoint)
            pairs.append(((x, y), (end_x, end_y)))
        y = block.h+1
        pt = block.coords_to_index(x, y)
        if block.m[pt-block.stride] != block.m[pt-block.stride-1]:
            _, endpoint = trace_path(block, pt-block.stride, -block.stride)
            end_x, end_y = block.index_to_coords(endpoint)
            pairs.append(((x, y), (end_x, end_y)))
    for y in range(block.h+1):
        x = block.w+1
        pt = block.coords_to_index(x, y)
        if block.m[pt-1] != block.m[pt-block.stride-1]:
            _, endpoint = trace_path(block, pt-1, -1)
            end_x, end_y = block.index_to_coords(endpoint)
            pairs.append(((x, y), (end_x, end_y)))

    paths = dict(pairs)
    for (x, y), (end_x, end_y) in pairs:
        assert paths[end_x, end_y] == (x, y)

    return paths


def bits_compatible(bits1, bits2):
    return not bool(
        (bits1 ^ bits2) &
        (bits1 ^ (bits1 << 1)) &
        (bits2 ^ (bits2 << 1)))


UP_CONN = -1
DOWN_CONN = 999
IN = object()
OUT = object()
BARRIER = object()
GATE_NAME = {None: None, IN: 'IN', OUT: 'OUT', BARRIER: 'BARRIER'}

class Propagator(object):
    def __init__(self, h):
        start = time.clock()
        self.h = h

        self.topo_index = {}
        self.index_topo = []
        self.topo_bits = []
        def rec(topo, stack):
            if len(topo) == self.h+1:
                if stack == []:
                    # fix opening brackets
                    topo = topo[:]
                    for i, e in enumerate(topo):
                        if e is not None and len(e) > 1:
                            if e.startswith('[') or e.startswith(']'):
                                q = int(e[1:])
                                assert topo[q] == e[0]
                                topo[q] += str(i)
                    topo = tuple(topo)
                    assert topo not in self.topo_index
                    self.topo_index[topo] = len(self.topo_index)
                    self.index_topo.append(topo)
                    self.topo_bits.append(self.bits_from_topo(topo))
                return
            rec(topo+[None], stack)
            if stack == []:
                rec(topo+['global'], [])
            if stack:
                rec(topo+stack[-1:], stack[:-1])
            if stack == [] or stack[-1].startswith('['):
                rec(topo+['['], stack+['['+str(len(topo))])
            if stack == [] or stack[-1].startswith(']'):
                rec(topo+[']'], stack+[']'+str(len(topo))])
        rec([], [])

        cnt = 0
        self.transition_table = []
        self.back_transitions = [{} for _ in self.index_topo]
        for index, topo in enumerate(self.index_topo):
            if index % 100 == 0:
                print>>sys.stderr, 100*index/len(self.index_topo), '%'
            t = {}
            self.transition_table.append(t)
            for (xor_bits, up_gate, down_gate), new_topo in self.enumerate_transitions(topo):
                tt = t.setdefault((up_gate, down_gate), {})
                assert xor_bits not in tt
                tt[xor_bits] = self.topo_index[new_topo]

                tt = self.back_transitions[self.topo_index[new_topo]].setdefault((up_gate, down_gate), [])
                tt.append(index)
                cnt += 1

        print>>sys.stderr, 'Propagator(%d) initialization took %s' % (self.h, time.clock() - start)
        print>>sys.stderr, len(self.index_topo), 'topos'
        print>>sys.stderr, cnt, 'transitions'

    def bits_from_topo(self, topo):
        result = 0
        q = False
        for i, t in enumerate(topo):
            if t is not None:
                q = not q
            if q:
                result |= 1 << (i+1)
        return result

    def get_connections(self, prev_bits, next_bits):
        assert 0 <= prev_bits <= 4 << self.h
        assert 0 <= next_bits <= 4 << self.h
        assert bits_compatible(prev_bits, next_bits)

        UL = 7 # 0b0111
        UR = 4 # 0b0100
        DL = 2 # 0b0010
        DR = 1 # 0b0001
        HOR = 3 # 0b0011
        VER = 5 # 0b0101

        if (prev_bits ^ next_bits) & 1:
            z = UP_CONN
        else:
            z = None

        connections = []
        for y in range(self.h+1):
            mask = ((prev_bits >> y) & 1) * 7  # 0b0111
            mask ^= ((next_bits >> y) & 1) << 2
            mask ^= ((prev_bits >> (y+1)) & 1) << 1
            mask ^= ((next_bits >> (y+1)) & 1) << 0
            assert mask not in [6, 9]  # [0b0110, 0b1001]

            if mask == VER:
                assert z is not None
                z = z
            elif mask == HOR:
                assert z is None
                connections.append((y, y+100))
                z = None
            elif mask == UL:
                assert z is not None
                connections.append((z, y))
                z = None
            elif mask == UR:
                assert z is not None
                connections.append((z, y+100))
                z = None
            elif mask == DL:
                assert z is None
                z = y
            elif mask == DR:
                assert z is None
                z = y+100
            elif mask == 0:
                pass
            else:
                assert False, bin(mask)

        if (prev_bits ^ next_bits) & (2 << self.h):
            assert z is not None
            connections.append((z, DOWN_CONN))
        else:
            assert z is None

        return connections

    def next_topo(self, topo, connections, up_gate=None, down_gate=None):
        conn_dict = dict(connections + map(reversed, connections))
        visited = set()
        def trace(q):
            q = conn_dict[q]
            if 0 <= q < 100 and topo[q].startswith('['):
                visited.add(q)
                qq = int(topo[q][1:])
                visited.add(qq)
                return trace(qq)
            else:
                return q

        # incorporate up_gate and down_gate into topo
        list_topo = list(topo)
        topo = dict(enumerate(topo))
        if up_gate is OUT:
            if 'global' not in list_topo:
                return
            i = list_topo.index('global')
            assert topo[i] == 'global'
            list_topo[i] = topo[i] = ']'+str(UP_CONN)
            topo[UP_CONN] = ']'+str(i)
        elif up_gate is IN:
            topo[UP_CONN] = 'global'
        elif up_gate is BARRIER:
            topo[UP_CONN] = ']'+str(DOWN_CONN)

        if down_gate is OUT:
            if 'global' not in list_topo:
                return
            i = self.h - list_topo[::-1].index('global')
            assert topo[i] == 'global'
            list_topo[i] = topo[i] = ']'+str(DOWN_CONN)
            topo[DOWN_CONN] = ']'+str(i)
        elif down_gate is IN:
            topo[DOWN_CONN] = 'global'
        elif down_gate is BARRIER:
            topo[DOWN_CONN] = ']'+str(UP_CONN)

        new_topo = ['uninitialized'] * (self.h+1)
        for start, gate in topo.items():
            if gate is None or gate.startswith('['):
                continue
            end = trace(start)
            if gate == 'global':
                if 100 <= end < 200:
                    new_topo[end-100] = 'global'
                else:
                    return
            elif gate.startswith(']'):
                counterpart_start = int(gate[1:])
                if counterpart_start > start:
                    counterpart_end = trace(counterpart_start)
                    if end == counterpart_start:
                        assert start == counterpart_end
                    else:
                        if not 100 <= end < 200:
                            return
                        if not 100 <= counterpart_end < 200:
                            return
                        assert end < counterpart_end

                        orig = end
                        i = end+1
                        while True:
                            if i-100 > self.h:
                                return
                            if i not in conn_dict:
                                i += 1
                                continue
                            if i in conn_dict and i != counterpart_end:
                                i2 = trace(i)
                                if 0 <= i2 < 100 and topo[i2].startswith(']'):
                                    i2 = trace(int(topo[i2][1:]))
                                    if not 100 <= i2 < 200:
                                        return
                                    i = i2+1
                                    continue

                            new_topo[orig-100] = ']'+str(i-100)
                            new_topo[i-100] = ']'+str(orig-100)

                            if i == counterpart_end:
                                break

                            i = trace(i)
                            if not 100 <= i < 200:
                                return
                            assert i <= counterpart_end

                            orig = i
                            i = i+1

        for y in range(self.h+1):
            if new_topo[y] != 'uninitialized':
                continue
            if y+100 not in conn_dict:
                new_topo[y] = None
            else:
                end = trace(y+100)
                if not 100 <= end < 200:
                    return
                new_topo[y] = '['+str(end-100)
        for y in range(self.h+1):
            if topo[y] is not None and topo[y].startswith('[') and y not in visited:
                return

        assert 'uninitialized' not in new_topo
        for y in range(self.h+1):
            assert (new_topo[y] is None) == (y+100 not in conn_dict)

        return tuple(new_topo)

    def enumerate_transitions(self, topo):
        bits = self.bits_from_topo(topo)
        gate_pairs = [
            (None, None), (None, IN), (None, OUT), (IN, None), (OUT, None),
            (IN, IN), (IN, OUT), (OUT, IN), (OUT, OUT),
            (BARRIER, BARRIER)]
        for up_gate, down_gate in gate_pairs:
            for xor_bits in range(0, 2 << self.h, 2):
                if up_gate is not None:
                    xor_bits |= 1
                if down_gate is not None:
                    xor_bits |= 2 << self.h
                new_bits = bits ^ xor_bits
                if not bits_compatible(bits, new_bits):
                    continue
                conns = self.get_connections(bits, new_bits)
                new_topo = self.next_topo(topo, conns, up_gate, down_gate)

                if new_topo is not None:
                    yield (xor_bits, up_gate, down_gate), new_topo


def num_bits(n):
    cnt = 0
    while n > 0:
        n &= n-1
        cnt += 1
    return cnt


prop_cache = {}

def dynamic(block, end_time=None):
    initial_score = block.get_score()

    paths = compute_paths(block)

    if len(paths) == 0:
        print>>sys.stderr, 'dynamic: block with no paths'
        return

    start_topo = [None]*(block.h+1)
    finish_topo = [None]*(block.h+1)
    for y in range(block.h+1):
        if (-1, y) in paths:
            end_x, end_y = paths[-1, y]
            if end_x == -1:
                start_topo[y] = ']' + str(end_y)
            else:
                start_topo[y] = 'global'

        if (block.w+1, y) in paths:
            end_x, end_y = paths[block.w+1, y]
            if end_x == block.w+1:
                finish_topo[y] = '[' + str(end_y)
            else:
                finish_topo[y] = 'global'

    prop = prop_cache.get(block.h)
    if prop is None:
        prop = prop_cache[block.h] = Propagator(block.h)

    start_topo = prop.topo_index[tuple(start_topo)]
    finish_topo = prop.topo_index[tuple(finish_topo)]

    start_bonus = 0
    start_penalty = 0

    # left goals
    for y in range(-1, block.h+1):
        pt = block.coords_to_index(-1, y)
        g = block.goal[pt]
        if g is not None:
            num_neighbors = (
                (block.m[pt] != block.m[pt-1])+
                (block.m[pt] != block.m[pt-block.stride])+
                (block.m[pt] != block.m[pt+block.stride]))
            if g == num_neighbors:
                start_penalty |= 1 << (y+1)
            elif g == num_neighbors+1:
                start_bonus |= 1 << (y+1)

    num_bits_ = map(num_bits, range(4 << block.h))

    gate_pairs = {}
    for x in range(block.w+1):
        up_gate = down_gate = None
        if (x, -1) in paths:
            end_x, end_y = paths[x, -1]
            if end_x < x:
                up_gate = OUT
            elif end_x > x:
                up_gate = IN
            else:
                up_gate = BARRIER
        if (x, block.h+1) in paths:
            end_x, end_y = paths[x, block.h+1]
            if end_x < x:
                down_gate = OUT
            elif end_x > x:
                down_gate = IN
            else:
                down_gate = BARRIER
        gate_pairs[x] = (up_gate, down_gate)

    reachable = [None] * (block.w+1)
    reachable[block.w] = set([finish_topo])
    for x in reversed(range(block.w)):
        s = reachable[x] = set()
        for t in reachable[x+1]:
            s.update(prop.back_transitions[t].get(gate_pairs[x+1], []))
    #for x in range(block.w+1):
    #    print len(reachable[x]),
    #print
    #raw_input()

    states = []
    states.append({(start_topo, start_bonus, start_penalty): (0, ())})
    for x in range(block.w+1):
        if end_time is not None and time.clock() > end_time:
            return
        states.append({})

        # up and down goals
        up_bonus = False
        up_penalty = False
        pt = block.coords_to_index(x-1, -1)
        g = block.goal[pt]
        if g is not None:
            num_neighbors = bool(block.m[pt] != block.m[pt-1])
            num_neighbors += bool(block.m[pt] != block.m[pt+1])
            num_neighbors += bool(block.m[pt] != block.m[pt-block.stride])
            if num_neighbors+1 == g:
                up_bonus = True
            elif num_neighbors == g:
                up_penalty = True
        down_bonus = False
        down_penalty = False
        pt = block.coords_to_index(x-1, block.h)
        g = block.goal[pt]
        if g is not None:
            num_neighbors = bool(block.m[pt] != block.m[pt-1])
            num_neighbors += bool(block.m[pt] != block.m[pt+1])
            num_neighbors += bool(block.m[pt] != block.m[pt+block.stride])
            if num_neighbors+1 == g:
                down_bonus = True
            elif num_neighbors == g:
                down_penalty = True

        goal0 = 0
        goal1 = 0
        goal2 = 0
        goal3 = 0
        for y in range(block.h):
            pt = block.coords_to_index(x, y)
            g = block.goal[pt]
            if g == 0:
                goal0 |= 2 << y
            elif g == 1:
                goal1 |= 2 << y
            elif g == 2:
                goal2 |= 2 << y
            elif g == 3:
                goal3 |= 2 << y

        num_states = 0
        num_transitions = 0

        for (topo, bonus, penalty), (cost, sol) in states[x].items():
            num_states += 1
            if PRINT_GRAPH:
                print>>sys.stderr, x, prop.index_topo[topo], bin(bonus)[::-1], bin(penalty)[::-1], cost

            bits = prop.topo_bits[topo]
            if up_bonus:
                if (bits ^ (bits >> 1)) & 1:
                    cost += 1
            elif up_penalty:
                if not ((bits ^ (bits >> 1)) & 1):
                    cost += 1
            if down_bonus:
                if (bits ^ (bits >> 1)) & (1 << block.h):
                    cost += 1
            elif down_penalty:
                if not ((bits ^ (bits >> 1)) & (1 << block.h)):
                    cost += 1

            bad_entry = (-1000000, None)

            zzz = prop.transition_table[topo].get(gate_pairs[x], {})
            for xor_bits, new_topo in zzz.items():
                if new_topo not in reachable[x]:
                    continue

                new_cost = cost
                new_cost += num_bits_[xor_bits & bonus]
                new_cost += num_bits_[~xor_bits & penalty]

                new_bits = bits ^ xor_bits

                neib_up = new_bits ^ (new_bits << 1)
                neib_down = neib_up >> 1
                neib_left = xor_bits

                n0 = ~neib_left
                n1 = neib_left

                n2 = n1 & neib_up
                n1 = (n1 & ~neib_up) | (n0 & neib_up)
                n0 &= ~neib_up

                n3 = n2 & neib_down
                n2 = (n2 & ~neib_down) | (n1 & neib_down)
                n1 = (n1 & ~neib_down) | (n0 & neib_down)
                n0 &= ~neib_down

                new_bonus = (n0 & goal1) | (n1 & goal2) | (n2 & goal3)
                new_penalty = (n0 & goal0) | (n1 & goal1) | (n2 & goal2) | (n3 & goal3)

                new_state = new_topo, new_bonus, new_penalty
                if PRINT_GRAPH:
                    print>>sys.stderr, '  ->', bin(xor_bits)[::-1], prop.index_topo[new_topo], bin(new_bonus)[::-1], bin(new_penalty)[::-1], new_cost

                num_transitions += 1
                qcost = states[x+1].get(new_state, bad_entry)[0]
                if new_cost > qcost:
                    states[x+1][new_state] = (new_cost, (sol, xor_bits))

        qq = states[x+1].items()
        qq.sort(key=lambda e: -e[1][0])
        qq = qq[:STATE_CUTOFF]
        states[x+1] = dict(qq)
        #print>>sys.stderr, '%s(%s)'%(num_states, num_transitions),
    #print>>sys.stderr

    #assert len(states[block.w+1]) == 1
    for (topo, bonus, penalty), (cost, sol) in states[block.w+1].items():
        if topo == finish_topo:
            # TODO: right goals

            bits = prop.bits_from_topo(prop.index_topo[start_topo])
            if block.m[block.coords_to_index(-1, -1)]:
                bits ^= (4 << block.h) - 1

            solution = []
            while sol != ():
                sol, x = sol
                solution.append(x)
            solution.reverse()
            for x, xor_bits in enumerate(solution):
                bits ^= xor_bits
                for y in range(block.h):
                    desired = bool(bits & (2 << y))
                    pt = block.coords_to_index(x, y)

                    if block.m[pt] != desired:
                        block.change(pt)
            break
    else:
        print>>sys.stderr, 'crucial states were truncated :('
        return
        assert False

    delta = block.get_score() - initial_score
    print>>sys.stderr, 'delta', delta
    assert delta >= 0


class FixTheFence(object):
    def findLoop(self, diagram):
        start = time.clock()

        h = len(diagram)
        w = len(diagram[0])

        whole = Block.make_empty(w, h)

        for x in range(w):
            for y in range(h):
                if diagram[y][x] != '-':
                    whole.goal[whole.coords_to_index(x, y)] = int(diagram[y][x])


        whole.change(whole.coords_to_index(0, 0))
        for y in range(h):
            whole.change(whole.coords_to_index(x/2, y))

        transposed = False
        for i in xrange(10**6):
            if time.clock() > start + TIME_LIMIT:
                break
            print>>sys.stderr, '---'
            offset = 0
            if i < 2:
                offset = 0
            elif i < 4:
                offset = whole.h % STRIP_WIDTH
                if offset == 0:
                    offset = random.randrange(1, STRIP_WIDTH)
            else:
                offset = random.randrange(STRIP_WIDTH)
            rng = range(offset, whole.h-STRIP_WIDTH+1, STRIP_WIDTH)
            if random.random() < 0.5:
                rng = reversed(rng)
            for y in rng:
                if time.clock() > start + TIME_LIMIT:
                    break
                sub = whole.get_subblock(0, y, whole.w, y+STRIP_WIDTH)
                dynamic(sub, end_time=start + TIME_LIMIT)
            whole.transpose()
            transposed = not transposed

        if transposed:
            whole.transpose()

        score = whole.get_score()
        print>>sys.stderr, 'final', score
        total_goals = sum(1 for g in whole.goal if g is not None)
        if total_goals > 0:
            print>>sys.stderr, 'score:', 1.0*whole.get_score()/total_goals

        for pt in whole.enum_points():
            if whole.m[pt]:
                x, y = whole.index_to_coords(pt)
                path, finish = trace_path(whole, pt+1, 1)
                assert finish == pt+1

                overtime = time.clock() - (start+TIME_LIMIT)
                #assert overtime < 0.5, overtime
                return '%s %s %s' % (y, x+1, path)
        else:
            assert False, 'no stuff found'


def main():
    start = time.clock()
    size = int(raw_input())
    lines = [raw_input().strip() for _ in range(size)]
    result = FixTheFence().findLoop(lines)
    print>>sys.stderr, 'it took', time.clock()-start
    print>>sys.stderr, num_changes / (time.clock()-start), 'changes per second'
    print result


if __name__ == '__main__':
    main()