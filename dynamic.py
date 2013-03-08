import random
import copy
import time

from ff import *
from sampling import *
from simple_solvers import *


PRINT_GRAPH = False


random.seed(1)


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
        for index, topo in enumerate(self.index_topo):
            if index % 100 == 0:
                print>>sys.stderr, 100*index/len(self.index_topo), '%'
            t = {}
            self.transition_table.append(t)
            for (xor_bits, up_gate, down_gate), new_topo in self.enumerate_transitions(topo):
                tt = t.setdefault((up_gate, down_gate), {})
                assert xor_bits not in tt
                tt[xor_bits] = self.topo_index[new_topo]
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

        UL = 0b0111
        UR = 0b0100
        DL = 0b0010
        DR = 0b0001
        HOR = 0b0011
        VER = 0b0101

        if (prev_bits ^ next_bits) & 1:
            z = UP_CONN
        else:
            z = None

        connections = []
        for y in range(self.h+1):
            mask = ((prev_bits >> y) & 1) * (0b0111)
            mask ^= ((next_bits >> y) & 1) << 2
            mask ^= ((prev_bits >> (y+1)) & 1) << 1
            mask ^= ((next_bits >> (y+1)) & 1) << 0
            assert mask not in [0b0110, 0b1001]

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
        #print '-----------'
        conn_dict = dict(connections + map(reversed, connections))
        visited = set()
        def trace(q):
            q = conn_dict[q]
            if 0 <= q < 100 and topo[q].startswith('['):
                visited.add(q)
                qq = int(topo[q][1:])
                #assert qq not in visited
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

        #print 'topo with gates:', sorted(topo.items())

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

                        #print 'ends', end, counterpart_end
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

        #print 'filling the rest', new_topo
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
        #print 'filled'
        # prevent cycles
        for y in range(self.h+1):
            if topo[y] is not None and topo[y].startswith('[') and y not in visited:
                return
        #print 'no cycles'

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

def dynamic(block):
    #print>>sys.stderr, 'dynamic for block'
    #block.show()
    paths = compute_paths(block)

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

    #print>>sys.stderr, 'start topo', start_topo
    #print>>sys.stderr, 'finish topo', finish_topo

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

    states = []
    states.append({(start_topo, start_bonus, start_penalty): (0, ())})
    for x in range(block.w+1):
        #print>>sys.stderr, x, len(states[-1])
        states.append({})

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

        for (topo, bonus, penalty), (cost, sol) in states[x].items():
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

            zzz = prop.transition_table[topo].get((up_gate, down_gate), {})
            for xor_bits, new_topo in zzz.items():
                if x == block.w and new_topo != finish_topo:
                    continue

                new_cost = cost
                new_cost += num_bits(xor_bits & bonus)
                new_cost += num_bits((~xor_bits) & penalty)

                new_bits = bits ^ xor_bits

                neib_up = new_bits ^ (new_bits << 1)
                neib_down = new_bits ^ (new_bits >> 1)
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

                qcost, _ = states[x+1].get(new_state, (-10e10, None))
                if new_cost > qcost:
                    states[x+1][new_state] = (new_cost, (sol, xor_bits))

    for (topo, bonus, penalty), (cost, sol) in states[block.w+1].items():
        if topo == finish_topo:
            # TODO: right goals

            #print 'reached'
            #print prop.index_topo[topo], bin(bonus)[::-1], bin(penalty)[::-1], cost
            bits = prop.bits_from_topo(prop.index_topo[start_topo])
            if block.m[block.coords_to_index(-1, -1)]:
                bits ^= (4 << block.h) - 1
            #print bin(bits)[::-1]

            solution = []
            while sol != ():
                sol, x = sol
                solution.append(x)
            solution.reverse()
            for x, xor_bits in enumerate(solution):
                bits ^= xor_bits
                #print bin(bits)[::-1]
                for y in range(block.h):
                    desired = bool(bits & (2 << y))
                    pt = block.coords_to_index(x, y)

                    if block.m[pt] != desired:
                        block.change(pt)

            #print 'optimized block:'
            #block.show()
            break
    else:
        assert False
    #print>>sys.stderr, 'dynamic done'


def checked_dynamic(block):
    dynamic(block)
    q = copy.deepcopy(block)
    impr = any(search_local_improvement(q, depth) for depth in range(8))
    if impr:
        assert q.get_score() > block.get_score()
        print>>sys.stderr, 'dyn', block.get_score()
        block.show()
        print>>sys.stderr, 'improved', q.get_score()
        q.show()
        assert False
    else:
        assert q.get_score() == block.get_score()


if __name__ == '__main__':

    sys.stdout = sys.stderr
    seed = 109
    while True:
        print '------------'
        random.seed(seed)
        print 'seed', seed
        seed += 1

        whole = Block.make_empty(10, 9)
        whole.change(whole.coords_to_index(0, 0))

        params = 0.5, 1, 1, 1, 0.5
        randomize_block_goal(whole.get_subblock(0, 0, whole.w-2, whole.h), *params)

        block = whole.get_subblock(2, 2, whole.w-2, whole.h-2)
        block.change(block.coords_to_index(0, 0))


        for _ in range(30):
            for pt in whole.enum_points():
                if whole.can_change(pt) and random.random() < 0.5:
                    whole.change(pt)

        whole.show()

        print>>sys.stderr

        backup = copy.deepcopy(whole)

        checked_dynamic(block)

        print 'full solution', whole.get_score()
        whole.show()

        print 'we wanted to solve', backup.get_score()
        backup.show()
