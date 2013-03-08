import random
import copy
import time

from ff import *
from sampling import *
from simple_solvers import *



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


def generate_crossings(block, paths):
    crossings = []
    for y in range(block.h+1):
        x = -1
        if (x, y) in paths:
            end_x, end_y = paths[x, y]
            if end_x >= 0:
                crossings.append(((x, y), (end_x, end_y)))
            else:
                assert end_x == x
    yield crossings[:]
    for x in range(block.w+1):
        y = -1
        if (x, y) in paths:
            end_x, end_y = paths[x, y]
            if end_x < x:
                assert crossings[0] == ((end_x, end_y), (x, y))
                crossings.pop(0)
            elif end_x > x:
                crossings.insert(0, ((x, y), (end_x, end_y)))
            else:
                assert crossings == []

        y = block.h+1
        if (x, y) in paths:
            end_x, end_y = paths[x, y]
            if end_x < x:
                assert crossings[-1] == ((end_x, end_y), (x, y))
                crossings.pop()
            elif end_x > x:
                crossings.append(((x, y), (end_x, end_y)))
            else:
                assert crossings == []

        yield crossings[:]

    for y in range(block.h+1):
        x = block.w+1
        if (x, y) in paths:
            end_x, end_y = paths[x, y]
            if end_x < x:
                assert crossings[0] == ((end_x, end_y), (x, y))
                crossings.pop(0)
            else:
                assert end_x == x
    assert crossings == []


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

class Propagator(object):
    def __init__(self, h):
        self.h = h

        self.topo_index = {}
        self.index_topo = []
        self.topo_bits = []
        def rec(topo, stack):
            if len(topo) == self.h+1:
                if stack == []:
                    # TODO: fix opening brackets
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

        self.transition_table = []
        for index, topo in enumerate(self.index_topo):
            t = {}
            self.transition_table.append(t)
            for (xor_bits, up_gate, down_gate), new_topo in self.enumerate_transitions(topo):
                #if (up_gate, down_gate) not in t:
                tt = t.setdefault((up_gate, down_gate), {})
                assert xor_bits not in tt
                tt[xor_bits] = self.topo_index[new_topo]

        print>>sys.stderr, 'Propagator(%d) created' % self.h

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
        conn_dict = dict(connections + map(reversed, connections))

        visited = set()

        def trace(q):
            q = conn_dict[q]
            if 0 <= q < 100 and topo[q].startswith('['):
                qq = int(topo[q][1:])
                #assert qq not in visited
                visited.add(qq)
                return trace(qq)
            else:
                return q

        new_topo = ['uninitialized'] * (self.h+1)

        failed = False
        for y in range(self.h+1):
            if topo[y] is None or topo[y].startswith('['):
                pass
            elif topo[y] == 'global':
                end = trace(y)
                if 100 <= end < 200:
                    new_topo[end-100] = 'global'
                elif end in [UP_CONN, DOWN_CONN]:
                    yy = {UP_CONN: up_gate, 999: down_gate}[end]
                    if end == UP_CONN:
                        gate = up_gate
                    else:
                        gate = down_gate
                    if gate is not OUT:
                        return
                else:
                    assert 0 <= end < 100, end
                    assert topo[end] == 'global' or topo[end].startswith(']')
                    return
            elif topo[y].startswith(']'):
                counterpart = int(topo[y][1:])
                end = trace(y)
                if 0 <= end < 100:
                    if end != counterpart:
                        return
                elif 100 <= end < 200:
                    end2 = trace(counterpart)
                    if not 100 <= end2 < 200:
                        return
                    else:
                        new_topo[end-100] = ']'+str(end2-100)
                        new_topo[end2-100] = ']'+str(end-100)
                else:
                    return
            else:
                assert False

        for start, gate in (UP_CONN, up_gate), (DOWN_CONN, down_gate):
            if gate is None:
                assert start not in conn_dict
                continue
            end = trace(start)
            #print 'trace', start, end
            if gate is OUT:
                if 0 <= end < 100:
                    if topo[end] != 'global':
                        return
                elif 100 <= end < 200:
                    if 'global' not in topo:
                        return
                    if start == UP_CONN:
                        g = topo.index('global')
                    elif start == DOWN_CONN:
                        g = len(topo) - 1 - topo[::-1].index('global')
                        assert topo[g] == 'global'
                    else:
                        assert False
                    end2 = trace(g)
                    if not 100 <= end2 < 200:
                        return
                    new_topo[end-100] = ']'+str(end2-100)
                    new_topo[end2-100] = ']'+str(end-100)
                else:
                    return
            elif gate is IN:
                if 100 <= end < 200:
                    new_topo[end-100] = 'global'
                else:
                    return
            elif gate is BARRIER:
                assert up_gate == down_gate == BARRIER
                if 'global' in topo:
                    return
                other_start = UP_CONN + DOWN_CONN - start
                other_end = trace(other_start)
                if end == other_start:
                    assert start == other_end
                elif 100 <= end < 200:
                    if 100 <= other_end < 200:
                        new_topo[end-100] = ']'+str(other_end-100)
                        new_topo[other_end-100] = ']'+str(end-100)
                    else:
                        return
                else:
                    return

        for y in range(self.h+1):
            if y+100 not in conn_dict:
                new_topo[y] = None
            else:
                end = trace(y+100)
                if 100 <= end < 200:
                    new_topo[end-100] = '['+str(y)

        # prevent cycles
        for y in range(self.h+1):
            if topo[y] is not None and topo[y].startswith('[') and y not in visited:
                return

        if not failed:
            assert 'uninitialized' not in new_topo
            return tuple(new_topo)
        else:
            return None

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
                #if new_topo is not None:
                    #assert new_topo in self.topo_index, (topo, bin(xor_bits)[::-1], new_topo)
                if new_topo in self.topo_index:
                    yield (xor_bits, up_gate, down_gate), new_topo
        #for qq in range(compatible):
        #    pass


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

        for (topo, bonus, penalty), (cost, sol) in states[x].items():
            #print x, prop.index_topo[topo], bin(bonus)[::-1], bin(penalty)[::-1], cost
            zzz = prop.transition_table[topo].get((up_gate, down_gate), {})
            for xor_bits, new_topo in zzz.items():
                new_cost = cost
                new_cost += num_bits(xor_bits & bonus)
                new_cost += num_bits((~xor_bits) & penalty)

                new_bonus = 0
                new_penalty = 0

                bits = prop.topo_bits[topo]
                new_bits = bits ^ xor_bits
                for y in range(block.h):
                    pt = block.coords_to_index(x, y)
                    g = block.goal[pt]
                    if g is not None:
                        d = new_bits ^ (new_bits << 1)

                        num_neighbors = bool(d & (2 << y)) + bool(d & (4 << y))
                        num_neighbors += bool(xor_bits & (2 << y))
                        if num_neighbors+1 == g:
                            new_bonus |= 2 << y
                        elif num_neighbors == g:
                            new_penalty |= 2 << y

                # up and down goals
                pt = block.coords_to_index(x, -1)
                g = block.goal[pt]
                if g is not None:
                    num_neighbors = bool(xor_bits & 1)
                    num_neighbors += bool(block.m[pt] != block.m[pt-block.stride])
                    num_neighbors += bool((new_bits ^ (new_bits >> 1)) & 1)
                    if num_neighbors+1 == g:
                        new_bonus |= 1
                    elif num_neighbors == g:
                        new_penalty |= 1
                pt = block.coords_to_index(x, block.h)
                g = block.goal[pt]
                if g is not None:
                    num_neighbors = bool(xor_bits & (2 << block.h))
                    num_neighbors += bool(block.m[pt] != block.m[pt+block.stride])
                    num_neighbors += bool((new_bits ^ (new_bits << 1)) & (2 << block.h))
                    if num_neighbors+1 == g:
                        new_bonus |= 2 << block.h
                    elif num_neighbors == g:
                        new_penalty |= 2 << block.h

                new_state = new_topo, new_bonus, new_penalty
                #print '  ->', bin(xor_bits)[::-1], prop.index_topo[new_topo], bin(new_bonus)[::-1], bin(new_penalty)[::-1], new_cost

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


def checked_dynamic(block):
    dynamic(block)
    q = copy.deepcopy(block)
    improve_by_levels(q, time.clock()+10)
    if q.get_score() > block.get_score():
        print 'dyn', block.get_score()
        block.show()
        print 'improved', q.get_score()
        q.show()
        assert False


if __name__ == '__main__':

    #prop = Propagator(7)
    #print len(prop.topo_index)
    #cnt = 0
    #for t in prop.transition_table:
    #    for gg in t:
    #        cnt += len(t[gg])
    #print cnt
    #exit()
    sys.stdout = sys.stderr

    whole = Block.make_empty(30, 6)
    whole.change(whole.coords_to_index(0, 0))

    params = 0.5, 1, 1, 1, 0.5
    randomize_block_goal(whole.get_subblock(0, 0, whole.w-1, whole.h), *params)

    block = whole.get_subblock(1, 1, whole.w-1, whole.h-1)
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

    #print trace_path(whole, whole.coords_to_index(2, 0), 1)

    #time.sleep(0.01)
    #print trace_path(block, block.coords_to_index(0, 0), -block.stride)