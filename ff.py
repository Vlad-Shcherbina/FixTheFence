import sys
import random
import time

import dynamic
from simple_solvers import *

TIME_LIMIT = 9


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
        for pt in self.enum_points():
            c = m[pt]
            g = goal[pt]
            if g is not None:
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

        strip_width = 3
        for y in range(0, whole.h, strip_width+1):
            print>>sys.stderr, 'y =', y
            if y+strip_width <= whole.h:
                block = whole.get_subblock(0, y, whole.w, y+strip_width)
                print>>sys.stderr, whole.get_score(), '->',
                dynamic.dynamic(block)
            print>>sys.stderr, whole.get_score()

        print>>sys.stderr, 'final', whole.get_score()

        for pt in whole.enum_points():
            if whole.m[pt]:
                x, y = whole.index_to_coords(pt)
                path, finish = trace_path(whole, pt+1, 1)
                assert finish == pt+1
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