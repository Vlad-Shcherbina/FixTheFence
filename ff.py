import sys
import random
import time

TIME_LIMIT = 8


random.seed(1)
num_changes = 0

dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]

def trace_matrix(w, m):
    assert len(m) % w == 0
    h = len(m) / w

    p0 = m.index(True)
    y0, x0 = divmod(p0, w)
    x = x0 + 1
    y = y0
    path = ['R']
    while x != x0 or y != y0:
        ul = x > 0 and y > 0 and m[(y-1)*w + x-1]
        ur = x < w and y > 0 and m[(y-1)*w + x]
        dl = x > 0 and y < h and m[y*w + x-1]
        dr = x < w and y < h and m[y*w + x]

        if ul != ur and path[-1] != 'D':
            path.append('U')
            y -= 1
            continue
        if dl != dr and path[-1] != 'U':
            path.append('D')
            y += 1
            continue
        if ul != dl and path[-1] != 'R':
            path.append('L')
            x -= 1
            continue
        if ur != dr and path[-1] != 'L':
            path.append('R')
            x += 1
            continue

        assert False

    return x0, y0, ''.join(path)


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

    def can_change(self, pt):
        m = self.m
        stride = self.stride
        if m[pt+1+stride] != m[pt+stride] == m[pt+1]: return False
        if m[pt-1+stride] != m[pt+stride] == m[pt-1]: return False
        if m[pt+1-stride] != m[pt-stride] == m[pt+1]: return False
        if m[pt-1-stride] != m[pt-stride] == m[pt-1]: return False
        return m[pt-1] != m[pt+1] or m[pt-stride] != m[pt+stride]

    def get_score(self):
        stride = self.stride
        m = self.m
        goal = self.goal
        result = 0
        for y in range(self.h):
            for pt in xrange(self.offset+stride*y, self.offset+stride*y+self.w):
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

    def optimize(self, iterations, temperature=0):
        score = self.get_score()

        if self.w * self.h == 0:
            return
        for i in xrange(iterations):
            pt = self.coords_to_index(
                random.randrange(self.w),
                random.randrange(self.h))
            if self.can_change(pt):
                d = self.score_diff(pt)
                if d >= 0 or random.random() < temperature:
                    self.change(pt)
                    score += d
                    self.update_best(score)

    def optimize_pairs(self, iterations, temperature=0):
        stride = self.stride
        if self.w <= 2:
            return
        if self.h <= 2:
            return

        score = self.get_score()

        for i in xrange(iterations):
            pt = self.coords_to_index(
                random.randrange(self.w-2)+1,
                random.randrange(self.h-2)+1)
            if self.can_change(pt):
                pts = [pt-1, pt+1, pt-stride, pt+stride]
                random.shuffle(pts)

                d1 = self.score_diff(pt)
                self.change(pt)

                for pt2 in pts:
                    if self.can_change(pt2):
                        d2 = self.score_diff(pt2)
                        if d1 + d2 >= 0 or random.random() < temperature:
                            self.change(pt2)
                            score += d1+d2
                            self.update_best(score)
                            break
                else:
                    self.change(pt)


class FixTheFence(object):
    def findLoop(self, diagram):
        start = time.clock()

        h = len(diagram)
        w = len(diagram[0])

        m = [False] * (w+2) * (h+2)
        m[1*(w+2) + 1] = True

        goal = [None] * (w+2) * (h+2)
        for x in range(w):
            for y in range(h):
                if diagram[y][x] != '-':
                    goal[(y+1)*(w+2) + x+1] = int(diagram[y][x])

        whole = Block(m, goal, stride=w+2, offset=w+3, w=w, h=h)

        k = 7
        centers = [(x, y) for x in range(w) for y in range(h) if x%k == k-1 and y%k == k-1]
        centers.sort(key=sum)
        for x, y in centers:
            sub = whole.get_subblock(max(0, x-k), max(0, y-k), min(w, x+k), min(h, y+k))
            sub.optimize(k*k*k*10, temperature=0.1)

        for i in xrange(1000):
            if time.clock() - start > TIME_LIMIT:
                break
            whole.optimize_pairs(w*h*3, 0.001)
            whole.optimize(w*h*2, 0.001)

        print>>sys.stderr, 'best', whole.best_score
        print>>sys.stderr, 'current', whole.get_score()
        whole.undo_to_best()
        print>>sys.stderr, 'rewinded', whole.get_score()

        whole.optimize(w*h*4)
        whole.optimize_pairs(w*h*2)
        print>>sys.stderr, 'best', whole.best_score
        print>>sys.stderr, 'final', whole.get_score()

        x, y, path = trace_matrix(whole.stride, whole.m)
        return '%s %s %s' % (y-1, x-1, path)


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