import sys
import random
import time

TIME_LIMIT = 9


random.seed(1)

dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]

def trace_matrix(m):
    h = len(m)
    w = len(m[0])

    def find_start():
        for y, line in enumerate(m):
            for x, occ in enumerate(line):
                if occ:
                    return x, y

    x0, y0 = find_start()
    x = x0 + 1
    y = y0
    path = ['R']
    while x != x0 or y != y0:
        ul = x > 0 and y > 0 and m[y-1][x-1]
        ur = x < w and y > 0 and m[y-1][x]
        dl = x > 0 and y < h and m[y][x-1]
        dr = x < w and y < h and m[y][x]

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


def can_change(m, x, y):
    #for dx in [-1, 1]:
    #    for dy in [-1, 1]:
    #        if m[y+dy][x+dx] != m[y+dy][x] == m[y][x+dx]:
    #            return False
    if m[y+1][x+1] != m[y+1][x] == m[y][x+1]: return False
    if m[y-1][x+1] != m[y-1][x] == m[y][x+1]: return False
    if m[y+1][x-1] != m[y+1][x] == m[y][x-1]: return False
    if m[y-1][x-1] != m[y-1][x] == m[y][x-1]: return False
    return m[y-1][x] != m[y+1][x] or m[y][x-1] != m[y][x+1]


def score_diff(m, goal, x, y):
    result = 0
    q = m[y][x]
    g = goal[y][x]
    if g is not None:
        count = (
            (q != m[y][x-1])+
            (q != m[y][x+1])+
            (q != m[y-1][x])+
            (q != m[y+1][x]))
        result += (4-count) == g
        result -= count == g
    for dx, dy in dirs:
        g = goal[y+dy][x+dx]
        if g is not None:
            qq = m[y+dy][x+dx]
            count = (
                (qq != m[y+dy+dy][x+dx+dx])+
                (qq != m[y+dy-dx][x+dx+dy])+
                (qq != m[y+dy+dx][x+dx-dy]))
            result += count + (q == qq) == g
            result -= count + (q != qq) == g
    return result


def print_matrix(m):
    for line in m:
        for q in line:
            print>>sys.stderr, int(q),
        print>>sys.stderr


class FixTheFence(object):
    def findLoop(self, diagram):
        start = time.clock()

        h = len(diagram)
        w = len(diagram[0])
        m = [[False]*(w+2) for _ in range(h+2)]
        m[1][1] = True

        goal = [[None]*(w+2) for _ in range(h+2)]
        for i in range(w):
            for j in range(h):
                if diagram[j][i] != '-':
                    goal[j+1][i+1] = int(diagram[j][i])

        frontier = set([(2, 1)])
        frontier_list = list(frontier)

        e = 0.8
        i = 0
        while True:
            i += 1
            if i % 1000 == 0:
                if time.clock() - start > TIME_LIMIT:
                    #print>>sys.stderr, i, 'steps'
                    break
            if i % len(frontier) == 0:
                frontier_list = list(frontier)
            x, y = random.choice(frontier_list)
            if x <= 0 or x > w or y <= 0 or y > h:
                continue

            if can_change(m, x, y):
                d = score_diff(m, goal, x, y)
                m[y][x] = not m[y][x]
                if d < 0 and random.random() > e:
                    m[y][x] = not m[y][x]
                else:
                    for dx, dy in dirs:
                        frontier.add((x+dx, y+dy))
                if d <= 0:
                    e *= 0.99999

        print>>sys.stderr, i / (time.clock() - start), 'steps per second'

        x, y, path = trace_matrix(m)
        return '%s %s %s' % (y-1, x-1, path)


def main():
    size = int(raw_input())
    lines = [raw_input().strip() for _ in range(size)]
    result = FixTheFence().findLoop(lines)
    #print>>sys.stderr, result
    print result


if __name__ == '__main__':
    main()