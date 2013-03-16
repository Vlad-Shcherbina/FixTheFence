import collections
import multiprocessing
import random
import subprocess
import sys
import time

from ff import *
from sampling import *


def eval_solution(block, solution):
    m = block.m
    for pt in block.enum_points():
        m[pt] = False

    y, x, path = solution.split()

    x = int(x)
    y = int(y)
    visited = set()
    start = x, y
    for d in path:
        assert 0 <= x <= block.w
        assert 0 <= y <= block.h
        assert (x, y) not in visited
        visited.add((x, y))
        if d == 'U':
            y -= 1
            m[block.coords_to_index(x, y)] = True
        elif d == 'D':
            m[block.coords_to_index(x, y)] = True
            y += 1
        elif d == 'L':
            x -= 1
        elif d == 'R':
            x += 1
        else:
            assert False
    assert start == (x, y)

    for y in range(block.h):
        q = False
        for x in range(block.w+1):
            pt = block.coords_to_index(x, y)
            if m[pt]:
                q = not q
            m[pt] = q
        assert not q

    for pt in block.enum_points():
        assert not m[pt] != m[pt+1] != m[pt+block.stride+1] != m[pt+block.stride]

    return block.get_score()


def run(block):
    assert block.h == block.w
    start = time.time()
    p = subprocess.Popen(
        ['./a.out'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    s = str(block.h)+'\n'
    for y in range(block.h):
        for x in range(block.w):
            g = block.goal[block.coords_to_index(x, y)]
            if g is None:
                s += '-'
            else:
                s += str(g)
        s += '\n'
    out, _ = p.communicate(s)
    assert time.time() - start <= 4
    return out


def evaluate_sample(seed):
    random.seed(seed)
    size = random.randrange(15, 101)
    params = random_parameters()

    block = Block.make_empty(size, size)
    randomize_block_goal(block, *params)

    solution = run(block)
    score = eval_solution(block, solution)

    counts = collections.Counter(block.goal[pt] for pt in block.enum_points())

    area = 1.0*size**2

    score /= area - counts[None]+1e-12
    return '{"seed": %d, "size": %d, "p_":%f, "p0":%f, "p1":%f, "p2":%f, "p3":%f, "score": %f}' % (
        seed, size,
        counts[None]/area,
        counts[0]/area,
        counts[1]/area,
        counts[2]/area,
        counts[3]/area,
        score)


def main():
    pool = multiprocessing.Pool(multiprocessing.cpu_count()-1)

    print ' '.join(sys.argv[1:])
    for result in pool.imap(evaluate_sample, xrange(1000)):
        print result


if __name__ == '__main__':
    main()

