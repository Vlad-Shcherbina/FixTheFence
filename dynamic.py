import random
import copy
import time

from ff import *
from sampling import *
from simple_solvers import *







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
