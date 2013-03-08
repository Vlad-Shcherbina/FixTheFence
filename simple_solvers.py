import random


def optimize(block, iterations, temperature=0):
    score = block.get_score()

    if block.w * block.h == 0:
        return
    for i in xrange(iterations):
        pt = block.coords_to_index(
            random.randrange(block.w),
            random.randrange(block.h))
        if block.can_change(pt):
            d = block.score_diff(pt)
            if d >= 0 or random.random() < temperature:
                block.change(pt)
                score += d
                block.update_best(score)


def optimize_pairs(block, iterations, temperature=0):
    stride = block.stride
    if block.w <= 2:
        return
    if block.h <= 2:
        return

    score = block.get_score()

    for i in xrange(iterations):
        pt = block.coords_to_index(
            random.randrange(block.w-2)+1,
            random.randrange(block.h-2)+1)
        if block.can_change(pt):
            pts = [pt-1, pt+1, pt-stride, pt+stride]
            random.shuffle(pts)

            d1 = block.score_diff(pt)
            block.change(pt)

            for pt2 in pts:
                if block.can_change(pt2):
                    d2 = block.score_diff(pt2)
                    if d1 + d2 >= 0 or random.random() < temperature:
                        block.change(pt2)
                        score += d1+d2
                        block.update_best(score)
                        break
            else:
                block.change(pt)


class FoundImprovement(Exception):
    pass


def search_local_improvement(block, depth):
    def rec(x, y, score_diff, depth):
        pt = block.coords_to_index(x, y)
        if not block.can_change(pt):
            return
        score_diff += block.score_diff(pt)
        block.change(pt)
        if score_diff > 0:
            raise FoundImprovement()
        if depth > 0:
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    if dx == dy == 0:
                        continue
                    if depth > 2 and random.random() > 0.2:
                        continue
                    x1 = x + dx
                    y1 = y + dy
                    if x1 < 0 or x1 >= block.w:
                        continue
                    if y1 < 0 or y1 >= block.h:
                        continue
                    rec(x1, y1, score_diff, depth-1)
        block.change(pt)

    improved = False
    for y in range(block.h):
        for x in range(block.w):
            try:
                rec(y, x, 0, depth)
            except FoundImprovement:
                improved = True
    return improved


def anneal(whole, end_time):
    k = 2
    for i in xrange(10**9):
        if time.clock() > end_time:
            break

        x = random.randrange(w)
        y = random.randrange(h)

        sub = whole.get_subblock(max(0, x-k), max(0, y-k), min(w, x+k), min(h, y+k))
        orig_score = sub.get_score()
        optimize(sub, k*k*50, temperature=0.5)
        sub.undo_to_best()
        optimize_pairs(sub, k*k*10, temperature=0.5)
        sub.undo_to_best()


def improve_by_levels(whole, end_time):
    k = 7
    centers = [(x, y) for x in range(w) for y in range(h) if x%k == k-1 and y%k == k-1]
    centers.sort(key=sum)
    for x, y in centers:
        sub = whole.get_subblock(max(0, x-k), max(0, y-k), min(w, x+k), min(h, y+k))
        optimize(sub, k*k*k*10, temperature=0.1)

    level = 0
    while True:
        if time.clock() > end_time:
            break
        print>>sys.stderr, level
        sys.stderr.flush()
        if search_local_improvement(whole, level):
            level = 0
            print>>sys.stderr, 'improved to', whole.get_score()
        else:
            level += 1