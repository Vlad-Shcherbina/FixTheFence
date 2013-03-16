import random


def random_parameters():
    fill = 0.1 + 0.9*random.random()
    zeros = random.random()
    ones = random.random()
    twos = random.random()
    threes = random.random()*0.5
    return fill, zeros, ones, twos, threes


def random_goal(fill, zeros, ones, twos, threes):
    if random.random() >= fill:
        return None

    p = zeros+ones+twos+threes
    x = random.random() * p
    if x < zeros:
        return 0
    elif x < zeros + ones:
        return 1
    elif x < zeros + ones + twos:
        return 2
    else:
        return 3


def randomize_block_goal(block, *params):
    for pt in block.enum_points():
        block.goal[pt] = random_goal(*params)
