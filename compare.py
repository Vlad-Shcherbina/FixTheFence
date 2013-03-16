import sys
import collections


class C(object):
    pass


class StatsCollector(object):
    def __init__(self):
        self.sum = 0.0
        self.count = 0
        self.sum2 = 0.0

    def add(self, x):
        self.count += 1
        self.sum += x
        self.sum2 += x*x

    def average(self):
        if self.count == 0:
            return 0.0
        return 1.0*self.sum/self.count

    def sigma(self):
        if self.count < 2:
            return 0.0
        ave = self.average()
        return (1.0*(self.sum2 - 2*self.sum*ave + self.count*ave**2) / (self.count-1))**0.5 / self.count**0.5


def bucket(entry):
    return entry.size//5*5
    d = ''
    if entry.p_ > 0.4:
        d += '_'
    if entry.p0 > 0.4:
        d += '0'
    if entry.p1 > 0.4:
        d += '1'
    if entry.p2 > 0.4:
        d += '2'
    if entry.p3 > 0.4:
        d += '3'
    return (entry.size//10*10, d)

def read_file(filename):
    result = {}
    with open(filename) as fin:
        fin.readline()
        for line in fin:
            c = C()
            c.__dict__.update(eval(line))
            result[c.seed] = c
    return result



def main():
    entries1 = read_file(sys.argv[1])
    entries2 = read_file(sys.argv[2])

    seeds = set(entries1) & set(entries2)
    print len(seeds)

    score_diff = collections.defaultdict(StatsCollector)
    for seed in seeds:
        score1 = entries1[seed].score
        score2 = entries2[seed].score
        score_diff[bucket(entries1[seed])].add(100*(1.0*score2/score1-1))


    for k, v in sorted(score_diff.items()):
        print '{:10}: {:.4}%+-{:.4} ({} entries)'.format(k, v.average(), v.sigma(), v.count)

if __name__ == '__main__':
    main()
