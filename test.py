import nose
from nose.tools import *

from ff import *
from dynamic import *


def test_conns():
    prop = Propagator(3)

    eq_(len(prop.topo_index), 56)

    conns = prop.get_connections(0b00000, 0b0000)
    eq_(conns, [])

    conns = prop.get_connections(0b01000, 0b00011)
    eq_(sorted(conns), [(-1, 101), (2, 3)])

    conns = prop.get_connections(0b00011, 0b00111)
    eq_(conns, [(1, 102)])


def test_next_topo():
    prop = Propagator(3)

    topo = [None, None, None, None]
    conns = [(-1, 101)]

    next_topo = prop.next_topo(topo, conns, up_gate=IN)
    eq_(next_topo, [None, 'global', None, None])

    next_topo = prop.next_topo(topo, conns, up_gate=OUT)
    eq_(next_topo, None)


    topo = ['global', None, None, None]
    conns = [(0, -1), (999, 102)]

    next_topo = prop.next_topo(topo, conns, up_gate=BARRIER, down_gate=BARRIER)
    eq_(next_topo, None)


    topo = [']2', None, ']0', 'global']
    conns = [(0, 101), (2, 102), (999, 3)]

    next_topo = prop.next_topo(topo, conns, down_gate=OUT)
    eq_(next_topo, [None, ']2', ']1', None])


def test_enumerate_transitions():
    prop = Propagator(1)

    for topo in [None, None], ['global', None], ['[1', '[0'], [']1', ']0']:
        print topo, ':'
        for (xor_bits, up_gate, down_gate), new_topo in prop.enumerate_transitions(topo):
            d = {IN: 'IN', OUT: 'OUT', BARRIER: 'BARRIER', None: None}
            up_gate = d[up_gate]
            down_gate = d[down_gate]
            print ' ', bin(xor_bits)[::-1], up_gate, down_gate, new_topo

    assert False



nose.run()