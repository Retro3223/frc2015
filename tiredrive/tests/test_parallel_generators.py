from tiredrive.robot import ParallelGenerators


def test_parallel_generators():
    """
    ParallelGenerators should consume generators in parallel
    """
    pg = ParallelGenerators()
    pg.add("1", range(4))
    pg.add("2", range(4, 8))
    assert {"1": 0, "2": 4} == pg.next()
    assert {"1": 1, "2": 5} == pg.next()
    assert {"1": 2, "2": 6} == pg.next()
    assert {"1": 3, "2": 7} == pg.next()
    assert {} == pg.next()


def test_overlap_generators():
    """
    ParallelGenerators should consume overlapping generators
    """
    pg = ParallelGenerators()
    pg.add("1", range(4))
    assert {"1": 0} == pg.next()
    assert {"1": 1} == pg.next()
    pg.add("2", range(3))
    assert {"1": 2, "2": 0} == pg.next()
    assert {"1": 3, "2": 1} == pg.next()
    assert {"2": 2} == pg.next()


def test_sequential_generators():
    """
    ParallelGenerators should consume generators in sequence
    """
    pg = ParallelGenerators()
    pg.add("1", range(4))
    pg.add("2", range(3))
    pg.after("2", "1")
    assert {"1": 0} == pg.next()
    assert {"1": 1} == pg.next()
    assert {"1": 2} == pg.next()
    assert {"1": 3} == pg.next()
    assert {} == pg.next()
    assert {"2": 0} == pg.next()
    assert {"2": 1} == pg.next()
    assert {"2": 2} == pg.next()


def loop(i=1):
    while True:
        yield i


def bad_eaglet(pg):
    """
    wait awhile, then throw "sibling" out of ParallelGenerators pg
    """
    for i in range(4):
        yield 2
    pg.cancel("sibling")
    for i in range(4):
        yield 2


def test_cancel_generator1():
    """
    generators run by ParallelGenerators should be able to cancel other
    generators
    """
    pg = ParallelGenerators()
    pg.add("eaglet", bad_eaglet(pg))
    pg.add("sibling", loop())

    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2} == pg.next()


def test_cancel_generator2():
    """
    canceling generators should trigger successor generators to run
    """
    pg = ParallelGenerators()
    pg.add("eaglet", bad_eaglet(pg))
    pg.add("sibling", loop())
    pg.add("nephew", loop(3), after="sibling")

    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "sibling": 1} == pg.next()
    assert {"eaglet": 2, "nephew": 3} == pg.next()
    assert {"eaglet": 2, "nephew": 3} == pg.next()
