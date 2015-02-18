class ParallelGenerators:
    def __init__(self):
        self.generators = {}
        self.afters = {}

    def add(self, name, generator, after=None):
        self.generators[name] = iter(generator)
        if after is not None:
            self.after(name, after)

    def after(self, succeed_name, precede_name):
        if precede_name not in self.afters:
            self.afters[precede_name] = []
        self.afters[precede_name].append(
            (succeed_name, self.generators[succeed_name]))
        del self.generators[succeed_name]

    def cancel(self, name):
        if name in self.afters:
            for (new_name, generator) in self.afters[name]:
                self.generators[new_name] = generator
        del self.generators[name]

    def next(self):
        results = {}
        items = list(self.generators.items())
        for (name, g) in items:
            try:
                val = g.__next__()
                results[name] = val
            except StopIteration:
                self.cancel(name)
        return results
