"""A module python_block.ari's python: block imports: one function it marks with @export, and one
the document exports by name (export: python_block_helpers.cube)."""

from _arihost import export


@export
def scale(x):
    return 10 * x


def cube(x):
    return x**3
