"""A program that imports a module nobody has (ctest arisrvx_check_missing_module): `arisrvx check`
reports it and exits 1."""

import arisrvx_no_such_module  # noqa: F401
