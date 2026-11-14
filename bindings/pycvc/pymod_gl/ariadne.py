"""pycvc_gl.ariadne: ``pycvc.ari`` under the name it first shipped under (#985).

It is the same module, not a copy: importing this one puts ``pycvc.ari`` in its place, so
``pycvc_gl.ariadne is pycvc.ari`` and its ``Host``, ``current_host()`` and ``export`` are the
same objects whichever name a program imports. The Host lives in pycvc so that a program under
``arix``, or a headless Host, needs no pycvc_gl; its window and offscreen modes still draw through
pycvc_gl. New code imports ``pycvc.ari``.
"""

import sys

import pycvc.ari

sys.modules[__name__] = pycvc.ari
