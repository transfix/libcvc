#!/bin/sh
# The long run of the design-time lock model (revision 5b; docs/STATE_EXEC_LOCKS_PROTOCOL.md
# section 7): every configuration, every scripted mode and three long random runs. See README.md.
#
# Run it from anywhere; it works in this directory. Phase 1 is bounded exploration (bound 4 for
# every configuration, bound 5 for core, destroy, gdestroy, handlers, attach, cache and l2) beside
# the first long random run; phases 2-4 explore every interleaving of scripted processes (single,
# nested, pairs); phase 5 is two more random seeds. The bound-5 runs outlast the rest of phase 1
# (cache and l2 by about an hour), so by default (OVERLAP=1) phases 2-5 start beside them. Markers
# in $R: PHASE1A (bound 4 and the first random run), PHASE1B (bound 5), PHASE2, PHASE3, PHASE4,
# ALLDONE; SUMMARY.txt lists every result.
#
# Cost with the defaults: about 2 hours on 22 cores, and about 15 GB of memory at the peak (the
# bound-5 explorations keep every state they visit).
#
#   PY       the Python 3 interpreter (default: python3, else python on PATH)
#   R        the results directory (default: results, here; not committed)
#   B4_JOBS  bound-4 explorations at once, and the first random run's workers (default 12)
#   W        workers for phases 2-5 (default 16: with the seven bound-5 runs, 23 processes)
#   OVERLAP  1 (default): phases 2-5 start beside the bound-5 runs; 0: they wait for them
#
# Expected: every scenario ok and every mutation caught by the violation it names (scenarios.py's
# last line reports 0 unexpected); no violation in any result except splitpost_mut's port-race
# (the mutation 'post-test-outside-mu'); every exploration complete; every result file present.
PY=${PY:-$(command -v python3 || command -v python)}
[ -n "$PY" ] || { echo "run_long.sh: set PY to a Python 3 interpreter" >&2; exit 1; }
B4_JOBS=${B4_JOBS:-12}
W=${W:-16}
OVERLAP=${OVERLAP:-1}
R=${R:-results}
# the bound-5 runs, longest first, and the bound-4 runs, longest first
B5="cache l2 handlers core destroy gdestroy attach"
B4="wide l2 priority cache three grantseeds coalesce l1 core two2 handlers noinh foreign destroy gdestroy splitpost2 attach splitpost splitpost_mut"
cd "$(dirname "$0")"
mkdir -p $R
export PY R B4 B5
date > $R/START
$PY -I scenarios.py > $R/scenarios.txt 2>&1
# phase 1: the bound-5 runs first (cache and l2 are the longest), in the background
(
  for c in $B5; do
    $PY -I explore.py $c 5 > $R/free_${c}5.json 2> $R/free_${c}5.err &
  done
  wait
  echo done > $R/PHASE1B
) &
B5PID=$!
# beside them, bound 4 for every configuration, longest first, B4_JOBS at a time
for c in $B4; do
  echo $c
done | xargs -P $B4_JOBS -I{} sh -c '$PY -I explore.py {} 4 > $R/free_{}4.json 2> $R/free_{}4.err'
# then the first long random run
$PY -I random_runs.py 6000000 $B4_JOBS 7 > $R/random.json 2> $R/random.err
echo done > $R/PHASE1A
[ "$OVERLAP" = 1 ] || wait $B5PID
# phase 2-4: every interleaving of scripted processes with faults
$PY -I explore_scripts.py single 0 1 $W > $R/single.json 2> $R/single.err
echo done > $R/PHASE2
$PY -I explore_scripts.py nested 1500 2 $W > $R/nested.json 2> $R/nested.err
echo done > $R/PHASE3
$PY -I explore_scripts.py pairs 1200 3 $W > $R/pairs.json 2> $R/pairs.err
echo done > $R/PHASE4
# phase 5: two more long random runs, other seeds
$PY -I random_runs.py 4000000 $W 101 > $R/random2.json 2> $R/random2.err
$PY -I random_runs.py 4000000 $W 9001 > $R/random3.json 2> $R/random3.err
wait $B5PID
# summary: every result, flagging anything unexpected
$PY -I - $R > $R/SUMMARY.txt 2>&1 <<'EOF'
import glob, json, os, sys
R = sys.argv[1]
bad = 0
# scenarios.py (rev 5b): each mutation line must say it was caught by the violation it names; a
# scenario must not report a violation, an assertion or a crash; the last line must count 0
# unexpected (no last line: scenarios.py died)
try:
    lines = open(os.path.join(R, 'scenarios.txt')).read().splitlines()
except Exception as e:
    lines = []
    print('UNEXPECTED scenarios: no scenarios.txt (%s)' % e); bad += 1
for line in lines:
    if line.startswith('mutation ') and 'caught as expected (' not in line:
        print('UNEXPECTED scenarios: ' + line.rstrip()); bad += 1
    if line.startswith('scenario_') and any(s in line for s in ('VIOLATION', 'ASSERT', 'CRASH')):
        print('UNEXPECTED scenarios: ' + line.rstrip()); bad += 1
tail = [l for l in lines if l.startswith('scenarios.py: ')]
if not tail or not tail[-1].endswith(' 0 unexpected'):
    print('UNEXPECTED scenarios: %s' % (tail[-1] if tail else 'no summary line (scenarios.py died)')); bad += 1
elif tail:
    print('scenarios              ok         ' + tail[-1])
# every result file the run makes must be there
want = (['free_%s4' % c for c in os.environ['B4'].split()] + ['free_%s5' % c for c in os.environ['B5'].split()]
        + ['random', 'single', 'nested', 'pairs', 'random2', 'random3'])
for name in want:
    if not os.path.exists(os.path.join(R, name + '.json')):
        print('%-22s MISSING' % name); bad += 1
for f in sorted(glob.glob(os.path.join(R, '*.json'))):
    name = os.path.basename(f)[:-5]
    try:
        d = json.load(open(f))
    except Exception as e:
        print('%-22s UNREADABLE (%s): the run died or is still going' % (name, e)); bad += 1
        continue
    v = sorted(d.get('violations', {}))
    size = d.get('states', d.get('steps'))
    done = d.get('complete', True)          # explore.py; scripted modes report incomplete combos
    note = ' (%d combos stopped at 400k states)' % d['incomplete'] if d.get('incomplete') else ''
    expect = ['port-race', 'port-race(drain)'] if name.startswith('free_splitpost_mut') else []
    ok = v == expect and done
    bad += not ok
    print('%-22s %-10s %10s states/steps %7ss complete=%s violations=%s%s'
          % (name, 'ok' if ok else 'UNEXPECTED', size, d.get('seconds'), done, v, note))
print('%d unexpected' % bad)
EOF
date > $R/ALLDONE
