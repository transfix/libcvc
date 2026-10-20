#!/usr/bin/env bash
# Health checks for publish-cvcpkg.yml's self-hosted CUDA lanes,
# [self-hosted, linux, cuda] and [self-hosted, windows, cuda].
#
# GitHub has no queue timeout. A job that no runner takes waits 24 h before
# GitHub cancels it, and its run waits with it (so does `gh run rerun --failed`,
# which needs the run to have finished). That happens when the runner is
# offline, and also when it is online and idle but GitHub sends it nothing:
# GitHub stops queueing jobs to a runner that is more than 30 days behind on
# runner updates, and the REST API does not report runner versions.
# sandipaws-libcvc-cuda sat like that from about 2026-10-02 to 2026-10-20,
# because its self-update kept failing on the box.
#
#   cuda-lane.sh probe  Writes linux= and windows= (true|false) to
#                       $GITHUB_OUTPUT. A lane is false when no matching runner
#                       is online (only checked with RUNNERS_TOKEN), or when a
#                       lane job in another run has waited STALE_MINUTES while
#                       no lane job runs anywhere.
#   cuda-lane.sh watch  Watches this run's CUDA jobs (LINUX/WINDOWS = the probe's
#                       outputs) and exits 0 once each has a runner. If one
#                       waits STALE_MINUTES while no lane job runs anywhere, it
#                       waits for the run's other jobs to finish, then cancels
#                       the run instead of leaving it open for 24 h.
#
# Env: GITHUB_REPOSITORY, GITHUB_RUN_ID; GH_TOKEN (actions:read, plus
# actions:write for `watch`); RUNNERS_TOKEN (optional, administration:read);
# STALE_MINUTES (default 60); POLL_SECONDS (default 60); WATCHDOG_JOB (`watch`:
# this job's name, so it does not wait for itself).
set -uo pipefail

repo="${GITHUB_REPOSITORY:?}"
run_id="${GITHUB_RUN_ID:?}"
stale_min="${STALE_MINUTES:-60}"
poll="${POLL_SECONDS:-60}"

# One TSV row per job: <lane> <status> <created epoch> <run id> <url> <name>.
# <lane> is linux|windows for a job whose runs-on names a CUDA lane, else "-".
# shellcheck disable=SC2016  # jq variables, not shell ones
JOB_ROWS='.jobs[]
  | ([.labels[]? | ascii_downcase]) as $l
  | (if (($l | index("self-hosted")) and ($l | index("cuda"))) then
       (if ($l | index("windows")) then "windows"
        elif ($l | index("linux")) then "linux" else "-" end)
     else "-" end) as $lane
  | [$lane, .status, (.created_at | fromdateiso8601), .run_id, .html_url, .name]
  | @tsv'

run_jobs() {  # $1 = run id
  gh api --paginate "repos/$repo/actions/runs/$1/jobs?per_page=100" --jq "$JOB_ROWS"
}

# Rows for every job of every unfinished run in the repo, this run included.
all_unfinished_jobs() {
  local ids id
  ids="$(for st in queued in_progress waiting; do
           gh api --paginate "repos/$repo/actions/runs?status=$st&per_page=100" \
             --jq '.workflow_runs[].id' || exit 1
         done | sort -u)" || return 1
  for id in $ids; do
    run_jobs "$id" || return 1
  done
}

# Runner JSON pages, or empty when RUNNERS_TOKEN is unset or cannot list them.
runners_json() {
  [ -n "${RUNNERS_TOKEN:-}" ] || return 0
  GH_TOKEN="$RUNNERS_TOKEN" gh api --paginate "repos/$repo/actions/runners" 2>/dev/null || true
}

runner_count() {  # $1 = runners json, $2 = lane, $3 = extra jq select
  jq -rs --arg os "$2" "
    [ .[].runners[]?
      | select(.status == \"online\")
      | select([.labels[].name | ascii_downcase] as \$l
               | (\$l | index(\"cuda\")) and (\$l | index(\$os)))
      | select($3)
    ] | length" <<<"$1"
}

# True when a job on this lane is running somewhere, so a queued one is just
# waiting its turn. $1 = job rows, $2 = runners json, $3 = lane.
lane_busy() {
  awk -F'\t' -v lane="$3" '$1 == lane && $2 == "in_progress" { found = 1 } END { exit !found }' <<<"$1" && return 0
  [ -n "$2" ] && [ "$(runner_count "$2" "$3" '.busy')" -gt 0 ]
}

probe() {
  local runners rows lane ok cutoff stale
  runners="$(runners_json)"
  if [ -z "$runners" ]; then
    echo "::warning::Cannot list self-hosted runners (needs administration:read; set CVCPKG_RUNNER_PAT), so an offline runner goes unnoticed. The stale-queue check still runs."
  fi
  if ! rows="$(all_unfinished_jobs)"; then
    echo "::warning::Cannot list this repo's unfinished jobs, so the stale-queue check is skipped."
    rows=""
  fi
  cutoff=$(( $(date -u +%s) - stale_min * 60 ))
  for lane in linux windows; do
    ok=true
    if [ -n "$runners" ] && [ "$(runner_count "$runners" "$lane" 'true')" -eq 0 ]; then
      echo "::warning::No ONLINE self-hosted [$lane, cuda] runner. Skipping that CUDA lane instead of queueing on it."
      ok=false
    fi
    if $ok; then
      stale="$(awk -F'\t' -v lane="$lane" -v run="$run_id" -v cutoff="$cutoff" \
        '$1 == lane && $2 == "queued" && $4 != run && $3 < cutoff { print $5; exit }' <<<"$rows")"
      if [ -n "$stale" ] && ! lane_busy "$rows" "$runners" "$lane"; then
        echo "::warning::A [self-hosted, $lane, cuda] job has waited over ${stale_min} min with no runner taking it, and no job is running on that lane: $stale . The runner may show online but take no jobs (GitHub stops sending jobs to runners more than 30 days behind on updates). Skipping that CUDA lane."
        ok=false
      fi
    fi
    echo "$lane: $ok"
    echo "$lane=$ok" >> "${GITHUB_OUTPUT:-/dev/null}"
  done
}

# Of the lanes in $1 (whose job here has waited past the threshold), print the
# ones with nothing running on them anywhere: those are stuck, not waiting
# their turn behind another run. Scans every unfinished run, so it is costly.
stuck_of() {
  local rows runners lane
  rows="$(all_unfinished_jobs || true)"
  runners="$(runners_json)"
  for lane in $1; do
    lane_busy "$rows" "$runners" "$lane" || printf ' %s' "$lane"
  done
}

watch() {
  local expected="" lane rows now open stuck="" stuck_lanes last_lanes="" last_scan=0 warned="" confirm=0 all
  [ "${LINUX:-false}" = true ] && expected="$expected linux"
  [ "${WINDOWS:-false}" = true ] && expected="$expected windows"
  [ -n "$expected" ] || { echo "No CUDA lane in this run."; return 0; }
  while :; do
    if ! rows="$(run_jobs "$run_id")"; then
      sleep "$poll"; continue
    fi
    now=$(date -u +%s)

    # Done once every expected lane has a job that left the queue.
    all=true
    for lane in $expected; do
      awk -F'\t' -v lane="$lane" '$1 == lane && $2 != "queued" { found = 1 } END { exit !found }' <<<"$rows" || all=false
    done
    if $all; then
      echo "Every CUDA lane in this run got a runner:$expected."
      return 0
    fi

    # Lanes whose job has waited past the threshold. The repo-wide scan in
    # stuck_of costs one API call per unfinished run, so repeat it at most
    # every 15 min: GITHUB_TOKEN gets 1000 calls an hour.
    stuck_lanes="$(awk -F'\t' -v cutoff=$(( now - stale_min * 60 )) \
      '$1 != "-" && $2 == "queued" && $3 < cutoff { print $1 }' <<<"$rows" | sort -u | xargs)"
    if [ -z "$stuck_lanes" ]; then
      stuck=""
    elif [ "$stuck_lanes" != "$last_lanes" ] || [ $(( now - last_scan )) -ge 900 ]; then
      stuck="$(stuck_of "$stuck_lanes")"
      last_scan=$now
    fi
    last_lanes="$stuck_lanes"

    # Jobs of this run that are still open, apart from this watchdog and
    # the queued lane jobs.
    open="$(awk -F'\t' -v me="${WATCHDOG_JOB:-}" \
      '$6 != me && $2 != "completed" && !($1 != "-" && $2 == "queued") { n++ } END { print n + 0 }' <<<"$rows")"

    if [ -z "$stuck" ]; then
      if [ "$open" -eq 0 ] && ! awk -F'\t' '$1 != "-" && $2 == "queued" { found = 1 } END { exit !found }' <<<"$rows"; then
        echo "Nothing left to watch: the CUDA lane jobs never queued (skipped?)."
        return 0
      fi
      confirm=0
    else
      if [ -z "$warned" ]; then
        echo "::warning::CUDA lane job(s) for$stuck have waited over ${stale_min} min with no runner taking them, and nothing runs on that lane. The runner may show online but take no jobs. Once this run's other jobs finish, the run will be cancelled rather than left open 24 h."
        warned=1
      fi
      if [ "$open" -eq 0 ]; then
        # A finished job's dependents can take a moment to be created, so
        # see "nothing else open" on two polls in a row before cancelling.
        confirm=$((confirm + 1))
        if [ "$confirm" -ge 2 ]; then
          stuck="$(stuck_of "$stuck_lanes")"   # fresh, right before acting
          last_scan=$now
        fi
        if [ "$confirm" -ge 2 ] && [ -n "$stuck" ]; then
          echo "::error::Cancelling run $run_id: the CUDA lane job(s) for$stuck never got a runner in ${stale_min}+ min, and every other job has finished. Check the runner's _diag logs (a failed self-update leaves it online but jobless), then re-run the failed jobs."
          gh api -X POST "repos/$repo/actions/runs/$run_id/cancel" >/dev/null
          return 1
        fi
      else
        confirm=0
      fi
    fi
    sleep "$poll"
  done
}

case "${1:-}" in
  probe) probe ;;
  watch) watch ;;
  *) echo "usage: $0 probe|watch" >&2; exit 2 ;;
esac
