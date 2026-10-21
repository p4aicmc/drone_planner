#!/usr/bin/env bash
# Temporal Fast Downward wrapper.
#
# Contract (same as OPTIC/generate_plan.sh): print the plan as
# "<start>: (<action> <args>) [<duration>]" lines on stdout and exit 0, or
# print "ERROR" plus diagnostics on stderr and exit non-zero. The caller
# (action_planner_node.generate_plan_custom_solver) only trusts stdout when
# the exit code is 0, so this script must never print a plan it did not
# produce in this very run.

DOMAIN_FILE="$1"
PROBLEM_FILE="$2"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TFD_HOME="$SCRIPT_DIR/downward"
OUTPUT_DIR="$SCRIPT_DIR/output"
PLAN_PREFIX="$OUTPUT_DIR/plan"

# Search parameters: y/Y greedy + ff heuristics, 'a' anytime search,
# T = seconds to give up while no plan is found, t = seconds to keep
# improving after the first plan, e/r/O/C as in the TFD README.
SEARCH_ARGS=(y Y a T 10 t 5 e r O 1 C 1)

fail() {
    echo "ERROR"
    echo "Error: $1" >&2
    shift
    for log in "$@"; do
        if [[ -s "$log" ]]; then
            echo "----- $(basename "$log") -----" >&2
            cat "$log" >&2
        fi
    done
    exit 1
}

if [[ ! -f "$DOMAIN_FILE" || ! -f "$PROBLEM_FILE" ]]; then
    fail "domain or problem file is missing (DOMAIN_FILE=$DOMAIN_FILE PROBLEM_FILE=$PROBLEM_FILE)"
fi

# Start from a clean slate: every artifact of this run lives in output/, and
# any plan file a previous run (or an old checkout) left under downward/ is
# removed so it can never be mistaken for this run's result.
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"
rm -f "$TFD_HOME"/plan "$TFD_HOME"/plan.*

echo "Running TFD:" >&2
echo "$TFD_HOME/search/search ${SEARCH_ARGS[*]} p $PLAN_PREFIX ($DOMAIN_FILE, $PROBLEM_FILE)" >&2

cd "$OUTPUT_DIR" || fail "cannot enter $OUTPUT_DIR"

if ! python3 "$TFD_HOME/translate/translate.py" "$DOMAIN_FILE" "$PROBLEM_FILE" \
        > translate.log 2>&1; then
    fail "TFD translate failed" translate.log
fi

if ! "$TFD_HOME/preprocess/preprocess" < output.sas > preprocess.log 2>&1; then
    fail "TFD preprocess failed" preprocess.log
fi

"$TFD_HOME/search/search" "${SEARCH_ARGS[@]}" p "$PLAN_PREFIX" < output > search.log 2>&1
search_rc=$?

# Exit codes of downward/search/planner.cc:
#   0   SOLVED / SOLVED_COMPLETE
#   1   FAILED: state space exhausted, no plan exists
#   137 SOLVED_TIMEOUT / FAILED_TIMEOUT: the T/t deadline hit; a plan may or
#       may not have been written, so the plan files decide.
case "$search_rc" in
    0|137) ;;
    1) fail "TFD found no plan (unsolvable problem)" search.log ;;
    *) fail "TFD search exited with code $search_rc" search.log ;;
esac

# Anytime search writes plan.1, plan.2, ... with strictly improving makespan;
# take the highest-numbered one. Without 'a' the file is just "plan".
best_plan=""
best_n=-1
for f in "$PLAN_PREFIX" "$PLAN_PREFIX".*; do
    [[ -s "$f" ]] || continue
    n="${f##*.}"
    [[ "$f" == "$PLAN_PREFIX" ]] && n=0
    [[ "$n" =~ ^[0-9]+$ ]] || continue
    if (( n > best_n )); then
        best_n=$n
        best_plan="$f"
    fi
done

if [[ -z "$best_plan" ]]; then
    fail "TFD search ended (exit $search_rc) without writing a plan" search.log
fi

echo "Using $(basename "$best_plan") (exit $search_rc)" >&2
cat "$best_plan"
