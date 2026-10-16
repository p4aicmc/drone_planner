#!/usr/bin/env bash
set -e

DOMAIN_FILE="$1"
PROBLEM_FILE="$2"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -rf "$SCRIPT_DIR/output"
mkdir -p "$SCRIPT_DIR/output"

if [[ ! -f "$DOMAIN_FILE" || ! -f "$PROBLEM_FILE" ]]; then
    echo "ERROR"
    echo "Error: domain or problem file is missing!"
    echo "DOMAIN_FILE=$DOMAIN_FILE"
    echo "PROBLEM_FILE=$PROBLEM_FILE"
    exit 1
fi

echo "Running OPTIC:"
echo "$SCRIPT_DIR/optic-clp -n1000000000 $DOMAIN_FILE $PROBLEM_FILE"

# This OPTIC build needs an explicit upper bound for fluent-based metrics.
# Search for improvements until the time limit, then return the last complete plan.
set +e
timeout "${OPTIC_TIMEOUT_SEC:-10s}" "$SCRIPT_DIR/optic-clp" -n1000000000 \
    "$DOMAIN_FILE" "$PROBLEM_FILE" \
    > "$SCRIPT_DIR/output/plan.txt" \
    2> "$SCRIPT_DIR/output/optic_error.txt"
optic_status=$?
set -e

if [[ "$optic_status" -ne 0 && "$optic_status" -ne 124 ]]; then

    echo "ERROR"
    echo "Error: OPTIC returned with error"
    echo "----- OPTIC STDERR -----"
    cat "$SCRIPT_DIR/output/optic_error.txt"
    echo "----- OPTIC STDOUT / PLAN -----"
    cat "$SCRIPT_DIR/output/plan.txt"
    exit 1
fi

plan=$(awk '
    /; Plan found with metric/ {candidate=""; reading=1; next}
    reading && /^[0-9]+\.[0-9]+:[[:space:]]+\([^)]+\)[[:space:]]+\[[0-9]+\.[0-9]+\]$/ {
        candidate = candidate $0 ORS
        next
    }
    reading && candidate != "" {plan=candidate; reading=0}
    END {printf "%s", plan}
' "$SCRIPT_DIR/output/plan.txt")

if [[ -z "$plan" ]]; then
    echo "ERROR"
    echo "Error: Solution not found in the plan file."
    echo "----- OPTIC STDERR -----"
    cat "$SCRIPT_DIR/output/optic_error.txt"
    echo "----- OPTIC STDOUT / PLAN -----"
    cat "$SCRIPT_DIR/output/plan.txt"
    exit 1
fi

printf '%s\n' "$plan"
