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
echo "$SCRIPT_DIR/optic-clp -b $DOMAIN_FILE $PROBLEM_FILE"

if ! "$SCRIPT_DIR/optic-clp" -b "$DOMAIN_FILE" "$PROBLEM_FILE" \
    > "$SCRIPT_DIR/output/plan.txt" \
    2> "$SCRIPT_DIR/output/optic_error.txt"; then

    echo "ERROR"
    echo "Error: OPTIC returned with error"
    echo "----- OPTIC STDERR -----"
    cat "$SCRIPT_DIR/output/optic_error.txt"
    echo "----- OPTIC STDOUT / PLAN -----"
    cat "$SCRIPT_DIR/output/plan.txt"
    exit 1
fi

if ! grep -q ";;;; Solution Found" "$SCRIPT_DIR/output/plan.txt"; then
    echo "ERROR"
    echo "Error: Solution not found in the plan file."
    echo "----- OPTIC STDERR -----"
    cat "$SCRIPT_DIR/output/optic_error.txt"
    echo "----- OPTIC STDOUT / PLAN -----"
    cat "$SCRIPT_DIR/output/plan.txt"
    exit 1
fi

awk '/;;;; Solution Found/ {found=1; count=3; next} found && count-- <= 0' "$SCRIPT_DIR/output/plan.txt"
