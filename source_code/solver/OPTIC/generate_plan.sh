#!/bin/bash

DOMAIN_FILE=$1
PROBLEM_FILE=$2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -rf "$SCRIPT_DIR/output"
mkdir "$SCRIPT_DIR/output"

if [[ ! -f "$DOMAIN_FILE" || ! -f "$PROBLEM_FILE" ]]; then
    echo "ERROR"
    echo "Error: domain or problem file is missing!"
    exit 1
fi


if !("$SCRIPT_DIR/optic-clp" -b "$DOMAIN_FILE" "$PROBLEM_FILE" > "$SCRIPT_DIR/output/plan.txt" 2>/dev/null); then
    echo "ERROR"
    echo "Error: Optic returned with error"
    exit 1
fi

if !(grep -q ";;;; Solution Found" "$SCRIPT_DIR/output/plan.txt"); then
    echo "ERROR"
    echo "Error: Solution not found in the plan file."
    exit 1
fi

awk '/;;;; Solution Found/ {found=1; count=3; next} found && count-- <= 0' "$SCRIPT_DIR/output/plan.txt"