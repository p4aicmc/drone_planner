#!/bin/bash

DOMAIN_FILE=$1
PROBLEM_FILE=$2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "$SCRIPT_DIR"
echo "$DOMAIN_FILE"
echo "$PROBLEM_FILE"

export TFD_HOME="$SCRIPT_DIR/downward"

rm -rf "$SCRIPT_DIR/output"
mkdir "$SCRIPT_DIR/output"
# rm -f "$TFD_HOME"/plan.*

if ! (cd "$SCRIPT_DIR/output" && python3 $TFD_HOME/translate/translate.py "$DOMAIN_FILE" "$PROBLEM_FILE"); then
    echo "ERROR"
    exit 1
fi

cd "$SCRIPT_DIR/output"

$TFD_HOME/preprocess/preprocess < output.sas > /dev/null 2>&1
$TFD_HOME/search/search y Y a T 10 t 5 e r O 1 C 1 p $TFD_HOME/plan < output > /dev/null 2>&1

cat $TFD_HOME/plan.1