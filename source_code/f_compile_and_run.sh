clear

export COMPILE_SCRIPT_JUMP_HARPIA_MSGS_COMPILATION_IF_ALREADY_IS=1
source ./compile.sh

export RUN_SCRIPT_DO_NOT_CLEAR=1
./run.sh ${1:+$1}