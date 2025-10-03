#!/bin/bash

# Exit if any command fails


set -e

SRC="/home/$USER/shared-folder/lis-2.1.10/lis-2.1.10/test/test1.c"
OUT="test1"

 echo "Loading environment and Lis module..."
 source /u/sw/etc/bash.bashrc
 module load gcc-glibc
 module load lis

# Check for required environment variables
if [ -z "$mkLisInc" ] || [ -z "$mkLisLib" ]; then
    echo "Error: mkLisInc and mkLisLib must be set."
    echo "Example: export mkLisInc=/path/to/include"
    echo "         export mkLisLib=/path/to/lib"
    exit 1
fi

# Compile the program
echo "Compiling test1.c..."
mpicc -DUSE_MPI -I"${mkLisInc}" -L"${mkLisLib}" -llis $SRC -o $OUT

# Check if compilation succeeded
if [ ! -f test1 ]; then
    echo "Compilation failed!"
    exit 1
fi

# Check if user provided input arguments
if [ $# -lt 3 ]; then
    echo "Usage: $0 MATRIX.mtx RHS_OPTION OUTPUT_FILE.mxt [HISTORY.txt]"
    exit 1
fi

# Set arguments
MATRIX="$1"
RHS_OPTION="$2"
OUTPUT_FILE="$3"
HISTORY_FILE="$4"
I="$5"  # Default history file
PRECOND="$6"
TOL="$7"

# Run the program with 4 MPI processes
echo "Running test1 with 4 MPI processes..."
mpirun -n 4 ./test1 "$MATRIX" "$RHS_OPTION" "$OUTPUT_FILE" "$HISTORY_FILE" -i "$I" -p "$PRECOND" -tol "$TOL"



