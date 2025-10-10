#!/bin/bash
source /u/sw/etc/profile
module load gcc-glibc
module load lis

#
mpirun -n 4 ./eigen1 L_s.mtx eigvec.txt hist.txt -e pi -i cg -p ic -etol 1.e-8