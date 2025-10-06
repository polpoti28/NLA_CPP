#!/bin/bash
source /u/sw/etc/profile
module load gcc-glibc
module load lis

cd ../data

# Since A2 is SPD we use the Conjugate Gradient method
# We use CROUT ILU as a preconditioner.

./test1 A2.mtx w.mtx sol.mtx history.txt -tol 1.0e-14 -i cg -p 8 -maxiter 100