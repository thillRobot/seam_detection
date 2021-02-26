#include <teaser/registration.h>

teaser::RobustRegistrationSolver::Params params;
teaser::RobustRegistrationSolver solver(params);
solver.solve(src, dst); // assuming src & dst are 3-by-N Eigen matrices
auto solution = solver.getSolution();