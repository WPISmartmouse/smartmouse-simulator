// hacks to get around horrible arduino code
#ifdef abs
#undef abs
#endif
#ifdef round
#undef round
#endif

#undef A1
#undef B1

#undef this_cannot_possibly_be_real

#include <Eigen/Dense>
