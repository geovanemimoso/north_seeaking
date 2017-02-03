#ifndef north_seeking_TYPES_HPP
#define north_seeking_TYPES_HPP

#include <base/Time.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
using namespace boost;
using namespace boost::accumulators;

namespace north_seeking {

    typedef accumulator_set<double, features<tag::density> > accumulator;
    typedef iterator_range<std::vector<std::pair<double, double> >::iterator > histogram_type;

    struct point {
        double x;
        double y;
        double error;
        base::Time sample_time;
    };
}

#endif
