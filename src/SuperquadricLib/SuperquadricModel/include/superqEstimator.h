#ifndef SUPERQESTIMATOR_H
#define SUPERQESTIMATOR_H

namespace SuperqEstimator {

/**
* \class SuperqEstimator::Superq_NLP
* \headerfile superqEstimator.h <SuperquadricModel/include/superqEstimator.h>
*
* \brief A class from SuperqEstimator namespace.
*
* This class implements an IpOpt optimizer for reconstructing a superquadric from point clouds.
*/
class Superq_NLP
{
public:
    /**
    * Constructor
    */
    Superq_NLP();

    /**
    * Destructory
    */
    virtual ~Superq_NLP();


};


}

#endif
