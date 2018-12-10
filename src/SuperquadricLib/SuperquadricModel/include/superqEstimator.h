#ifndef SUPERQESTIMATOR_H
#define SUPERQESTIMATOR_H

namespace SuperqModel {

/**
* \class SuperqModel::Superq_NLP
* \headerfile superqEstimator.h <SuperquadricModel/include/superqEstimator.h>
*
* \brief A class from SuperqModel namespace.
*
* This class implements an IpOpt optimizer for reconstructing a superquadric from point clouds.
*/
class SuperqEstimator
{
public:
    /**
    * Constructor
    */
    SuperqEstimator();

    /**
    * Destructory
    */
    virtual ~SuperqEstimator();


};


}

#endif
