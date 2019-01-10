#ifndef GRASPCOMPUTATION_H
#define GRASPCOMPUTATION_H

#include "superquadricEstimator.h"
#include "graspPoses.h"

namespace SuperqGrasp {

/**
* \class LibTemplateCMake::aClass
* \headerfile template-lib.h <TemplateLib/templatelib.h>
*
* \brief A class from LibTemplateCMake namespace.
*
* This class that does a summation.
*/
class graspComputation : public Ipopt::TNLP
{
public:
};


class EstimatorApp
{
public:
     SuperqGrasp::GraspPoses computeGraspPoses(SuperqModel::IpoptParam &pars, SuperqModel::Superquadric);
};


}

#endif // LIB_TEMPLATE_CMAKE_H
