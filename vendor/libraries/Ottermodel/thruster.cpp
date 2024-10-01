#include "thruster.hpp"
namespace Ottermodel
{
  namespace Thruster
  {
//! Model converting force to thrust actuation level (Untrimmed)
//! @param[in] force value of force currently in the motor
//! @return thrust actuation.
float forceToThrust(float force){
    if(force > 0) {
    const float weights[] = {0.01137,-7.549e-05,1.86e-07};
    return weights[0]*force+weights[1]*force*force+weights[2]*force*force*force; // ax+bx^2+cx^3
    } else if(force < 0) {
    const float weights[] = {0.01912, 0.0002268, 1.012e-06};
    return weights[0]*force+weights[1]*force*force+weights[2]*force*force*force; // ax+bx^2+cx^3
    } else { // Force is 0
    return 0.0;
    }
    return 0.0;
}
//! Model converting thrust actuation level to force (Untrimmed)
//! @param[in] thrust value of thrust currently in the motor
//! @return force.
float thrustToForce(float thrust){
    const float fweight[] = {57.04, -60.18, 3.17, 8.67, -0.0534, -2.691, 2.9435, -1.5335, 0.3177}; 
    const float bweight[] = {-121.213, -182.64, -77.6, -5.177, 0.03, 1.9913, 1.798,  0.981, 0.2564}; 

    if(thrust > 0) { // Forward Thrust
      return (fweight[0]*std::pow(thrust,4) + fweight[1]*std::pow(thrust,3) + fweight[2]*std::pow(thrust,2) + fweight[3]*thrust + fweight[4])
      /(std::pow(thrust,4) + fweight[5]*std::pow(thrust,3) + fweight[6]*std::pow(thrust,2) + fweight[7]*thrust + fweight[8]);

    } else if(thrust < 0) { // Backward Thrust
      return (bweight[0]*std::pow(thrust,4) + bweight[1]*std::pow(thrust,3) + bweight[2]*std::pow(thrust,2) + bweight[3]*thrust + bweight[4])
      /(std::pow(thrust,4) + bweight[5]*std::pow(thrust,3) + bweight[6]*std::pow(thrust,2) + bweight[7]*thrust + bweight[8]);
    } else { // Force is 0
      return 0.0;
    }
    return 0.0;
}
//! Model converting thrust actuation level to force (Untrimmed)
//! @param[in] thrust value of thrust currently in the motor
//! @return force.
float thrustToForceOld(float thrust){
    if(thrust > 0) { // Forward Thrust
      const float weights[] = {-146.4,948.7,-556.3}; 
      return weights[0]*thrust+weights[1]*thrust*thrust+weights[2]*thrust*thrust*thrust; // ax+bx^2+cx^3
    } else if(thrust < 0) { // Backward Thrust
      const float weights[] = {-99.73, -609.1, -378.6};
      return weights[0]*thrust+weights[1]*thrust*thrust+weights[2]*thrust*thrust*thrust; // ax+bx^2+cx^3
    } else { // Force is 0
      return 0.0;
    }
    return 0.0;
}
  }
}