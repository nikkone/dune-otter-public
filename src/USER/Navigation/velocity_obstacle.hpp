/**
 *    \file   velocity_obstacle.hpp
 *    \brief  Declares the Velocity Obstacle collision avoidance algorithm class.
 *    \author Melih Akdağ.
 */

#ifndef DUNE_NAVIGATION_VELOCITY_OBSTACLE_HPP_INCLUDED_
#define DUNE_NAVIGATION_VELOCITY_OBSTACLE_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <Eigen/Dense>
#include <eFLL/Fuzzy.h>

namespace DUNE
{
  namespace Navigation
  {
	  // Export DLL Symbol.
    class DUNE_DLL_SYM velocityObstacle;
    
		class velocityObstacle
		{
			public:
			/// Constructor
			velocityObstacle();
		
			/// Destructor
			~velocityObstacle();
		
		
			void create(double D_CLOSE, double D_SAFE, double KAPPA, double K_P, int VO_METHOD);
		
			std::tuple<double, double, double> velocityUpdate(double psi_des, double U_des, const std::vector<double>& asv_state, const Math::Matrix& obst_states);
			
			Eigen::Vector2d Vs_opt_prev;

			Eigen::VectorXd Chi_ca_;
			Eigen::VectorXd P_ca_;
			
			/**
			 * @brief Returns the distance where COLREGS are said to apply [m].
			 */
			double getDClose();
			/**
	 		* @brief Returns the minimum distance which is considered as safe [m].
	 		*/
			double getDSafe();
			/**
			 * @brief Returns the cost of not complying with the COLREGS.
			 */
			double getKappa();
			/**
			 * @brief Returns the cost of deviating from the nominal speed.
			 */
			double getKP();
			/**
			 * @brief Returns the VO method.
			 */
			int getVoMethod();
			
			void setDClose(double d_close);
			void setDSafe(double d_safe);
			void setKappa(double kappa);
			void setKdP(double K_P);
			void setVoMethod(int vo_method);

			double D_CLOSE_;
			int vo_method;

			private:
			Eigen::Vector2d computeVelocityDesired(double psi_des, double U_des);
			std::tuple<double, double> calculateDesiredCourseAndSpeed(const Eigen::Vector2d& V_opt);
			std::tuple<double, double, double> intersect(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Vs, double psi_des, double U_des, const Math::Matrix& VO_all);
			bool in_between(double theta_right, double theta_dif, double theta_left);
			double distance(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Po);
			std::tuple<double, double> calculateCPA(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Vs, const Eigen::Vector2d& Po, const Eigen::Vector2d& Vo);
			inline double normalize_angle(double angle); 
			inline double normalize_angle_360(double angle);

			double D_SAFE_;
			double KAPPA_;
			double K_P_;
		};
  }
}

#endif