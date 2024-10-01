/**
 *    \file   velocity_obstacle.cpp
 *    \brief  Defines the Velocity Obstacle collision avoidance algorithm class.
 *    \author Melih Akdağ.
 */

// Local headers.
#include <USER/Navigation/velocity_obstacle.hpp>
//#include <DUNE/DUNE.hpp>
#include <eFLL/Fuzzy.h>

static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = 180.0f/M_PI;

namespace DUNE
{
  namespace Navigation
  {
    //! Constructor.
    velocityObstacle::velocityObstacle(void):
    D_CLOSE_(0.0),
    D_SAFE_(0.0),
	KAPPA_(0.0),
	K_P_(0.0)
    {}

    //! Destructor.
    velocityObstacle::~velocityObstacle()
    {
    }
	
	void
	velocityObstacle::create(double D_CLOSE, double D_SAFE, double KAPPA, double K_P, int VO_METHOD)
	{
		D_CLOSE_ = D_CLOSE;
		D_SAFE_ = D_SAFE;
		KAPPA_ = KAPPA;
		K_P_ = K_P;

		Chi_ca_.resize(13);
		Chi_ca_ << -90.0,-75.0,-60.0,-45.0,-30.0,-15.0,0.0,15.0,30.0,45.0,60.0,75.0,90.0;
		Chi_ca_ *= DEG2RAD;

		P_ca_.resize(4);
		P_ca_ << 0.0, 0.25, 0.5, 1.0;

		vo_method = VO_METHOD; 	// VO=1, RVO=2
	}


	Eigen::Vector2d
	velocityObstacle::computeVelocityDesired(double psi_des, double U_des)
	{
		Eigen::Vector2d V_des;
		V_des(0) = U_des * std::cos(psi_des);
		V_des(1) = U_des * std::sin(psi_des);
		return V_des;
	}


	std::tuple<double, double>  
	velocityObstacle::calculateDesiredCourseAndSpeed(const Eigen::Vector2d& V_opt)
	{
		double U_des = V_opt.norm();
		double psi_des = atan2(V_opt(1), V_opt(0));
		psi_des = normalize_angle(psi_des);
		return std::make_tuple(psi_des, U_des);
	}



	std::tuple<double, double, double>  
	velocityObstacle::velocityUpdate(double psi_des, double U_des, const std::vector<double>& asv_state, const Math::Matrix& obst_states)
	{
		Eigen::Vector2d Vs, Ps, Vo, Po, trans_Vo_Vs;

		trans_Vo_Vs(0) = 0.0;
		trans_Vo_Vs(1) = 0.0;

		double dist, theta_o_s, theta_obst, theta_obst_left, theta_obst_right, psi_desired, U_desired, cost;
		
		// A matrix with a row of [trans_Vo_Vs(0), trans_Vo_Vs(1), bound_left(0), bound_left(1), bound_right(0), bound_right(1), dist, 2*obs_radius] for each obstacle
		Math::Matrix VO_all;
		VO_all.resize(obst_states.rows(), 13); 

		Ps(0) = asv_state[0];
		Ps(1) = asv_state[1];
		Vs(0) = asv_state[3] * std::cos(asv_state[2]);
		Vs(1) = asv_state[3] * std::sin(asv_state[2]);
		
		double obs_radius = D_SAFE_;

		// Create velocity obstacles for each obstacle
		for (int i=0; i<obst_states.rows(); i++)
		{
			Po(0) = obst_states(i,0);
			Po(1) = obst_states(i,1);
			Vo(0) = obst_states(i,11)*std::cos(Angles::radians(obst_states(i,10)));
			Vo(1) = obst_states(i,11)*std::sin(Angles::radians(obst_states(i,10)));

			if (vo_method==1)	// Classical VO
			{
				trans_Vo_Vs(0) = Ps(0)+Vo(0);
				trans_Vo_Vs(1) = Ps(1)+Vo(1);
			}	
			else if (vo_method==2)	// Reciprocal VO
			{
				trans_Vo_Vs(0) = Ps(0)+0.5*(Vo(0)+Vs(0));
				trans_Vo_Vs(1) = Ps(1)+0.5*(Vo(1)+Vs(1));
			}

			dist = distance(Ps, Po);
			theta_o_s = atan2(Po(1)-Ps(1), Po(0)-Ps(0));
			if (obs_radius > dist) //if (2*obs_radius > dist)
			{
				dist = obs_radius; //dist = 2*obs_radius;
			}
			theta_obst = asin(obs_radius/dist); //theta_obst = asin(2*obs_radius/dist);
			theta_obst_left = theta_o_s + theta_obst;
			theta_obst_right = theta_o_s - theta_obst;

			VO_all(i, 0) = trans_Vo_Vs(0);
			VO_all(i, 1) = trans_Vo_Vs(1);
			VO_all(i, 2) = std::cos(theta_obst_left);
			VO_all(i, 3) = std::sin(theta_obst_left);
			VO_all(i, 4) = std::cos(theta_obst_right);
			VO_all(i, 5) = std::sin(theta_obst_right);
			VO_all(i, 6) = dist;
			VO_all(i, 7) = obs_radius; //VO_all(i, 7) = 2*obs_radius;
			VO_all(i, 8) = Vo(0);
			VO_all(i, 9) = Vo(1);
			VO_all(i, 10) = Po(0);
			VO_all(i, 11) = Po(1);
			VO_all(i, 12) = obst_states(i, 16); // Self vessel's COLREG rule responsibility towards the obstacle
		}
		std::tie(psi_desired, U_desired, cost) = intersect(Ps, Vs, psi_des, U_des, VO_all);
		
		double psi_os_temp = normalize_angle(psi_desired) - normalize_angle(psi_des); //asv_state[2];
		double u_os_temp = U_desired / U_des; //asv_state[3];
		//std::cout << "Psi_d:" << psi_des << " + Psi_off:" << psi_os_temp << " = " << Angles::normalizeRadian(psi_des+psi_os_temp) << " Psi_des:" << psi_desired << std::endl;
		//std::cout << "U_d:" << U_des << " + U_off:" << u_os_temp << " = "  << U_des*u_os_temp << " U_des:" << U_desired << std::endl; 
		return std::make_tuple(psi_os_temp, u_os_temp, cost);
	}



	std::tuple<double, double, double> 
	velocityObstacle::intersect(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Vs, double psi_des, double U_des, const Math::Matrix& VO_all)
	{
		Eigen::Vector2d V_desired = computeVelocityDesired(psi_des, U_des);

		double psi_desired, U_desired;
		//double norm_v = std::sqrt(pow(V_desired(0),2) + pow(V_desired(1),2));
		std::vector<Eigen::Vector2d> suitable_V;
		std::vector<Eigen::Vector2d> unsuitable_V;
		Eigen::Vector2d new_v;

		for (int i=0; i<Chi_ca_.size(); i++)
		{
			for (int j=0; j<P_ca_.size(); j++)
			{
				new_v(0) = U_des*P_ca_[j] * std::cos(Angles::normalizeRadian(psi_des+Chi_ca_[i]));
				new_v(1) = U_des*P_ca_[j] * std::sin(Angles::normalizeRadian(psi_des+Chi_ca_[i]));
				bool suit = true;
				
				for (int k=0; k<VO_all.rows(); k++)
				{
					double trans_Vo_Vs_0 = VO_all(k,0);
					double trans_Vo_Vs_1 = VO_all(k,1);
					double theta_obst_left_0 = VO_all(k,2);
					double theta_obst_left_1 = VO_all(k,3);
					double theta_obst_right_0 = VO_all(k,4);
					double theta_obst_right_1 = VO_all(k,5);
					double dif_0 = new_v(0)+Ps(0)-trans_Vo_Vs_0;
					double dif_1 = new_v(1)+Ps(1)-trans_Vo_Vs_1;
					double theta_dif = atan2(dif_1, dif_0);
					double theta_right = atan2(theta_obst_right_1, theta_obst_right_0);
					double theta_left = atan2(theta_obst_left_1, theta_obst_left_0);
					
					//if ( (VO_all(k,6) <= D_CLOSE_) && (in_between(theta_right, theta_dif, theta_left)) )
					if (in_between(theta_right, theta_dif, theta_left))
					{
						suit = false;
						break;
					}
				}
				if (suit)
				{
					suitable_V.push_back(new_v);
				}
				else
				{
					unsuitable_V.push_back(new_v);
				}
			}
		}
		new_v(0) = V_desired(0);
		new_v(1) = V_desired(1);
		bool suit = true;
		for (int i=0; i<VO_all.rows(); i++)
		{
			double trans_Vo_Vs_0 = VO_all(i,0);
			double trans_Vo_Vs_1 = VO_all(i,1);
			double theta_obst_left_0 = VO_all(i,2);
			double theta_obst_left_1 = VO_all(i,3);
			double theta_obst_right_0 = VO_all(i,4);
			double theta_obst_right_1 = VO_all(i,5);
			double dif_0 = new_v(0)+Ps(0)-trans_Vo_Vs_0;
			double dif_1 = new_v(1)+Ps(1)-trans_Vo_Vs_1;
			double theta_dif = atan2(dif_1, dif_0);
			double theta_right = atan2(theta_obst_right_1, theta_obst_right_0);
			double theta_left = atan2(theta_obst_left_1, theta_obst_left_0);
			if (in_between(theta_right, theta_dif, theta_left))
			{
				suit = false;
				break;
			}
		}
		if (suit)
		{
			suitable_V.push_back(new_v);
		}
		else
		{
			unsuitable_V.push_back(new_v);
		}

		Eigen::Vector2d Vs_opt; 
		double cost = std::numeric_limits<double>::infinity();

		if (!suitable_V.empty()) 	// Suitable velocity found
		{
			for (const auto& vel_suitable : suitable_V)
			{
				// Check whether ownship is turning to port
				double psi_ref = atan2(V_desired(1), V_desired(0));
				psi_ref = normalize_angle(psi_ref);
				
				double psi_s = atan2(Vs(1), Vs(0));
				psi_s = normalize_angle(psi_s);

				double psi_next = atan2(vel_suitable(1), vel_suitable(0));
				psi_next = normalize_angle(psi_next);

				double mu = 0.0;
				bool port_turn = false;
				if (normalize_angle(psi_next - psi_s) < 0)
				{
					port_turn = true;
				}

				double cost_ = 0.0;
				
				for (int i=0; i<VO_all.rows(); i++)
				{
					Eigen::Vector2d Vo, Po;
					Vo(0) = VO_all(i,8);
					Vo(1) = VO_all(i,9);
					Po(0) = VO_all(i,10);
					Po(1) = VO_all(i,11);

					double rule = VO_all(i, 12); // rule => 0.0=None, 1.0=HO-GW, 2.0=ON-SO, 3.0=OG, 4.0=CR-SO, 5.0=CR-GW

					if ((VO_all(i,6) <= D_CLOSE_) && (rule == 1.0 || rule == 4.0 || rule == 5.0) && port_turn==true)
					{
						mu = KAPPA_*std::fabs(normalize_angle(psi_s - psi_next));
					}
					else
					{
						mu = 0.1*KAPPA_*std::fabs(normalize_angle(psi_s - psi_next));
					}

					cost_ = K_P_*std::pow((1-vel_suitable.norm()),2) + mu;

					//std::cout << "Psi_next:" << normalize_angle_360(psi_next)*RAD2DEG << " Cost_obst:" <<  cost_obst << " Cost:" << cost_ <<std::endl;

					if (cost_ < cost)
					{
						cost = cost_;
						Vs_opt = vel_suitable;
					}
				}
			}
		}
		else 	// Suitable velocity not found
		{
			Eigen::Vector2d Vs_opt_temp;  // Temporary variable for storing the optimal velocity
    		double min_tc_sum = std::numeric_limits<double>::infinity();  // Initialize with infinity
	
        	for (const auto& unsuit_v : unsuitable_V) 
        	{
        		double tc_sum = 0.0;  // Accumulator for tc values
        	    
        	    for (int i=0; i<VO_all.rows(); i++)
        	    {
        	        double trans_Vo_Vs_0 = VO_all(i,0);
					double trans_Vo_Vs_1 = VO_all(i,1);
					double theta_obst_left_0 = VO_all(i,2);
					double theta_obst_left_1 = VO_all(i,3);
					double theta_obst_right_0 = VO_all(i,4);
					double theta_obst_right_1 = VO_all(i,5);
        	        double dist = VO_all(i,6);
        	        double rad = VO_all(i,7);
        	        double dif_0 = unsuit_v(0) + Ps(0) - trans_Vo_Vs_0;
        	        double dif_1 = unsuit_v(1) + Ps(1) + trans_Vo_Vs_1;
        	        double theta_dif = atan2(dif_1, dif_0);
        	        double theta_right = atan2(theta_obst_right_1, theta_obst_right_0);
        	        double theta_left = atan2(theta_obst_left_1, theta_obst_left_0);
	
        	        if (in_between(theta_right, theta_dif, theta_left)) 
        	        {
        	            double small_theta = std::abs(theta_dif - 0.5 * (theta_left + theta_right));
	
        	            if (std::abs(dist * std::sin(small_theta)) >= rad) 
        	            {
        	                rad = std::abs(dist * std::sin(small_theta));
        	            }
	
        	            double big_theta = std::asin(std::abs(dist * std::sin(small_theta)) / rad);
        	            double dist_tg = std::abs(dist * std::cos(small_theta)) - std::abs(rad * std::cos(big_theta));
	
        	            if (dist_tg < 0) 
        	            {
        	                dist_tg = 0;
        	            }

        	            double tc_v = dist_tg / (std::sqrt(pow(dif_0, 2) + pow(dif_1, 2)));
        	            tc_sum += tc_v;
        	        }
        	    }
        	    // If the current tc_sum is smaller than the minimum, update Vs_opt_temp
        		if (tc_sum < min_tc_sum) 
        		{
        		    min_tc_sum = tc_sum;
        		    Vs_opt_temp = unsuit_v;
        		}
        	}
        	Vs_opt = Vs_opt_temp;
		} 
		std::tie(psi_desired, U_desired) = calculateDesiredCourseAndSpeed(Vs_opt);
		return std::make_tuple(psi_desired, U_desired, cost);
	}


	bool 
	velocityObstacle::in_between(double theta_right, double theta_dif, double theta_left) 
	{
    	if (std::abs(theta_right - theta_left) <= M_PI) 
    	{
    	    if (theta_right <= theta_dif && theta_dif <= theta_left)
    	    { 
    	    	return true;
    		}
    		else
    		{
    			return false;
    		}
    	} 
    	else 
    	{
    	    if (theta_left < 0 && theta_right > 0) 
    	    {
    	        theta_left += 2 * M_PI;
    	        if (theta_dif < 0) 
    	        {
    	        	theta_dif += 2 * M_PI;
    	        }
    	        if (theta_right <= theta_dif && theta_dif <= theta_left)
    	        {
    	        	return true;
    	        }
    	        else
    	        {
    	        	return false;
    	        }
    	    }
    	    if (theta_left > 0 && theta_right < 0) 
    	    {
    	        theta_right += 2 * M_PI;
    	        if (theta_dif < 0) 
    	        {
    	        	theta_dif += 2 * M_PI;
    	        }
    	        if (theta_left <= theta_dif && theta_dif <= theta_right)
    	        {
    	        	return true;
    	        }
    	        else
    	        {
    	        	return false;
    	        }
    	    }
else
    	    {
    	    	return false;
    	    }
    	}
	}


	double 
	velocityObstacle::distance(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Po)
	{
		Eigen::Vector2d d;
		double dist;
		d(0) = Po(0)-Ps(0);
		d(1) = Po(1)-Ps(1);
		dist = d.norm();
		return dist;
	}


	
	std::tuple<double, double> 
	velocityObstacle::calculateCPA(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Vs, const Eigen::Vector2d& Po, const Eigen::Vector2d& Vo)
	{
		double delta_alpha = 0.0;
		double delta_chi = 0.0;
		double D_r, U_r, alpha_r, chi_r, beta, dcpa, tcpa;

		// Distance between self and targetship
        D_r = distance(Ps, Po);

        // Relative speed between self and targetship
        U_r = distance(Vs, Vo);

        // Alpha_r (True bearing of the targetship)
		if ((Po(1) - Ps(1) >= 0.0) && (Po(0) - Ps(0) >= 0.0))
		{
		    delta_alpha = 0.0;
		}
		else if ((Po(1) - Ps(1) >= 0.0) && (Po(0) - Ps(0)) < 0)
		{
		    delta_alpha = 0.0;
		}
		else if ((Po(1) - Ps(1) < 0.0) && (Po(0) - Ps(0)) < 0)
		{
		    delta_alpha = 2*M_PI;
		}
		else if ((Po(1) - Ps(1) < 0.0) && (Po(0) - Ps(0)) >= 0)
		{
		    delta_alpha = 2*M_PI;
		}
		alpha_r = atan2((Po(1)-Ps(1)), (Po(0)-Ps(0))) + delta_alpha; 

		// Chi_r (Relative course of targetship - from 0 to U_r)
		if ((Vo(1) - Vs(1) >= 0) && (Vo(0) - Vs(0) >= 0))
		{
		    delta_chi = 0;
		}
		else if ((Vo(1) - Vs(1) >= 0) && (Vo(0) - Vs(0) < 0))
		{
		    delta_chi = 0;
		}
		else if ((Vo(1) - Vs(1) < 0) && (Vo(0) - Vs(0) < 0))
		{
		    delta_chi = 2*M_PI;
		}
		else if ((Vo(1) - Vs(1) < 0) && (Vo(0) - Vs(0) >= 0))
		{
		    delta_chi = 2*M_PI;
		}
		chi_r = atan2((Vo(1)-Vs(1)), (Vo(0)-Vs(0))) + delta_chi;

		// beta
        beta = chi_r - alpha_r - M_PI;

        // DCPA and TCPA
        dcpa = std::abs(D_r * sin(beta));
        tcpa = (D_r * cos(beta)) / (std::abs(U_r)+1);

        return std::make_tuple(dcpa, tcpa);
	}


	// Normalize angle
	inline double 
	velocityObstacle::normalize_angle(double angle)
	{
		while(angle <= -M_PI) angle += 2*M_PI;
		while (angle > M_PI) angle -= 2*M_PI;
		return angle;
	}

	inline double 
	velocityObstacle::normalize_angle_360(double angle)
	{
		angle = fmod(angle,2*M_PI);
		if(angle < 0)
		angle += 2*M_PI;
		return angle;
	}

  }
}