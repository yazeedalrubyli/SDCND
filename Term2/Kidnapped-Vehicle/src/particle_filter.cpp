/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

// Thanks to Jeremy Shannon, Some ideas taken from his repo.
// https://github.com/jeremy-shannon/CarND-Kidnapped-Vehicle-Project

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/*
     1. Set the number of particles.
     2. Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
     4. Add random Gaussian noise to each particle.
     */
    
    num_particles = 100;
    default_random_engine gen;
    
    // Create Normal (Gaussian) Distribution to Each Particle
    normal_distribution<double> dist_x(0, std[0]);
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);
    
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = x;
        p.y = y;
        p.theta = theta;
        p.weight = 1.0;
        
        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);
        
        particles.push_back(p);
    }
    
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    /*
     1. Add measurements to each particle.
     2. Add random Gaussian noise.
     
     NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
     Useful Links:
     http://www.cplusplus.com/reference/random/default_random_engine/
     http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     */
    
    default_random_engine gen;
    
    // Create Normal (Gaussian) Distribution to Each Particle
    normal_distribution<double> x(0, std_pos[0]);
    normal_distribution<double> y(0, std_pos[1]);
    normal_distribution<double> theta(0, std_pos[2]);
    
    for (int i = 0; i < num_particles; i++) {
        
        if (fabs(yaw_rate) < 0.00001) {
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        }
        else {
            particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }
        
        particles[i].x += x(gen);
        particles[i].y += y(gen);
        particles[i].theta += theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    /*
     1. Find the predicted measurement that is closest to each observed measurement.
     2. assign the observed measurement to this particular landmark.
     
     NOTE: this method will NOT be called by the grading code. But you will probably find it useful to implement this method and use it as a helper during the updateWeights phase.
     */
    for (unsigned int i = 0; i < observations.size(); i++) {
        
        LandmarkObs obs = observations[i];
        
        double min_dis = numeric_limits<double>::max();
        
        int map_id = -1;
        
        for (unsigned int j = 0; j < predicted.size(); j++) {
            LandmarkObs p = predicted[j];
            
            double current_dis = dist(obs.x, obs.y, p.x, p.y);
            
            if (current_dis < min_dis) {
                min_dis = current_dis;
                map_id = p.id;
            }
        }
        
        observations[i].id = map_id;
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    /*
     1. Update the weights of each particle using a mult-variate Gaussian distribution.
     2. assign the observed measurement to this particular landmark.
     
     NOTE: You can read more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located according to the MAP'S coordinate system. You will need to transform between the two systems. The following is a good resource for the theory: https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm and the following is a good resource for the actual equation to implement (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html
     */
    
    for (int i = 0; i < num_particles; i++) {
        
        double particle_x = particles[i].x;
        double particle_y = particles[i].y;
        double particle_theta = particles[i].theta;
        
        vector<LandmarkObs> predictions;
        
        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            
            float landmark_x = map_landmarks.landmark_list[j].x_f;
            float landmark_y = map_landmarks.landmark_list[j].y_f;
            int landmark_id = map_landmarks.landmark_list[j].id_i;

            if (fabs(landmark_x - particle_x) <= sensor_range && fabs(landmark_y - particle_y) <= sensor_range) {
                
                predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
            }
        }
        
        vector<LandmarkObs> obs_transformed;
        for (unsigned int j = 0; j < observations.size(); j++) {
            double t_x = cos(particle_theta)*observations[j].x - sin(particle_theta)*observations[j].y + particle_x;
            double t_y = sin(particle_theta)*observations[j].x + cos(particle_theta)*observations[j].y + particle_y;
            obs_transformed.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
        }

        dataAssociation(predictions, obs_transformed);
        
        particles[i].weight = 1.0;
        
        for (unsigned int j = 0; j < obs_transformed.size(); j++) {
            
            double obs_x, obs_y, pr_x, pr_y;
            obs_x = obs_transformed[j].x;
            obs_y = obs_transformed[j].y;
            
            int associated_prediction = obs_transformed[j].id;
            
            for (unsigned int k = 0; k < predictions.size(); k++) {
                if (predictions[k].id == associated_prediction) {
                    pr_x = predictions[k].x;
                    pr_y = predictions[k].y;
                }
            }
            
            double s_x = std_landmark[0];
            double s_y = std_landmark[1];
            double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-obs_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-obs_y,2)/(2*pow(s_y, 2))) ) );
            
            particles[i].weight *= obs_w;
        }
    }
}

void ParticleFilter::resample() {
	/*
     1. Resample particles with replacement with probability proportional to their weight.
     
     NOTE: You may find std::discrete_distribution helpful here.
     http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
    vector<Particle> new_particles;

    default_random_engine gen;

    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }

    discrete_distribution<int> discrete_dist(0, num_particles-1);
    auto index = discrete_dist(gen);

    double max_weight = *max_element(weights.begin(), weights.end());

    uniform_real_distribution<double> unirealdist(0.0, max_weight);

    double beta = 0.0;

    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }

    particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
