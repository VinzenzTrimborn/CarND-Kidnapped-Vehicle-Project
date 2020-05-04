/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 50;
  std::default_random_engine gen;
  double std_x, std_y, std_theta;
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];
  
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);
  
  particles.clear();
  weights.clear();
  
  for(int i = 0; i < num_particles; i++){
    double sample_x = dist_x(gen);
    double sample_y = dist_y(gen);
    double sample_theta = dist_theta(gen);
    
    Particle particle;
    particle.id = i;
    particle.x = sample_x;
    particle.y = sample_y;
    particle.theta = sample_theta;
    particle.weight = 1.0;
    
    particles.push_back(particle);
    weights.push_back(1.0);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  std::default_random_engine gen;
  double std_x, std_y, std_theta;
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];
  
  std::normal_distribution<double> dist_x(0, std_x);
  std::normal_distribution<double> dist_y(0, std_y);
  std::normal_distribution<double> dist_theta(0, std_theta);
  
  
  for(int i = 0; i < num_particles; i++){
     Particle& particle = particles[i];
     double theta_0 = particle.theta;
    
    if(fabs(yaw_rate) < 0.00001){
      particle.x = velocity * delta_t * cos(theta_0);
      particle.y = velocity * delta_t * sin(theta_0);
    }else{
     particle.x += (velocity/yaw_rate)*(sin(theta_0+yaw_rate*delta_t)-sin(theta_0));
     particle.y += (velocity/yaw_rate)*(cos(theta_0)-cos(theta_0+yaw_rate*delta_t));
     particle.theta += yaw_rate*delta_t;
    
     particle.x += dist_x(gen);
     particle.y += dist_y(gen);
     particle.theta += dist_theta(gen);
    }
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  weights.clear();
  for(int i = 0; i < particles.size(); i++){
    double probability=1.0;
    vector<LandmarkObs> observations_cor;
    vector<LandmarkObs> landmarks;
    
    //transform the observations. I need this loop since the transformation for every landmark is the same. It would be pointless to do this in the next loop
    for(int j = 0; j < observations.size(); j++){
      LandmarkObs observation_cor;
      //transform the observations
      observation_cor.x = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
      observation_cor.y = particles[i].y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;
      observation_cor.id = -1;
      observations_cor.push_back(observation_cor);
    }
    
    //I go through all the landmarks.
    for(int j = 0; j < map_landmarks.landmark_list.size(); j++){
      double delta = dist(particles[i].x,particles[i].y,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f);
       //get Landmarks from the map which are in sensor range
      if(delta < sensor_range){
        LandmarkObs landmark;
        landmark.id = map_landmarks.landmark_list[j].id_i;
        landmark.x = map_landmarks.landmark_list[j].x_f;
        landmark.y = map_landmarks.landmark_list[j].y_f;
        landmarks.push_back(landmark);
        
        //go through all observations and compute the distance from the landmark to the observation
        vector<double> distances;
        for(int k = 0; k < observations.size(); k++){
          double distance = dist(landmark.x,landmark.y,observations_cor[k].x,observations_cor[k].y);
          distances.push_back(distance);
        }
        //find the observation that is closest to the landmark
        auto smallest = std::min_element(distances.begin(),distances.end());
        LandmarkObs nearest_neighbor_observation = observations_cor[std::distance(distances.begin(), smallest)];
        
        double sigma_x = std_landmark[0];
        double sigma_y = std_landmark[1];
        double sigma_x2 = pow(sigma_x,2);
        double sigma_y2 = pow(sigma_y,2);
        double normalizer = (1.0/2.0*M_PI*sigma_x*sigma_y);
        
        double x_ob = nearest_neighbor_observation.x;
        double y_ob = nearest_neighbor_observation.y;
        
        double q1=(pow((x_ob- landmark.x),2)/(2.0 * sigma_x2));
        double q2=(pow((y_ob-landmark.y),2)/(2.0 * sigma_y2));
        double e = exp(-1.0*(q1+q2));
        probability *= normalizer *e;
      }
    }
   weights.push_back(probability);
   particles[i].weight = probability;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<> distribution(weights.begin(), weights.end());
  vector<Particle> new_particles;
  int index;
  
  for (int i = 0; i < num_particles; i++) {
    index = distribution(gen);
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
