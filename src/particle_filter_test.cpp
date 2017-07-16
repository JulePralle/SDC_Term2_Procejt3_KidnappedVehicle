/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).




	num_particles = 30;                  // number of particles
	//Particle particle;
	particles.resize(num_particles);
	weights.resize(num_particles);


	// Create normal distributions
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Set initial position for each particle based on GPS, adding Gaussian noise

	default_random_engine gen;  //random Gaussian noise

	for (int i = 0; i < num_particles; i++)
	{
		double sample_x, sample_y, sample_theta;

		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);

		particles[i].id = i;
		particles[i].x = sample_x;
		particles[i].y = sample_y;
		particles[i].theta = sample_theta;
		particles[i].weight = 1;

	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;


	// Predict each particle position using the motion model and adding random Gaussian noise
	for (int i = 0; i < num_particles; i++)
	{
		double pred_x;
		double pred_y;
		double pred_theta;

		if (fabs(yaw_rate) > 0.001)
		{
			pred_x = particles[i].x + ((velocity / yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta)));
			pred_y = particles[i].y + ((velocity / yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)));
			pred_theta = particles[i].theta + yaw_rate*delta_t;
		}
		else
		{
			pred_x = particles[i].x + (velocity *delta_t * cos(particles[i].theta));
			pred_y = particles[i].y + (velocity *delta_t * sin(particles[i].theta));
			pred_theta = particles[i].theta;
		}

        // Create normal distributions
		normal_distribution<double> dist_x(pred_x, std_pos[0]);
		normal_distribution<double> dist_y(pred_y, std_pos[1]);
		normal_distribution<double> dist_theta(pred_theta, std_pos[2]);

		double sample_x, sample_y, sample_theta;

		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);

		particles[i].x = sample_x;
		particles[i].y = sample_y;
		particles[i].theta = sample_theta;

	}

}

void ParticleFilter::dataAssociation(Map map_landmarks, std::vector<LandmarkObs>& observations_transf, double sensor_range, Particle& particle) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	// create vectors and values for the nexts steps
	std::vector<double> distances;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
	int min_dist_id = 0;
	distances.resize(observations_transf.size());


    //create the first distance to compare to the next ones
    distances[0] = dist(observations_transf[0].x, observations_transf[0].y, map_landmarks.landmark_list.x_f, map_landmarks.landmark_list[i].y_f);
    min_dist_id = 0;
    for (int k = 1; k < observations_transf.size(); k++)
    {
        distances[k] = dist(observations_transf[k].x, observations_transf[k].y, map_landmarks.landmark_list.x_f, map_landmarks.landmark_list[i].y_f);
        // find nearest observation (neighbor) for each Landmark in sensor range
        if (distances[k] < distances[min_dist_id])
        {
            min_dist_id = k;
        }
    }
    // Assign the landmark to the closest observation
    observations_transf[min_dist_id].id = map_landmarks.landmark_list.id_i;
    associations.push_back(map_landmarks.landmark_list.id_i);
    sense_x.push_back(observations_transf[min_dist_id].x);
    sense_y.push_back(observations_transf[min_dist_id].y);


	// Save associations for the particle
	SetAssociations(particle, associations, sense_x, sense_y);


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// For each particle:
	for (size_t i = 0; i < num_particles; i++)
	{
		// Transform observations to MAP coordinates
		std::vector<LandmarkObs> observations_transf(observations.size());
		for (int a = 0; a < observations.size(); a++)
		{
			observations_transf[a].x = particles[i].x + observations[a].x * cos(particles[i].theta) - observations[a].y * sin(particles[i].theta);
			observations_transf[a].y = particles[i].y + observations[a].x * sin(particles[i].theta) + observations[a].y * cos(particles[i].theta);
			observations_transf[a].id = observations[a].id;
		}


		for (int l = 0; l < map_landmarks.landmark_list.size(); l++)
        {
            double dist_lm_p = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f);
            // Check if particle distance to landmark is in sensor range
            if (dist_lm_p < sensor_range)
            {
                // Associate Landmarks with closest transfered observation
                dataAssociation(map_landmarks, observations_transf,sensor_range,particles[i]);

            }



		// Calculate new weight for each particle
		double prob = 1.0;
		for (int a = 0; a < observations.size(); a++)
		{
			int associated_landmark = observations_transf[a].id;
			if (associated_landmark > 0)
			{
				double x = observations_transf[a].x;
				double y = observations_transf[a].y;
				double x_pred = map_landmarks.landmark_list[associated_landmark - 1].x_f;
				double y_pred = map_landmarks.landmark_list[associated_landmark - 1].y_f;

				// calculate new weights using the multivariante Gaussian probability density function
				prob *= exp(-0.5 * ((pow((x - x_pred), 2) / (std_landmark[0]* std_landmark[0])) + (pow((y - y_pred), 2) / (std_landmark[1]* std_landmark[1])))) / (2 * M_PI*std_landmark[0] * std_landmark[1]);
			}

		}
		//saving the probability as weights
		particles[i].weight = prob;
		weights[i] = prob;
	}



}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


	// Resample particles based on current weights, particles with less weight are more likely to disapear
	vector<Particle> new_particles(num_particles);
	default_random_engine gen;
	std::discrete_distribution<> dis_dist(weights.begin(), weights.end());
	std::uniform_real_distribution<double> uni_dist(0.0, 1.0);

	double beta = 0.0;
	double max_weight = 0;

	for (int i = 0; i < num_particles; i++)
	{
		int index = dis_dist(gen);
		new_particles[i] = particles[index];
		new_particles[i].id = i;
	}

	particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
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
