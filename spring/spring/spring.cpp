#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

  // declare variables
  double m, k, x, v, t_max, dt, t, a;
  vector<double> t_list, x_list, v_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;

  // simulation time and timestep
  t_max = 1000;
  dt = 0.001;
/*
  // Euler integration
  for (t = 0; t <= t_max; t = t + dt) {

    // append current state to trajectories
    t_list.push_back(t);
    x_list.push_back(x);
    v_list.push_back(v);

    // calculate new position and velocity
    a = -k * x / m;
    x = x + dt * v;
    v = v + dt * a;

  }

  // Write the trajectories to file
  ofstream fout;
  fout.open("/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/spring/trajectories1.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }
   */
    
    
    int i = 0;
    // Verlet integration
    for (t = 0; t <= t_max; t = t + dt) {
        if (i == 0){
            // append current state to trajectories
            t_list.push_back(t);
            x_list.push_back(x);
            v_list.push_back(v);

            // calculate new position and velocity
            a = -k * x / m;
            x = x + dt * v;
            v = v + dt * a;
            i++;
        }
        else{
            t_list.push_back(t);
            x_list.push_back(x);
            v_list.push_back(v);
            
            a = -k * x / m;
            x = 2 * x - x_list[i - 1] + a * (dt * dt);
            v = (x - x_list[i]) / dt;
            i++;
        }

    }

    // Write the trajectories to file
    ofstream fout;
    fout.open("/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/spring/trajectories2.txt");
    if (fout) { // file opened successfully
      for (int i = 0; i < t_list.size(); i = i + 1) {
        fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
      }
    } else { // file did not open successfully
      cout << "Could not open trajectory file for writing" << endl;
    }
    

  /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
