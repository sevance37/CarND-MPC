#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"]; 
          double a = j[1]["throttle"];
          
          // transform the way points into the cars frame of reference
          size_t n_pts = ptsx.size();
          Eigen::VectorXd traj_x(n_pts);
          Eigen::VectorXd traj_y(n_pts);
          for (size_t i = 0; i < n_pts; i++) { 
            traj_x[i] =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-py)*sin(psi);
            traj_y[i] = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-py)*cos(psi);
          }
          
          // fit a cubic polynomail to the waypoints.
          auto coeffs = polyfit(traj_x, traj_y, 3);
          
          // adjust for latency of 100ms
          double Lf = 2.67;
          double dt = 0.1;        // the latency of 100ms
          v = v*(1609.34/3600.0); // change from miles per hour to meters per second
          delta = -delta;         // adjust from steering angle to psi
          
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          double f_0 = polyeval(coeffs,0);
          double f_p_0 = coeffs[1];
          double psi_des_0 = atan(f_p_0);
          double epsi_0 = 0 - psi_des_0;
          
          double x_l = 0 + v*cos(0)*dt;
          double y_l = 0 + v*sin(0)*dt;
          double psi_l = 0 + (v/Lf)*delta*dt;
          double v_l = v + a*dt;
          double epsi_l = epsi_0 + (v/Lf)*delta*dt;
          double cte_l = (f_0 - 0) + v*sin(epsi_0)*dt;
          
          Eigen::VectorXd state(6);
          state << x_l, y_l, psi_l, v_l, cte_l, epsi_l;
          //cout << "state: " << state << endl;
          
          // Optimize
          auto vars = mpc.Solve(state, coeffs);
          
          double steer_value = -vars[0]/deg2rad(25);
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          //Display the MPC predicted trajectory 
          size_t n_mpc = (vars.size()-2)/2;
          vector<double> mpc_x_vals(n_mpc);
          vector<double> mpc_y_vals(n_mpc);
          for (size_t i = 0; i < n_mpc; i++) {
            mpc_x_vals[i] =  (vars[2*i+2] - x_l)*cos(psi_l) + (vars[2*i+3] - y_l)*sin(psi_l);
            mpc_y_vals[i] = -(vars[2*i+2] - x_l)*sin(psi_l) + (vars[2*i+3] - y_l)*cos(psi_l);
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          //Display the waypoints/reference line
          int n_nxt = n_pts;
          vector<double> next_x_vals(n_nxt);
          vector<double> next_y_vals(n_nxt);
          for (int i = 0; i < n_nxt; i++) { 
            next_x_vals[i] =  (traj_x[i] - x_l)*cos(psi_l) + (traj_y[i] - y_l)*sin(psi_l);
            next_y_vals[i] = -(traj_x[i] - x_l)*sin(psi_l) + (traj_y[i] - y_l)*cos(psi_l);
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
