// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <Eigen/Dense>
// #include "ct/core/core.h"
// #include <ct/optcon/optcon.h>
// using namespace std;
// using namespace Eigen;
// class ChannelCommand {
// public:
//     ChannelCommand(int sample_count_to_send, int id)
//         : sample_count_to_send_(sample_count_to_send), id_(id) {}
//     int get_sample_count() const {
//         return sample_count_to_send_;
//     }
//     int get_id() const {
//         return id_;
//     }
// private:
//     int sample_count_to_send_;
//     int id_;
// };
// class ChannelInfo {
// public:
//     ChannelInfo(int samples, int time_since_last_messages_ms, int id)
//         : samples_(samples), time_since_last_messages_ms_(time_since_last_messages_ms), id_(id) {}
//     int get_samples() const {
//         return samples_;
//     }
//     int get_last_time() const {
//         return time_since_last_messages_ms_;
//     }
//     int get_id() const {
//         return id_;
//     }
// private:
//     int samples_;
//     int time_since_last_messages_ms_;
//     int id_;
// };
// // Simulated prioritize function that distributes static_rate proportionally
// vector<ChannelCommand> prioritize(vector<ChannelInfo> channel_infos, int static_rate) {
//   auto channelInfos = channel_infos;
//   const int numChannels = 30;
//   const int stateDim = numChannels * 2;
//   const int controlDim = numChannels;
//   Eigen::VectorXd x(stateDim);
//   for (int i = 0; i < numChannels; i++) {
//       x(2 * i)     = static_cast<double>(channelInfos[i].get_last_time());
//       x(2 * i + 1) = 0.0;  // Assuming no previous rate is stored yet.
//   }
//   // Build the system dynamics matrices A and B.
//   // For each channel i, we have:
//   //   [ t_{k+1} ]   [ 1   -1 ] [ t_{k} ]   [  0 ] u_i
//   //   [ r_{k+1} ] = [ 0    0 ] [ r_{k} ] + [  1 ]
//   Eigen::MatrixXd A = Eigen::MatrixXd::Zero(stateDim, stateDim);
//   Eigen::MatrixXd B = Eigen::MatrixXd::Zero(stateDim, controlDim);
//   for (int i = 0; i < numChannels; i++) {
//       // Dynamics for waiting time: t_{k+1} = t_k - r_k
//       A(2 * i, 2 * i)     = 1.0;
//       A(2 * i, 2 * i + 1) = -1.0;
//       // Dynamics for current rate: r_{k+1} = u_i  (no dependence on previous rate)
//       // B directly maps u_i to r_{k+1}
//       B(2 * i + 1, i) = 1.0;
//   }
//   Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(stateDim, stateDim);
//   Eigen::MatrixXd R = Eigen::MatrixXd::Zero(controlDim, controlDim);
//   // we can fix this to be whatever
//   vector<double> q_importance = {
//        5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
//        2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1
//   };
//   //we need to determine these
//   vector<double> q_rates = {
//     5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
//     2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
//     1, 1, 1, 1, 1, 1, 1, 1, 1, 1
//   };
//   vector<double> r_size = {
//     5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
//     2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
//     1, 1, 1, 1, 1, 1, 1, 1, 1, 1
//   };
//   // these are pretty arbiraty also
//   for (int i = 0; i < numChannels; i++) {
//       Q(2 * i, 2 * i) = q_importance[i];
//       Q(2 * i + 1, 2 * i + 1) = q_rates[i];
//       R(i, i) = r_size[i];
//       //Q(2 * i, 2 * i)         = q_importance[i];
//       //Q(2 * i + 1, 2 * i + 1)   = 0.5;
//       //R(2 * i, 2 * i)         = r_size[i];
//       //R(2 * i + 1, 2 * i + 1)   = 0.5;
//   }
//   // ----------------------------------------------------------------------------------
//   // Compute the LQR gain using the Control Toolbox.
//   // ----------------------------------------------------------------------------------
//   ct::optcon::LQR<60, 30> lqrSolver;
//   ct::core::FeedbackMatrix<60, 30> K;
//   lqrSolver.compute(Q, Rmat, A, B, K);
//   // u tells us how many messages/samples to send per channel.
//   Eigen::Matrix<double, 30, 1> u = -K * x;
//   vector<ChannelCommand> channel_commands;
//   for (int i = 0; i < numChannels; i++) {
//       int sample_count = std::max(0, static_cast<int>(round(u(i))));
//       channel_commands.push_back(ChannelCommand(sample_count, i));
//   }
//   return channel_commands;
// }
// void test_prioritize() {
//     const int num_channels = 30;
//     const int static_rate = 15000; // static capable rate of 15000 messages per second
//     // Define sample rates.
//     const int high_rate = 2000;
//     const int medium_rate = 100;
//     const int low_rate = 10;
//     // Open CSV file for output.
//     ofstream csv_file("results.csv");
//     csv_file << "Phase,ChannelID,InputRate,TimeSinceLast,StaticRate,OutputSamples\n";
//     // Define 3 phases with rotated rates.
//     for (int phase = 1; phase <= 3; phase++) {
//         vector<ChannelInfo> channel_infos;
//         for (int i = 0; i < num_channels; i++) {
//             int input_rate = 0;
//             // Determine which group the channel belongs to:
//             // group 0: channels 0–9, group 1: channels 10–19, group 2: channels 20–29.
//             int group = i / 10;
//             if (phase == 1) {
//                 // Phase 1: group 0: high, group 1: medium, group 2: low.
//                 if (group == 0)
//                     input_rate = high_rate;
//                 else if (group == 1)
//                     input_rate = medium_rate;
//                 else
//                     input_rate = low_rate;
//             } else if (phase == 2) {
//                 // Phase 2: group 0: low, group 1: high, group 2: medium.
//                 if (group == 0)
//                     input_rate = low_rate;
//                 else if (group == 1)
//                     input_rate = high_rate;
//                 else
//                     input_rate = medium_rate;
//             } else if (phase == 3) {
//                 // Phase 3: group 0: medium, group 1: low, group 2: high.
//                 if (group == 0)
//                     input_rate = medium_rate;
//                 else if (group == 1)
//                     input_rate = low_rate;
//                 else
//                     input_rate = high_rate;
//             }
//             // All channels have time since last message = 1.
//             channel_infos.push_back(ChannelInfo(input_rate, 1, i));
//         }
//         // Call the prioritize function.
//         vector<ChannelCommand> commands = prioritize(channel_infos, static_rate);
//         // Write one CSV row per channel.
//         for (size_t i = 0; i < commands.size(); i++) {
//             ChannelInfo info = channel_infos[i];
//             ChannelCommand command = commands[i];
//             csv_file << phase << ","
//                      << info.get_id() << ","
//                      << info.get_samples() << ","
//                      << info.get_last_time() << ","
//                      << static_rate << ","
//                      << command.get_sample_count() << "\n";
//         }
//     }
//     csv_file.close();
//     cout << "Test complete. Results saved to results.csv" << endl;
// }
int main() {
//     test_prioritize();
    return 0;
}


