#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include “ct/core/core.h”
#include <ct/optcon/optcon.h>
using namespace std;
using namespace Eigen;
class ChannelCommand {
    public:
    ChannelCommand(int sample_count_to_send, int id):sample_count_to_send_(sample_count_to_send), id_(id)  {
    };
    private:
    int sample_count_to_send_;
    int id_;
    };
    class ChannelInfo {
    public:
    ChannelInfo(int samples, int time_since_last_messages_ms, int id):samples_(samples), time_since_last_messages_ms_(time_since_last_messages_ms),  id_(id)  {
    };
    int get_samples() {return samples_;}
    int get_last_time() {return time_since_last_messages_ms_;}
    int get_id() {return id_;}
    private:
    int samples_;
    int time_since_last_messages_ms_;
    int id_;
    };
    vector<ChannelCommand> prioritize(vector<ChannelInfo> channel_infos, int rate) {
        channel_infos.at(0).get_id();
        vector<ChannelCommand> channel_commands = vector<ChannelCommand>();
        channel_commands.push_back(ChannelCommand(1,1));
        return channel_commands;
    }













int main() {
    // assume we have 30 channels; for each we have a ChannelInfo with:
    // - sample count (dummy value, e.g. 1)
    // - time_since_last_messages (for demonstration, we use an increasing value)
    // - an id (here simply 0 to 29)
    const int numChannels = 30;
    vector<ChannelInfo> channelInfos;
    for (int i = 0; i < numChannels; i++) {
        channelInfos.push_back(ChannelInfo(1, i * 10, i));
    }
    // ----------------------------------------------------------------------------------
    // LQR Setup:
    //
    // We now model each channel using a two-dimensional state:
    //   x_i = [ waiting_time; current_rate ]
    //
    // and assume the following simple dynamics for each channel (with unit sampling time):
    //   waiting_time_{k+1} = waiting_time_k - current_rate_k
    //   current_rate_{k+1} = u_i
    //
    // In the combined system for all channels, the state vector x has dimension 2*numChannels,
    // and the control input u is a numChannels-dimensional vector.
    // ----------------------------------------------------------------------------------
    const int stateDim = numChannels * 2;
    const int controlDim = numChannels;
    Eigen::VectorXd x(stateDim);
    for (int i = 0; i < numChannels; i++) {
        x(2 * i)     = static_cast<double>(channelInfos[i].get_last_time());
        x(2 * i + 1) = 0.0;  // Assuming no previous rate is stored yet.
    }
    // Build the system dynamics matrices A and B.
    // For each channel i, we have:
    //   [ t_{k+1} ]   [ 1   -1 ] [ t_{k} ]   [  0 ] u_i
    //   [ r_{k+1} ] = [ 0    0 ] [ r_{k} ] + [  1 ]
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(stateDim, stateDim);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(stateDim, controlDim);
    for (int i = 0; i < numChannels; i++) {
        // Dynamics for waiting time: t_{k+1} = t_k - r_k
        A(2 * i, 2 * i)     = 1.0;
        A(2 * i, 2 * i + 1) = -1.0;
        // Dynamics for current rate: r_{k+1} = u_i  (no dependence on previous rate)
        // B directly maps u_i to r_{k+1}
        B(2 * i + 1, i) = 1.0;
    }
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(stateDim, stateDim);
    Eigen::MatrixXd Rmat = Eigen::MatrixXd::Zero(controlDim, controlDim);
    // we can fix this to be whatever
    vector<double> q_weight = {
         1, 2, 2, 2, 2, 4, 5, 5, 4, 3,
         1, 1, 1, 1, 2, 1, 2, 2, 2, 2,
         2, 9, 8, 7, 6, 5, 4, 3, 2, 1
    };
    // these are pretty arbiraty also
    for (int i = 0; i < numChannels; i++) {
        Q(2 * i, 2 * i)         = q_weight[i];
        Q(2 * i + 1, 2 * i + 1)   = 0.5;
        Rmat(i, i) = 1.0 / q_weight[i];
    }
    // ----------------------------------------------------------------------------------
    // Compute the LQR gain using the Control Toolbox.
    // ----------------------------------------------------------------------------------
    ct::optcon::LQR<60, 30> lqrSolver;
    ct::core::FeedbackMatrix<60, 30> K;
    lqrSolver.compute(Q, Rmat, A, B, K);
    // u tells us how many messages/samples to send per channel.
    Eigen::Matrix<double, 30, 1> u = -K * x;
    vector<ChannelCommand> channel_commands;
    for (int i = 0; i < numChannels; i++) {
        int sample_count = std::max(0, static_cast<int>(round(u(i))));
        channel_commands.push_back(ChannelCommand(sample_count, i));
    }
     cout << “Computed Channel Commands:” << endl;
    for (const auto& cmd : channel_commands) {
        cout << “Channel ” << cmd.get_channel_id()
             << ” -> Sample Count: ” << cmd.get_sample_count() << endl;
    }
    return 0;
}