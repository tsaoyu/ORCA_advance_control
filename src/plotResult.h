/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/
#include <fstream>  
#include <string>

template <size_t STATE_DIM, size_t CONTROL_DIM>
void plotResultsOscillator(const ct::core::StateVectorArray<STATE_DIM>& stateArray,
    const ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>& fbcontrolArray,
    const ct::core::ControlVectorArray<CONTROL_DIM>& ffcontrolArray,
    const ct::core::TimeArray& timeArray)
{

    using namespace ct::core;

    try
    {
        plot::ion();
        plot::figure();

        if (timeArray.size() != stateArray.size())
        {
            std::cout << timeArray.size() << std::endl;
            std::cout << stateArray.size() << std::endl;
            throw std::runtime_error("Cannot plot data, x and t not equal length");
        }

        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> time_state;
        for (size_t j = 0; j < stateArray.size(); j++)
        {
            position.push_back(stateArray[j](0));
            velocity.push_back(stateArray[j](1));
            time_state.push_back(timeArray[j]);
        }

        std::vector<double> fbcontrol;
        std::vector<double> ffcontrol;
        std::vector<double> time_control;
        for (size_t j = 0; j < ffcontrolArray.size(); j++)
        {
            fbcontrol.push_back(fbcontrolArray[j](0));
            ffcontrol.push_back(ffcontrolArray[j](0));
            time_control.push_back(timeArray[j]);
        }

        plot::subplot(4, 1, 1);
        plot::plot(time_state, position);
        plot::title("position");

        plot::subplot(4, 1, 2);
        plot::plot(time_state, velocity);
        plot::title("velocity");

        plot::subplot(4, 1, 3);
        plot::plot(time_control, ffcontrol);
        plot::title("feedforward control");

        plot::subplot(4, 1, 4);
        plot::plot(time_control, fbcontrol);
        plot::title("feedback control");


        plot::show();
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void plotResultsROV(const ct::core::StateVectorArray<STATE_DIM>& stateArray,
    const ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>& fbcontrolArray,
    const ct::core::ControlVectorArray<CONTROL_DIM>& ffcontrolArray,
    const ct::core::TimeArray& timeArray,
    bool save_matrix_to_file = false)
{

    using namespace ct::core;

    try
    {
        plot::ion();
        plot::figure();

        if (timeArray.size() != stateArray.size())
        {
            std::cout << timeArray.size() << std::endl;
            std::cout << stateArray.size() << std::endl;
            throw std::runtime_error("Cannot plot data, x and t not equal length");
        }
        std::vector<double> state_x;
        std::vector<double> state_y;
        std::vector<double> state_z;
        std::vector<double> state_phi;
        std::vector<double> state_theta;
        std::vector<double> state_psi;
   

        std::vector<double> state_u;
        std::vector<double> state_v;
        std::vector<double> state_w;
        std::vector<double> state_p;
        std::vector<double> state_q;
        std::vector<double> state_r;

        std::vector<double> time_state;

        for (size_t j = 0; j < stateArray.size(); j++)

        {

            state_u.push_back(stateArray[j](0));
            state_v.push_back(stateArray[j](1));
            state_w.push_back(stateArray[j](2));
            state_p.push_back(stateArray[j](3));
            state_q.push_back(stateArray[j](4));
            state_r.push_back(stateArray[j](5));


            state_x.push_back(stateArray[j](6));
            state_y.push_back(stateArray[j](7));
            state_z.push_back(stateArray[j](8));
            state_phi.push_back(stateArray[j](9));
            state_theta.push_back(stateArray[j](10));
            state_psi.push_back(stateArray[j](11));

            time_state.push_back(timeArray[j]);
        }   

        // std::vector<double> feedback_u;
        // std::vector<double> feedback_v;
        // std::vector<double> feedback_w;
        // std::vector<double> feedback_p;
        // std::vector<double> feedback_q;
        // std::vector<double> feedback_r;

        // std::vector<double> feedback_x;
        // std::vector<double> feedback_y;
        // std::vector<double> feedback_z;
        // std::vector<double> feedback_phi;
        // std::vector<double> feedback_theta;
        // std::vector<double> feedback_psi;

        // std::vector<double> feedforward_u;
        // std::vector<double> feedforward_v;
        // std::vector<double> feedforward_w;
        // std::vector<double> feedforward_p;
        // std::vector<double> feedforward_q;
        // std::vector<double> feedforward_r;

        // std::vector<double> feedforward_x;
        // std::vector<double> feedforward_y;
        // std::vector<double> feedforward_z;
        // std::vector<double> feedforward_phi;
        // std::vector<double> feedforward_theta;
        // std::vector<double> feedforward_psi;

        std::vector<double> feedforward_X;
        std::vector<double> feedforward_Y;
        std::vector<double> feedforward_Z;
        std::vector<double> feedforward_YAW;

        std::vector<double> time_control;
        for (size_t j = 0; j < ffcontrolArray.size(); j++)
        {
            // feedback_u.push_back(fbcontrolArray[j](0));
            // feedback_v.push_back(fbcontrolArray[j](1));
            // feedback_w.push_back(fbcontrolArray[j](2));
            // feedback_p.push_back(fbcontrolArray[j](3));
            // feedback_q.push_back(fbcontrolArray[j](4));
            // feedback_r.push_back(fbcontrolArray[j](5));

            // feedback_x.push_back(fbcontrolArray[j](6));
            // feedback_y.push_back(fbcontrolArray[j](7));
            // feedback_z.push_back(fbcontrolArray[j](8));
            // feedback_phi.push_back(fbcontrolArray[j](9));
            // feedback_theta.push_back(fbcontrolArray[j](10));
            // feedback_psi.push_back(fbcontrolArray[j](11));


            // feedforward_u.push_back(ffcontrolArray[j](0));
            // feedforward_v.push_back(ffcontrolArray[j](1));
            // feedforward_w.push_back(ffcontrolArray[j](2));
            // feedforward_p.push_back(ffcontrolArray[j](3));
            // feedforward_q.push_back(ffcontrolArray[j](4));
            // feedforward_r.push_back(ffcontrolArray[j](5));

            // feedforward_x.push_back(ffcontrolArray[j](6));
            // feedforward_y.push_back(ffcontrolArray[j](7));
            // feedforward_z.push_back(ffcontrolArray[j](8));
            // feedforward_phi.push_back(ffcontrolArray[j](9));
            // feedforward_theta.push_back(ffcontrolArray[j](10));
            // feedforward_psi.push_back(ffcontrolArray[j](11));

            feedforward_X.push_back(ffcontrolArray[j](0));
            feedforward_Y.push_back(ffcontrolArray[j](1));
            feedforward_Z.push_back(ffcontrolArray[j](2));
            feedforward_YAW.push_back(ffcontrolArray[j](3));

            time_control.push_back(timeArray[j]);
        }

        plot::subplot(3, 4, 1);
        plot::plot(time_state, state_x);
        plot::title("x");

        plot::subplot(3, 4, 2);
        plot::plot(time_state, state_y);
        plot::title("y");
        
        plot::subplot(3, 4, 3);
        plot::plot(time_state, state_z);
        plot::title("z");

        plot::subplot(3, 4, 4);
        plot::plot(time_state, state_psi);
        plot::title("psi");

        
        plot::subplot(3, 4, 5);
        plot::plot(time_state, state_u);
        plot::title("u");

        plot::subplot(3, 4, 6);
        plot::plot(time_state, state_v);
        plot::title("v");
        
        plot::subplot(3, 4, 7);
        plot::plot(time_state, state_w);
        plot::title("w");

        plot::subplot(3, 4, 8);
        plot::plot(time_state, state_r);
        plot::title("r");
        

        plot::subplot(3, 4, 9);
        plot::plot(time_control, feedforward_X);
        plot::title("x");

        plot::subplot(3, 4, 10);
        plot::plot(time_control, feedforward_Y);
        plot::title("y");
        
        plot::subplot(3, 4, 11);
        plot::plot(time_control, feedforward_Z);
        plot::title("z");

        plot::subplot(3, 4, 12);
        plot::plot(time_control, feedforward_YAW);
        plot::title("psi");
        
        

        plot::show();
        if (save_matrix_to_file)
            return;
    } catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

}

void write_csv(std::ofstream & file, std::vector<double> vals, std::string name){
    // Make a CSV file with one column of integer values
    // filename - the name of the file
    // colname - the name of the one and only column
    // vals - an integer vector of values
    
    // Send data to the stream
    file << name << ",";

    for(int i = 0; i < vals.size() - 1; ++i)
    {
        file << vals.at(i) << ",";
    }
    file << vals.back() << "\n";

}


template <size_t STATE_DIM, size_t CONTROL_DIM>
void saveResultROV(const ct::core::StateVectorArray<STATE_DIM>& stateArray,
    const ct::core::FeedbackArray<STATE_DIM, CONTROL_DIM>& fbcontrolArray,
    const ct::core::ControlVectorArray<CONTROL_DIM>& ffcontrolArray,
    const ct::core::TimeArray& timeArray)
{

    using namespace ct::core;

 
    std::vector<double> state_x;
    std::vector<double> state_y;
    std::vector<double> state_z;
    std::vector<double> state_phi;
    std::vector<double> state_theta;
    std::vector<double> state_psi;


    std::vector<double> state_u;
    std::vector<double> state_v;
    std::vector<double> state_w;
    std::vector<double> state_p;
    std::vector<double> state_q;
    std::vector<double> state_r;

    std::vector<double> time_state;

    for (size_t j = 0; j < stateArray.size(); j++)

    {

        state_u.push_back(stateArray[j](0));
        state_v.push_back(stateArray[j](1));
        state_w.push_back(stateArray[j](2));
        state_p.push_back(stateArray[j](3));
        state_q.push_back(stateArray[j](4));
        state_r.push_back(stateArray[j](5));


        state_x.push_back(stateArray[j](6));
        state_y.push_back(stateArray[j](7));
        state_z.push_back(stateArray[j](8));
        state_phi.push_back(stateArray[j](9));
        state_theta.push_back(stateArray[j](10));
        state_psi.push_back(stateArray[j](11));

        time_state.push_back(timeArray[j]);
    }   

    std::vector<double> feedforward_X;
    std::vector<double> feedforward_Y;
    std::vector<double> feedforward_Z;
    std::vector<double> feedforward_YAW;

    std::vector<double> time_control;
    for (size_t j = 0; j < ffcontrolArray.size(); j++)
    {
        feedforward_X.push_back(ffcontrolArray[j](0));
        feedforward_Y.push_back(ffcontrolArray[j](1));
        feedforward_Z.push_back(ffcontrolArray[j](2));
        feedforward_YAW.push_back(ffcontrolArray[j](3));

        time_control.push_back(timeArray[j]);
    }

    std::ofstream myfile("/home/yu/Playground/result.csv", std::ofstream::app);
    write_csv(myfile, time_control, "time");
    write_csv(myfile, state_u, "u");
    write_csv(myfile, state_v, "v");
    write_csv(myfile, state_w, "w");
    write_csv(myfile, state_r, "r");

    write_csv(myfile, state_x, "x");
    write_csv(myfile, state_y, "y");
    write_csv(myfile, state_z, "z");
    write_csv(myfile, state_psi, "psi");

    write_csv(myfile, feedforward_X, "X");
    write_csv(myfile, feedforward_Y, "Y");
    write_csv(myfile, feedforward_Z, "Z");
    write_csv(myfile, feedforward_YAW, "YAW");
    

    myfile.close();
   
}