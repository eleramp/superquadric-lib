/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <cstdlib>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/clustering.h>

#include <SuperquadricLibModel/superquadricEstimator.h>
#include <SuperquadricLibVis/visRenderer.h>
#include <SuperquadricLibGrasp/graspComputation.h>
#include "src/SuperquadricPipelineDemo_IDL.h"

using namespace std;
using namespace SuperqModel;
using namespace SuperqVis;
using namespace SuperqGrasp;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::eigen;
using namespace iCub::ctrl;

/****************************************************************/
enum class WhichHand
{
    HAND_RIGHT,
    HAND_LEFT,
    BOTH,
    NONE
};

/****************************************************************/
string prettyError(const char* func_name, const string &message)
{
    //  Nice formatting for errors
    stringstream error;
    error << "[" << func_name << "] " << message;
    return error.str();
};

/****************************************************************/
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,", ", ", ", "", "", " [ ", "]");

/****************************************************************/
class SuperquadricPipelineDemo : public RFModule, SuperquadricPipelineDemo_IDL
{
    string moduleName;

    RpcClient point_cloud_rpc;
    RpcClient action_render_rpc;
    RpcClient reach_calib_rpc;
    RpcClient table_calib_rpc;
    RpcClient sfm_rpc;

    // Connection to image
    BufferedPort<ImageOf<PixelRgb>> img_in;

    RpcServer user_rpc;

    string object_class;
    vector<string> classes;

    bool closing;

    string robot;
    WhichHand grasping_hand;
    string control_arms;

    PolyDriver left_arm_client, right_arm_client;
    ICartesianControl *icart_right, *icart_left;

    Vector home_pos_left;
    Vector home_pos_right;
    Vector home_o_left;
    Vector home_o_right;

    Matrix grasper_specific_transform_right;
    Matrix grasper_specific_transform_left;
    Vector grasper_approach_parameters_right;
    Vector grasper_approach_parameters_left;

    //  visualization parameters
    int x, y, h, w;
    bool fixate_object;

    // Image crop
    int u_i,v_i, u_f, v_f;

    string best_hand;
    bool single_superq;

    // Take tool trajectory
    double max_traj_value;
    vector<PointD> take_tool_trajectory;

    // PointCloud filtering
    map<string,double> pc_filter_params;

    // Superquadric-lib objects
    SuperqModel::PointCloud point_cloud;
    vector<Superquadric> superqs;
    GraspResults grasp_res_hand1, grasp_res_hand2;
    SuperqEstimatorApp estim;
    GraspEstimatorApp grasp_estim;
    Visualizer vis;

    map<string,double> sq_model_params;
    map<string,double> sq_grasp_params;
    Eigen::Vector4d plane;
    Eigen::Vector3d displacement;
    Eigen::VectorXd hand_superq_params;
    Eigen::MatrixXd bounds_right, bounds_left;
    Eigen::MatrixXd bounds_constr_right, bounds_constr_left;

    bool configure(ResourceFinder &rf) override;
    bool updateModule() override;
    double getPeriod() override;
    bool close() override;
    bool attach(RpcServer &source);
    bool interruptModule();

    /* idl methods */

    // set-get methods
    bool set_approach(const string& hand, const vector<double> &value);
    vector<double> get_approach(const string& hand);
    vector<PointD> get_tool_trajectory();
    bool set_tool_trajectory(const vector<PointD> &points);
    bool clear_tool_trajectory();
    map<string,double> get_pc_filter_params();
    map<string,double> get_sq_model_params();
    map<string,double> get_sq_grasp_params();
    bool set_pc_filter_param(const string &param_name, double value);
    bool set_sq_model_param(const string &param_name, double value);
    bool set_sq_grasp_param(const string &param_name, double value);
    vector<double> get_hand_sq_params();
    bool set_hand_sq_params(const vector<double> &values);
    bool set_single_superq(const bool value);
    string get_superq_mode();
    std::vector<int> get_sfm_region();
    bool set_sfm_region(const double u_i,const double v_i,const double u_f,const double v_f);
    map<string,PointD> get_best_grasping_position();

    // action methos
    bool from_off_file(const string &object_file, const string &hand);
    bool compute_superq_and_pose(const string &object_name, const string &hand);
    bool grasp();
    bool take_tool();
    bool open_hand(const string &hand);
    bool drop();
    bool home();
    bool quit();

    /* */
    bool requestPointCloud(const string &object);
    bool acquireFromSFM();
    void filterPC(vector<Vector> &point_cloud, vector<vector<unsigned char>> &colors);
    void removeOutliers(vector<Vector> &point_cloud, vector<vector<unsigned char>> &all_colors);
    bool set_grasping_hand(const string &hand);
    void computeSuperqAndGrasp(bool choose_hand);
    bool isInClasses(const string &obj_name);
    void getTable();

    bool computePoseHat(GraspResults &grasp_res, ICartesianControl *icart);
    void computePoseHatR1(GraspResults &grasp_res, const string &hand);


    bool executeGrasp(Vector &pose, string &best_hand);

    bool fixReachingOffset(const Vector &poseToFix, Vector &poseFixed,
                           const bool invert=false);

    void setGraspContext(ICartesianControl *icart);
    Vector eigenToYarp(Eigen::VectorXd &v);
    deque<Eigen::Vector3d> vectorYarptoEigen(vector<Vector> &yarp_points);

};

    /****************************************************************/
    bool SuperquadricPipelineDemo::configure(ResourceFinder &rf)
    {
        moduleName = rf.check("name", Value("superquadric-lib-demo")).toString();
        if(!rf.check("robot"))
        {
            robot = (rf.check("sim")? "icubSim" : "icub");
        }
        else
        {
            robot = rf.find("robot").asString();
        }

        yInfo() << "Opening module for connection with robot" << robot;

        control_arms = rf.check("control_arms", Value("both")).toString();

        x = rf.check("x", Value(0)).asInt();
        y = rf.check("y", Value(0)).asInt();
        w = rf.check("width", Value(600)).asInt();
        h = rf.check("height", Value(600)).asInt();

        u_i = rf.check("u_i", Value(50)).asInt();
        v_i = rf.check("v_i", Value(20)).asInt();
        u_f = rf.check("u_f", Value(280)).asInt();
        v_f = rf.check("v_f", Value(190)).asInt();

        vis.setPosition(x,y);
        vis.setSize(w,h);

        Vector grasp_specific_translation(3, 0.0);
        Vector grasp_specific_orientation(4, 0.0);
        Bottle *list = rf.find("grasp_trsfm_right").asList();
        bool valid_grasp_specific_transform = true;

        if(list)
        {
            if(list->size() == 7)
            {
                for(int i = 0; i < 3; i++) grasp_specific_translation[i] = list->get(i).asDouble();
                for(int i = 0; i < 4; i++) grasp_specific_orientation[i] = list->get(3 + i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid grasp_trsfm_right dimension in config. Should be 7.");
                valid_grasp_specific_transform = false;
            }
        }
        else valid_grasp_specific_transform = false;

        if( (!valid_grasp_specific_transform) && ((robot == "icubSim") || (robot == "icub")) )
        {
            yInfo() << "Loading grasp_trsfm_right default value for iCub";
            grasp_specific_translation[0] = -0.01;
            grasp_specific_orientation[1] = 1;
            grasp_specific_orientation[3] = - 38.0 * M_PI/180.0;
        }

        grasper_specific_transform_right = axis2dcm(grasp_specific_orientation);
        grasper_specific_transform_right.setSubcol(grasp_specific_translation, 0, 3);
        yInfo() << "Grabber specific transform for right arm loaded\n" << grasper_specific_transform_right.toString();

        grasp_specific_translation.zero();
        grasp_specific_orientation.zero();
        list = rf.find("grasp_trsfm_left").asList();
        valid_grasp_specific_transform = true;

        if(list)
        {
            if(list->size() == 7)
            {
                for(int i = 0; i < 3; i++) grasp_specific_translation[i] = list->get(i).asDouble();
                for(int i = 0; i < 4; i++) grasp_specific_orientation[i] = list->get(3+i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid grasp_trsfm_left dimension in config. Should be 7.");
                valid_grasp_specific_transform = false;
            }
        }
        else valid_grasp_specific_transform = false;

        if( (!valid_grasp_specific_transform) && ((robot == "icubSim") || (robot == "icub")) )
        {
            yInfo() << "Loading grasp_trsfm_left default value for iCub";
            grasp_specific_translation[0] = -0.01;
            grasp_specific_orientation[1] = 1;
            grasp_specific_orientation[3] = + 38.0 * M_PI/180.0;
        }

        grasper_specific_transform_left = axis2dcm(grasp_specific_orientation);
        grasper_specific_transform_left.setSubcol(grasp_specific_translation, 0,3);
        yInfo() << "Grabber specific transform for left arm loaded\n" << grasper_specific_transform_left.toString();

        list = rf.find("approach_right").asList();

        grasper_approach_parameters_right.resize(4, 0.0);
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i = 0; i < 4; i++) grasper_approach_parameters_right[i] = list->get(i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid approach_right dimension in config. Should be 4.");
            }
        }
        else if((robot == "icubSim") || (robot == "icub"))
        {
            grasper_approach_parameters_right[0] = -0.05;
            grasper_approach_parameters_right[1] = 0.0;
            grasper_approach_parameters_right[2] = 0.0;
            grasper_approach_parameters_right[3] = 0.0;
        }
        yInfo() << "Grabber specific approach for right arm loaded\n" << grasper_approach_parameters_right.toString();

        list = rf.find("approach_left").asList();
        grasper_approach_parameters_left.resize(4, 0.0);
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i=0 ; i<4 ; i++) grasper_approach_parameters_left[i] = list->get(i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid approach_left dimension in config. Should be 4.");
            }
        }
        else if((robot == "icubSim") || (robot == "icub"))
        {
            grasper_approach_parameters_left[0] = -0.05;
            grasper_approach_parameters_left[1] = 0.0;
            grasper_approach_parameters_left[2] = +0.0;
            grasper_approach_parameters_left[3] = 0.0;
        }
        yInfo() << "Grabber specific approach for left arm loaded\n" << grasper_approach_parameters_left.toString();

        //  open the necessary ports
        point_cloud_rpc.open("/" + moduleName + "/pointCloud:rpc");
        action_render_rpc.open("/" + moduleName + "/actionRenderer:rpc");
        reach_calib_rpc.open("/" + moduleName + "/reachingCalibration:rpc");
        table_calib_rpc.open("/" + moduleName + "/tableCalib:rpc");
        user_rpc.open("/" + moduleName + "/cmd:rpc");
        sfm_rpc.open("/" + moduleName + "/sfm:rpc");
        img_in.open("/" + moduleName + "/img:i");


        //  open clients when using iCub

        if((robot == "icubSim") || (robot == "icub"))
        {
            Property optionLeftArm, optionRightArm;

            optionLeftArm.put("device", "cartesiancontrollerclient");
            optionLeftArm.put("remote", "/" + robot + "/cartesianController/left_arm");
            optionLeftArm.put("local", "/" + moduleName + "/cartesianClient/left_arm");

            optionRightArm.put("device", "cartesiancontrollerclient");
            optionRightArm.put("remote", "/" + robot + "/cartesianController/right_arm");
            optionRightArm.put("local", "/" + moduleName + "/cartesianClient/right_arm");

            if ((control_arms == "both") || (control_arms == "left"))
            {
                if (!left_arm_client.open(optionLeftArm))
                {
                    yError() << prettyError( __FUNCTION__, "Could not open cartesian solver client for left arm");
                    return false;
                }
            }
            if ((control_arms == "both") || (control_arms == "right"))
            {
                if (!right_arm_client.open(optionRightArm))
                {
                    if (left_arm_client.isValid())
                    {
                        left_arm_client.close();
                    }
                    yError() << prettyError( __FUNCTION__, "Could not open cartesian solver client for right arm");
                    return false;
                }
            }

            if (left_arm_client.isValid())
            {
                left_arm_client.view(icart_left);
                icart_left->getPose(home_pos_left, home_o_left);
            }

            if (right_arm_client.isValid())
            {
                right_arm_client.view(icart_right);
                icart_right->getPose(home_pos_right, home_o_right);
            }

        }

        //  attach callback
        attach(user_rpc);

        fixate_object = false;

        /* ------ take_tool trajectory ------ */

        max_traj_value = rf.check("max_value", Value(0.2)).asDouble();

        list->clear();
        list = rf.find("tool_trajectory").asList();
        PointD p;
        if (list)
        {
            if (list->size()%3 == 0)
            {
                for(int i = 0 ; i < list->size() ; i+=3)
                {
                    p = {list->get(i).asDouble(), list->get(i+1).asDouble(), list->get(i+2).asDouble()};
                    take_tool_trajectory.push_back(p);
                }
            }
        }
        else
        {
            PointD p1 = {0.1, 0.0, 0.0};
            PointD p2 = {0.0, 0.2, 0.0};
            PointD p3 = {0.0, 0.0, 0.05};
            take_tool_trajectory.push_back(p1);
            take_tool_trajectory.push_back(p2);
            take_tool_trajectory.push_back(p3);
        }

        /* ------ PointCloud filtering ------ */

        pc_filter_params["radius_dbscan"] = rf.check("radius_dbscan", Value(0.01)).asDouble();
        pc_filter_params["points_dbscan"] = rf.check("points_dbscan", Value(10)).asInt();
        pc_filter_params["sfm_range"] = rf.check("sfm_range", Value(0.04)).asDouble();

        /* ------ Set Superquadric Model parameters ------ */

        int print_level_superq = rf.check("print_level_superq", Value(0)).asInt();
        object_class = rf.check("object_class", Value("default")).toString();
        single_superq = rf.check("single_superq", Value(true)).asBool();
        sq_model_params["tol"] = rf.check("tol_superq", Value(1e-5)).asDouble();
        sq_model_params["optimizer_points"] = rf.check("optimizer_points", Value(50)).asInt();
        sq_model_params["random_sampling"] = rf.check("random_sampling", Value(true)).asBool();

        estim.SetNumericValue("tol", sq_model_params["tol"]);
        estim.SetIntegerValue("print_level", print_level_superq);
        estim.SetStringValue("object_class", object_class);
        estim.SetIntegerValue("optimizer_points", int(sq_model_params["optimizer_points"]));
        estim.SetBoolValue("random_sampling", bool(sq_model_params["random_sampling"]));

        bool merge_model = rf.check("merge_model", Value(true)).asBool();
        sq_model_params["minimum_points"] = rf.check("minimum_points", Value(150)).asInt();
        sq_model_params["fraction_pc"] = rf.check("fraction_pc", Value(8)).asInt();
        sq_model_params["tol_threshold_axissuperq"] = rf.check("tol_threshold_axissuperq", Value(0.7)).asDouble();
        sq_model_params["threshold_section1"] = rf.check("threshold_section1", Value(0.6)).asDouble();
        sq_model_params["threshold_section2"] = rf.check("threshold_section2", Value(0.03)).asDouble();

        estim.SetBoolValue("merge_model", merge_model);
        estim.SetIntegerValue("minimum_points", sq_model_params["minimum_points"]);
        estim.SetIntegerValue("fraction_pc", sq_model_params["fraction_pc"]);
        estim.SetNumericValue("threshold_axis", sq_model_params["tol_threshold_axissuperq"]);
        estim.SetNumericValue("threshold_section1", sq_model_params["threshold_section1"]);
        estim.SetNumericValue("threshold_section2", sq_model_params["threshold_section2"]);

        /* ------  Set Superquadric Grasp parameters ------ */

        int print_level_grasp = rf.check("print_level_grasp", Value(0)).asInt();
        sq_grasp_params["tol_grasp"] = rf.check("tol_grasp", Value(1e-5)).asDouble();
        sq_grasp_params["constr_tol"] = rf.check("constr_tol", Value(1e-4)).asDouble();
        sq_grasp_params["max_superq"] = rf.check("max_superq", Value(4)).asInt();

        grasp_estim.SetIntegerValue("print_level", print_level_grasp);
        grasp_estim.SetNumericValue("tol", sq_grasp_params["tol_grasp"]);
        grasp_estim.SetIntegerValue("max_superq", int(sq_grasp_params["max_superq"]));
        grasp_estim.SetNumericValue("constr_tol", sq_grasp_params["constr_tol"]);
        grasp_estim.SetStringValue("left_or_right", "right");
        grasping_hand = WhichHand::HAND_RIGHT;

        // set plane
        list = rf.find("plane_table").asList();
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i = 0 ; i < 4 ; i++) plane(i) = list->get(i).asDouble();
            }
        }
        else
            plane << 0.0, 0.0, 1.0, 0.20;

        grasp_estim.setVector("plane", plane);

        // set displacement
        list = rf.find("displacement").asList();
        if (list)
        {
            if (list->size() == 3)
            {
                for(int i = 0 ; i < 3 ; i++) displacement(i) = list->get(i).asDouble();
            }
        }
        else
            displacement << 0.0, 0.0, 0.0;

        grasp_estim.setVector("displacement", displacement);

        // set hand_superq_params
        hand_superq_params.resize(11);

        list = rf.find("hand").asList();
        if (list)
        {
            if (list->size() == 11)
            {
                for(int i = 0 ; i < 11 ; i++) hand_superq_params(i) = list->get(i).asDouble();
            }
        }
        else
            hand_superq_params << 0.03, 0.06, 0.03, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        grasp_estim.setVector("hand", hand_superq_params);

        // set bounds right
        bounds_right.resize(6,2);
        list = rf.find("bounds_right").asList();
        if (list)
        {
            if (list->size() == 12)
            {
                for(int i = 0 ; i < 12 ; i = i+2)
                {
                    if ( i % 2 == 0 )
                        bounds_right(i/2,0) = list->get(i).asDouble();
                    else
                        bounds_right(i/2,1) = list->get(i).asDouble();
                }
            }
        }
        else
            bounds_right << -0.5, 0.0, -0.2, 0.2, -0.3, 0.3, -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;

        grasp_estim.setMatrix("bounds_right", bounds_right);

        // set bounds left
        bounds_left.resize(6,2);
        list = rf.find("bounds_left").asList();
        if (list)
        {
            if (list->size() == 12)
            {
                for(int i = 0 ; i < 12 ; i = i+2)
                {
                    if ( i % 2 == 0 )
                        bounds_left(i/2,0) = list->get(i).asDouble();
                    else
                        bounds_left(i/2,1) = list->get(i).asDouble();
                }
            }
        }
        else
            bounds_left << -0.5, 0.0, -0.2, 0.2, -0.3, 0.3, -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;

        grasp_estim.setMatrix("bounds_left", bounds_left);

        // set bounds constraint right
        bounds_constr_right.resize(8,2);
        list = rf.find("bounds_constr_right").asList();
        if (list)
        {
            if (list->size() == 16)
            {
                for(int i = 0 ; i < 16 ; i++)
                {
                    if ( i % 2 == 0 )
                        bounds_constr_right(i/2,0) = list->get(i).asDouble();
                    else
                        bounds_constr_right(i/2,1) = list->get(i).asDouble();
                }
            }
        }
        else
        {
            bounds_constr_right << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.001,
                    10.0, 0.0, 1.0, 0.00001, 10.0, 0.00001, 10.0, 0.00001, 10.0;
        }

        grasp_estim.setMatrix("bounds_constr_right", bounds_constr_right);

        // set bounds constraint left
        bounds_constr_left.resize(8,2);
        list = rf.find("bounds_constr_left").asList();
        if (list)
        {
            if (list->size() == 16)
            {
                for(int i = 0 ; i < 16 ; i++)
                {
                    if ( i % 2 == 0 )
                        bounds_constr_left(i/2,0) = list->get(i).asDouble();
                    else
                        bounds_constr_left(i/2,1) = list->get(i).asDouble();
                }
            }
        }
        else
        {
            bounds_constr_left << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.01,
                    10.0, 0.0, 1.0, 0.00001, 10.0, 0.00001, 10.0, 0.00001, 10.0;
        }
        grasp_estim.setMatrix("bounds_constr_left", bounds_constr_left);

        classes.push_back("box");
        classes.push_back("sphere");
        classes.push_back("cylinder");
        // Start visualization
        vis.visualize();

        return true;
    }


    /****************************************************************/
    bool SuperquadricPipelineDemo::updateModule()
    {
        return false;
    }

    /****************************************************************/
    double SuperquadricPipelineDemo::getPeriod()
    {
        return 1.0;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::interruptModule()
    {
        point_cloud_rpc.interrupt();
        action_render_rpc.interrupt();
        reach_calib_rpc.interrupt();
        user_rpc.interrupt();
        table_calib_rpc.interrupt();
        closing = true;

        return true;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::close()
    {
        point_cloud_rpc.close();
        action_render_rpc.close();
        reach_calib_rpc.close();
        user_rpc.close();
        table_calib_rpc.close();

        if (left_arm_client.isValid())
        {
            left_arm_client.close();
        }
        if (right_arm_client.isValid())
        {
            right_arm_client.close();
        }

        return true;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::quit()
    {
        yInfo() << "asking to stop module";
        stopModule(false);
        return true;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_approach(const string& hand, const vector<double> &value)
    {
        if (value.size() == 4)
        {
            if(!hand.compare("right"))
            {
                for(int i=0 ; i<4 ; i++) grasper_approach_parameters_right[i] = value[i];
            }
            else if(!hand.compare("left"))
            {
                for(int i=0 ; i<4 ; i++) grasper_approach_parameters_left[i] = value[i];
            }
            else
            {
                yError() << "Invalid hand.";
                return false;
            }
        }
        else
        {
            yError() << "Invalid approach dimension. Should be 4.";
            return false;
        }
        return true;
    }

    /****************************************************************/
    vector<double> SuperquadricPipelineDemo::get_approach(const string& hand)
    {
        vector<double> output;

        if(!hand.compare("right"))
        {
            for(double & p: grasper_approach_parameters_right) output.push_back(p);
        }
        else if(!hand.compare("left"))
        {
            for(double & p: grasper_approach_parameters_left) output.push_back(p);
        }
        else
        {
            yError() << "Invalid hand.";
        }

        return output;
    }

    /****************************************************************/
    vector<PointD> SuperquadricPipelineDemo::get_tool_trajectory()
    {
        return take_tool_trajectory;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_tool_trajectory(const vector<PointD> &points)
    {
        take_tool_trajectory.clear();

        PointD p1;
        for(const PointD& p: points)
        {
            p1.x = sign(p.x) * min(max_traj_value, abs(p.x));
            p1.y = sign(p.y) * min(max_traj_value, abs(p.y));
            p1.z = sign(p.z) * min(max_traj_value, abs(p.z));
            take_tool_trajectory.push_back(p);
        }

        return true;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::clear_tool_trajectory()
    {
        take_tool_trajectory.clear();
        return true;
    }

    /****************************************************************/
    map<string,double> SuperquadricPipelineDemo::get_pc_filter_params()
    {
        return pc_filter_params;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_pc_filter_param(const string &param_name, double value)
    {
        if(sq_model_params.find(param_name) == sq_model_params.end())
        {
            yError() << param_name << " is unkown.";
            return false;
        }

        sq_model_params[param_name] = value;
        return true;
    }

    /****************************************************************/
    map<string,double> SuperquadricPipelineDemo::get_sq_model_params()
    {
        return sq_model_params;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_sq_model_param(const string &param_name, double value)
    {
        if(sq_model_params.find(param_name) == sq_model_params.end())
        {
            yError() << param_name << " is unkown.";
            return false;
        }

        sq_model_params[param_name] = value;
        return true;
    }

    /****************************************************************/
    map<string,double> SuperquadricPipelineDemo::get_sq_grasp_params()
    {
        return sq_grasp_params;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_sq_grasp_param(const string &param_name, double value)
    {
        if(sq_grasp_params.find(param_name) == sq_grasp_params.end())
        {
            yError() << param_name << " is unkown.";
            return false;
        }

        sq_grasp_params[param_name] = value;
        return true;
    }

    /****************************************************************/
    vector<double> SuperquadricPipelineDemo::get_hand_sq_params()
    {
        vector<double> output_vec;
        output_vec.resize(hand_superq_params.size());
        Eigen::VectorXd::Map(&output_vec[0], hand_superq_params.size()) = hand_superq_params;
        return output_vec;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_hand_sq_params(const vector<double> &values)
    {
        hand_superq_params.resize(values.size());
        hand_superq_params = Eigen::VectorXd::Map(&values[0], hand_superq_params.size());
        return true;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_single_superq(const bool value)
    {
        single_superq = value;
        return true;
    }

    /****************************************************************/
    string SuperquadricPipelineDemo::get_superq_mode()
    {
        return single_superq? "singleSuperQuadric" : "multipleSuperQuadric";
    }

    /****************************************************************/
    std::vector<int> SuperquadricPipelineDemo::get_sfm_region()
    {
        return vector<int>{u_i,v_i,u_f,v_f};
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_sfm_region(const double u_i,const double v_i,const double u_f,const double v_f)
    {
        if(u_i < 0 || v_i < 0 || u_f < 0 || v_f < 0)
        {
            yError() << "Invalid pixel values. Keeping current image region";
            return false;
        }

        this->u_i = u_i;
        this->v_i = v_i;
        this->u_f = u_f;
        this->v_f = v_f;
        return true;
    }

    /****************************************************************/
    map<string,PointD> SuperquadricPipelineDemo::get_best_grasping_position()
    {
        GraspPoses best_graspPose;
        map<string,PointD> output_map;

        if(grasp_res_hand1.grasp_poses.size()==0)
        {
            yWarning() << "no grasping poses available. They need to be computed.";
            return output_map;
        }

        if (grasping_hand == WhichHand::BOTH && best_hand == "left")
        {
            best_graspPose = grasp_res_hand2.grasp_poses[grasp_res_hand2.best_pose];
        }
        else
        {
            best_graspPose = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose];
        }

        PointD position;
        if(best_graspPose.position.size() > 0)
        {
            position.x = best_graspPose.position(0);
            position.y = best_graspPose.position(1);
            position.z = best_graspPose.position(2);
            output_map[best_hand] = position;
        }
        return output_map;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::set_grasping_hand(const string &hand)
    {
        if (!hand.compare("right"))
        {
            if(!control_arms.compare("left"))
            {
                yWarning() << "the only controllable hand is " << control_arms;
                return false;
            }

            grasping_hand = WhichHand::HAND_RIGHT;

            grasp_estim.SetStringValue("left_or_right", hand);
            return true;
        }
        else if (!hand.compare("left"))
        {
            if(!control_arms.compare("right"))
            {
                yWarning() << "the only controllable hand is " << control_arms;
                return false;
            }

            grasping_hand = WhichHand::HAND_LEFT;

            grasp_estim.SetStringValue("left_or_right", hand);
            return true;
        }
        else if (!hand.compare("both"))
        {
            if(control_arms.compare("both"))
            {
                yWarning() << "the only controllable hand is " << control_arms;
                return false;
            }

            grasping_hand = WhichHand::BOTH;

            grasp_estim.SetStringValue("left_or_right", "right");
            return true;
        }
        else
            yInfo() << "set_grasping_hand: unvalid value. Possible values are: right, left, both";
            return false;

    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::from_off_file(const string &object_file, const string &hand)
    {
        if(!set_grasping_hand(hand))
            return false;

        vector<Vector> points_yarp;
        vector<vector<unsigned char>> all_colors;

        ifstream fin(object_file);
        if (!fin.is_open())
        {
            yError() << "Unable to open file \"" << object_file;
            return 0;
        }

        Vector p(3);
        vector<unsigned int> c_(3);
        vector<unsigned char> c(3);

        // Read point cloud from file
        string line;
        while (getline(fin,line))
        {
            istringstream iss(line);
            if (!(iss >> p(0) >> p(1) >> p(2)))
                break;
            points_yarp.push_back(p);

            fill(c_.begin(),c_.end(),120);
            iss >> c_[0] >> c_[1] >> c_[2];
            c[0] = (unsigned char)c_[0];
            c[1] = (unsigned char)c_[1];
            c[2] = (unsigned char)c_[2];

            if (c[0] == c[1] && c[1] == c[2])
            {
                c[0] = 50;
                c[1] = 100;
                c[2] = 0;
            }

            all_colors.push_back(c);
        }

        // filtering
        removeOutliers(points_yarp, all_colors);

        deque<Eigen::Vector3d> points_eigen = vectorYarptoEigen(points_yarp);

        point_cloud.setPoints(points_eigen);
        point_cloud.setColors(all_colors);

        if (point_cloud.getNumberPoints() > 0)
        {
            computeSuperqAndGrasp(false);
            return true;
        }

        return false;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::compute_superq_and_pose(const string &object_name, const string &hand)
    {
        if(!set_grasping_hand(hand))
            return false;

        fixate_object = true;

        bool ok;

        if (object_name != "hanging_tool")
            ok = requestPointCloud(object_name);
        else
            ok = acquireFromSFM();

        if (isInClasses(object_name))
            object_class = object_name;

        if (ok == true && point_cloud.getNumberPoints() > 0)
        {
            computeSuperqAndGrasp(true);
        }

        return ok;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::grasp()
    {
        if(grasp_res_hand1.grasp_poses.size()==0)
        {
            yError() << "no grasping poses available. They need to be computed.";
            return false;
        }

        GraspPoses best_graspPose = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose];

        if (grasping_hand == WhichHand::BOTH && best_hand == "left")
        {
            best_graspPose = grasp_res_hand2.grasp_poses[grasp_res_hand2.best_pose];
        }

        Eigen::VectorXd pose;
        pose.resize(7);
        pose.head(3) = best_graspPose.getGraspPosition();
        pose.tail(4) = best_graspPose.getGraspAxisAngle();
        Vector best_pose = eigenToYarp(pose);

        cout << "|| ---------------------------------------------------- ||" << endl;
        yInfo() << " || Best pose selected: " << best_pose.toString();
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << endl << endl;

        return executeGrasp(best_pose, best_hand);
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::drop()
    {
        if (action_render_rpc.getOutputCount() > 0)
        {
            Bottle cmd, reply;
            cmd.addVocab(Vocab::encode("drop"));
            action_render_rpc.write(cmd, reply);
            if (reply.get(0).asVocab() == Vocab::encode("ack"))
                return true;
            else
                return false;
        }
        else
            return false;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::home()
    {
        if(robot == "icubSim")
        {
            if(best_hand == "right")
            {
                // store context
                int context_backup;
                icart_right->storeContext(&context_backup);
                setGraspContext(icart_right);

                yInfo() << "home pose: " << home_pos_right.toString() << " " << home_o_right.toString();
                icart_right->goToPoseSync(home_pos_right, home_o_right);
                icart_right->waitMotionDone();

                // retrieve context
                icart_right->restoreContext(context_backup);
                icart_right->deleteContext(context_backup);
            }
            else if (best_hand == "left")
            {
                // store context
                int context_backup;
                icart_left->storeContext(&context_backup);
                setGraspContext(icart_left);

                yInfo() << "home pose: " << home_pos_left.toString() << " " << home_o_left.toString();
                icart_left->goToPoseSync(home_pos_left, home_o_left);
                icart_left->waitMotionDone();

                // retrieve context
                icart_left->restoreContext(context_backup);
                icart_left->deleteContext(context_backup);
            }
            return true;
        }

        else if (action_render_rpc.getOutputCount() > 0)
        {
            Bottle cmd, reply;

            cmd.addVocab(Vocab::encode("home"));
            cmd.addString("head");
            cmd.addString("arms");

            action_render_rpc.write(cmd, reply);
            if (reply.get(0).asVocab() == Vocab::encode("ack"))
                return true;
            else
                return false;
        }
        else
            return false;
    }

    /************************************************************************/
    bool SuperquadricPipelineDemo::requestPointCloud(const string &object)
    {
        Bottle cmd_request;
        Bottle cmd_reply;

        //  if fixate_object is given, look at the object before acquiring the point cloud
        if (fixate_object)
        {
            if(action_render_rpc.getOutputCount() < 1)
            {
                yError() << prettyError( __FUNCTION__,  "requestRefreshPointCloud: no connection to action rendering module");
                return false;
            }

            cmd_request.addVocab(Vocab::encode("look"));
            cmd_request.addString(object);
            cmd_request.addString("wait");

            action_render_rpc.write(cmd_request, cmd_reply);
            if (cmd_reply.get(0).asVocab() != Vocab::encode("ack"))
            {
                yError() << prettyError( __FUNCTION__,  "Didn't manage to look at the object");
                return false;
            }
        }

        point_cloud.deletePoints();
        cmd_request.clear();
        cmd_reply.clear();

        yarp::sig::PointCloud<DataXYZRGBA> pc;

        cmd_request.addString("get_point_cloud");
        cmd_request.addString(object);

        if(point_cloud_rpc.getOutputCount() < 1)
        {
            yError() << prettyError( __FUNCTION__,  "requestRefreshPointCloud: no connection to point cloud module");
            return false;
        }

        point_cloud_rpc.write(cmd_request, cmd_reply);

        //  cheap workaround to get the point cloud
        Bottle* pcBt = cmd_reply.get(0).asList();
        bool success = pc.fromBottle(*pcBt);

        vector<Vector> acquired_points;
        vector<vector<unsigned char>> acquired_colors;

        Vector point(3);
        vector<unsigned char> c;
        c.resize(3);

        for (size_t i = 0; i < pc.size(); i++)
        {
            point(0)=pc(i).x; point(1)=pc(i).y; point(2)=pc(i).z;
            c[0] = pc(i).r;
            c[1] = pc(i).g;
            c[2] = pc(i).b;

            acquired_points.push_back(point);
            acquired_colors.push_back(c);
        }

        // filtering
        removeOutliers(acquired_points, acquired_colors);

        deque<Eigen::Vector3d> eigen_points = vectorYarptoEigen(acquired_points);

        if (success && (eigen_points.size() > 0))
        {
            point_cloud.setPoints(eigen_points);
            point_cloud.setColors(acquired_colors);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::acquireFromSFM()
    {
        point_cloud.deletePoints();

        vector<Vector> acquired_points;
        vector<vector<unsigned char>> acquired_colors;

        Bottle cmd_request;
        Bottle cmd_reply;
        cmd_request.clear();
        cmd_reply.clear();
        cmd_request.addString("Points");
        Bottle pointsList;

        // Consider points inside the image region [u_i,v_i], [u_f,v_f]
        for (int i = u_i; i < u_f; i++)
        {
            for (int j = v_i; j < v_f; j++)
            {
                Bottle &points = pointsList.addList();
                cmd_request.addInt(i);
                cmd_request.addInt(j);
                points.addInt(i);
                points.addInt(j);
            }
        }

        // Get depth from sfm module
        if (!sfm_rpc.write(cmd_request, cmd_reply))
        {
            yError() << " Problems in getting points from SFM ";
            return false;
        }

        // Get RGB image
        if (img_in.getInputCount() < 1)
        {
            yError() << prettyError( __FUNCTION__,  "no connection on RGB image port");
            return false;
        }

        ImageOf<PixelRgb> *inCamImg = img_in.read();

        // Create the point cloud
        for (int p_i = 0; p_i < cmd_reply.size()/3 ; p_i ++)
        {
            // Get 3D point
            Vector point(3,0.0);

            point[0] = cmd_reply.get(p_i*3).asDouble();
            point[1] = cmd_reply.get(p_i*3 + 1).asDouble();
            point[2] = cmd_reply.get(p_i*3 + 2).asDouble();

            if (norm(point) == 0.0 || (point[0] > -0.2) || (point[0] < -0.6) || (point[2] < -0.2))
                continue;

            // Get color
            Bottle *point2D = pointsList.get(p_i).asList();
            PixelRgb point_rgb = inCamImg->pixel(point2D->get(0).asInt(), point2D->get(1).asInt());

            vector<unsigned char> color(3);

            color[0] = point_rgb.r;
            color[1] = point_rgb.g;
            color[2] = point_rgb.b;

            acquired_points.push_back(point);
            acquired_colors.push_back(color);
        }

        // filtering
        filterPC(acquired_points, acquired_colors);

        deque<Eigen::Vector3d> eigen_points = vectorYarptoEigen(acquired_points);

        point_cloud.setPoints(eigen_points);
        point_cloud.setColors(acquired_colors);

        if (point_cloud.getNumberPoints() > 0)
            return true;
        else
            return false;
    }

    /****************************************************************/
    void SuperquadricPipelineDemo::filterPC(vector<Vector> &point_cloud, vector<vector<unsigned char>> &colors)
    {
        double x_max = point_cloud[0][0];

        vector<Vector> new_point_cloud;
        vector<vector<unsigned char>> new_colors;

        for (size_t i = 1; i < point_cloud.size(); i++)
        {
            if (point_cloud[i][0] > x_max)
                x_max = point_cloud[i][0];
        }

        yDebug() << "X max " << x_max;

        yDebug() << "x_max - 0.04" << x_max - 0.04;

        for (size_t i = 0; i < point_cloud.size(); i++)
        {
            if (point_cloud[i][0] > x_max - pc_filter_params["sfm_range"])
            {
                new_point_cloud.push_back(point_cloud[i]);

                vector<unsigned char> color(3);
                color[0] = colors[i][0];
                color[1] = colors[i][1];
                color[2] = colors[i][2];

                new_colors.push_back(color);
            }
        }

        point_cloud.clear();
        colors.clear();

        point_cloud = new_point_cloud;
        colors = new_colors;

    }

    /****************************************************************/
    void SuperquadricPipelineDemo::removeOutliers(vector<Vector> &point_cloud, vector<vector<unsigned char>> &all_colors)
    {
        double t0=Time::now();

        vector<Vector> in_points;
        vector<vector<unsigned char>> in_colors;

        Property options;
        options.put("epsilon", pc_filter_params["radius_dbscan"]);
        options.put("minpts", int(pc_filter_params["points_dbscan"]));

        DBSCAN dbscan;
        map<size_t,set<size_t>> clusters=dbscan.cluster(point_cloud, options);

        size_t largest_class; size_t largest_size=0;
        for (auto it=begin(clusters); it!=end(clusters); it++)
        {
            if (it->second.size()>largest_size)
            {
                largest_size=it->second.size();
                largest_class=it->first;
            }
        }

        auto &c=clusters[largest_class];
        for (size_t i=0; i<point_cloud.size(); i++)
        {
            if (c.find(i)!=end(c))
            {
                in_points.push_back(point_cloud[i]);
                in_colors.push_back(all_colors[i]);
            }
        }

        double t1=Time::now();

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Outliers removed                                      : "<< point_cloud.size() - in_points.size() << endl;
        cout << "|| ---------------------------------------------------- ||" << endl<<endl;


        point_cloud.clear();
        all_colors.clear();

        point_cloud = in_points;
        all_colors = in_colors;

    }

    /************************************************************************/
    void SuperquadricPipelineDemo::computeSuperqAndGrasp(bool choose_hand)
    {
        // Reset visualizer for new computations
        vis.resetSuperq();
        vis.resetPoses();
        vis.resetPoints();

        // Visualize acquired point cloud
        vis.addPoints(point_cloud, false);

        /*  ------ Compute superq ------  */

        if (single_superq || object_class != "default")
        {
            estim.SetStringValue("object_class", object_class);
            superqs = estim.computeSuperq(point_cloud);
            vis.addPoints(point_cloud, true);
        }
        else
        {
            superqs = estim.computeMultipleSuperq(point_cloud);
            vis.addPoints(point_cloud, false);
        }

        // Visualize downsampled point cloud and estimated superq
        vis.addSuperq(superqs);

        /* ------ Compute grasp poses ------ */

        // Get real value of table height
        getTable();

        grasp_res_hand1 = grasp_estim.computeGraspPoses(superqs);

        // Show computed grasp pose and plane
        vis.addPoses(grasp_res_hand1.grasp_poses);
        vis.addPlane(grasp_estim.getPlaneHeight());

        // Compute and show grasp pose for the other hand
        if (grasping_hand == WhichHand::BOTH)
        {
            grasp_estim.SetStringValue("left_or_right", "left");
            grasp_res_hand2 = grasp_estim.computeGraspPoses(superqs);
            vis.addPoses(grasp_res_hand1.grasp_poses, grasp_res_hand2.grasp_poses);
        }

        /* ------ Estimate pose cost ------ */

        // Compute pose hat
        if ((robot == "icubSim") || (robot == "icub"))
        {
            if (grasping_hand == WhichHand::BOTH)
            {
                bool success = false;
                if (left_arm_client.isValid())
                {
                    success = computePoseHat(grasp_res_hand2, icart_left);
                }

                if (right_arm_client.isValid())
                {
                    success = computePoseHat(grasp_res_hand1, icart_right);
                }
            }
            else if (grasping_hand == WhichHand::HAND_RIGHT)
            {
                if (right_arm_client.isValid())
                {
                    //right_arm_client.view(icart_right);
                    computePoseHat(grasp_res_hand1, icart_right);
                }
            }
            else if (grasping_hand == WhichHand::HAND_LEFT)
            {
                if (left_arm_client.isValid())
                {
                    //left_arm_client.view(icart_left);
                    computePoseHat(grasp_res_hand1, icart_left);
                }
            }
        }
        else // TODO extend to R1 (see cardinal-grasp-points)
        {
            if(action_render_rpc.getOutputCount()<1)
            {
                yError() << prettyError( __FUNCTION__,  "getPoseCostFunction: no connection to action rendering module");
            }
            else
            {
                if (grasping_hand == WhichHand::BOTH)
                {
                    computePoseHatR1(grasp_res_hand1, "right");
                    computePoseHatR1(grasp_res_hand2, "left");
                }
                else if (grasping_hand == WhichHand::HAND_RIGHT)
                {
                    computePoseHatR1(grasp_res_hand1, "right");
                }
                else if (grasping_hand == WhichHand::HAND_LEFT)
                {
                    computePoseHatR1(grasp_res_hand1, "left");
                }
            }
        }

        // Refine pose cost
        grasp_estim.refinePoseCost(grasp_res_hand1);

        if (grasping_hand == WhichHand::BOTH)
        {
            grasp_estim.refinePoseCost(grasp_res_hand2);
            vis.addPoses(grasp_res_hand1.grasp_poses, grasp_res_hand2.grasp_poses);
        }
        else
        {
            vis.addPoses(grasp_res_hand1.grasp_poses);
        }

        /* ------ Select best pose ------ */

        GraspPoses best_graspPose;
        if (grasping_hand == WhichHand::HAND_RIGHT)
        {
            best_hand = "right";
            best_graspPose = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose];
            vis.highlightBestPose("right", "right", grasp_res_hand1.best_pose);
        }
        else if (grasping_hand == WhichHand::HAND_LEFT)
        {
            best_hand = "left";
            best_graspPose = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose];
            vis.highlightBestPose("left", "left", grasp_res_hand1.best_pose);
        }

        if(grasping_hand == WhichHand::BOTH)
        {
            int best_right = grasp_res_hand1.best_pose;
            int best_left = grasp_res_hand2.best_pose;

            if (grasp_res_hand1.grasp_poses[best_right].cost < grasp_res_hand2.grasp_poses[best_left].cost)
            {
                best_hand = "right";
                best_graspPose = grasp_res_hand1.grasp_poses[best_right];
                vis.highlightBestPose("right", "both", best_right);
            }
            else
            {
                best_hand = "left";
                best_graspPose = grasp_res_hand2.grasp_poses[best_left];
                vis.highlightBestPose("left", "both", best_left);
            }
        }

        /* ------ Calibrated pose ------ */

        // TODO See if calibration is necessary and in case adapt fixReachingOffset
        // to the tool scenario

        Eigen::VectorXd pose;
        pose.resize(7);
        pose.head(3) = best_graspPose.getGraspPosition();
        pose.tail(4) = best_graspPose.getGraspAxisAngle();
        Vector best_pose = eigenToYarp(pose);

        Vector old_pose = best_pose;
        //
        cout<< "|| Pose to be fixed with calibration offsets              :" << toEigen(old_pose).format(CommaInitFmt)<< endl;
        fixReachingOffset(old_pose, best_pose);
        cout<< "|| Fixed pose                                             :" << toEigen(best_pose).format(CommaInitFmt)<< endl;

    }

    /****************************************************************/
    void SuperquadricPipelineDemo::getTable()
    {
        bool table_ok = false;
        if (robot != "icubSim" && table_calib_rpc.getOutputCount() > 0)
        {
            Bottle table_cmd, table_rply;
            table_cmd.addVocab(Vocab::encode("get"));
            table_cmd.addString("table");

            table_calib_rpc.write(table_cmd, table_rply);
            if (Bottle *payload = table_rply.get(0).asList())
            {
                if (payload->size() >= 2)
                {
                    plane(0) = 0.0;
                    plane(1) = 0.0;
                    plane(2) = 1.0;

                    plane(3) = -payload->get(1).asDouble() + 0.035;
                    table_ok = true;

                    grasp_estim.setVector("plane", plane);
                }
            }
        }
        if (!table_ok)
        {
            cout << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Unable to retrieve object table                      ||" << endl;
            cout << "|| Unsig default value                                  : " << -plane[3] << endl;
            cout << "|| ---------------------------------------------------- ||" << endl << endl;
        }
        else
        {
            cout << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Unsig plane value                                    : " << -plane[3] << endl;
            cout << "|| ---------------------------------------------------- ||" << endl << endl;
        }
    }

    /****************************************************************/
    void SuperquadricPipelineDemo::setGraspContext(ICartesianControl *icart)
    {
        //  set up the context for the grasping planning and execution
        //  enable all joints
        Vector dof;
        icart->getDOF(dof);
        Vector new_dof(10, 1);
        new_dof(1) = 0.0; //disable roll of the torso (lateral movement)
        icart->setDOF(new_dof, dof);
        icart->setPosePriority("position");
        icart->setInTargetTol(0.001);
    }

    /****************************************************************/
    Vector SuperquadricPipelineDemo::eigenToYarp(Eigen::VectorXd &v)
    {
        Vector x;
        x.resize(v.size());

        for (size_t i = 0; i< x.size(); i++)
        {
            x[i] = v[i];
        }

        return x;
    }

    /****************************************************************/
    deque<Eigen::Vector3d> SuperquadricPipelineDemo::vectorYarptoEigen(vector<Vector> &yarp_points)
    {
        deque<Eigen::Vector3d> eigen_points;

        for (size_t i = 0; i < yarp_points.size(); i++)
        {
            eigen_points.push_back(toEigen(yarp_points[i]));
        }

        return eigen_points;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::computePoseHat(GraspResults &grasp_res, ICartesianControl *icart)
    {
        for (size_t i = 0; i < 2; i++) //grasp_res.grasp_poses.size()
        {
            int context_backup;
            icart->storeContext(&context_backup);
            //  set up the context for the computation of the candidates
            setGraspContext(icart);

            Eigen::VectorXd desired_pose = grasp_res.grasp_poses[i].getGraspPosition();
            Eigen::VectorXd desired_or = grasp_res.grasp_poses[i].getGraspAxisAngle();

            Vector x_d = eigenToYarp(desired_pose);
            Vector o_d = eigenToYarp(desired_or);

            Vector x_d_hat, o_d_hat, q_d_hat;

            bool success = icart->askForPose(x_d, o_d, x_d_hat, o_d_hat, q_d_hat);

            //  restore previous context
            icart->restoreContext(context_backup);
            icart->deleteContext(context_backup);

            if(!success)
            {
                yError() << prettyError( __FUNCTION__,  "computePoseHat: could not communicate with kinematics module");
                return false;
            }

            Eigen::VectorXd pose_hat = toEigen(x_d_hat);
            Eigen::VectorXd or_hat = toEigen(o_d_hat);

            Eigen::VectorXd robot_pose;
            robot_pose.resize(7);
            robot_pose.head(3) = pose_hat;
            robot_pose.tail(4) = or_hat;

            grasp_res.grasp_poses[i].setGraspParamsHat(robot_pose);
        }


    }

    /****************************************************************/
    void SuperquadricPipelineDemo::computePoseHatR1(GraspResults &grasp_res, const string &hand)
    {
        for (size_t i = 0; i < grasp_res.grasp_poses.size(); i++)
        {
            Eigen::VectorXd desired_pose = grasp_res.grasp_poses[i].getGraspPosition();
            Eigen::VectorXd desired_or = grasp_res.grasp_poses[i].getGraspAxisAngle();

            Vector x_d = eigenToYarp(desired_pose);
            Vector o_d = eigenToYarp(desired_or);

            Bottle cmd, reply;
            cmd.addVocab(Vocab::encode("ask"));
            Bottle &subcmd = cmd.addList();
            for(int i=0 ; i<3 ; i++) subcmd.addDouble(x_d[i]);
            for(int i=0 ; i<4 ; i++) subcmd.addDouble(o_d[i]);

            cmd.addString(hand);

            action_render_rpc.write(cmd, reply);

            if(reply.size()<1)
            {
                yError() << prettyError( __FUNCTION__,  "getPoseCostFunction: empty reply from action rendering module");
            }

            if(reply.get(0).asVocab() != Vocab::encode("ack"))
            {
                yError() << prettyError( __FUNCTION__,  "getPoseCostFunction: invalid reply from action rendering module:") << reply.toString();
            }

            if(reply.size()<3)
            {
                yError() << prettyError( __FUNCTION__,  "getPoseCostFunction: invlaid reply size from action rendering module") << reply.toString();
            }

            if(!reply.check("x"))
            {
                yError() << prettyError( __FUNCTION__,  "getPoseCostFunction: invalid reply from action rendering module: missing x:") << reply.toString();
            }

            Bottle *position = reply.find("x").asList();
            Eigen::VectorXd robot_pose;
            robot_pose.resize(7);
            for(int i=0 ; i<3 ; i++) robot_pose(i) = position->get(i).asDouble();
            for(int i=3 ; i<7 ; i++) robot_pose(i) = position->get(i).asDouble();

            // for (size_t i = 0; i < grasp_res.grasp_poses.size(); i++)
            // {
            grasp_res.grasp_poses[i].setGraspParamsHat(robot_pose);
            // }
        }
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::take_tool()
    {
        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,", ", ", ", "", "", " [ ", "]");

        Vector grasping_current_pos(3), grasping_current_o(4);
        if (best_hand == "right")
        {
            // store context
            int context_backup;
            icart_right->storeContext(&context_backup);
            setGraspContext(icart_right);

            for(const PointD& p : take_tool_trajectory)
            {
                icart_right->getPose(grasping_current_pos, grasping_current_o);

                grasping_current_pos[0] += p.x;
                grasping_current_pos[1] += p.y;
                grasping_current_pos[2] += p.z;

                cout << "|| ---------------------------------------------------- ||"  << endl;
                cout << "|| moving to: " << toEigen(grasping_current_pos).format(CommaInitFmt) << endl;

                icart_right->goToPoseSync(grasping_current_pos, grasping_current_o);
                icart_right->waitMotionDone();
            }

            // retrieve context
            icart_right->restoreContext(context_backup);
            icart_right->deleteContext(context_backup);
        }

        else if (best_hand == "left")
        {
            // store context
            int context_backup;
            icart_left->storeContext(&context_backup);
            setGraspContext(icart_left);

            for(const PointD& p : take_tool_trajectory)
            {
                icart_left->getPose(grasping_current_pos, grasping_current_o);

                grasping_current_pos[0] += p.x;
                grasping_current_pos[1] -= p.y;
                grasping_current_pos[2] += p.z;

                cout << "|| ---------------------------------------------------- ||"  << endl;
                cout << "|| moving to: " << toEigen(grasping_current_pos).format(CommaInitFmt) << endl;

                icart_left->goToPoseSync(grasping_current_pos, grasping_current_o);
                icart_left->waitMotionDone();
            }

            // retrieve context
            icart_left->restoreContext(context_backup);
            icart_left->deleteContext(context_backup);
        }

        // send robot to home position and return
        return this->home();
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::open_hand(const string &hand)
    {
        if (action_render_rpc.getOutputCount() > 0)
        {
            Bottle cmd, reply;
            cmd.addVocab(Vocab::encode("hand"));
            cmd.addString("open_hand");
            if(hand.compare("right") && hand.compare("left"))
            {
                yWarning() << "Specified hand is unknown. Opening the hand in use";
            }
            else
            {
                yInfo() << "opening hand " << hand;
                cmd.addString(hand);
            }

            action_render_rpc.write(cmd, reply);
            if (reply.get(0).asVocab() == Vocab::encode("ack"))
                return true;
            else
                return false;
        }
        else
            return false;
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::executeGrasp(Vector &pose, string &best_hand)
    {
        if(robot == "icubSim")
        {
            //  simulation context, suppose there is no actionsRenderingEngine running
            if (best_hand == "right")
            {
                // store context
                int context_backup;
                icart_right->storeContext(&context_backup);
                setGraspContext(icart_right);

                Vector previous_x(3), previous_o(4);
                icart_right->getPose(previous_x, previous_o);

                // intermediate pose
                Vector pose_intermediate(7,0.0);
                pose_intermediate.setSubvector(0,pose.subVector(0,2) + grasper_approach_parameters_right.subVector(0,2));
                pose_intermediate.setSubvector(3, pose.subVector(3,6));

                cout << "|| ---------------------------------------------------- ||"  << endl;
                cout << "|| Right hand reaching  intermediate                    :" << toEigen(pose_intermediate).format(CommaInitFmt) << endl;

                icart_right->goToPoseSync(pose_intermediate.subVector(0, 2), pose_intermediate.subVector(3,6));
                icart_right->waitMotionDone();

                // final pose
                cout << "|| Right hand reaching                                  :" << toEigen(pose).format(CommaInitFmt) << endl;
                icart_right->goToPoseSync(pose.subVector(0, 2), pose.subVector(3,6));
                icart_right->waitMotionDone();

                // retrieve context
                icart_right->restoreContext(context_backup);
                icart_right->deleteContext(context_backup);

                return true;
            }
            else if (best_hand == "left")
            {
                // store context
                int context_backup;
                icart_left->storeContext(&context_backup);
                setGraspContext(icart_left);

                // intermediate pose
                Vector pose_intermediate(7,0.0);
                pose_intermediate.setSubvector(0,pose.subVector(0,2) + grasper_approach_parameters_left.subVector(0,2));
                pose_intermediate.setSubvector(3, pose.subVector(3,6));

                cout << "|| ---------------------------------------------------- ||"  << endl;
                cout << "|| Left hand reaching  pose intermediate                :" << toEigen(pose_intermediate).format(CommaInitFmt) << endl;

                Vector previous_x(3), previous_o(4);
                icart_left->getPose(previous_x, previous_o);

                icart_left->goToPoseSync(pose_intermediate.subVector(0, 2), pose_intermediate.subVector(3,6));
                icart_left->waitMotionDone();

                // final pose
                cout << "|| ---------------------------------------------------- ||"  << endl;
                cout << "|| Left hand reaching  pose                             :" << toEigen(pose).format(CommaInitFmt) << endl;
                icart_left->goToPoseSync(pose.subVector(0, 2), pose.subVector(3,6));
                icart_left->waitMotionDone();

                // retrieve context
                icart_left->restoreContext(context_backup);
                icart_left->deleteContext(context_backup);

                return true;
            }
        }
        else
        {
            //  communication with actionRenderingEngine/cmd:io
            //  grasp("cartesian" x y z gx gy gz theta) ("approach" (-0.05 0 +-0.05 0.0)) "left"/"right"
            Bottle command, reply;

            command.addString("grasp");
            Bottle &ptr = command.addList();

            ptr.addString("cartesian");
            for(double & p: pose) ptr.addDouble(p);

            Bottle &ptr1 = command.addList();
            ptr1.addString("approach");

            Bottle &ptr2 = ptr1.addList();
            if (best_hand == "left")
            {
                for(double & p: grasper_approach_parameters_left) ptr2.addDouble(p);
                command.addString("left");
            }
            else
            {
                for(double & p: grasper_approach_parameters_right) ptr2.addDouble(p);
                command.addString("right");
            }

            //Prevent robot for bringing back home after grasp
            command.addString("still");

            yInfo() << command.toString();
            action_render_rpc.write(command, reply);
            if (reply.get(0).asVocab() == Vocab::encode("ack"))
                return true;
            else
                return false;
        }

    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::isInClasses(const string &obj_name)
    {
        auto it = find(classes.begin(), classes.end(), obj_name);
        if (it != classes.end())
        {
            cout << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Object name is one of the classes                    ||" << endl;
            cout << "|| ---------------------------------------------------- ||" << endl << endl;
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool SuperquadricPipelineDemo::attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool SuperquadricPipelineDemo::fixReachingOffset(const Vector &poseToFix, Vector &poseFixed,
                           const bool invert)
    {
        //  fix the pose offset according to iolReachingCalibration
        //  pose is supposed to be (x y z gx gy gz theta)
        if ((robot == "r1" || robot == "icub") && reach_calib_rpc.getOutputCount() > 0)
        {
            Bottle command, reply;

            command.addString("get_location_nolook");
            if (best_hand == "left")
                command.addString("iol-left");

            else
                command.addString("iol-right");

            command.addDouble(poseToFix(0));    //  x value
            command.addDouble(poseToFix(1));    //  y value
            command.addDouble(poseToFix(2));    //  z value
            command.addInt(invert?1:0);         //  flag to invert input/output map

            reach_calib_rpc.write(command, reply);

            //  incoming reply is going to be (success x y z)
            if (reply.size() < 2)
            {
                yError() << prettyError( __FUNCTION__,  "Failure retrieving fixed pose");
                return false;
            }
            else if (reply.get(0).asVocab() == Vocab::encode("ok"))
            {
                poseFixed = poseToFix;
                poseFixed(0) = reply.get(1).asDouble();
                poseFixed(1) = reply.get(2).asDouble();
                poseFixed(2) = reply.get(3).asDouble();
                return true;
            }
            else
            {
                yWarning() << "Couldn't retrieve fixed pose. Continuing with unchanged pose";
                return true;
            }
        }
        else
        {
            //  if we are working with the simulator or there is no calib map, the pose doesn't need to be corrected
            poseFixed = poseToFix;
            yWarning() << "Connection to iolReachingCalibration not detected or calibration map not present: pose will not be changed";
            return true;
        }
    }


int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.setDefaultContext("Superquadric-Lib-Demo");
    rf.setDefaultConfigFile("config-icub.ini");
    rf.configure(argc, argv);

    if (!yarp.checkNetwork())
    {
        yError() << prettyError(__FUNCTION__, "YARP network not detected. Check nameserver");
        return EXIT_FAILURE;
    }

    SuperquadricPipelineDemo disp;

    return disp.runModule(rf);
}
