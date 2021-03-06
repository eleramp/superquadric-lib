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
  * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
  */

#include <iostream>

#include <SuperquadricLibModel/options.h>

using namespace std;

Options::Options()
{
    pars.tol = 1e-5;
    pars.acceptable_iter = 0;
    pars.max_iter = 1000000;
    pars.max_cpu_time = 5.0;
    pars.mu_strategy = "adaptive";
    pars.nlp_scaling_method = "gradient-based";
    pars.hessian_approximation = "limited-memory";
    pars.print_level = 0;
    pars.object_class = "default";
    pars.optimizer_points = 50;
    pars.random_sampling = true;
}

/****************************************************************/
bool Options::SetNumericValue(const string &tag, const double &value)
{
    if (tag == "tol")
    {
        pars.tol = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Tolerance set                                        : " << pars.tol <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "constr_tol")
    {
        pars.constr_tol = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Constraint tolerance set                             : " << pars.constr_tol <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "max_cpu_time")
    {
        pars.max_cpu_time = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Max cpu time set                                     : " << pars.max_cpu_time <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    // Multiple superquadric estimation
    else if (tag == "threshold_axis")
    {
        m_pars.threshold_axis = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Threshold axis set                                   : " << m_pars.threshold_axis <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "threshold_section1")
    {
        m_pars.threshold_section1 = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Threshold section no.1 set                           : " << m_pars.threshold_section1 <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "threshold_section2")
    {
        m_pars.threshold_section2 = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Threshold section no.2 set                           : " << m_pars.threshold_section2 <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else
    {
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Not valid tag for numeric variable!                    " <<  endl << endl;
        return false;
    }
}

/****************************************************************/
bool Options::SetIntegerValue(const string &tag, const int &value)
{
    // Common params
    if (tag == "acceptable_iter")
    {
        pars.acceptable_iter = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Acceptable iter set                                  : " << pars.acceptable_iter <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "max_iter")
    {
        pars.max_iter = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Max iteration set                                    : " << pars.max_iter <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "print_level")
    {
        pars.print_level = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Print level set                                      : " << pars.print_level <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    // Superquadric estimation
    else if (tag == "optimizer_points")
    {
        pars.optimizer_points = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Optimizer points set                                 : " << pars.optimizer_points <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    // Multiple superquadric estimation
    else if (tag == "minimum_points")
    {
        m_pars.minimum_points = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Minimum points for computation set                   : " << m_pars.minimum_points <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "fraction_pc")
    {
        m_pars.fraction_pc = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Desired fraction of point cloud set                  : " << m_pars.fraction_pc <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    // Grasp commputation
    else if (tag == "max_superq")
    {
        g_params.max_superq = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Max superq set                                       : " << g_params.max_superq <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else
    {
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Not valid tag for integer variable!                    " << endl << endl;
        return false;
    }
}
/****************************************************************/
bool Options::SetBoolValue(const string &tag, const bool &value)
{
    if (tag == "random_sampling")
    {
        pars.random_sampling = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Random sampling set                                  : " << pars.random_sampling <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "merge_model")
    {
        m_pars.merge_model = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Merge model set                                      : " << m_pars.merge_model <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else
    {
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Not valid tag for bool variable!                    " << endl << endl;
        return false;
    }
}
/****************************************************************/
bool Options::SetStringValue(const string &tag, const string &value)
{
    // Common params
    if (tag == "mu_strategy")
    {
        pars.mu_strategy = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Mu strategy set                                      : " << pars.mu_strategy <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "nlp_scaling_method")
    {
        pars.nlp_scaling_method = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Nlp scaling method set                               : " << pars.nlp_scaling_method <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else if (tag == "print_lehessian_approximationvel")
    {
        pars.hessian_approximation = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Hessian approximation set                            : " << pars.hessian_approximation <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    // Superquadric estimation
    else if (tag == "object_class")
    {
        pars.object_class = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Object class set                                     : " << pars.object_class <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    // Grasp commputation
    else if (tag == "left_or_right")
    {
        g_params.left_or_right = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Hand for grasp computation set                       : " << g_params.left_or_right <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

        return true;
    }
    else
    {
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Not valid tag for string variable!                    " << endl << endl;
        return false;
    }
}
