/*
 * Attractor.cpp
 *
 * Description: Attractor function definitions.
 *
 *     Authors: Guillermo Oliver, Pablo Lanillos, Gordon Cheng
 *
 * Copyright 2019 Institute for Cognitive Systems, Technical University of Munich. All rights reserved.
 *
 * This file is part of the ActiveInferenceICub project.
 *
 * ActiveInferenceICub is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "FirstDerivative.h" //functions from first derivative
#include "Attractor.h"

//Alternative: Hessian approximation using H = (J^T)*(J)

//----------------------------------------------------------------------------------
//Attractor terms ------------------------------------------------------------------
//----------------------------------------------------------------------------------

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

//----------------------------------------------------------------------------------
//Attractor terms ------------------------------------------------------------------
//----------------------------------------------------------------------------------

//3d jacobian right arm ------------------------------------------------------------
void calc_3d_jacobian(vector<double> mu, vector<double> a)
{
	//calculate derivative of rototranslational matrix
	calc_d_3d_rototranslational_matrix_mu1(mu, a);
	calc_d_3d_rototranslational_matrix_mu2(mu, a);
	calc_d_3d_rototranslational_matrix_mu3(mu, a);
	
	//calculate the two parts of the jacobian
	ra_3d_jacobian_mu1 = ra_d1rtm * ra_repv;
	ra_3d_jacobian_mu2 = ra_d2rtm * ra_repv;
	ra_3d_jacobian_mu3 = ra_d3rtm * ra_repv;
	
	//build the jacobian
	ra_3d_jacobian(0,0) = ra_3d_jacobian_mu1(0)/k;
	ra_3d_jacobian(1,0) = ra_3d_jacobian_mu1(1)/k;
	ra_3d_jacobian(2,0) = ra_3d_jacobian_mu1(2)/k;
	ra_3d_jacobian(0,1) = ra_3d_jacobian_mu2(0)/k;
	ra_3d_jacobian(1,1) = ra_3d_jacobian_mu2(1)/k;
	ra_3d_jacobian(2,1) = ra_3d_jacobian_mu2(2)/k;
	ra_3d_jacobian(0,2) = ra_3d_jacobian_mu3(0)/k;
	ra_3d_jacobian(1,2) = ra_3d_jacobian_mu3(1)/k;
	ra_3d_jacobian(2,2) = ra_3d_jacobian_mu3(2)/k;
}

void calc_3d_jacobian_origin(vector<double> mu, vector<double> a)
{
	//end-effector position to origin
	Vector4d ra_origin;
	ra_origin(0) = 0;
	ra_origin(1) = 0;
	ra_origin(2) = 0;
	ra_origin(3) = 1;	

	//calculate derivative of rototranslational matrix
	calc_d_3d_rototranslational_matrix_mu1(mu, a);
	calc_d_3d_rototranslational_matrix_mu2(mu, a);
	calc_d_3d_rototranslational_matrix_mu3(mu, a);
	
	//calculate the two parts of the jacobian
	ra_3d_jacobian_mu1_origin = ra_d1rtm * ra_origin;
	ra_3d_jacobian_mu2_origin = ra_d2rtm * ra_origin;
	ra_3d_jacobian_mu3_origin = ra_d3rtm * ra_origin;

	//build the jacobian
	ra_3d_jacobian_origin(0,0) = ra_3d_jacobian_mu1_origin(0)/k;
	ra_3d_jacobian_origin(1,0) = ra_3d_jacobian_mu1_origin(1)/k;
	ra_3d_jacobian_origin(2,0) = ra_3d_jacobian_mu1_origin(2)/k;
	ra_3d_jacobian_origin(0,1) = ra_3d_jacobian_mu2_origin(0)/k;
	ra_3d_jacobian_origin(1,1) = ra_3d_jacobian_mu2_origin(1)/k;
	ra_3d_jacobian_origin(2,1) = ra_3d_jacobian_mu2_origin(2)/k;
	ra_3d_jacobian_origin(0,2) = ra_3d_jacobian_mu3_origin(0)/k;
	ra_3d_jacobian_origin(1,2) = ra_3d_jacobian_mu3_origin(1)/k;
	ra_3d_jacobian_origin(2,2) = ra_3d_jacobian_mu3_origin(2)/k;
}

//3d hessian terms -----------------------------------------------------------------
void calc_3d_hessian_terms(vector<double> mu, vector<double> a)
{
	//calculate second derivatives of rototranslational matrix
	calc_dd_3d_rototranslational_matrix_mu11(mu, a);
	calc_dd_3d_rototranslational_matrix_mu12(mu, a);
	calc_dd_3d_rototranslational_matrix_mu22(mu, a);
	calc_dd_3d_rototranslational_matrix_mu23(mu, a);
	calc_dd_3d_rototranslational_matrix_mu33(mu, a);
	calc_dd_3d_rototranslational_matrix_mu13(mu, a);
	
	//calculate the six parts of the three hessians
	ra_3d_hessian_mu11 = ra_d11rtm * ra_repv;
	ra_3d_hessian_mu12 = ra_d12rtm * ra_repv;
	ra_3d_hessian_mu22 = ra_d22rtm * ra_repv;
	ra_3d_hessian_mu23 = ra_d23rtm * ra_repv;
	ra_3d_hessian_mu33 = ra_d33rtm * ra_repv;
	ra_3d_hessian_mu13 = ra_d13rtm * ra_repv;
}

void calc_3d_hessian_terms_origin(vector<double> mu, vector<double> a)
{
	//end-effector position to origin
	Vector4d ra_origin;
	ra_origin(0) = 0;
	ra_origin(1) = 0;
	ra_origin(2) = 0;
	ra_origin(3) = 1;

	//calculate second derivatives of rototranslational matrix
	calc_dd_3d_rototranslational_matrix_mu11(mu, a);
	calc_dd_3d_rototranslational_matrix_mu12(mu, a);
	calc_dd_3d_rototranslational_matrix_mu22(mu, a);
	calc_dd_3d_rototranslational_matrix_mu23(mu, a);
	calc_dd_3d_rototranslational_matrix_mu33(mu, a);
	calc_dd_3d_rototranslational_matrix_mu13(mu, a);
	
	//calculate the three parts of the two hessians
	ra_3d_hessian_mu11_origin = ra_d11rtm * ra_origin;
	ra_3d_hessian_mu12_origin = ra_d12rtm * ra_origin;
	ra_3d_hessian_mu22_origin = ra_d22rtm * ra_origin;
	ra_3d_hessian_mu23_origin = ra_d23rtm * ra_origin;
	ra_3d_hessian_mu33_origin = ra_d33rtm * ra_origin;
	ra_3d_hessian_mu13_origin = ra_d13rtm * ra_origin;
}

//3d hessian mu1 -------------------------------------------------------------------
void calc_3d_hessian_mu1(vector<double> mu, vector<double> a)
{
	calc_3d_hessian_terms(mu, a);

	//build the first hessian
	ra_3d_hessian_mu1(0,0) = ra_3d_hessian_mu11(0)/k;
	ra_3d_hessian_mu1(1,0) = ra_3d_hessian_mu11(1)/k;
	ra_3d_hessian_mu1(2,0) = ra_3d_hessian_mu11(2)/k;
	ra_3d_hessian_mu1(0,1) = ra_3d_hessian_mu12(0)/k;
	ra_3d_hessian_mu1(1,1) = ra_3d_hessian_mu12(1)/k;
	ra_3d_hessian_mu1(2,1) = ra_3d_hessian_mu12(2)/k;
	ra_3d_hessian_mu1(0,2) = ra_3d_hessian_mu13(0)/k;
	ra_3d_hessian_mu1(1,2) = ra_3d_hessian_mu13(1)/k;
	ra_3d_hessian_mu1(2,2) = ra_3d_hessian_mu13(2)/k;
}

void calc_3d_hessian_mu1_origin(vector<double> mu, vector<double> a)
{
	calc_3d_hessian_terms_origin(mu, a);
	
	//build the first hessian
	ra_3d_hessian_mu1_origin(0,0) = ra_3d_hessian_mu11_origin(0)/k;
	ra_3d_hessian_mu1_origin(1,0) = ra_3d_hessian_mu11_origin(1)/k;
	ra_3d_hessian_mu1_origin(2,0) = ra_3d_hessian_mu11_origin(2)/k;
	ra_3d_hessian_mu1_origin(0,1) = ra_3d_hessian_mu12_origin(0)/k;
	ra_3d_hessian_mu1_origin(1,1) = ra_3d_hessian_mu12_origin(1)/k;
	ra_3d_hessian_mu1_origin(2,1) = ra_3d_hessian_mu12_origin(2)/k;
	ra_3d_hessian_mu1_origin(0,2) = ra_3d_hessian_mu13_origin(0)/k;
	ra_3d_hessian_mu1_origin(1,2) = ra_3d_hessian_mu13_origin(1)/k;
	ra_3d_hessian_mu1_origin(2,2) = ra_3d_hessian_mu13_origin(2)/k;
}

//3d hessian mu2 -------------------------------------------------------------------
void calc_3d_hessian_mu2(vector<double> mu, vector<double> a)
{
	calc_3d_hessian_terms(mu, a);

	//build the second hessian
	ra_3d_hessian_mu2(0,0) = ra_3d_hessian_mu12(0)/k;
	ra_3d_hessian_mu2(1,0) = ra_3d_hessian_mu12(1)/k;
	ra_3d_hessian_mu2(2,0) = ra_3d_hessian_mu12(2)/k;
	ra_3d_hessian_mu2(0,1) = ra_3d_hessian_mu22(0)/k;
	ra_3d_hessian_mu2(1,1) = ra_3d_hessian_mu22(1)/k;
	ra_3d_hessian_mu2(2,1) = ra_3d_hessian_mu22(2)/k;
	ra_3d_hessian_mu2(0,2) = ra_3d_hessian_mu23(0)/k;
	ra_3d_hessian_mu2(1,2) = ra_3d_hessian_mu23(1)/k;
	ra_3d_hessian_mu2(2,2) = ra_3d_hessian_mu23(2)/k;	
}

void calc_3d_hessian_mu2_origin(vector<double> mu, vector<double> a)
{
	calc_3d_hessian_terms_origin(mu, a);

	//build the second hessian
	ra_3d_hessian_mu2_origin(0,0) = ra_3d_hessian_mu12_origin(0)/k;
	ra_3d_hessian_mu2_origin(1,0) = ra_3d_hessian_mu12_origin(1)/k;
	ra_3d_hessian_mu2_origin(2,0) = ra_3d_hessian_mu12_origin(2)/k;
	ra_3d_hessian_mu2_origin(0,1) = ra_3d_hessian_mu22_origin(0)/k;
	ra_3d_hessian_mu2_origin(1,1) = ra_3d_hessian_mu22_origin(1)/k;
	ra_3d_hessian_mu2_origin(2,1) = ra_3d_hessian_mu22_origin(2)/k;	
	ra_3d_hessian_mu2_origin(0,2) = ra_3d_hessian_mu23_origin(0)/k;
	ra_3d_hessian_mu2_origin(1,2) = ra_3d_hessian_mu23_origin(1)/k;
	ra_3d_hessian_mu2_origin(2,2) = ra_3d_hessian_mu23_origin(2)/k;	
}

//3d hessian mu3 -------------------------------------------------------------------
void calc_3d_hessian_mu3(vector<double> mu, vector<double> a)
{
	calc_3d_hessian_terms(mu, a);

	//build the third hessian
	ra_3d_hessian_mu3(0,0) = ra_3d_hessian_mu13(0)/k;
	ra_3d_hessian_mu3(1,0) = ra_3d_hessian_mu13(1)/k;
	ra_3d_hessian_mu3(2,0) = ra_3d_hessian_mu13(2)/k;
	ra_3d_hessian_mu3(0,1) = ra_3d_hessian_mu23(0)/k;
	ra_3d_hessian_mu3(1,1) = ra_3d_hessian_mu23(1)/k;
	ra_3d_hessian_mu3(2,1) = ra_3d_hessian_mu23(2)/k;
	ra_3d_hessian_mu3(0,2) = ra_3d_hessian_mu33(0)/k;
	ra_3d_hessian_mu3(1,2) = ra_3d_hessian_mu33(1)/k;
	ra_3d_hessian_mu3(2,2) = ra_3d_hessian_mu33(2)/k;		
}

void calc_3d_hessian_mu3_origin(vector<double> mu, vector<double> a)
{
	calc_3d_hessian_terms_origin(mu, a);

	//build the third hessian
	ra_3d_hessian_mu3_origin(0,0) = ra_3d_hessian_mu13_origin(0)/k;
	ra_3d_hessian_mu3_origin(1,0) = ra_3d_hessian_mu13_origin(1)/k;
	ra_3d_hessian_mu3_origin(2,0) = ra_3d_hessian_mu13_origin(2)/k;
	ra_3d_hessian_mu3_origin(0,1) = ra_3d_hessian_mu23_origin(0)/k;
	ra_3d_hessian_mu3_origin(1,1) = ra_3d_hessian_mu23_origin(1)/k;
	ra_3d_hessian_mu3_origin(2,1) = ra_3d_hessian_mu23_origin(2)/k;	
	ra_3d_hessian_mu3_origin(0,2) = ra_3d_hessian_mu33_origin(0)/k;
	ra_3d_hessian_mu3_origin(1,2) = ra_3d_hessian_mu33_origin(1)/k;
	ra_3d_hessian_mu3_origin(2,2) = ra_3d_hessian_mu33_origin(2)/k;	
}

//3d jacobian left arm -------------------------------------------------------------
void calc_3d_jacobian_b(vector<double> mu, vector<double> a)
{
    //calculate derivative of rototranslational matrix
    calc_d_3d_rototranslational_matrix_mu1b(mu, a);
    calc_d_3d_rototranslational_matrix_mu2b(mu, a);
    calc_d_3d_rototranslational_matrix_mu3b(mu, a);

    //calculate the two parts of the jacobian
    la_3d_jacobian_mu1 = la_d1rtm * la_repv;
    la_3d_jacobian_mu2 = la_d2rtm * la_repv;
    la_3d_jacobian_mu3 = la_d3rtm * la_repv;

    //build the jacobian
    la_3d_jacobian(0,0) = la_3d_jacobian_mu1(0)/k;
    la_3d_jacobian(1,0) = la_3d_jacobian_mu1(1)/k;
    la_3d_jacobian(2,0) = la_3d_jacobian_mu1(2)/k;
    la_3d_jacobian(0,1) = la_3d_jacobian_mu2(0)/k;
    la_3d_jacobian(1,1) = la_3d_jacobian_mu2(1)/k;
    la_3d_jacobian(2,1) = la_3d_jacobian_mu2(2)/k;
    la_3d_jacobian(0,2) = la_3d_jacobian_mu3(0)/k;
    la_3d_jacobian(1,2) = la_3d_jacobian_mu3(1)/k;
    la_3d_jacobian(2,2) = la_3d_jacobian_mu3(2)/k;
}

void calc_3d_jacobian_origin_b(vector<double> mu, vector<double> a)
{
    //end-effector position to origin
    Vector4d la_origin;
    la_origin(0) = 0;
    la_origin(1) = 0;
    la_origin(2) = 0;
    la_origin(3) = 1;

    //calculate derivative of rototranslational matrix
    calc_d_3d_rototranslational_matrix_mu1b(mu, a);
    calc_d_3d_rototranslational_matrix_mu2b(mu, a);
    calc_d_3d_rototranslational_matrix_mu3b(mu, a);

    //calculate the two parts of the jacobian
    la_3d_jacobian_mu1_origin = la_d1rtm * la_origin;
    la_3d_jacobian_mu2_origin = la_d2rtm * la_origin;
    la_3d_jacobian_mu3_origin = la_d3rtm * la_origin;

    //build the jacobian
    la_3d_jacobian_origin(0,0) = la_3d_jacobian_mu1_origin(0)/k;
    la_3d_jacobian_origin(1,0) = la_3d_jacobian_mu1_origin(1)/k;
    la_3d_jacobian_origin(2,0) = la_3d_jacobian_mu1_origin(2)/k;
    la_3d_jacobian_origin(0,1) = la_3d_jacobian_mu2_origin(0)/k;
    la_3d_jacobian_origin(1,1) = la_3d_jacobian_mu2_origin(1)/k;
    la_3d_jacobian_origin(2,1) = la_3d_jacobian_mu2_origin(2)/k;
    la_3d_jacobian_origin(0,2) = la_3d_jacobian_mu3_origin(0)/k;
    la_3d_jacobian_origin(1,2) = la_3d_jacobian_mu3_origin(1)/k;
    la_3d_jacobian_origin(2,2) = la_3d_jacobian_mu3_origin(2)/k;
}

//visual jacobian ------------------------------------------------------------------
void calc_visual_jacobian(vector<double> mu, vector<double> a)
{	
/*	//set end-effector position to origin
	ra_repv(0) = 0;
	ra_repv(1) = 0;
	ra_repv(2) = 0;
	ra_repv(3) = 1;*/

	//calculate all parameters and projections
	calc_le_camera_projection(mu,a);
	calc_d_le_camera_projection_mu1(mu,a);
	calc_d_le_camera_projection_mu2(mu,a);
	calc_d_le_camera_projection_mu3(mu,a);
	calc_camera_parameters(cam_projection);
	calc_d_camera_parameters_mu1(cam_projection, d1_cam_projection);
	calc_d_camera_parameters_mu2(cam_projection, d2_cam_projection);
	calc_d_camera_parameters_mu3(cam_projection, d3_cam_projection);

/*	//set end-effector position back to marker
	ra_repv(0) = xh;
	ra_repv(1) = yh;
	ra_repv(2) = zh;
	ra_repv(3) = 1;*/
	
	//build the visual jacobian
	ra_visual_jacobian(0,0) = d_vision_u_mu1(mu,a);
	ra_visual_jacobian(1,0) = d_vision_v_mu1(mu,a);
	ra_visual_jacobian(0,1) = d_vision_u_mu2(mu,a);
	ra_visual_jacobian(1,1) = d_vision_v_mu2(mu,a);
	ra_visual_jacobian(0,2) = d_vision_u_mu3(mu,a);
	ra_visual_jacobian(1,2) = d_vision_v_mu3(mu,a);
}

void calc_visual_jacobian_head(vector<double> mu, vector<double> a)
{	
	//calculate all parameters and projections
	calc_le_camera_origin(mu, a);
	calc_d_le_camera_origin_mu1(mu, a);
	calc_d_le_camera_origin_mu2(mu, a);
	calc_camera_parameters(le_projection);
	calc_d_camera_parameters_mu1(le_projection, d1_le_projection);
	calc_d_camera_parameters_mu2(le_projection, d2_le_projection);
	
	//build the visual jacobian
	ra_visual_jacobian_head(0,0) = d_vision_u_mu1_head(mu,a);
	ra_visual_jacobian_head(1,0) = d_vision_v_mu1_head(mu,a);
	ra_visual_jacobian_head(0,1) = d_vision_u_mu2_head(mu,a);
	ra_visual_jacobian_head(1,1) = d_vision_v_mu2_head(mu,a);
}

void calc_visual_jacobian_b(vector<double> mu, vector<double> a)
{
/*	//set end-effector position to origin
    ra_repv(0) = 0;
    ra_repv(1) = 0;
    ra_repv(2) = 0;
    ra_repv(3) = 1;*/

    //calculate all parameters and projections
    calc_le_camera_projection_b(mu,a);
    calc_d_le_camera_projection_mu1_b(mu,a);
    calc_d_le_camera_projection_mu2_b(mu,a);
    calc_d_le_camera_projection_mu3_b(mu,a);
    calc_camera_parameters_b(cam_projection_b);
    calc_d_camera_parameters_mu1_b(cam_projection_b, d1_cam_projection_b);
    calc_d_camera_parameters_mu2_b(cam_projection_b, d2_cam_projection_b);
    calc_d_camera_parameters_mu3_b(cam_projection_b, d3_cam_projection_b);

/*	//set end-effector position back to marker
    ra_repv(0) = xh;
    ra_repv(1) = yh;
    ra_repv(2) = zh;
    ra_repv(3) = 1;*/

    //build the visual jacobian
    la_visual_jacobian(0,0) = d_vision_u_mu1_b(mu,a);
    la_visual_jacobian(1,0) = d_vision_v_mu1_b(mu,a);
    la_visual_jacobian(0,1) = d_vision_u_mu2_b(mu,a);
    la_visual_jacobian(1,1) = d_vision_v_mu2_b(mu,a);
    la_visual_jacobian(0,2) = d_vision_u_mu3_b(mu,a);
    la_visual_jacobian(1,2) = d_vision_v_mu3_b(mu,a);
}

//visual hessian terms -------------------------------------------------------------
void calc_visual_hessian_terms(vector<double> mu, vector<double> a)
{
	double d11xpp, d22xpp, d12xpp, d11ypp, d22ypp, d12ypp;
	double d13xpp, d33xpp, d23xpp, d13ypp, d33ypp, d23ypp;
	
/*	//set end-effector position to origin
	ra_repv(0) = 0;
	ra_repv(1) = 0;
	ra_repv(2) = 0;
	ra_repv(3) = 1;*/

	//calculate all parameters and projections
	calc_le_camera_projection(mu,a);
	calc_d_le_camera_projection_mu1(mu,a);
	calc_d_le_camera_projection_mu2(mu,a);
	calc_d_le_camera_projection_mu3(mu,a);
	calc_d_le_camera_projection_mu11(mu,a);
	calc_d_le_camera_projection_mu22(mu,a);
	calc_d_le_camera_projection_mu12(mu,a);
	calc_d_le_camera_projection_mu23(mu,a);
	calc_d_le_camera_projection_mu33(mu,a);
	calc_d_le_camera_projection_mu13(mu,a);
	calc_camera_parameters(cam_projection);
	calc_d_camera_parameters_mu1(cam_projection, d1_cam_projection);
	calc_d_camera_parameters_mu2(cam_projection, d2_cam_projection);
	calc_d_camera_parameters_mu3(cam_projection, d3_cam_projection);
	calc_d_camera_parameters_mu11(cam_projection, d1_cam_projection, d11_cam_projection);
	calc_d_camera_parameters_mu22(cam_projection, d2_cam_projection, d22_cam_projection);
	calc_d_camera_parameters_mu12(cam_projection, d1_cam_projection, d2_cam_projection, d12_cam_projection);
	calc_d_camera_parameters_mu23(cam_projection, d2_cam_projection, d3_cam_projection, d23_cam_projection);
	calc_d_camera_parameters_mu33(cam_projection, d3_cam_projection, d33_cam_projection);
	calc_d_camera_parameters_mu13(cam_projection, d1_cam_projection, d3_cam_projection, d13_cam_projection);

/*	//set end-effector position back to marker
	ra_repv(0) = xh;
	ra_repv(1) = yh;
	ra_repv(2) = zh;
	ra_repv(3) = 1;*/
	
	//calculate second order derivatives of visual projections
	d11xpp = d11xp*ck + 2*d1xp*d1k + xp*d11k + d11px + d11pxx;
	d22xpp = d22xp*ck + 2*d2xp*d2k + xp*d22k + d22px + d22pxx;
	d11ypp = d11yp*ck + 2*d1yp*d1k + yp*d11k + d11py + d11pyy;
	d22ypp = d22yp*ck + 2*d2yp*d2k + yp*d22k + d22py + d22pyy;
	d12xpp = d12xp*ck + d1xp*d2k + d2xp*d1k + xp*d12k + d12px + d12pxx;
	d12ypp = d12yp*ck + d1yp*d2k + d2yp*d1k + yp*d12k + d12py + d12pyy;
	d33xpp = d33xp*ck + 2*d3xp*d3k + xp*d33k + d33px + d33pxx;
	d33ypp = d33yp*ck + 2*d3yp*d3k + yp*d33k + d33py + d33pyy;
	d13xpp = d13xp*ck + d1xp*d2k + d3xp*d1k + xp*d13k + d13px + d13pxx;
	d13ypp = d13yp*ck + d1yp*d2k + d3yp*d1k + yp*d13k + d13py + d13pyy;
	d23xpp = d23xp*ck + d2xp*d2k + d3xp*d1k + xp*d23k + d23px + d23pxx;
	d23ypp = d23yp*ck + d2yp*d2k + d3yp*d1k + yp*d23k + d23py + d23pyy;
		
	//calculate the six parts of the three hessians
	ra_visual_hessian_mu11(0) = fx*d11xpp;
	ra_visual_hessian_mu11(1) = fy*d11ypp;
	ra_visual_hessian_mu22(0) = fx*d22xpp;
	ra_visual_hessian_mu22(1) = fy*d22ypp;
	ra_visual_hessian_mu12(0) = fx*d12xpp;
	ra_visual_hessian_mu12(1) = fy*d12ypp;
	ra_visual_hessian_mu13(0) = fx*d13xpp;
	ra_visual_hessian_mu13(1) = fy*d13ypp;
	ra_visual_hessian_mu23(0) = fx*d23xpp;
	ra_visual_hessian_mu23(1) = fy*d23ypp;
	ra_visual_hessian_mu33(0) = fx*d33xpp;
	ra_visual_hessian_mu33(1) = fy*d33ypp;

}

//visual hessian mu1 ---------------------------------------------------------------
void calc_visual_hessian_mu1(vector<double> mu, vector<double> a)
{
	//calculate hessian terms
	calc_visual_hessian_terms(mu, a);	

	//build the first hessian
	ra_visual_hessian_mu1(0,0) = ra_visual_hessian_mu11(0);
	ra_visual_hessian_mu1(1,0) = ra_visual_hessian_mu11(1);
	ra_visual_hessian_mu1(0,1) = ra_visual_hessian_mu12(0);
	ra_visual_hessian_mu1(1,1) = ra_visual_hessian_mu12(1);
	ra_visual_hessian_mu1(0,2) = ra_visual_hessian_mu13(0);
	ra_visual_hessian_mu1(1,2) = ra_visual_hessian_mu13(1);
}

//visual hessian mu2 ---------------------------------------------------------------
void calc_visual_hessian_mu2(vector<double> mu, vector<double> a)
{
	//calculate hessian terms
	calc_visual_hessian_terms(mu, a);
	
	//build the second hessian
	ra_visual_hessian_mu2(0,0) = ra_visual_hessian_mu12(0);
	ra_visual_hessian_mu2(1,0) = ra_visual_hessian_mu12(1);
	ra_visual_hessian_mu2(0,1) = ra_visual_hessian_mu22(0);
	ra_visual_hessian_mu2(1,1) = ra_visual_hessian_mu22(1);
	ra_visual_hessian_mu2(0,2) = ra_visual_hessian_mu23(0);
	ra_visual_hessian_mu2(1,2) = ra_visual_hessian_mu23(1);
}

//visual hessian mu3 ---------------------------------------------------------------
void calc_visual_hessian_mu3(vector<double> mu, vector<double> a)
{
	//calculate hessian terms
	calc_visual_hessian_terms(mu, a);
	
	//build the second hessian
	ra_visual_hessian_mu3(0,0) = ra_visual_hessian_mu13(0);
	ra_visual_hessian_mu3(1,0) = ra_visual_hessian_mu13(1);
	ra_visual_hessian_mu3(0,1) = ra_visual_hessian_mu23(0);
	ra_visual_hessian_mu3(1,1) = ra_visual_hessian_mu23(1);
	ra_visual_hessian_mu3(0,2) = ra_visual_hessian_mu33(0);
	ra_visual_hessian_mu3(1,2) = ra_visual_hessian_mu33(1);
}

//3d -------------------------------------------------------------------------------
void calc_attractor_3d_vector(vector<double> mu, vector<double> a)
{
	Vector3d calc_3d_pos;
	Vector3d attr_3d_pos;
	
/*	//set end-effector position to origin
	ra_repv(0) = 0;
	ra_repv(1) = 0;
	ra_repv(2) = 0;
	ra_repv(3) = 1;*/

	//calculate 3d position
	calc_3d_pos(0) = calc_3d_position_x(mu,a);
	calc_3d_pos(1) = calc_3d_position_y(mu,a);
	calc_3d_pos(2) = calc_3d_position_z(mu,a);
	
/*	//set end-effector position back to marker
	ra_repv(0) = xh;
	ra_repv(1) = yh;
	ra_repv(2) = zh;
	ra_repv(3) = 1;*/
	
	//build attractor position
	attr_3d_pos(0) = ro2;
	attr_3d_pos(1) = ro3;
	attr_3d_pos(2) = ro4;
	
	//calculate attractor vector
	ra_3d_attr = ro1 * (attr_3d_pos - calc_3d_pos);
}

double calc_attractor_3d_f1(vector<double> mu, vector<double> a)
{
	double attr_3d_f1;
	
	calc_attractor_3d_vector(mu, a);
	
	ra_3d_f_attr = ra_3d_jacobian.transpose() * ra_3d_attr;
	
	attr_3d_f1 = ra_3d_f_attr(0);
	
	return attr_3d_f1;
	
}

double calc_attractor_3d_f2(vector<double> mu, vector<double> a)
{
	double attr_3d_f2;
	
	calc_attractor_3d_vector(mu, a);
	
	ra_3d_f_attr = ra_3d_jacobian.transpose() * ra_3d_attr;

	attr_3d_f2 = ra_3d_f_attr(1);
	
	return attr_3d_f2;
}

double calc_attractor_3d_f3(vector<double> mu, vector<double> a)
{
	double attr_3d_f3;
	
	calc_attractor_3d_vector(mu, a);
	
	ra_3d_f_attr = ra_3d_jacobian.transpose() * ra_3d_attr;

	attr_3d_f3 = ra_3d_f_attr(2);
	
	return attr_3d_f3;
}

double d_attractor_3d_f1_mu1(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f1_mu1;
	Vector3d d_calc_3d_pos_mu1;
	
	d_calc_3d_pos_mu1(0) = d_3d_position_x_mu1(mu,a);
	d_calc_3d_pos_mu1(1) = d_3d_position_y_mu1(mu,a);
	d_calc_3d_pos_mu1(2) = d_3d_position_z_mu1(mu,a);
	
	d1_3d_f_attractor = ra_3d_hessian_mu1.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu1;
	
	d_attr_3d_f1_mu1 = d1_3d_f_attractor(0);
	
	return d_attr_3d_f1_mu1;
	
}

double d_attractor_3d_f2_mu1(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f2_mu1;
	Vector3d d_calc_3d_pos_mu1;
	
	d_calc_3d_pos_mu1(0) = d_3d_position_x_mu1(mu,a);
	d_calc_3d_pos_mu1(1) = d_3d_position_y_mu1(mu,a);
	d_calc_3d_pos_mu1(2) = d_3d_position_z_mu1(mu,a);
	
	d1_3d_f_attractor = ra_3d_hessian_mu1.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu1;
	
	d_attr_3d_f2_mu1 = d1_3d_f_attractor(1);
	
	return d_attr_3d_f2_mu1;
}

double d_attractor_3d_f3_mu1(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f3_mu1;
	Vector3d d_calc_3d_pos_mu1;
	
	d_calc_3d_pos_mu1(0) = d_3d_position_x_mu1(mu,a);
	d_calc_3d_pos_mu1(1) = d_3d_position_y_mu1(mu,a);
	d_calc_3d_pos_mu1(2) = d_3d_position_z_mu1(mu,a);
	
	d1_3d_f_attractor = ra_3d_hessian_mu1.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu1;
	
	d_attr_3d_f3_mu1 = d1_3d_f_attractor(2);
	
	return d_attr_3d_f3_mu1;
}

double d_attractor_3d_f1_mu2(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f1_mu2;
	Vector3d d_calc_3d_pos_mu2;
	
	d_calc_3d_pos_mu2(0) = d_3d_position_x_mu2(mu,a);
	d_calc_3d_pos_mu2(1) = d_3d_position_y_mu2(mu,a);
	d_calc_3d_pos_mu2(2) = d_3d_position_z_mu2(mu,a);
	
	d2_3d_f_attractor = ra_3d_hessian_mu2.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu2;
	
	d_attr_3d_f1_mu2 = d2_3d_f_attractor(0);
	
	return d_attr_3d_f1_mu2;
}

double d_attractor_3d_f2_mu2(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f2_mu2;
	Vector3d d_calc_3d_pos_mu2;

	d_calc_3d_pos_mu2(0) = d_3d_position_x_mu2(mu,a);
	d_calc_3d_pos_mu2(1) = d_3d_position_y_mu2(mu,a);
	d_calc_3d_pos_mu2(2) = d_3d_position_z_mu2(mu,a);
	
	d2_3d_f_attractor = ra_3d_hessian_mu2.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu2;
	
	d_attr_3d_f2_mu2 = d2_3d_f_attractor(1);
	
	return d_attr_3d_f2_mu2;
}

double d_attractor_3d_f3_mu2(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f3_mu2;
	Vector3d d_calc_3d_pos_mu2;

	d_calc_3d_pos_mu2(0) = d_3d_position_x_mu2(mu,a);
	d_calc_3d_pos_mu2(1) = d_3d_position_y_mu2(mu,a);
	d_calc_3d_pos_mu2(2) = d_3d_position_z_mu2(mu,a);
	
	d2_3d_f_attractor = ra_3d_hessian_mu2.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu2;
	
	d_attr_3d_f3_mu2 = d2_3d_f_attractor(2);
	
	return d_attr_3d_f3_mu2;
}

double d_attractor_3d_f1_mu3(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f1_mu3;
	Vector3d d_calc_3d_pos_mu3;
	
	d_calc_3d_pos_mu3(0) = d_3d_position_x_mu3(mu,a);
	d_calc_3d_pos_mu3(1) = d_3d_position_y_mu3(mu,a);
	d_calc_3d_pos_mu3(2) = d_3d_position_z_mu3(mu,a);
	
	d3_3d_f_attractor = ra_3d_hessian_mu3.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu3;
	
	d_attr_3d_f1_mu3 = d3_3d_f_attractor(0);
	
	return d_attr_3d_f1_mu3;
}

double d_attractor_3d_f2_mu3(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f2_mu3;
	Vector3d d_calc_3d_pos_mu3;

	d_calc_3d_pos_mu3(0) = d_3d_position_x_mu3(mu,a);
	d_calc_3d_pos_mu3(1) = d_3d_position_y_mu3(mu,a);
	d_calc_3d_pos_mu3(2) = d_3d_position_z_mu3(mu,a);
	
	d3_3d_f_attractor = ra_3d_hessian_mu3.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu3;
	
	d_attr_3d_f2_mu3 = d3_3d_f_attractor(1);
	
	return d_attr_3d_f2_mu3;
}

double d_attractor_3d_f3_mu3(vector<double> mu, vector<double> a)
{
	double d_attr_3d_f3_mu3;
	Vector3d d_calc_3d_pos_mu3;

	d_calc_3d_pos_mu3(0) = d_3d_position_x_mu3(mu,a);
	d_calc_3d_pos_mu3(1) = d_3d_position_y_mu3(mu,a);
	d_calc_3d_pos_mu3(2) = d_3d_position_z_mu3(mu,a);
	
	d3_3d_f_attractor = ra_3d_hessian_mu3.transpose()*ra_3d_attr 
		- ro1*ra_3d_jacobian.transpose()*d_calc_3d_pos_mu3;
	
	d_attr_3d_f3_mu3 = d3_3d_f_attractor(2);
	
	return d_attr_3d_f3_mu3;
}

//Visual ---------------------------------------------------------------------------
//Right arm
void calc_attractor_visual_vector(vector<double> mu, vector<double> a)
{
	Vector2d calc_visual_pos;
	Vector2d attr_visual_pos;
	
/*	//set end-effector position to origin
	ra_repv(0) = 0;
	ra_repv(1) = 0;
	ra_repv(2) = 0;
	ra_repv(3) = 1;*/

	//calculate projected position
	calc_visual_pos(0) = calc_vision_u(mu,a);
	calc_visual_pos(1) = calc_vision_v(mu,a);
	
/*	//set end-effector position back to marker
	ra_repv(0) = xh;
	ra_repv(1) = yh;
	ra_repv(2) = zh;
	ra_repv(3) = 1;*/
	
	//build attractor position
    attr_visual_pos(0) = att_pos(0);
    attr_visual_pos(1) = att_pos(1);
	
	//calculate visual attractor vector
	ra_visual_attr = ro1 * (attr_visual_pos - calc_visual_pos);
	//ra_visual_attr(0) = ro1*0;
	//ra_visual_attr(1) = ro1*100;

	//cout << " ->>>> " << ro1 << " " << calc_visual_pos.transpose() << " " << attr_visual_pos.transpose() << " " <<  ra_visual_attr.transpose() << endl;
	
}

double calc_attractor_visual_f1(vector<double> mu, vector<double> a)
{
	double attr_visual_f1;
	
	calc_attractor_visual_vector(mu, a);
	calc_visual_jacobian(mu, a);
	
	//transpose -> inverse -> pseudoinverse
	ra_visual_f_attr = pseudoinverse(ra_visual_jacobian) * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.inverse() * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.transpose() * ra_visual_attr;
	
	//cout << ra_visual_jacobian.transpose() << endl << ra_visual_attr << endl;

	attr_visual_f1 = ra_visual_f_attr(0);
	
	return attr_visual_f1;
	
}

double calc_attractor_visual_f2(vector<double> mu, vector<double> a)
{
	double attr_visual_f2;
	
	//calc_attractor_visual_vector(mu, a);
	//calc_visual_jacobian(mu, a);
	
	//transpose -> inverse -> pseudoinverse
	//ra_visual_f_attr = pseudoinverse(ra_visual_jacobian) * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.inverse() * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.transpose() * ra_visual_attr;

	attr_visual_f2 = ra_visual_f_attr(1);
	
	return attr_visual_f2;
}

double calc_attractor_visual_f3(vector<double> mu, vector<double> a)
{
	double attr_visual_f3;
	
	//calc_attractor_visual_vector(mu, a);
	//calc_visual_jacobian(mu, a);
	
	//transpose -> inverse -> pseudoinverse
	//ra_visual_f_attr = pseudoinverse(ra_visual_jacobian) * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.inverse() * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.transpose() * ra_visual_attr;

	attr_visual_f3 = ra_visual_f_attr(2);
	
	return attr_visual_f3;
}

double d_attractor_visual_f1_mu1(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f1_mu1;
	Vector2d d_calc_visual_mu1;
	
	calc_visual_hessian_mu1(mu, a);

	d_calc_visual_mu1(0) = d_vision_u_mu1(mu,a);
	d_calc_visual_mu1(1) = d_vision_v_mu1(mu,a);
	
	d1_visual_f_attractor = ra_visual_hessian_mu1.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu1;
	
	d_attr_visual_f1_mu1 = d1_visual_f_attractor(0);
	
	return d_attr_visual_f1_mu1;
	
}

double d_attractor_visual_f2_mu1(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f2_mu1;
	Vector2d d_calc_visual_mu1;

	calc_visual_hessian_mu1(mu, a);

	d_calc_visual_mu1(0) = d_vision_u_mu1(mu,a);
	d_calc_visual_mu1(1) = d_vision_v_mu1(mu,a);
	
	d1_visual_f_attractor = ra_visual_hessian_mu1.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu1;
	
	d_attr_visual_f2_mu1 = d1_visual_f_attractor(1);
	
	return d_attr_visual_f2_mu1;
}

double d_attractor_visual_f3_mu1(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f3_mu1;
	Vector2d d_calc_visual_mu1;

	calc_visual_hessian_mu1(mu, a);

	d_calc_visual_mu1(0) = d_vision_u_mu1(mu,a);
	d_calc_visual_mu1(1) = d_vision_v_mu1(mu,a);
	
	d1_visual_f_attractor = ra_visual_hessian_mu1.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu1;
	
	d_attr_visual_f3_mu1 = d1_visual_f_attractor(2);
	
	return d_attr_visual_f3_mu1;
}

double d_attractor_visual_f1_mu2(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f1_mu2;
	Vector2d d_calc_visual_mu2;
	
	calc_visual_hessian_mu2(mu, a);

	d_calc_visual_mu2(0) = d_vision_u_mu2(mu,a);
	d_calc_visual_mu2(1) = d_vision_v_mu2(mu,a);
	
	d2_visual_f_attractor = ra_visual_hessian_mu2.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu2;
	
	d_attr_visual_f1_mu2 = d2_visual_f_attractor(0);
	
	return d_attr_visual_f1_mu2;
}

double d_attractor_visual_f2_mu2(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f2_mu2;
	Vector2d d_calc_visual_mu2;

	calc_visual_hessian_mu2(mu, a);

	d_calc_visual_mu2(0) = d_vision_u_mu2(mu,a);
	d_calc_visual_mu2(1) = d_vision_v_mu2(mu,a);
	
	d2_visual_f_attractor = ra_visual_hessian_mu2.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu2;
	
	d_attr_visual_f2_mu2 = d2_visual_f_attractor(1);
	
	return d_attr_visual_f2_mu2;
}

double d_attractor_visual_f3_mu2(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f3_mu2;
	Vector2d d_calc_visual_mu2;

	calc_visual_hessian_mu2(mu, a);

	d_calc_visual_mu2(0) = d_vision_u_mu2(mu,a);
	d_calc_visual_mu2(1) = d_vision_v_mu2(mu,a);
	
	d2_visual_f_attractor = ra_visual_hessian_mu2.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu2;
	
	d_attr_visual_f3_mu2 = d2_visual_f_attractor(2);
	
	return d_attr_visual_f3_mu2;
}

double d_attractor_visual_f1_mu3(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f1_mu3;
	Vector2d d_calc_visual_mu3;
	
	calc_visual_hessian_mu3(mu, a);

	d_calc_visual_mu3(0) = d_vision_u_mu3(mu,a);
	d_calc_visual_mu3(1) = d_vision_v_mu3(mu,a);
	
	d2_visual_f_attractor = ra_visual_hessian_mu3.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu3;
	
	d_attr_visual_f1_mu3 = d2_visual_f_attractor(0);
	
	return d_attr_visual_f1_mu3;
}

double d_attractor_visual_f2_mu3(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f2_mu3;
	Vector2d d_calc_visual_mu3;

	calc_visual_hessian_mu3(mu, a);

	d_calc_visual_mu3(0) = d_vision_u_mu3(mu,a);
	d_calc_visual_mu3(1) = d_vision_v_mu3(mu,a);
	
	d2_visual_f_attractor = ra_visual_hessian_mu3.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu3;
	
	d_attr_visual_f2_mu3 = d2_visual_f_attractor(1);
	
	return d_attr_visual_f2_mu3;
}

double d_attractor_visual_f3_mu3(vector<double> mu, vector<double> a)
{
	double d_attr_visual_f3_mu3;
	Vector2d d_calc_visual_mu3;

	calc_visual_hessian_mu3(mu, a);

	d_calc_visual_mu3(0) = d_vision_u_mu3(mu,a);
	d_calc_visual_mu3(1) = d_vision_v_mu3(mu,a);
	
	d2_visual_f_attractor = ra_visual_hessian_mu3.transpose()*ra_visual_attr 
		- ro1*ra_visual_jacobian.transpose()*d_calc_visual_mu3;
	
	d_attr_visual_f3_mu3 = d2_visual_f_attractor(2);
	
	return d_attr_visual_f3_mu3;
}

//Left arm
void calc_attractor_visual_vector_b(vector<double> mu, vector<double> a)
{
    Vector2d calc_visual_pos;
    Vector2d attr_visual_pos;

/*	//set end-effector position to origin
    ra_repv(0) = 0;
    ra_repv(1) = 0;
    ra_repv(2) = 0;
    ra_repv(3) = 1;*/

    //calculate projected position
    calc_visual_pos(0) = calc_vision_u_b(mu,a);
    calc_visual_pos(1) = calc_vision_v_b(mu,a);

/*	//set end-effector position back to marker
    ra_repv(0) = xh;
    ra_repv(1) = yh;
    ra_repv(2) = zh;
    ra_repv(3) = 1;*/

    //build attractor position
    attr_visual_pos(0) = att_pos(0);
    attr_visual_pos(1) = att_pos(1);

    //calculate visual attractor vector
    la_visual_attr = ro1 * (attr_visual_pos - calc_visual_pos);
    //ra_visual_attr(0) = ro1*0;
    //ra_visual_attr(1) = ro1*100;

    //cout << " ->>>> " << ro1 << " " << calc_visual_pos.transpose() << " " << attr_visual_pos.transpose() << " " <<  ra_visual_attr.transpose() << endl;

}

double calc_attractor_visual_f1b(vector<double> mu, vector<double> a)
{
    double attr_visual_f1;

    calc_attractor_visual_vector_b(mu, a);
    calc_visual_jacobian_b(mu, a);

    //pseudoinverse
    la_visual_f_attr = pseudoinverse(la_visual_jacobian) * la_visual_attr;

    attr_visual_f1 = la_visual_f_attr(0);

    return attr_visual_f1;

}

double calc_attractor_visual_f2b(vector<double> mu, vector<double> a)
{
    double attr_visual_f2;

    //calculated in previous

    attr_visual_f2 = la_visual_f_attr(1);

    return attr_visual_f2;
}

double calc_attractor_visual_f3b(vector<double> mu, vector<double> a)
{
    double attr_visual_f3;

    //calculated in previous

    attr_visual_f3 = la_visual_f_attr(2);

    return attr_visual_f3;
}

//Visual recognition ---------------------------------------------------------------
//Right arm
void calc_attractor_hsv_vector(vector<double> mu, vector<double> a)
{
	Vector2d calc_visual_pos;
	Vector2d attr_visual_pos;

/*	//set end-effector position to origin
	ra_repv(0) = 0;
	ra_repv(1) = 0;
	ra_repv(2) = 0;
	ra_repv(3) = 1;*/

	//calculate projected position
	calc_visual_pos(0) = calc_vision_u(mu,a);
	calc_visual_pos(1) = calc_vision_v(mu,a);
	
/*	//set end-effector position back to marker
	ra_repv(0) = xh;
	ra_repv(1) = yh;
	ra_repv(2) = zh;
	ra_repv(3) = 1;*/

	//build attractor position
	attr_visual_pos(0) = att_pos(0);
	attr_visual_pos(1) = att_pos(1);
	
	//calculate visual attractor vector
	ra_visual_attr = ro1 * (attr_visual_pos - calc_visual_pos);
	
}

double calc_attractor_hsv_f1(vector<double> mu, vector<double> a)
{
	double attr_visual_f1;
	
	calc_attractor_hsv_vector(mu, a);
	calc_visual_jacobian(mu, a);
	
	//transpose -> inverse -> pseudoinverse
	ra_visual_f_attr = pseudoinverse(ra_visual_jacobian) * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.inverse() * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.transpose() * ra_visual_attr;
	
	//cout << ra_visual_attr << endl;
	//cout << ra_visual_f_attr << endl;

	attr_visual_f1 = ra_visual_f_attr(0);
	
	return attr_visual_f1;
	
}

double calc_attractor_hsv_f2(vector<double> mu, vector<double> a)
{
	double attr_visual_f2;

	//calc_attractor_hsv_vector(mu, a);
	//calc_visual_jacobian(mu, a);
	
	//transpose -> inverse -> pseudoinverse
	//ra_visual_f_attr = pseudoinverse(ra_visual_jacobian) * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.inverse() * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.transpose() * ra_visual_attr;

	attr_visual_f2 = ra_visual_f_attr(1);
	
	return attr_visual_f2;
}

double calc_attractor_hsv_f3(vector<double> mu, vector<double> a)
{
	double attr_visual_f3;

	//calc_attractor_hsv_vector(mu, a);
	//calc_visual_jacobian(mu, a);
	
	//transpose -> inverse -> pseudoinverse
	//ra_visual_f_attr = pseudoinverse(ra_visual_jacobian) * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.inverse() * ra_visual_attr;
	//ra_visual_f_attr = ra_visual_jacobian.transpose() * ra_visual_attr;

	attr_visual_f3 = ra_visual_f_attr(2);
	
	return attr_visual_f3;
}

void calc_attractor_hsv_vector_b(vector<double> mu, vector<double> a)
{
    Vector2d calc_visual_pos;
    Vector2d attr_visual_pos;

/*	//set end-effector position to origin
    ra_repv(0) = 0;
    ra_repv(1) = 0;
    ra_repv(2) = 0;
    ra_repv(3) = 1;*/

    //calculate projected position
    calc_visual_pos(0) = calc_vision_u_b(mu,a);
    calc_visual_pos(1) = calc_vision_v_b(mu,a);

/*	//set end-effector position back to marker
    ra_repv(0) = xh;
    ra_repv(1) = yh;
    ra_repv(2) = zh;
    ra_repv(3) = 1;*/

    //build attractor position
    attr_visual_pos(0) = att_pos(0);
    attr_visual_pos(1) = att_pos(1);

    //calculate visual attractor vector
    la_visual_attr = ro1 * (attr_visual_pos - calc_visual_pos);

}

double calc_attractor_hsv_f1b(vector<double> mu, vector<double> a)
{
    double attr_visual_f1;

    calc_attractor_hsv_vector_b(mu, a);
    calc_visual_jacobian_b(mu, a);

    //transpose -> inverse -> pseudoinverse
    la_visual_f_attr = pseudoinverse(la_visual_jacobian) * la_visual_attr;

    attr_visual_f1 = la_visual_f_attr(0);

    return attr_visual_f1;

}

double calc_attractor_hsv_f2b(vector<double> mu, vector<double> a)
{
    double attr_visual_f2;

    attr_visual_f2 = la_visual_f_attr(1);

    return attr_visual_f2;
}

double calc_attractor_hsv_f3b(vector<double> mu, vector<double> a)
{
    double attr_visual_f3;

    attr_visual_f3 = la_visual_f_attr(2);

    return attr_visual_f3;
}


//head -----------------------------------------------------------------------------
void calc_attractor_head_vector(vector<double> mu, vector<double> a)
{
	Vector2d center_visual_pos;
	Vector2d attr_visual_pos;

	//calculate projected position
	center_visual_pos(0) = cx;
	center_visual_pos(1) = cy;

	//build attractor position
	attr_visual_pos(0) = att_pos(0);
	attr_visual_pos(1) = att_pos(1);
	
	//calculate visual attractor vector
	//inverse order: center is the fixed position, attractor changes with motion
	head_visual_attr = ro1_head * (center_visual_pos - attr_visual_pos);

	if (publish)
	{
		Bottle& output_center_x = plot_center_x.prepare();
		output_center_x.clear();
		output_center_x.addDouble(cx);
		plot_center_x.write();
		Bottle& output_center_y = plot_center_y.prepare();
		output_center_y.clear();
		output_center_y.addDouble(cy);
		plot_center_y.write();
	}

}

double calc_attractor_head_f1(vector<double> mu, vector<double> a)
{
	double attr_head_f1;
	
	calc_attractor_head_vector(mu, a);
	calc_visual_jacobian_head(mu, a);
	
	//inverse
	head_visual_f_attr = ra_visual_jacobian_head.inverse() * head_visual_attr;
	
	attr_head_f1 = head_visual_f_attr(0);
	
	return attr_head_f1;
	
}

double calc_attractor_head_f2(vector<double> mu, vector<double> a)
{
	double attr_head_f2;
	
	calc_attractor_head_vector(mu, a);
	calc_visual_jacobian_head(mu, a);
	
	//inverse
	head_visual_f_attr = ra_visual_jacobian_head.inverse() * head_visual_attr;
	
	attr_head_f2 = head_visual_f_attr(1);
	
	return attr_head_f2;
	
}
