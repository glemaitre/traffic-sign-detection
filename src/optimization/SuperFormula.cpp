/*

By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2015,
      Guillaume Lemaitre (g.lemaitre58@gmail.com),
      Johan Massich (mailsik@gmail.com),
      Gerard Bahi (zomeck@gmail.com),
      Yohan Fougerolle (Yohan.Fougerolle@u-bourgogne.fr).
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include "math_utils.h"
#include "SuperFormula.h"
#include "random-standalone.h"
#include "timer.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <algorithm>

#include <iomanip>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <limits>
//#include <GL/glut.h>


using namespace Eigen;

extern Random R1;

//---------------------------------------------------------------------
//
// Rational 2D Gielis Curves
//
//---------------------------------------------------------------------
void RationalSuperShape2D :: Init( double a, double b, double n1,double n2,double n3,double p, double q , double thtoffset, double phioffset, double xoffset, double yoffset, double zoffset){
    Parameters.clear();
    Parameters.push_back(a);
    Parameters.push_back(b);
    Parameters.push_back(n1);
    Parameters.push_back(n2);
    Parameters.push_back(n3);
    Parameters.push_back(p);
    Parameters.push_back(q);
    Parameters.push_back(thtoffset);
    Parameters.push_back(phioffset);
    Parameters.push_back(xoffset);
    Parameters.push_back(yoffset);
    Parameters.push_back(zoffset);
}
void RationalSuperShape2D :: Init( double a, double b, double n1,double n2,double n3)
{
    Init(
                a, b, //scale
                n1, n2, n3, //shape
                Get_p(), Get_q(), //sym
                0,0, //rota
                0,0,0); //trans
}
RationalSuperShape2D :: RationalSuperShape2D(){
    Init(
                1, 1, //scale
                2, 2, 2, //shape
                4, 1, //sym
                0,0, //rota
                0,0,0); //trans
}
RationalSuperShape2D :: RationalSuperShape2D(double a, double b, double n1,double n2,double n3,double p, double q, double thtoffset, double phioffset, double xoffset, double yoffset, double zoffset){
    Init(a, b, n1, n2, n3, p, q, thtoffset, phioffset,xoffset, yoffset, zoffset);
}
RationalSuperShape2D :: ~RationalSuperShape2D() {
    Parameters.clear();
    PointList.clear();
    FaceList.clear();
    NormalList.clear();
}
double RationalSuperShape2D :: radius ( const double angle ){
    double tmp_angle = Get_p() * angle * 0.25 / Get_q() ;
    double tmp1( cos(tmp_angle) );
    double tmp2( sin(tmp_angle) );
    //if( tmp1 != 0)
    tmp1 = pow(fabs(tmp1),Get_n2()) / Get_a();
    //if( tmp2 != 0)
    tmp2 = pow(fabs(tmp2),Get_n3()) / Get_b();
    if( tmp1 + tmp2 !=0 ) return( pow( (tmp1 + tmp2), -1.0/Get_n1() ) );
    else	{std::cout<<"ERROR RADIUS NULL"<<std::endl;return 0;}
}
// void RationalSuperShape2D :: Aff(){
// for (unsigned int i=0; i<Parameters.size(); i++)
// {
// std::cout << setiosflags(ios::fixed) << setprecision(6)<<Parameters[i] << " ";
// }
// std::cout << std::endl;
// }
//Potential fields
double RationalSuperShape2D :: ImplicitFunction1( const Vector2d P, std::vector<double> &Dffinal) {
    Dffinal.clear();
    // nothing computable, return zero values, zero partial derivatives
    // the point will have no effect on the ongoing computations
    if ( P[0] == 0 && P[1] == 0)
    {
        // Df/Dx, Df/Dy, Df/Dr set to zero...
        for (int i=0; i<3; i++) Dffinal.push_back(0);
        return 0;
    }
    std::vector<double> f, Ddum;
    double x(P[0]), y(P[1]), PSL(P.squaredNorm()), PL(sqrt(PSL)), dthtdx (-y/PSL), dthtdy (x/PSL), R,drdth;
    std::vector< std::vector<double> > Df;
    //assert angular values between [0, 2q*Pi]
    double tht (atan2(y,x)), thtbase(tht);
    if (tht<0) thtbase += 2.*M_PI;
    //compute all intersections and associated partial derivatives
    for (int i=0; i<Get_q(); i++)
    {
        tht = thtbase + i*2.*M_PI;
        R = radius(tht);
        f.push_back(R - PL); //store function
        // store partial derivatives
        std::vector<double> rowi;
        drdth = DrDtheta(tht);
        rowi.push_back( drdth*dthtdx - cos(tht)); //df/dx
        rowi.push_back( drdth*dthtdy - sin(tht)); //df/dy
        rowi.push_back( 1. ); //df/dr
        Df.push_back(rowi);
    }
    //bubble sort, not really efficient but acceptable for such small arrays
    for(int i=0; i<Get_q()-1; i++)
        for (int j=i+1; j<Get_q(); j++)
            if (f[i]<f[j])
            {
                //std::swap values of f[i] and f[j]
                std::swap(f[i],f[j]);
                //std::swap rows Df[i] and Df[j]
                Df[i].swap(Df[j]);
            }
    //Compute resulting Rfunction
    std::vector<double> Df1; //std::vectorfor df/dxi
    //iterative evaluation of:
    // -the resulting R-functions
    // -the associated partial derivatives
    double f1,fdum;
    f1 = f[0]; // first value of f
    Df1 = Df[0]; // first associated row with partial derivatives
    //combine functions as (...((F1 v F2) v F3 ) v F4) v ...)
    for(int i=1; i<Get_q(); i++) // for all intersections
    {
        //compute R-function, sets all partial derivatives
        //fdum and Ddum temporary results of the union from F1 to Fi
        RpUnion(f1, f[i], Df1, Df[i], fdum, Ddum);
        //update results in f1 and Df1, and iterate
        f1 = fdum;
        Df1 = Ddum;
    }
    //final partial derivatives df/dxi after R-functions
    Dffinal = Df1;
    //clear arrays
    f.clear(); Df.clear(); Ddum.clear(); Df1.clear();
    //return results
    return f1;
}
double RationalSuperShape2D :: ImplicitFunction2( const Vector2d P, std::vector<double> &Dffinal){
    Dffinal.clear();
    // nothing computable, return zero values, zero partial derivatives
    // the point will have no effect on the ongoing computations
    if ( P[0] == 0 && P[1] == 0)
    {
        // Df/Dx, Df/Dy, Df/Dr set to zero...
        for (int i=0; i<3; i++) Dffinal.push_back(0);
        return 0;
    }
    std::vector<double> f, Ddum;
    double x(P[0]), y(P[1]), PSL(P.squaredNorm()), PL(sqrt(PSL)), dthtdx (-y/PSL), dthtdy (x/PSL), R,drdth;
    std::vector<std::vector<double> > Df;
    //assert angular values between [0, 2q*Pi]
    double tht (atan2(y,x)), thtbase(tht);
    if (tht<0) thtbase += 2.*M_PI;
    //compute all intersections and associated gradient values
    for (int i=0; i<Get_q(); i++)
    {
        tht = thtbase + i*2.*M_PI;
        R = radius(tht);
        drdth = DrDtheta(tht);
        f.push_back(1. - PL/R); //store function
        // store partial derivatives
        std::vector<double> rowi;
        drdth = DrDtheta(tht);
        rowi.push_back( - ( x*R/PL - drdth*dthtdx*PL )/(R*R) ); //df/dx
        rowi.push_back( - ( y*R/PL - drdth*dthtdy*PL )/(R*R) ); //df/dy
        rowi.push_back( PL/(R*R) ); //df/dr
        Df.push_back(rowi);
    }
    //bubble sort, not really efficient but acceptable for such small arrays
    for(int i=0; i<Get_q()-1; i++)
        for (int j=i+1; j<Get_q(); j++)
            if (f[i]<f[j])
            {
                //std::swap values of f[i] and f[j]
                std::swap(f[i],f[j]);
                //std::swap rows Df[i] and Df[j]
                Df[i].swap(Df[j]);
            }
    //Compute resulting Rfunction
    std::vector<double> Df1; //std::vectorfor df/dxi
    //iterative evaluation of:
    // -the resulting R-functions
    // -the associated partial derivatives
    double f1,fdum;
    f1 = f[0]; // first value of f
    Df1 = Df[0]; // first associated row with partial derivatives
    //combine functions as (...((F1 v F2) v F3 ) v F4) v ...)
    for(int i=1; i<Get_q(); i++) // for all intersections
    {
        //compute R-function, sets all partial derivatives
        //fdum and Ddum temporary results of the union from F1 to Fi
        RpUnion(f1, f[i], Df1, Df[i], fdum, Ddum);
        //update results in f1 and Df1, and iterate
        f1 = fdum;
        Df1 = Ddum;
    }
    //final partial derivatives df/dxi after R-functions
    Dffinal = Df1;
    //clear arrays
    f.clear(); Df.clear(); Ddum.clear(); Df1.clear();
    //return results
    return f1;
}
double RationalSuperShape2D :: ImplicitFunction3( const Vector2d P, std::vector<double> &Dffinal){
    Dffinal.clear();
    // nothing computable, return zero values, zero partial derivatives
    // the point will have no effect on the ongoing computations
    if ( P[0] == 0 && P[1] == 0)
    {
        // Df/Dx, Df/Dy, Df/Dr set to zero...
        for (int i=0; i<3; i++) Dffinal.push_back(0);
        return 0;
    }
    std::vector<double> f, Ddum;
    double x(P[0]), y(P[1]), PSL(P.squaredNorm()), dthtdx (-y/PSL), dthtdy (x/PSL), R,drdth;
    std::vector< std::vector<double> > Df;
    //assert angular values between [0, 2q*Pi]
    double tht (atan2(y,x)), thtbase(tht);
    if (tht<0) thtbase += 2.*M_PI;
    //compute all intersections and associated gradient values
    for (int i=0; i<Get_q(); i++)
    {
        tht = thtbase + i*2.*M_PI;
        R = radius(tht);
        drdth = DrDtheta(tht);
        f.push_back( log( R*R / PSL)); //store function
        // store partial derivatives
        drdth = DrDtheta(tht);
        std::vector<double> rowi;
        rowi.push_back( -2.*(x*R - PSL * drdth*dthtdx)/(R*PSL) ); //df/dx
        rowi.push_back( -2.*(y*R - PSL * drdth*dthtdy)/(R*PSL) ); //df/dy
        rowi.push_back( 2./R ); //df/dr
        Df.push_back(rowi);
    }
    //bubble sort, not really efficient but acceptable for such small arrays
    for(int i=0; i<Get_q()-1; i++)
        for (int j=i+1; j<Get_q(); j++)
            if (f[i]<f[j])
            {
                //std::swap values of f[i] and f[j]
                std::swap(f[i],f[j]);
                //std::swap rows Df[i] and Df[j]
                Df[i].swap(Df[j]);
            }
    //Compute resulting Rfunction
    std::vector<double> Df1; //std::vectorfor df/dxi
    //iterative evaluation of:
    // -the resulting R-functions
    // -the associated partial derivatives
    double f1,fdum;
    f1 = f[0]; // first value of f
    Df1 = Df[0]; // first associated row with partial derivatives
    //combine functions as (...((F1 v F2) v F3 ) v F4) v ...)
    for(int i=1; i<Get_q(); i++) // for all intersections
    {
        //compute R-function, sets all partial derivatives
        //fdum and Ddum temporary results of the union from F1 to Fi
        RpUnion(f1, f[i], Df1, Df[i], fdum, Ddum);
        //update results in f1 and Df1, and iterate
        f1 = fdum;
        Df1 = Ddum;
    }
    //final partial derivatives df/dxi after R-functions
    Dffinal = Df1;
    //clear arrays
    f.clear(); Df.clear(); Ddum.clear(); Df1.clear();
    //return results
    return f1;
}
double RationalSuperShape2D :: DrDtheta(double tht)
{
    // analytic
    /*
double n1(Get_n1()), n2(Get_n2()), n3(Get_n3()), m(Get_p()),q(Get_q()),a(Get_a()),b(Get_b());
double c ( cos( m*0.25*tht / q)), C( fabs(c)), s( sin( m*0.25*tht / q)), S( fabs(s));
if(C<EPSILON || S<EPSILON) return 0;
double DC (-0.25*m*s/q), DS (0.25*m*c/q);
return
(-1./n1) *
pow( pow(C,n2) / a + pow(S,n3)/b , -1./n1 - 1.)*
(
-0.25*m*pow(C,n2)*(n2*tan(m*tht*0.25/q)) / (a*q)
+
0.25*m*pow(S,n3)*(n3/tan(m*tht*0.25/q)) / (b*q)
);
*/
    double r0,r1,r2,r3;
    double delta(1e-3);
    r0 = radius(tht - 2*delta);
    r1 = radius(tht - delta);
    r2 = radius(tht + delta);
    r3 = radius(tht + 2*delta);
    Vector4d V(
                r0,
                -8*r1,
                8*r2,
                -r3
                );
    return V.sum() / (12.*delta);
}
double RationalSuperShape2D :: DrDa(const double tht)
{
    //analytic version
    /*
double C = fabs( cos( Get_p() * tht * 0.25 / Get_q())) ;
double S = fabs( sin( Get_p() * tht * 0.25 / Get_q())) ;
double aCn2 = pow(C, Get_n2())/Get_a();
double bSn3 = pow(S, Get_n3())/Get_b();
return aCn2 * pow( aCn2 + bSn3, -1./Get_n1() - 1.) / (Get_n1()*Get_a());
*/
    //discrete approximation
    double olda(Get_a());
    double r0,r1,r2,r3;
    double delta(1e-3);
    Set_a(olda - 2*delta); r0 = radius(tht);
    Set_a(olda - delta); r1 = radius(tht);
    Set_a(olda + delta); r2 = radius(tht);
    Set_a(olda + 2*delta); r3 = radius(tht);
    Set_a(olda);
    Vector4d V(
                r0,
                -8*r1,
                8*r2,
                -r3
                );
    return V.sum() / (12.*delta);
}
double RationalSuperShape2D :: DrDb(const double tht)
{
    //analytic
    /*
double C = fabs( cos( Get_p() * tht * 0.25 / Get_q())) ;
double S = fabs( sin( Get_p() * tht * 0.25 / Get_q())) ;
double aCn2 = pow(C, Get_n2())/Get_a();
double bSn3 = pow(S, Get_n3())/Get_b();
return bSn3 * pow( aCn2 + bSn3, -1./Get_n1() - 1.) / (Get_n1()*Get_b());
*/
    //discrete approximation
    double oldb(Get_b());
    double r0,r1,r2,r3;
    double delta(1e-3);
    Set_b(oldb - 2*delta); r0 = radius(tht);
    Set_b(oldb - delta); r1 = radius(tht);
    Set_b(oldb + delta); r2 = radius(tht);
    Set_b(oldb + 2*delta); r3 = radius(tht);
    Set_b(oldb);
    Vector4d V(
                r0,
                -8*r1,
                8*r2,
                -r3
                );
    return V.sum() / (12.*delta);
}
double RationalSuperShape2D :: DrDn1(const double tht)
{
    //analytic
    /*
double R(radius(tht));
double rn1 = pow(R, -Get_n1());
if (R>EPSILON){
//return log ( pow ( R, -1./Get_n1()))*R;
return log(rn1) * R /(Get_n1()*Get_n1());
}
//std::cout <<"rhaaaa"<<std::endl;
return 0;
*/
    //discrete approximation
    double oldn1(Get_n1());
    double r0,r1,r2,r3;
    double delta(1e-3);
    Set_n1(oldn1 - 2*delta); r0 = radius(tht);
    Set_n1(oldn1 - delta); r1 = radius(tht);
    Set_n1(oldn1 + delta); r2 = radius(tht);
    Set_n1(oldn1 + 2*delta); r3 = radius(tht);
    Set_n1(oldn1);
    Vector4d V(
                r0,
                -8*r1,
                8*r2,
                -r3
                );
    return V.sum() / (12.*delta);
}
double RationalSuperShape2D :: DrDn2(const double tht)
{
    //analytic
    /*
double C = fabs( cos( Get_p() * tht * 0.25 / Get_q())) ;
double S = fabs( sin( Get_p() * tht * 0.25 / Get_q())) ;
double aCn2 = pow(C, Get_n2())/Get_a();
double bSn3 = pow(S, Get_n3())/Get_b();
if(C <EPSILON) return 0;
return
- pow( aCn2 + bSn3, -1./Get_n1() - 1.)
*
log(C)*aCn2 / Get_n1();
*/
    //discrete approximation
    double oldn2(Get_n2());
    double r0,r1,r2,r3;
    double delta(1e-3);
    Set_n2(oldn2 - 2*delta); r0 = radius(tht);
    Set_n2(oldn2 - delta); r1 = radius(tht);
    Set_n2(oldn2 + delta); r2 = radius(tht);
    Set_n2(oldn2 + 2*delta); r3 = radius(tht);
    Set_n2(oldn2);
    Vector4d V(
                r0,
                -8*r1,
                8*r2,
                -r3
                );
    return V.sum() / (12.*delta);
}
double RationalSuperShape2D :: DrDn3(const double tht)
{
    //analytic
    /*
double C = fabs( cos( Get_p() * tht * 0.25 / Get_q())) ;
double S = fabs( sin( Get_p() * tht * 0.25 / Get_q())) ;
double aCn2 = pow(C, Get_n2())/Get_a();
double bSn3 = pow(S, Get_n3())/Get_b();
if(S < EPSILON) return 0;
return
- pow( aCn2 + bSn3, -1./Get_n1() - 1.)
*
log(S)*bSn3 / Get_n1();
*/
    //discrete approx
    double oldn3(Get_n3());
    double r0,r1,r2,r3;
    double delta(1e-3);
    Set_n3(oldn3 - 2*delta); r0 = radius(tht);
    Set_n3(oldn3 - delta); r1 = radius(tht);
    Set_n3(oldn3 + delta); r2 = radius(tht);
    Set_n3(oldn3 + 2*delta); r3 = radius(tht);
    Set_n3(oldn3);
    Vector4d V(
                r0,
                -8*r1,
                8*r2,
                -r3
                );
    return V.sum() / (12.*delta);
}
void RpUnion(double f1, double f2, std::vector<double> Df1, std::vector<double> Df2, double &f, std::vector<double> &Df)
{
    assert(Df1.size() == Df2.size());
    Df.clear();
    f = f1+f2+sqrt(f1*f1+f2*f2);
    if(f1 != 0 || f2 != 0) // function differentiable
        for(unsigned int i=0; i<Df1.size(); i++)
            Df.push_back( Df1[i] + Df2[i] + (f1*Df1[i]+f2*Df2[i])/sqrt(f1*f1+f2*f2) );
    else //function not differentiable, set everything to zero
        for(unsigned int i=0; i<Df1.size(); i++)
            Df.push_back( 0 );
}
void RpIntersection(double f1, double f2, std::vector<double> Df1, std::vector<double> Df2, double &f, std::vector<double> &Df)
{
    assert(Df1.size() == Df2.size());
    Df.clear();
    f = f1+f2-sqrt(f1*f1+f2*f2);
    if(f1 != 0 || f2 != 0) // function differentiable
        for(unsigned int i=0; i<Df.size(); i++)
            Df.push_back ( Df1[i] + Df2[i] - (f1*Df1[i]+f2*Df2[i])/sqrt(f1*f1+f2*f2) ) ;
    else //function not differentiable, set everything to zero
        for(unsigned int i=0; i<Df1.size(); i++)
            Df.push_back( 0 );
}
void RationalSuperShape2D :: Optimize5D(
        std::string outfilename,
        std::vector< Vector2d, aligned_allocator< Vector2d> > Data,
        double &err ,
        int functionused
        )
{
    double NewChiSquare, ChiSquare(1e15), OldChiSquare(1e15);
    std::ofstream logfile;
    logfile.open(outfilename.c_str());
    bool STOP(false);
    double oldparams[] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //dj[16];
    double LAMBDA_INCR(10), lambda(pow(LAMBDA_INCR, -6));
    MatrixXd alpha, alpha2;
    alpha = MatrixXd::Zero(5,5);
    alpha2 = MatrixXd::Zero(5,5);
    VectorXd beta, beta2, dj, oldbeta, sigma;
    beta = VectorXd::Zero(5);
    beta2 = VectorXd::Zero(5);
    dj = VectorXd::Zero(5);
    oldbeta = VectorXd::Zero(5);
    // trial = VectorXd::Zero(5);
    // sigma = VectorXd::Zero(5);
    logfile << *this;

    std::vector<double> Df;
    int itnum = 0;
    for(itnum=0; itnum<1000 && STOP==false; itnum++) {
        //store oldparams
        for(size_t i=0; i<Parameters.size(); i++) oldparams[i] = Parameters[i];
        alpha.setZero();
        beta.setZero();
        alpha2.setZero();
        beta2.setZero();
        //std::cout <<"TRIAL:"<<*this<<std::endl;
        bool outofbounds(false);
        //std::cout <<"Norm on in Opt2 : "<<Normalization<<std::endl;
        ChiSquare = XiSquare5D(Data,
                               alpha,
                               beta,
                               functionused, //implicit function1
                               true); //update vectors
        //
        // add Lambda to diagonla elements and solve the matrix
        //
        //Linearization of Hessian, cf Numerical Recepies
        for(int k=0; k<5; k++)
        {
            alpha(k,k) *= 1. + lambda; //multiplicative factor to make diagonal dominant
            alpha(k,k) += lambda; //additive factor to avoid rank deficient matrix
        }
        //solve system
        alpha.ldlt().solveInPlace(beta);
        //coefficients a and b in [0.01, 100] to preserve numerical accuracy
        outofbounds = Parameters[0] + beta[0] < 0.01 || Parameters[0] + beta[0] > 1000 ||
                Parameters[1] + beta[1] < 0.01 || Parameters[1] + beta[1] > 1000 ||
                Parameters[2] + beta[2] < 0.1 || Parameters[2] + beta[2]> 1000 ||
                Parameters[3] + beta[3] < 0.1 || Parameters[3] + beta[3]> 1000 ||
                Parameters[4] + beta[4] < 0.1 || Parameters[4] + beta[4]> 1000;
        if( !outofbounds )
        {
            Set_a(Parameters[0] + beta[0]);
            Set_b(Parameters[1] + beta[1]);
            // coefficients n1 in [1., 1000]
            // setting n1<1. leads to strong numerical instabilities
            Set_n1( Parameters[2] + beta[2] );
            // coefficients n2,n3 in [0.001, 1000]
            Set_n2( Parameters[3] + beta[3]);
            Set_n3( Parameters[4] + beta[4]);
        }
        //
        // Evaluate chisquare with new values
        //
        OldChiSquare=ChiSquare;
        NewChiSquare=XiSquare5D(Data,
                                alpha2,
                                beta2,
                                functionused, //implicit function
                                false); // update alpha and beta or not
        //
        // check if better result
        //
        if( NewChiSquare>0.999*OldChiSquare ) // new result sucks-->restore old params and try with lambda 10 times bigger
        {
            lambda *=LAMBDA_INCR;
            for(size_t i=0; i<Parameters.size(); i++) Parameters[i]=oldparams[i];
        }
        else //successful iteration
        {
            // huge improvement, something may have been wrong
            // this may arise during the first iterations
            // n1 may literally explode, or tend to 0...
            // in such case, the next iteration is successful but leads to a local minimum
            // ==> it is better to verify the result with a smaller step
            // if indeed it was a correct iteration, then it will pass the next time
            if (NewChiSquare <= 0.01*OldChiSquare) //99% improvement, impossible
            {
                lambda *=LAMBDA_INCR; // reduce the step within the search direction
                for(size_t i=0; i<Parameters.size(); i++) Parameters[i]=oldparams[i]; // restore old parameters
            }
            else
            {
                //correct and realistic improvement
                logfile << *this;
                lambda /=LAMBDA_INCR;
            }
        }
        STOP = lambda > 1e15 || NewChiSquare < 1e-5; // very small displacement ==> local convergence
    } //end for(...
    err = ChiSquare;
    logfile << *this;
    logfile.close();
}
double RationalSuperShape2D :: XiSquare5D(
        const std::vector< Vector2d, aligned_allocator< Vector2d> > Data,
        MatrixXd &alpha,
        VectorXd &beta,
        int functionused,
        bool update) {
    //five dimensional optimization: a, b, n1, n2, n3 are optimized
    VectorXd dj; dj = VectorXd::Zero(5);
    Matrix3d Tr,Rot;
    //functions pointer
    double (RationalSuperShape2D ::*pt2ConstMember)(const Vector2d P, std::vector<double> &Dffinal) = NULL;
    switch (functionused){
    case 1 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction1; }break;
    case 2 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction2; }break;
    case 3 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction3; }break;
    default : pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction1;
    }
    std::vector<double> Df;
    double x, y, tht, ChiSquare(1e15), f(0),
            x0(Get_xoffset()),y0(Get_yoffset()),tht0(Get_thtoffset());
    //clean memory
    if ( update ) {
        alpha.setZero();
        beta.setZero();
    }
    //evaluate ChiSquare, components for the beta and matrix alpha
    ChiSquare=0;
    //First define inverse translation T-1
    Tr.setZero();
    Rot.setZero();
    Tr << 1 , 0 , -x0 ,
            0 , 1 , -y0 ,
            0 , 0 , 1;
    //Then define inverse rotation, i.e. transposed rotation
    Rot << cos(tht0) , sin(tht0) , 0 ,
            -sin(tht0) , cos(tht0) , 0 ,
            0 , 0 , 1;
    //summation of squared potentials
    for(size_t i=0; i<Data.size(); i++ ){
        double DfDr;
        //global inverse transform is T * R
        Vector2d dum(Data[i]);
        Vector3d dum2(dum[0],dum[1],1);
        //apply inverse transform
        Vector3d dum3 ( Rot * (Tr * dum2));
        Vector2d P( dum3[0], dum3[1]); //2D point in canonical referential
        //get partial derivatives for canonical point
        x = P[0]; y = P[1];
        // avoid division by 0 ==> numerical stability
        if (P.norm()<EPSILON) continue; // avoids division by zero
        tht = atan2(y,x); if( tht<0) tht+=2.*M_PI;
        //theta = Arctan(Y/X)

        //avoid non differentiable cases
        f = (*this.*pt2ConstMember)(P, Df); // call to the implicit function
        //
        //compute elements beta[i][0] and alpha[i][j]
        //
        //==> requires partial derivatives!!
        DfDr = Df[2]; //Df/Dr stored at index 2 in array Df during the call to ImplicitFunction1-2-3
        // F1 = R-PL ==> DfDr = 1. ;
        // F2 = 1-PL/R ==> DfDr = PL/R\B2 ;
        // F3 = log ( R\B2/PSL) ==> DfDr = 2/R
        dj.setZero();
        //df/da = df/dr * dr/da
        dj[0] = DfDr * DrDa(tht) ;
        //df/db = df/dr * dr/db
        dj[1] = DfDr * DrDb(tht) ;
        //df/dn1 = df/dr * dr/dn1
        dj[2] = DfDr * DrDn1(tht);
        //df/dn2 = df/dr * dr/dn2
        dj[3] = DfDr * DrDn2(tht);
        //df/dn3 = df/dr * dr/dn3
        dj[4] = DfDr * DrDn3(tht);
        ChiSquare += f*f;
        if(update){
            //gradient beta
            for(int k=0;k<5; k++)
            {
                beta[k] -= f * dj[k];
            }
            //compute approximation of Hessian matrix
            for(int k=0; k<5; k++)
                for(int j=0; j<5; j++)
                    alpha(k,j) += dj[k]*dj[j];
        }
    }//for all vertices
    return ChiSquare;
}
void RationalSuperShape2D :: Optimize7D(
        std::string outfilename,
        std::vector< Vector2d, aligned_allocator< Vector2d> > Data,
        double &err ,
        int functionused
        )
{
    double NewChiSquare, ChiSquare(1e15), OldChiSquare(1e15);
    std::ofstream logfile;
    logfile.open(outfilename.c_str());
    bool STOP(false);
    double oldparams[] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//, dj[16];
    double LAMBDA_INCR(10), lambda(pow(LAMBDA_INCR, -6));
    MatrixXd alpha, alpha2;
    alpha = MatrixXd::Zero(7,7);
    alpha2 = MatrixXd::Zero(7,7);
    VectorXd beta, beta2, dj, oldbeta, trial, sigma;
    beta = VectorXd::Zero(7);
    beta2 = VectorXd::Zero(7);
    dj = VectorXd::Zero(7);
    oldbeta = VectorXd::Zero(7);
    trial = VectorXd::Zero(7);
    sigma = VectorXd::Zero(7);
    logfile << *this;

    int itnum = 0;
    for(itnum=0; itnum<1000 && STOP==false; itnum++) {
        //store oldparams
        for(size_t i=0; i<Parameters.size(); i++) oldparams[i]=Parameters[i];
        alpha.setZero();
        beta.setZero();
        alpha2.setZero();
        beta2.setZero();
        //std::cout <<"TRIAL:"<<*this<<std::endl;
        bool outofbounds(false);
        //std::cout <<"Norm on in Opt2 : "<<Normalization<<std::endl;
        ChiSquare = XiSquare7D(Data,
                               alpha,
                               beta,
                               functionused, //implicitf cuntion1
                               true); //update vectors
        //
        // add Lambda to diagonla elements and solve the matrix
        //
        //Linearization of Hessian, cf Numerical Recepies

        for(int k=0; k<7; k++)
        {
            alpha(k,k) *= 1. + lambda; //multiplicative factor to make diagonal dominant
            alpha(k,k) += lambda; //additive factor to avoid rank deficient matrix
        }
        //std::cout << alpha << std::endl<<std::endl;
        //std::cout << beta << std::endl<<std::endl;
        //solve system
        alpha.ldlt().solveInPlace(beta);
        //coefficients a and b in [0.01, 100]
        outofbounds = Parameters[0] + beta[0] < 0.01 || Parameters[0] + beta[0] > 1000 ||
                Parameters[1] + beta[1] < 0.01 || Parameters[1] + beta[1] > 1000 ||
                Parameters[2] + beta[2] < 0.1 || Parameters[2] + beta[2]> 1000 ||
                Parameters[3] + beta[3] < 0.1 || Parameters[3] + beta[3]> 1000 ||
                Parameters[4] + beta[4] < 0.1 || Parameters[4] + beta[4]> 1000;
        if( !outofbounds )
        {
            Set_a( Parameters[0] + beta[0]);
            Set_b( Parameters[1] + beta[1]);
            // coefficients n1 in [1., 1000]
            // setting n1<1. leads to strong numerical instabilities
            Set_n1( Parameters[2] + beta[2] );
            // coefficients n2,n3 in [0.001, 1000]
            Set_n2( Parameters[3] + beta[3]);
            Set_n3( Parameters[4] + beta[4]);
            // coefficients x0 and y0
            //truncate translation to avoid huge gaps
            beta[5] = std::min(0.05, std::max(-0.05, beta[5]));
            beta[6] = std::min(0.05, std::max(-0.05, beta[6]));
            Parameters[9] += beta[5];
            Parameters[10] += beta[6];
        }
        //
        // Evaluate chisquare with new values
        //
        OldChiSquare = ChiSquare;
        NewChiSquare = XiSquare7D(Data,
                                  alpha2,
                                  beta2,
                                  functionused, //implicitf cuntion1
                                  false);
        if( NewChiSquare>0.999*OldChiSquare ) // new result sucks-->restore old params and try with lambda 10 times bigger
        {
            lambda *= LAMBDA_INCR;
            for(size_t i=0; i<Parameters.size(); i++) Parameters[i]=oldparams[i];
        }
        else //successful iteration
        {
            // huge improvement, something may have been wrong
            // this may arise during the first iterations
            // n1 may literally explode, or tend to 0...
            // in such case, the next iteration is successful but leads to a local minimum
            // ==> it is better to verify the result with a smaller step
            // if indeed it was a correct iteration, then it will pass the next time
            if (NewChiSquare <= 0.01*OldChiSquare) //99% improvement, impossible
            {
                lambda *= LAMBDA_INCR; // reduce the step within the search direction
                for(size_t i=0; i<Parameters.size(); i++) Parameters[i]=oldparams[i]; // restore old parameters
            }
            else
            {
                //correct and realistic improvement
                logfile << *this;
                lambda /=LAMBDA_INCR;
            }
        }
        STOP = lambda > 1e15 || NewChiSquare < 1e-5;
    } //end for(...
    err = ChiSquare;
    logfile << *this;
    logfile.close();
}
double RationalSuperShape2D :: XiSquare7D(
        const std::vector< Vector2d, aligned_allocator< Vector2d> > Data,
        MatrixXd &alpha,
        VectorXd &beta,
        int functionused,
        bool update) {
    VectorXd dj; dj = VectorXd::Zero(7);
    Matrix3d Tr,Rot, dTrdx0, dTrdy0;
    //functions pointer
    double (RationalSuperShape2D ::*pt2ConstMember)(const Vector2d P, std::vector<double> &Dffinal) = NULL;
    switch (functionused){
    case 1 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction1; }break;
    case 2 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction2; }break;
    case 3 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction3; }break;
    default : pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction1;
    }
    std::vector<double> Df;
    double x, y, tht, dthtdx, dthtdy,dthtdx0, dthtdy0, drdth,
            ChiSquare(1e15),
            f(0),
            x0(Get_xoffset()),y0(Get_yoffset()),tht0(Get_thtoffset()),
            dxdx0,dxdy0, dydx0,dydy0;
    //clean memory
    if(update) {
        alpha.setZero();
        beta.setZero();
    }
    //evaluate ChiSquare, components for the beta and matrix alpha
    ChiSquare=0;
    //First define inverse translation T-1
    Tr.setZero();
    Rot.setZero();
    Tr << 1 , 0 , -x0 ,
            0 , 1 , -y0 ,
            0 , 0 , 1;
    //derivatives of translation matrix are constant and can be computed once for all
    dTrdx0 << 0 , 0 , -1 , 0 , 0 , 0 , 0 , 0 , 1;
    dTrdy0 << 0 , 0 , 0 , 0 , 0 , -1 , 0 , 0 , 1;
    //Then define inverse rotation, i.e. transposed rotation
    Rot << cos(tht0) , sin(tht0) , 0 ,
            -sin(tht0) , cos(tht0) , 0 ,
            0 , 0 , 1;
    for(size_t i=0; i<Data.size(); i++){
        double DfDr;
        //global inverse transform is T * R
        Vector2d dum(Data[i]);
        Vector3d dum2(dum[0],dum[1],1);
        //apply inverse transform
        Vector3d dum3 ( Rot * (Tr * dum2));
        Vector2d P(dum3[0],dum3[1]); //2D point in canonical ref
        //get partial derivatives for canonical point
        Vector3d dPdx0 ( Rot * (dTrdx0 * dum2) );
        Vector3d dPdy0 ( Rot * (dTrdy0 * dum2) );
        //simplify notations
        dxdx0 = dPdx0[0];
        dxdy0 = dPdy0[0];
        dydx0 = dPdx0[1];
        dydy0 = dPdy0[1];
        x = P[0]; y = P[1];
        // avoid division by 0 ==> numerical stability
        if (P.norm()<EPSILON) continue; // avoids division by zero
        tht = atan2(y,x); if( tht < 0 ) tht+=2.*M_PI;

        //theta = Arctan(Y/X)
        dthtdx = -sin(tht) ;//-y / (x*x+y*y);
        dthtdy = cos(tht); //x / (x*x+y*y);
        //partial derivatives of theta regarding x offset, y offset, and angular offset
        dthtdx0 = dthtdx * dxdx0 + dthtdy * dydx0;
        dthtdy0 = dthtdx * dxdy0 + dthtdy * dydy0;
        //avoid non differentiable cases
        drdth = DrDtheta(tht);
        f = (*this.*pt2ConstMember)(P, Df); // call to the implicit function
        //
        //compute elements beta[i][0] and alpha[i][j]
        //
        //==> requires partial derivatives!!
        DfDr = Df[2]; //Df/Dr stored at index 2 in array Df during the call to ImplicitFunction1-2-3
        // F1 = R-PL ==> DfDr = 1. ;
        // F2 = 1-PL/R ==> DfDr = PL/R\B2 ;
        // F3 = log ( R\B2/PSL) ==> DfDr = 2/R
        dj.setZero();
        //df/da = df/dr * dr/da
        dj[0] = DfDr * DrDa(tht) ;
        //df/db = df/dr * dr/db
        dj[1] = DfDr * DrDb(tht) ;
        //df/dn1 = df/dr * dr/dn1
        dj[2] = DfDr * DrDn1(tht);
        //df/dn2 = df/dr * dr/dn2
        dj[3] = DfDr * DrDn2(tht);
        //df/dn3 = df/dr * dr/dn3
        dj[4] = DfDr * DrDn3(tht);
        //df/dx0 = df/dr * dr/dtht *dtht/dx0
        dj[5]= DfDr * drdth*dthtdx0;
        //df/dy0 = df/dr * dr/dtht *dtht/dy0
        dj[6]= DfDr * drdth*dthtdy0;
        //squared summation
        ChiSquare += f*f;
        if(update){
            for(int k=0; k<7; k++)
            {
                beta[k] -= f*dj[k];
            }
            //compute approximation of Hessian matrix
            for(int k=0; k<7; k++)
                for(int j=0; j<7; j++)
                    alpha(k,j) += dj[k]*dj[j];
        }
    }//for all vertices
    return ChiSquare;
}
void RationalSuperShape2D :: Optimize8D(
        std::vector< Vector2d, aligned_allocator< Vector2d> > Data,
        double &err ,
        int functionused
        )
{
    double NewChiSquare, ChiSquare(1e15), OldChiSquare(1e15);
    // ofstream logfile;
    // logfile.open(outfilename.c_str());
    bool STOP(false);
    double oldparams[] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//, dj[16];
    double LAMBDA_INCR(10);
    double lambda(pow(LAMBDA_INCR, -6));
    MatrixXd alpha, alpha2;
    alpha = MatrixXd::Zero(8,8);
    alpha2 = MatrixXd::Zero(8,8);
    VectorXd beta, beta2, dj, oldbeta, trial, sigma;
    beta = VectorXd::Zero(8);
    beta2 = VectorXd::Zero(8);
    dj = VectorXd::Zero(8);
    oldbeta = VectorXd::Zero(8);
    trial = VectorXd::Zero(8);
    sigma = VectorXd::Zero(8);

    // logfile << *this;
    int itnum = 0;
    for(itnum=0; itnum<1000 && STOP==false; itnum++) {
        //store oldparams
        for(size_t i=0; i<Parameters.size(); i++) oldparams[i]=Parameters[i];
        alpha.setZero();
        beta.setZero();
        alpha2.setZero();
        beta2.setZero();
        //std::cout <<"TRIAL:"<<*this<<std::endl;
        bool outofbounds(false);
        //std::cout <<"Norm on in Opt2 : "<<Normalization<<std::endl;
        ChiSquare = XiSquare8D(Data,
                               alpha,
                               beta,
                               functionused, //implicitf cuntion1
                               true); //update vectors
        //
        // add Lambda to diagonla elements and solve the matrix
        //
        //Linearization of Hessian, cf Numerical Recepies
        for(int k=0; k<8; k++)
        {
            alpha(k,k) *= 1. + lambda; //multiplicative factor to make diagonal dominant
            alpha(k,k) += lambda; //additive factor to avoid rank deficient matrix
        }
        //solve system
        alpha.ldlt().solveInPlace(beta);
        //coefficients a and b in [0.01, 100]
        outofbounds = Parameters[0] + beta[0] < 0.01 || Parameters[0] + beta[0] > 1000 ||
                Parameters[1] + beta[1] < 0.01 || Parameters[1] + beta[1] > 1000 ||
                Parameters[2] + beta[2] < 0.1 || Parameters[2] + beta[2]> 1000 ||
                Parameters[3] + beta[3] < 0.1 || Parameters[3] + beta[3]> 1000 ||
                Parameters[4] + beta[4] < 0.1 || Parameters[4] + beta[4]> 1000;
        if( !outofbounds ) {
            Set_a( Parameters[0] + beta[0]);
            Set_b( Parameters[1] + beta[1]);
            // coefficients n1 in [1., 1000]
            // setting n1<1. leads to strong numerical instabilities
            Set_n1( Parameters[2] + beta[2] );
            // coefficients n2,n3 in [0.001, 1000]
            Set_n2( Parameters[3] + beta[3]);
            Set_n3( Parameters[4] + beta[4]);
            // coefficients x0 and y0
            //truncate translation to avoid huge gaps
            beta[5] = std::min(0.05, std::max(-0.05, beta[5]));
            beta[6] = std::min(0.05, std::max(-0.05, beta[6]));
            Parameters[9] += beta[5];
            Parameters[10] += beta[6];
            //same for rotational offset tht0
            beta[7] = std::min(M_PI/50., std::max(-M_PI/50., beta[7]));
            Parameters[7] += beta[7];
        }
        //
        // Evaluate chisquare with new values
        //
        OldChiSquare = ChiSquare;
        NewChiSquare=XiSquare8D(Data,
                                alpha2,
                                beta2,
                                functionused, //implicitf cuntion1
                                false);
        //
        // check if better result
        //
        if( NewChiSquare>0.999*OldChiSquare ) // new result sucks-->restore old params and try with lambda 10 times bigger
        {
            lambda *=LAMBDA_INCR;
            for(size_t i=0; i<Parameters.size(); i++) Parameters[i]=oldparams[i];
        }
        else //successful iteration
        {
            // huge improvement, something may have been wrong
            // this may arise during the first iterations
            // n1 may literally explode, or tend to 0...
            // in such case, the next iteration is successful but leads to a local minimum
            // ==> it is better to verify the result with a smaller step
            // if indeed it was a correct iteration, then it will pass the next time
            if (NewChiSquare <= 0.01*OldChiSquare) //99% improvement, impossible
            {
                lambda *=LAMBDA_INCR; // reduce the step within the search direction
                for(size_t i=0; i<Parameters.size(); i++) Parameters[i]=oldparams[i]; // restore old parameters
            }
            else
            {
                //correct and realistic improvement
                // logfile << *this;
                lambda /=LAMBDA_INCR;
            }
        }
        STOP = lambda > 1e15 || NewChiSquare < 1e-5; // very small displacement ==> local convergence
    } //end for(...
    err = ChiSquare;
    // logfile << *this;
    // logfile.close();
}
double RationalSuperShape2D :: XiSquare8D(
        const std::vector< Vector2d, aligned_allocator< Vector2d> > Data,
        MatrixXd &alpha,
        VectorXd &beta,
        int functionused,
        bool update) {
    VectorXd dj; dj = VectorXd::Zero(8);
    Matrix3d Tr,Rot, dTrdx0, dTrdy0, dRotdtht0;
    //functions pointer
    double (RationalSuperShape2D ::*pt2ConstMember)(const Vector2d P, std::vector<double> &Dffinal) = NULL;
    switch (functionused){
    case 1 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction1; }break;
    case 2 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction2; }break;
    case 3 :{ pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction3; }break;
    default : pt2ConstMember = &RationalSuperShape2D :: ImplicitFunction1;
    }
    std::vector<double> Df;
    double  x, y, tht, dthtdx, dthtdy,dthtdx0, dthtdy0, drdth,
            ChiSquare(1e15),
            f(0),
            x0(Get_xoffset()),y0(Get_yoffset()),tht0(Get_thtoffset()),
            dxdx0,dxdy0,dxdtht0, dydx0,dydy0,dydtht0, dthtdtht0;
    //clean memory
    if(update)
    {
        alpha.setZero();
        beta.setZero();
    }
    //evaluate ChiSquare, components for the beta and matrix alpha
    ChiSquare=0;
    //First define inverse translation T-1
    Tr.setZero();
    Rot.setZero();
    dRotdtht0.setZero();
    Tr << 1 , 0 , -x0 ,
            0 , 1 , -y0 ,
            0 , 0 , 1;
    //derivatives of translation matrix are constant and can be computed once for all
    dTrdx0 << 0 , 0 , -1 , 0 , 0 , 0 , 0 , 0 , 1;
    dTrdy0 << 0 , 0 , 0 , 0 , 0 , -1 , 0 , 0 , 1;
    //Then define inverse rotation, i.e. transposed rotation
    Rot << cos(tht0) , sin(tht0) , 0 ,
            -sin(tht0) , cos(tht0) , 0 ,
            0 , 0 , 1;
    //get partial derivatives
    dRotdtht0 << -sin(tht0) , cos(tht0) , 0 ,
            -cos(tht0) , -sin(tht0) , 0 ,
            0 , 0 , 1;
    for(size_t i=0; i<Data.size(); i++){
        double DfDr;
        //global inverse transform is T * R
        Vector2d dum(Data[i]);
        Vector3d dum2(dum[0],dum[1],1);
        //apply inverse transform
        Vector3d dum3 ( Rot * (Tr * dum2));
        //get partial derivatives for canonical point
        Vector3d dPdx0 ( Rot * (dTrdx0 * dum2) );
        Vector3d dPdy0 ( Rot * (dTrdy0 * dum2) );
        Vector3d dPdtht0 (dRotdtht0 * (Tr *dum2) );
        Vector2d P(dum3[0],dum3[1]);
        //simplify notations
        dxdx0 = dPdx0[0];
        dxdy0 = dPdy0[0];
        dxdtht0 = dPdtht0[0];
        dydx0 = dPdx0[1];
        dydy0 = dPdy0[1];
        dydtht0 = dPdtht0[1];
        x = P[0]; y = P[1];
        // avoid division by 0 ==> numerical stability
        if (P.norm()<EPSILON) continue; // avoids division by zero
        tht = atan2(y,x); if( tht<0) tht+=2.*M_PI;

        //theta = Arctan(Y/X)
        dthtdx = -sin(tht) ;//-y / (x*x+y*y);
        dthtdy = cos(tht); //x / (x*x+y*y);
        //partial derivatives of theta regarding x offset, y offset, and angular offset
        dthtdx0 = dthtdx * dxdx0 + dthtdy * dydx0;
        dthtdy0 = dthtdx * dxdy0 + dthtdy * dydy0;
        dthtdtht0 = dthtdx * dxdtht0+ dthtdy * dydtht0;
        //avoid non differentiable cases
        drdth = DrDtheta(tht);
        f = (*this.*pt2ConstMember)(P, Df); // call to the implicit function
        //
        //compute elements beta[i][0] and alpha[i][j]
        //
        //==> requires partial derivatives!!
        DfDr = Df[2]; //Df/Dr stored at index 2 in array Df during the call to ImplicitFunction1-2-3
        // F1 = R-PL ==> DfDr = 1. ;
        // F2 = 1-PL/R ==> DfDr = PL/R\B2 ;
        // F3 = log ( R\B2/PSL) ==> DfDr = 2/R
        //GetPartialDerivatives(tht, DrDa, DrDb, DrDn1, DrDn2, DrDn3);
        //for(int toto=0; toto<8; toto++) dj[toto]=0;
        dj.setZero();
        //df/da = df/dr * dr/da
        dj[0] = DfDr * DrDa(tht) ;
        //df/db = df/dr * dr/db
        dj[1] = DfDr * DrDb(tht) ;
        //df/dn1 = df/dr * dr/dn1
        dj[2] = DfDr * DrDn1(tht);
        //df/dn2 = df/dr * dr/dn2
        dj[3] = DfDr * DrDn2(tht);
        //df/dn3 = df/dr * dr/dn3
        dj[4] = DfDr * DrDn3(tht);
        //df/dx0 = df/dr * dr/dtht *dtht/dx0
        dj[5]= DfDr * drdth*dthtdx0;
        //df/dy0 = df/dr * dr/dtht *dtht/dy0
        dj[6]= DfDr * drdth*dthtdy0;
        //df/dth0 = dfdr * dr/dtht * dtht/dtht0
        dj[7]= DfDr * drdth*dthtdtht0;
        ChiSquare += f*f;
        if( update ){

            for(int k=0; k<8; k++)
            {
                beta[k] -= f*dj[k];
            }
            //compute approximation of Hessian matrix

            for(int k=0; k<8; k++)
                for(int j=0; j<8; j++)
                    alpha(k,j) += dj[k]*dj[j];
        }
    }//for all vertices
    return ChiSquare;
}
Vector2d RationalSuperShape2D :: ClosestPoint( Vector2d P, int itmax){
    // P is supposed to be expressed in canonical referential
    double tht = atan2(P[1],P[0]); if (tht<0) tht +=2*M_PI;
    double error(1), change(1);
    int it(0);
    while (error >= 0.001 && fabs(change)>1e-6 && it < itmax){
        // we would like to optimize the distance function
        // change = l' / l'' where l is distance function with argument phi
        // find the change of phi
        // but by somehow we feel like to ignore the sign of f''
        // dl_dphi(parameters, point, phi);
        double f_prime = Deriv1(tht, P);
        //parameters, point, phi, delta);
        // d2l_dphi2(parameters, point, phi, delta);
        double f_2prime = Deriv2(tht, P);
        change = f_prime / fabs(f_2prime);
        error = fabs(change);
        change = change / 10; // slowly move on
        tht = tht-change;
        it++;
    }
    // std::cout <<"PPoint found"<<std::endl;
    Vector2d H (Point(tht));
    //H = Rot.transpose()*H + Vector2d(Get_xoffset(), Get_yoffset());
    return H;
}
// void RationalSuperShape2D :: writeFile(std::string fileName){
// ofstream logfile;
// logfile.open(fileName.c_str());
// for ( int i=0; i<Parameters.size(); i++)
// {
// logfile << setiosflags(ios::fixed) << setprecision(6)<<Parameters[i] << " ";
// }
// logfile << std::endl;
// logfile.close();
// }
bool RationalSuperShape2D :: ErrorMetric (std::vector< Vector2d, aligned_allocator< Vector2d> > Data, Vector4d &Mean, Vector4d &Var)
{
    //Bring back data into canonical referential
    double x0(Get_xoffset()), y0(Get_yoffset()), tht0(Get_thtoffset());
    //allocate matrices (homogenous coordinate, so 3x3 matrix)
    Matrix3d Tr,Rot;
    //First define inverse translation T-1
    Tr.setZero();
    Tr << 1 , 0 , -x0 ,
            0 , 1 , -y0 ,
            0 , 0 , 1;
    //Now define inverse rotation, i.e. transposed rotation
    Rot.setZero();
    Rot << cos(tht0) , sin(tht0) , 0 ,
            -sin(tht0) , cos(tht0) , 0 ,
            0 , 0 , 1;
    //now process all the data and store the point in a local array
    std::vector< Vector2d, aligned_allocator< Vector2d> > CanonicalData;
    // use 1 array of std::vector4d to reduce computational load
    std::vector< Vector4d, aligned_allocator< Vector4d> > dumarray;
    for( size_t i=0; i<Data.size(); i++ ){
        //global inverse transform is T * R
        Vector3d dum(Data[i][0], Data[i][1], 1.);
        //apply inverse transform
        Vector3d dum2 ( Rot * (Tr * dum));
        CanonicalData.push_back( Vector2d (dum2[0], dum2[1]) ); //2D point in canonical referential
    }
    /*
glColor3f(0,0,0);
glPointSize(2);
glBegin(GL_POINTS);
for( int i=0; i<Data.size(); i++ ){
//global inverse transform is T * R
Vector2d dum(CanonicalData[i]);
glVertex3f(dum[0],dum[1],0);
}
glEnd();
*/
    //Init Mean and Var
    Mean = Vector4d(0,0,0,0);
    Var = Mean;
    std::vector<double> Dffinal;//dummy local variable to store partial derivatives, unused in this function
    //compute Mean
    /*
glColor3f(1,0,0);
glLineWidth(2);
glBegin(GL_LINES);
*/
    for (size_t i=0; i< CanonicalData.size(); i++)
    {
        //get point
        Vector2d P ( CanonicalData[i] );
        //compute shortest point
        Vector2d H ( ClosestPoint(P) );
        /*
glVertex3f( P[0], P[1], 0);
glVertex3f( H[0], H[1], 0);
*/
        //compute everything for this 2d point)
        Vector4d dumvector4d(
                    pow ( ImplicitFunction1(P, Dffinal), 2) ,
                    pow ( ImplicitFunction2(P, Dffinal), 2) ,
                    pow ( ImplicitFunction3(P, Dffinal), 2) ,
                    (P-H).norm() //euclidean distance
                    );
        //compute directly the 3 implicit functions implemented
        Mean += dumvector4d;
        dumarray.push_back(dumvector4d); //store results for variance computation without recomputing everything
    }
    // glEnd();
    Mean /= CanonicalData.size();
    //Compute Var
    for (size_t i=0; i< CanonicalData.size(); i++)
    {
        //compute variance from previously saved results
        Var[0] += pow( Mean[0] - dumarray[i][0], 2);
        Var[1] += pow( Mean[1] - dumarray[i][1], 2);
        Var[2] += pow( Mean[2] - dumarray[i][2], 2);
        Var[3] += pow( Mean[3] - dumarray[i][3], 2);
    }
    Var /= CanonicalData.size()- 1;
    return true;
}
