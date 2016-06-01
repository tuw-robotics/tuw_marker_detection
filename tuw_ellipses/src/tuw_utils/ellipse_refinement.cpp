/**
 * $Id$
 *
 * (C) 2014, Johann Prankl, Aitor Aldoma Buchaca
 */


#include "tuw_utils/ellipse_refinement.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
//#include <opencv2/highgui/highgui.hpp>


namespace tuw
{

template<typename T>
inline bool isZero(const T d)
{
    return fabs(d) <= std::numeric_limits<T>::epsilon();
}

/**
 * scale angle to [0..2pi[
 */
inline double scaleAngle_0_2pi(double a)
{
    while(a >= 2.*M_PI) a -= 2.*M_PI;
    while(a < 0.) a += 2.*M_PI;
    return a;
}

using namespace std;

/**
 * Constructor/ destructor
 */
EllipseRefinement::EllipseRefinement(const Parameter &_param)
    : param(_param)
{

}

EllipseRefinement::~EllipseRefinement()
{

}

/**
 * Calculate exact ellipse parameters of the given ellipse hypothesis using gradient images
 */
bool EllipseRefinement::refine(const cv::Mat_<short> &im_dx, const cv::Mat_<short> &im_dy,
                                   const std::vector<cv::Point2f> &points, Ellipse &ellipse)
{
    // get region around the ellipse edgels where to compute the dual ellipse
    // that means, pixels where the gradient is higher than the given threshold and the distance
    // to the ellipse borders along the line joining the pixel and the ellipse center
    // is less than a certain radius (3px for example)

    double minx=10000, maxx=0, miny=10000, maxy=0;
    double minGradPoints=10000,maxGradPoints=0;

    for(unsigned i=0; i<points.size(); i++)
    {
        const cv::Point2f &pt = points[i];

        if(pt.x < minx) minx=pt.x;
        if(pt.y < miny) miny=pt.y;
        if(pt.x > maxx) maxx=pt.x;
        if(pt.y > maxy) maxy=pt.y;
        float a =   im_dx(pt.y,pt.x);
        float b =   im_dy(pt.y,pt.x);
        float grad = sqrt(a*a + b*b);
        if(minGradPoints > grad) minGradPoints = grad;
        if(grad > maxGradPoints) maxGradPoints = grad;
    }

    double gradThreshold = (minGradPoints + (maxGradPoints - minGradPoints)/2)/2;
    gradThreshold = param.min_gradient; // minimum gradient to use a pixel
    // it does not make sense to use different values for ellipseRand and rand
    int ellipseRand=param.border; // maximum distance to given hypothesis
    int rand=param.border; // window border size to search for border points (hypothesis parameter A+rand X B+rand)
    pointsToUse.clear();
    double sumx=0,sumy=0;
    // searching within the window for points to use
    short min_y = (short)(miny-rand);
    short max_y = (short)(maxy+rand);
    short min_x = (short)(minx-rand);
    short max_x = (short)(maxx+rand);
    if (min_y < 0) min_y = 0;
    if (max_y >= im_dx.rows) max_y = (short)im_dx.rows-1;
    if (min_x < 0) min_x = 0;
    if (max_x >= im_dx.cols) max_x = (short)im_dx.cols-1;

    for(short j=min_y; j < max_y; j++)
    {
        for(short i=min_x; i < max_x; i++)
        {
            float a =   im_dx(j,i);
            float b =   im_dy(j,i);
            float grad = sqrt(a*a + b*b);
            if(gradThreshold < grad &&
                    ellipse.insideEllipse(ellipse.a+ellipseRand,ellipse.b+ellipseRand,ellipse.x,ellipse.y, ellipse.phi,i,j) &&
                    !ellipse.insideEllipse(ellipse.a-ellipseRand,ellipse.b-ellipseRand,ellipse.x,ellipse.y, ellipse.phi,i,j))
            {
                pointsToUse.push_back(cv::Point2d(i,j));
                sumx+=i;
                sumy+=j;
            }
        }
    }

    // now the pointsToUse array exists (points with a distance smaller than ellipseRand from the hypothesis)
    // normalisation (to ensure numeric stability)
    double mx = sumx/pointsToUse.size();
    double my = sumy/pointsToUse.size();

    // compute scaling factors
    double lengths=0;
    for(unsigned i=0; i<pointsToUse.size(); i++)
    {
        double a = pointsToUse[i].x-mx;
        double b = pointsToUse[i].y-my;

        lengths+= sqrt(a*a + b*b);
    }

    double ss = sqrt(2.) / (lengths/pointsToUse.size());
    Eigen::Matrix3d H;
    H << ss, 0, -ss*mx, 0, ss, -ss*my, 0, 0, 1;
    Eigen::Matrix3d Hinv = H.inverse();
    Eigen::Matrix3d HinvT = Hinv.transpose();


    // transformation finished
    /*
    Establish the least-square-system.

    In the following, [] means the definition of a vector or matrix and [x]^T its
    the transpose of [x]

    A point xi = [ui,vi,1]^T (in homogenous coordinates) lies on the conic C iff
    it satisfies xi^T*C*xi=0
    C is:
    A 		B/2		D/2
    B/2	 	C	  	E/2
    D/2		E/2		 F
    The ConicParameters = [A,B,C,D,E,F] of the conic C can be obtained by least square techniques
    by minimizing f(ConicParameters of C):

    f(ConicParameters of C) = sum_{iterate points}(w_i*(x_i^T*C*x_i)^2)

    Following part is based in the duality relationship of projective geometry where the role of
    homogenous points and lines can be interchanged.

    In the dual space, a line l_i = [a_i,b_i,c_i]^T is tangent to the dual of the conic C*
    iff it satisfies l_i^T*(C*)*l_i=0 where C* = inv(C)
    As in the point space, the DualConicParameters = [A*,B*,C*,D*,E*,F*] of the dual conic C*
    can be obtained similarly by finding f where f(DualConicParameters of C) reaches a minimum:

    f(DualConicParameters of C) = sum_{iterate lines}(w_i*(l_i^T*(C*)*l_i)^2) (Eq 1)

    li = [a_i,b_i,c_i]^T are obtained directly from the image gradient as follows:
    a_i = dx_i
    b_i = dy_i
    c_i = -[dx_i,dy_i]^T*[x_i,y_i]

    when the image gradient at [x_i,y_i] is not null, it defines the normal orientation of a line
    passing through [x_i,y_i]

    Operating in Eq 1, we obtain the following normal equations:
    [sum{iterate lines}(sqr(w_i)*Ki*Ki^T)][DualConicParameters] = 0

    where Ki = [sqr(a_i),a_i*b_i, sqr(b_i), a_i*c_i, b_i*c_i, c_i*2]^T

    By using the constraint F* = 1 to solve the system and adding the ellipse discriminant 4*A*C - sqr(B) = 1,
    we obtain the final system to find DualConicParameters' = [A'*,B'*,C'*,D'*,E'*,F'*]:

    sum{iterate lines}(-sqr(w_i)*K'i*sqr(c_i)) = 0

    which is the final system solved in the following code.
    Further references to the method can be found in:
    http://vision.gel.ulaval.ca/~hebert/pdf/ouellet07Ellipses.pdf
    */

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(5,5);
    Eigen::VectorXd E = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd Kprima(5);
    Eigen::MatrixXd res1(5,5);
    Eigen::Vector3d li;

    std::vector<Eigen::Vector3d> tangentLines((unsigned)pointsToUse.size());

    for (unsigned i=0; i<pointsToUse.size(); i++)
    {
        const cv::Point2d &pt = pointsToUse[i];
        Kprima.setZero();
        res1.setZero();
        float a =   im_dx((short)pt.y,(short)pt.x);
        float b =   im_dy((short)pt.y,(short)pt.x);
        float grad = sqrt(a*a + b*b);

        // compute tangent lines to the dual conic directly from the image gradient
        li << double(im_dx((short)pt.y,(short)pt.x)), double(im_dy((short)pt.y,(short)pt.x)),
           -( double(im_dx((short)pt.y,(short)pt.x))*pt.x + double(im_dy((short)pt.y,(short)pt.x))*pt.y );

        // scale line such norm(a_i,b_i)=1
        li /= grad;

        // normalize lines like L'=H^-T * L (-T is the inverse transposed)
        li = HinvT * li;

        //save tangent line
        tangentLines[i] = li;

        double weight = grad*grad;
        Kprima[0] = li[0]*li[0];
        Kprima[1] = li[0]*li[1];
        Kprima[2] = li[1]*li[1];
        Kprima[3] = li[0]*li[2];
        Kprima[4] = li[1]*li[2];

        res1 = weight*Kprima*Kprima.transpose();
        M += res1;

        Kprima[0] *= -weight*li[2]*li[2];
        Kprima[1] *= -weight*li[2]*li[2];
        Kprima[2] *= -weight*li[2]*li[2];
        Kprima[3] *= -weight*li[2]*li[2];
        Kprima[4] *= -weight*li[2]*li[2];

        E += Kprima;
    }

    // solve the least square system using the pseudo inverse technique
    // params = inv(M^T*M)*M^T*E

    Eigen::MatrixXd MT(5,5);
    MT = M.transpose();

    Eigen::MatrixXd MPerMT(5,5), MPerMTinv(5,5);

    MPerMT = MT*M;
    MPerMTinv = MPerMT.inverse();
    MPerMT = MPerMTinv*MT;

    Eigen::VectorXd params(5);
    params = MPerMT*E;

    Eigen::Matrix3d CNorm;
    CNorm << params[0],    params[1]/2., params[3]/2.,
          params[1]/2., params[2],    params[4]/2.,
          params[3]/2., params[4]/2., 1;

    //unnormalize params
    Eigen::Matrix3d CNormMalHInv;
    CNormMalHInv = Hinv*CNorm*HinvT;

    //compute error in the dual space
    //the error is proportional to distance between a tangent li and its pole xi=(C*)*li

    double dSumDistance=0;
    Eigen::Vector3d pole;

    for (unsigned i=0; i<pointsToUse.size(); i++)
    {
        const Eigen::Vector3d &l = tangentLines[i];
        //distance between li and pole
        pole = CNorm*l;

        double x0 = pole[0]/pole[2];
        double y0 = pole[1]/pole[2];
        double a = l[0];
        double b = l[1];
        double d = abs(l[0] * x0 + l[1] * y0 + l[2]) / sqrt(a*a+b*b);
        dSumDistance += d;
    }

    ellipse.fit_error = dSumDistance / double(pointsToUse.size());

    // by inverting the dual conic matrix C* we obtain the original
    // conic parameters we were seeking for

    Eigen::Matrix3d C;
    C = CNormMalHInv.inverse();

    double Ac,Bc,Cc,Dc,Ec,Fc;
    Ac = C(0,0);
    Bc = C(0,1);
    Cc = C(1,1);
    Dc = C(2,0);
    Ec = C(1,2);
    Fc = C(2,2);

    // calculate ellipse parameters x/y/A/B/phi from conic equation Ax^2+Bxy+Cy^2....+F=0
    bool ell_ok = ellipse.computeAndSetGeomFromConic(Ac,Bc,Cc,Dc,Ec,Fc);

    return ell_ok;
}



/**
 * Constructor/ destructor
 */
EllipseRefinement::Ellipse::Ellipse()
    : id(0), x(0), y(0), a(0), b(0), phi(0), fit_error(0.), support(0.), a2(0), a4(0), b2(0), b4(0), x2(0), y2(0)
{}

EllipseRefinement::Ellipse::Ellipse(const Ellipse &e)
    : id(e.id), x(e.x), y(e.y), a(e.a), b(e.b), phi(e.phi), fit_error(e.fit_error), support(e.support), a2(e.a2), a4(e.a4), b2(e.b2), b4(e.b4), x2(e.x2), y2(e.y2){
}

void EllipseRefinement::Ellipse::setEllipse(const double &_x, const double &_y, const double &_a, const double &_b, const double &_phi) {
    x = _x, y = _y, a = _a, b = _b, phi = _phi;
    a2 = a*a;
    a4 = a2*a2;
    b2 = b*b;
    b4 = b2*b2;
    x2 = 0;
    y2 = 0;
}

/**
 * Sets the ellipse parameter to get ready for detection based on an opencv rectancle
 */
void EllipseRefinement::Ellipse::setEllipse(const cv::RotatedRect& rect) {
    x = rect.center.x;
    y = rect.center.y;
    // box size is double the axis lengths
    a = double(rect.size.width)/2.;
    b = double(rect.size.height)/2.;
    // note: the angle returned is in degrees!
    phi = scaleAngle_0_2pi(rect.angle*M_PI/180.);
    // note: for unknown reasons sometimes a < b!
    if(a < b)
    {
        swap(a, b);
        phi = scaleAngle_0_2pi(phi + M_PI_2);
    }
    setEllipse(x, y, a, b, phi);
}

/**
 * Compute support
 */
unsigned EllipseRefinement::Ellipse::ellipseSupport(const std::vector<cv::Point2f> &points, double inl_dist, std::vector<bool> &inl_idx)
{
    double co = cos(-phi), si = sin(-phi), n, d, dist;
    unsigned z;
    unsigned nbInl=0;

    cv::Point2f p, q;

    fit_error = 0.;
    inl_idx.resize(points.size());

    for(z=0; z<points.size(); z++)
    {
        // transform point in image coords to ellipse coords
        // Note that this piece of code is called often.
        // Implementing this explicitely here is faster than using
        // TransformToEllipse(), as cos and sin only need to be evaluated once.
        //p = Vector2(points[z].x,points[z].y);
        p=points[z];
        p.x -= x;
        p.y -= y;
        q.x = co*p.x - si*p.y;
        q.y = si*p.x + co*p.y;
        // calculate absolute distance to ellipse
        if(isZero(q.x) && isZero(q.y))
            dist = b;
        else
        {
            x2 = q.x * q.x;
            y2 = q.y * q.y;
            n = fabs(x2/a2 + y2/b2 - 1.);
            d = 2.*sqrt(x2/a4 + y2/b4);
            dist = n/d;
        }
        if(dist <= inl_dist)
        {
            inl_idx[z]=true;
            nbInl++;
            fit_error += dist;
        }
        else
            inl_idx[z]=false;
    }

    double circumference = ellipseCircumference(a, b);

    if (nbInl>0 && !isZero(circumference))
    {
        fit_error /= (double)nbInl;
        support = (double)nbInl / circumference;
    }
    else
    {
        fit_error = DBL_MAX;
        support=0;
    }

    return nbInl;
}


void EllipseRefinement::Ellipse::get(cv::RotatedRect &r) {
    r.center.x = x, r.center.y = y;
    r.size.width = a*2., r.size.height = b*2.;
    r.angle = phi*180./M_PI;
}


/**
 * Calculate ellipse parameters x/y/A/B/phi from conic equation Ax^2+Bxy+Cy^2....+F=0
 */
bool EllipseRefinement::Ellipse::computeAndSetGeomFromConic(double Ac, double Bc, double Cc, double Dc, double Ec, double Fc) {

    double BcBcAcCc = (Bc*Bc - Ac*Cc);
    if (BcBcAcCc == 0) {
        return false;
    }
    double x0 = (Cc*Dc - Bc*Ec) / BcBcAcCc;
    double y0 = (Ac*Ec - Bc*Dc) / BcBcAcCc;
    double a0, a0_2, b0, b0_2, phi0;
    double d = Ac-Cc;
    double SqrAcCc4BcBc = d*d + 4*Bc*Bc;
    if (SqrAcCc4BcBc < 0) {
        return false;
    }
    a0_2 = (2*(Ac*Ec*Ec + Cc*Dc*Dc + Fc*Bc*Bc - 2*Bc*Dc*Ec - Ac*Cc*Fc)) / ((Bc*Bc - Ac*Cc)*((sqrt(SqrAcCc4BcBc)) - (Ac+Cc)));
    b0_2 = (2*(Ac*Ec*Ec + Cc*Dc*Dc + Fc*Bc*Bc - 2*Bc*Dc*Ec - Ac*Cc*Fc)) / ((Bc*Bc - Ac*Cc)*(-(sqrt(SqrAcCc4BcBc)) - (Ac+Cc)));
    if ((a0_2 < 0) || (b0_2 < 0)) {
        return false;
    }
    a0 = sqrt(a0_2);
    b0 = sqrt(b0_2);

    //   double tempPhi=phi;
    phi0=0;
    if (Bc == 0) {
        if (Ac < Cc) {
            //cout << "1" << endl;
            phi0=M_PI_2;
        }
    } else {
        double z = (Ac-Cc) / (2*Bc);
        if (Ac < Cc) {
            phi0=0.5*(atan(1./z));
            //cout << "m2" << endl;
        }
        if (Ac > Cc) {
            phi0=M_PI_2 + 0.5*(atan(1./z));
            //cout << "m3" << endl;
        }
    }

    x = x0;
    y = y0;
    a = a0;
    b = b0;

    //phi = scaleAngle_0_2pi(phi0);
    if (a < b) {
        swap(a, b);
        //phi = scaleAngle_0_2pi(phi + M_PI_2);
    }

    //phi=tempPhi;

    return true;
}

/**
 * Approximation to ellipse circumference.
 * (from Bartsch: Mathematische Formeln, Buch- und Zeit-Verlagsgesellschaft Koeln, 1988, p. 221)
 */
double EllipseRefinement::Ellipse::ellipseCircumference(double a, double b)
{
    return M_PI*(1.5*(a + b) - sqrt(a*b));
}


/**
 * Returns true if the position x/y is inside the given ellipse using the ellipse equation
 */
bool EllipseRefinement::Ellipse::insideEllipse(double _a, double _b, double _x0, double _y0, double _phi, double _x, double _y) const
{
    double dx = ((_x - _x0)*cos(_phi) + (_y-_y0)*sin(_phi)) / _a;
    double dy = (-(_x - _x0)*sin(_phi) + (_y-_y0)*cos(_phi)) / _b;
    double distance = dx * dx + dy * dy;
    return (distance < 1.0) ? 1 : 0;

}
}  // -- THE END --

