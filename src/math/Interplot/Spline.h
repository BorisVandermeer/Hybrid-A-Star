/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A simple cubic spline interpolation library without external 
 *      dependencies.
 * 
 * Reference : 
 *      https://blog.csdn.net/qq_21933647/article/details/103500428
 * 
*********************************************************************/

#ifndef _INTERPLOATION_SPLINE_H_
#define _INTERPLOATION_SPLINE_H_

#include<vector>
#include<Points/PointsType.h>

namespace Interplot
{

namespace Utils
{
    // Disposable Band Matrix class to solve "Ax=b" problems by LU decompose.
    class band_matrix {
    public:
        std::vector<std::vector<double> > m_upper;  // upper band
        std::vector<std::vector<double> > m_lower;  // lower & diag band
        bool decomposed_flag_ = false; 
    public:
        band_matrix(){};                             
        band_matrix(int dim, int n_u, int n_l);      
        ~band_matrix(){};
        bool isDecomposed() const {return decomposed_flag_;}
        void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
        int dim() const;                             // matrix dimension
        int num_upper() const {return m_upper.size() - 1;}
        int num_lower() const {return m_lower.size() - 1;}
        // access operator
        double &operator()(int i, int j);            // write
        double operator()(int i, int j) const;      // read
        // we can store an additional diogonal (in m_lower)
        double &saved_diag(int i);
        double saved_diag(int i) const;
        void lu_decompose();
        // solves Rx=y
        std::vector<double> r_solve(const std::vector<double> &b) const; 
        // solves Ly=b
        std::vector<double> l_solve(const std::vector<double> &b) const;
        // solves Ax=y
        std::vector<double> lu_solve(const std::vector<double> &b);
    };
} // namespace Utils

    
    class Spline{
    public:
        enum BoundType {
            FirstOrderDer = 1,
            SecondOrderDer = 2
        };

        typedef Points::PosPoint2Ds RefPoints;

    public:
        // Use Natural Spline as Default
        Spline():
            m_left(SecondOrderDer),
            m_right(SecondOrderDer),
            m_left_value(0.0),
            m_right_value(0.0),
            m_force_linear_extrapolation(false) {}
 
        void set_boundary(BoundType left, double left_value, BoundType right,
                            double right_value,
                            bool force_linear_extrapolation = false);
        // void set_points(const std::vector<double>& x, const std::vector<double>& y,bool cubic_spline = true);
        void set_points(const RefPoints &points, bool cubic_spline = true );
                        
        double operator()(double x) const;
        double getDeriv(int order, double x) const;

    private:
        RefPoints PointsData;
        std::vector<double> m_a, m_b, m_c;        // spline coefficients
        double m_b0, m_c0;                     // for left extrapol
        BoundType m_left, m_right;
        double m_left_value, m_right_value;
        bool m_force_linear_extrapolation;
    };
    
} // namespace Interplot


#endif