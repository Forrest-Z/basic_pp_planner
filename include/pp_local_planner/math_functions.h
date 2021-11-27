// C++ program to find the CIRCUMCENTER of a
// triangle
#include <iostream>
#include <cfloat>



using namespace std;



namespace math_functions{

    #define pdd pair<double, double>

    // Function to find the line given two points
    void lineFromPoints(pdd P, pdd Q, double &a,
                            double &b, double &c)
    {
        a = Q.second - P.second;
        b = P.first - Q.first;
        c = a*(P.first)+ b*(P.second);
    }

    // Function which converts the input line to its
    // perpendicular bisector. It also inputs the points
    // whose mid-point lies on the bisector
    void perpendicularBisectorFromLine(pdd P, pdd Q,
                    double &a, double &b, double &c)
    {
        pdd mid_point = make_pair((P.first + Q.first)/2,
                                (P.second + Q.second)/2);

        // c = -bx + ay
        c = -b*(mid_point.first) + a*(mid_point.second);

        double temp = a;
        a = -b;
        b = temp;
    }

    // Returns the intersection point of two lines
    pdd lineLineIntersection(double a1, double b1, double c1,
                            double a2, double b2, double c2)
    {
        double determinant = a1*b2 - a2*b1;
        if (determinant == 0)
        {
            // The lines are parallel. This is simplified
            // by returning a pair of FLT_MAX
            return make_pair(FLT_MAX, FLT_MAX);
        }

        else
        {
            double x = (b2*c1 - b1*c2)/determinant;
            double y = (a1*c2 - a2*c1)/determinant;
            return make_pair(x, y);
        }
    }

    bool findCircumCenter(pdd P, pdd Q, pdd R, pair<double, double> &centre_)
    {
        // Line PQ is represented as ax + by = c
        double a, b, c;
        lineFromPoints(P, Q, a, b, c);

        // Line QR is represented as ex + fy = g
        double e, f, g;
        lineFromPoints(Q, R, e, f, g);

        // Converting lines PQ and QR to perpendicular
        // vbisectors. After this, L = ax + by = c
        // M = ex + fy = g
        perpendicularBisectorFromLine(P, Q, a, b, c);
        perpendicularBisectorFromLine(Q, R, e, f, g);

        // The point of intersection of L and M gives
        // the circumcenter
        pdd circumcenter =
            lineLineIntersection(a, b, c, e, f, g);

        if (circumcenter.first == FLT_MAX &&
            circumcenter.second == FLT_MAX)
        {
            cout << "The two perpendicular bisectors "
                    "found come parallel" << endl;
            cout << "Thus, the given points do not form "
                    "a triangle and are collinear" << endl;
        
            return false;
        }

        else
        {
            cout << "The circumcenter of the triangle PQR is: ";
            cout << "(" << circumcenter.first << ", "
                << circumcenter.second << ")" << endl;

            centre_ = make_pair(circumcenter.first, circumcenter.second);
            return true;
        }
    }
};

