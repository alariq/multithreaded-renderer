#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double rand01(void) {
    return (double)rand() / (double)RAND_MAX;
}

static double eval_original(double t0, double t1, double t2, double t3,
                            double x,
                            double p1, double p2, double p3, double p4) {
    double A1 = (t1 - x) / (t1 - t0) * p1 + (x - t0) / (t1 - t0) * p2;
    double A2 = (t2 - x) / (t2 - t1) * p2 + (x - t1) / (t2 - t1) * p3;
    double A3 = (t3 - x) / (t3 - t2) * p3 + (x - t2) / (t3 - t2) * p4;
    double B1 = (t2 - x) / (t2 - t0) * A1 + (x - t0) / (t2 - t0) * A2;
    double B2 = (t3 - x) / (t3 - t1) * A2 + (x - t1) / (t3 - t1) * A3;
    double C  = (t2 - x) / (t2 - t1) * B1 + (x - t1) / (t2 - t1) * B2;
    return C;
}

static void eval_term3(double f1, double df1,
                       double f2, double df2,
                       double f3, double df3,
                       double denom,
                       double* value,
                       double* derivative) {
    *value = (f1 * f2 * f3) / denom;
    *derivative = (df1 * f2 * f3 + f1 * df2 * f3 + f1 * f2 * df3) / denom;
}

/*
Derivative equations (multiplied-difference form):

C(x) = k1(x)*p1 + k2(x)*p2 + k3(x)*p3 + k4(x)*p4
dC/dx = k1'(x)*p1 + k2'(x)*p2 + k3'(x)*p3 + k4'(x)*p4

k1 = (t2-x)(t2-x)(t1-x) / ((t2-t1)(t2-t0)(t1-t0))
k2 = (t2-x)(t2-x)(x-t0)/((t2-t1)(t2-t0)(t1-t0))
   + (t2-x)(t2-x)(x-t0)/((t2-t1)(t2-t1)(t2-t0))
   + (x-t1)(t3-x)(t2-x)/((t2-t1)(t2-t1)(t3-t1))
k3 = (t2-x)(x-t0)(x-t1)/((t2-t1)(t2-t1)(t2-t0))
   + (x-t1)(x-t1)(t3-x)/((t2-t1)(t2-t1)(t3-t1))
   + (x-t1)(x-t1)(t3-x)/((t2-t1)(t3-t1)(t3-t2))
k4 = (x-t1)(x-t1)(x-t2)/((t2-t1)(t3-t1)(t3-t2))

Each k_i' is computed by product rule on each 3-factor numerator:
d(f1*f2*f3)/dx = f1'*f2*f3 + f1*f2'*f3 + f1*f2*f3'
*/
static void compute_grouped_coeffs_and_derivative(double t0, double t1,
                                                  double t2, double t3,
                                                  double x,
                                                  double* k1, double* k2,
                                                  double* k3, double* k4,
                                                  double* dk1, double* dk2,
                                                  double* dk3, double* dk4) {
    const double d10 = t1 - t0;
    const double d21 = t2 - t1;
    const double d32 = t3 - t2;
    const double d20 = t2 - t0;
    const double d31 = t3 - t1;

    const double t2mx = t2 - x;
    const double t1mx = t1 - x;
    const double t3mx = t3 - x;
    const double xmt0 = x - t0;
    const double xmt1 = x - t1;
    const double xmt2 = x - t2;

    {
        const double denom = d21 * d20 * d10;
        eval_term3(t2mx, -1.0, t2mx, -1.0, t1mx, -1.0, denom, k1, dk1);
    }

    {
        double v1, v2, v3;
        double d1, d2, d3;

        eval_term3(t2mx, -1.0, t2mx, -1.0, xmt0, 1.0,
                   d21 * d20 * d10, &v1, &d1);
        eval_term3(t2mx, -1.0, t2mx, -1.0, xmt0, 1.0,
                   d21 * d21 * d20, &v2, &d2);
        eval_term3(xmt1, 1.0, t3mx, -1.0, t2mx, -1.0,
                   d21 * d21 * d31, &v3, &d3);

        *k2 = v1 + v2 + v3;
        *dk2 = d1 + d2 + d3;
    }

    {
        double v1, v2, v3;
        double d1, d2, d3;

        eval_term3(t2mx, -1.0, xmt0, 1.0, xmt1, 1.0,
                   d21 * d21 * d20, &v1, &d1);
        eval_term3(xmt1, 1.0, xmt1, 1.0, t3mx, -1.0,
                   d21 * d21 * d31, &v2, &d2);
        eval_term3(xmt1, 1.0, xmt1, 1.0, t3mx, -1.0,
                   d21 * d31 * d32, &v3, &d3);

        *k3 = v1 + v2 + v3;
        *dk3 = d1 + d2 + d3;
    }

    {
        const double denom = d21 * d31 * d32;
        eval_term3(xmt1, 1.0, xmt1, 1.0, xmt2, 1.0, denom, k4, dk4);
    }
}

int main(void) {
    const int num_tests = 200000;
    const double eps = 1e-10;
    const double derivative_eps = 1e-6;
    double max_abs_err = 0.0;
    double max_abs_derivative_err = 0.0;

    srand((unsigned int)time(NULL));

    for (int i = 0; i < num_tests; ++i) {
        /* Build strictly increasing knots. */
        double t0 = -5.0 + 10.0 * rand01();
        double dt1 = 0.05 + 2.0 * rand01();
        double dt2 = 0.05 + 2.0 * rand01();
        double dt3 = 0.05 + 2.0 * rand01();
        double t1 = t0 + dt1;
        double t2 = t1 + dt2;
        double t3 = t2 + dt3;

        /* Typical Catmull-Rom usage: x in [t1, t2]. */
        double u = rand01();
        double x = t1 + u * (t2 - t1);

        double p1 = -100.0 + 200.0 * rand01();
        double p2 = -100.0 + 200.0 * rand01();
        double p3 = -100.0 + 200.0 * rand01();
        double p4 = -100.0 + 200.0 * rand01();

        double corig = eval_original(t0, t1, t2, t3, x, p1, p2, p3, p4);

        double k1, k2, k3, k4, dk1, dk2, dk3, dk4;
        compute_grouped_coeffs_and_derivative(
            t0, t1, t2, t3, x, &k1, &k2, &k3, &k4, &dk1, &dk2, &dk3, &dk4);
        double cgroup = k1 * p1 + k2 * p2 + k3 * p3 + k4 * p4;
        double dgroup = dk1 * p1 + dk2 * p2 + dk3 * p3 + dk4 * p4;

        double err = fabs(corig - cgroup);
        if (err > max_abs_err) {
            max_abs_err = err;
        }

        if (err > eps) {
            fprintf(stderr,
                    "FAILED at test %d\n"
                    "corig=%.17g cgroup=%.17g err=%.3e\n"
                    "k=[%.17g %.17g %.17g %.17g]\n",
                    i, corig, cgroup, err, k1, k2, k3, k4);
            return 1;
        }

        {
            const double h = 1e-4 * (t2 - t1);
            const double c_p2h = eval_original(t0, t1, t2, t3, x + 2.0 * h, p1, p2, p3, p4);
            const double c_ph = eval_original(t0, t1, t2, t3, x + h, p1, p2, p3, p4);
            const double c_mh = eval_original(t0, t1, t2, t3, x - h, p1, p2, p3, p4);
            const double c_m2h = eval_original(t0, t1, t2, t3, x - 2.0 * h, p1, p2, p3, p4);
            const double dnum = (-c_p2h + 8.0 * c_ph - 8.0 * c_mh + c_m2h) / (12.0 * h);
            const double derr = fabs(dnum - dgroup);

            if (derr > max_abs_derivative_err) {
                max_abs_derivative_err = derr;
            }

            if (derr > derivative_eps) {
                fprintf(stderr,
                        "FAILED DERIVATIVE at test %d\n"
                        "dnum=%.17g dgroup=%.17g err=%.3e\n"
                        "dk=[%.17g %.17g %.17g %.17g]\n",
                        i, dnum, dgroup, derr, dk1, dk2, dk3, dk4);
                return 1;
            }
        }
    }

    printf("PASS: %d random tests\n", num_tests);
    printf("max abs error: %.3e\n", max_abs_err);
    printf("max abs derivative error: %.3e\n", max_abs_derivative_err);
    return 0;
}
