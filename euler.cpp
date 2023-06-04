#include <bits/stdc++.h>

double ComputeS0(double x) {
  double ans = 0;
  double dx = x / 1000000;
  for (double x0 = dx; x0 <= x; x0 += dx) {
    ans += std::sin(x0 * x0) * dx;
  }
  return ans;
}

double ComputeS1(double x) {
  double a = 1.0 / 3.0 * x * x * x;
  double ans = a;
  for (int n = 1; n <= 20; ++n) {
    a *= -1.0;
    a /= 2 * n;
    a /= 2 * n + 1;
    a *= 4 * (n - 1) + 3;
    a /= 4 * n + 3;
    a *= (x * x) * (x * x);
    ans += a;
    printf("a = %.20lf\n", a);
  }
  return ans;
}

const double pi = acos(-1);

double ComputeAnotherS2(double x) {
  std::complex<double> z1 {0.5, 0.5};
  std::complex<double> z2 {0.0, 1.0 / (pi * x)};
  std::complex<double> z3 {std::cos(x * x * pi / 2), std::sin(x * x * pi / 2)};
  std::complex<double> ans = z1 - z2 * z3;
  return ans.imag();
}

double ComputeAnotherS3(double x) {
  return 0.5 - 1.0 / (pi * x) * std::cos(pi / 2 * x * x);
}

double ComputeAnotherS4(double x) {
  double pix2 = pi * x * x;
  std::complex<double> b(1.0, -pix2);
  std::complex<double> cc = 1e20;
  std::complex<double> d = 1.0 / b;
  std::complex<double> h = 1.0 / b;
  int n = -1;
  for (int k = 2; k <= 15; k++) {
    n += 2;
    double a = -n * (n + 1);
    b += 4.0;
    d = 1.0 / (a * d + b);
    cc = b + a / cc;
    std::complex<double> del = cc * d;
    h *= del;
  }
  h *= std::complex<double>(x, -x);
  std::complex<double> ans = std::complex<double>(0.5, 0.5) *
      (1.0 - std::complex<double>(std::cos(0.5 * pix2), std::sin(0.5 * pix2)) * h);
  return ans.imag();
}

int main() {
  printf("%.12lf\n", ComputeS0(1.0));
  printf("%.12lf\n", ComputeS1(1.0));

  printf("%.12lf\n", ComputeS0(3.0));
  printf("%.12lf\n", ComputeS1(3.0));
  printf("%.12lf\n", ComputeAnotherS2(3.0 / sqrt(pi / 2)) * sqrt(pi / 2));
  printf("%.12lf\n", ComputeAnotherS3(3.0 / sqrt(pi / 2)) * sqrt(pi / 2));
  printf("%.12lf\n", ComputeAnotherS4(3.0 / sqrt(pi / 2)) * sqrt(pi / 2));
  printf("%.12lf\n", sqrt(pi / 8.0));

  printf("%.12lf\n", ComputeS0(1000));
  printf("%.12lf\n", ComputeAnotherS4(1000 / sqrt(pi / 2)) * sqrt(pi / 2));
  return 0;
}

