/*
 * @Author: Xiawenlong-bug 2473833028@qq.com
 * @Date: 2024-09-13 13:32:53
 * @LastEditors: Xiawenlong-bug 2473833028@qq.com
 * @LastEditTime: 2024-09-13 14:42:42
 * @FilePath: /optim_hw1/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;
double getFvalue(Vector2d &x, double tau, Vector2d &d) {
    Vector2d x_new = x + tau * d;
    return 100 * pow((pow(x_new(0), 2) - x_new(1)), 2) + pow((x_new(0) - 1), 2);
};
int main(){
    Vector2d x;
    double f = 100 * pow((pow(x(0), 2) - x(1)), 2) + pow((x(0) - 1), 2);
    Vector2d df;
    df << 400 * x(0) * (pow(x(0), 2) - x(1)) + 2 * (x(0) - 1), -200 * (pow(x(0), 2) - x(1));
    x << 0, 0;
    while(df.norm() > 1e-4){
        Matrix2d H,M;
        H << 1200 * pow(x(0), 2) - 400 * x(1) + 2, -400 * x(0), -400 * x(0), 200;
        M = H.inverse();
        Vector2d d = -M * df;
        double tau = 1.0;
        double rho = 1e-4;
        while (getFvalue(x,tau,d)>getFvalue(x,0,d)+rho*tau*d.dot(df)){
            tau /= 2;
        }
        x += tau * d;
        f = 100 * pow((pow(x(0), 2) - x(1)), 2) + pow((x(0) - 1), 2);
        df << 400 * x(0) * (pow(x(0), 2) - x(1)) + 2 * (x(0) - 1), -200 * (pow(x(0), 2) - x(1));
    }
    printf("final x*= %f,%f\n",x(0),x(1));
    printf("final y*= %f\n", f);
}
