// test_ankle_kinematics.cpp  –  C++ clone of the Python check
#include <cstdio>
#include <cmath>
#include <array>
#include "ankle_kinematics.hpp"

using ankle_kinematics::vec2;
using ankle_kinematics::mat2;

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double DEG2RAD = M_PI / 180.0;

/* ---------- finite-difference Jacobian (for a quick error check) -------- */
static mat2 J_fd_motor(double pitch,double roll,bool left,double h=1e-8)
{
    const ankle_kinematics::Geom& G = left ? ankle_kinematics::LEFT : ankle_kinematics::RIGHT;
    vec2 f0 = ankle_kinematics::motorAngles(pitch,       roll,       G);
    vec2 f5 = ankle_kinematics::motorAngles(pitch + h,   roll,       G);
    vec2 f6 = ankle_kinematics::motorAngles(pitch,       roll + h,   G);
    mat2 J { { { (f5[0]-f0[0])/h , (f6[0]-f0[0])/h },
               { (f5[1]-f0[1])/h , (f6[1]-f0[1])/h } } };
    return left ? J : mat2{ J[1], J[0] };    // swap rows for right leg
}

/* ---------- helpers to print std::array ---------------------------- */
/* -------- helper -------------------------------- */
static void printDeg(const char* lbl,const vec2& v)
{ printf("%-18s %+9.3f  %+9.3f (deg)\n",lbl,v[0]*RAD2DEG,v[1]*RAD2DEG); }
static void printDegPS(const char* lbl,const vec2& v)
{ printf("%-18s %+9.3f  %+9.3f (deg/s)\n",lbl,v[0]*RAD2DEG,v[1]*RAD2DEG); }

int main()
{
    /* command pose ---------------------------------------------------- */
    vec2 cmd{ 0*DEG2RAD , 0*DEG2RAD };          // roll, pitch

    vec2 tmL = ankle_kinematics::leftMotorAngles (cmd[0],cmd[1]);
    vec2 tmR = ankle_kinematics::rightMotorAngles(cmd[0],cmd[1]);

    vec2 footL = ankle_kinematics::leftFootAngles (*tmL.data(),*(tmL.data()+1));
    vec2 footR = ankle_kinematics::rightFootAngles(*tmR.data(),*(tmR.data()+1));

    printf("Cmd               : %+9.3f  %+9.3f (deg)\n",
           cmd[0]*RAD2DEG,cmd[1]*RAD2DEG);
    printDeg("theta_m left",  tmL);
    printDeg("theta_m right", tmR);
    printDeg("Left back",     footL);
    printDeg("Right back",    footR);

    /* velocity round-trip -------------------------------------------- */
    vec2 footVelCmd{ 1.0*DEG2RAD , -2.0*DEG2RAD };
    vec2 thetaDot = ankle_kinematics::motorVelFromFoot(cmd[0],cmd[1],
                                           footVelCmd[0],footVelCmd[1],true);
    vec2 footVelBack = ankle_kinematics::footVelFromMotor(cmd[0],cmd[1],
                                              thetaDot[0],thetaDot[1],true);
    printDegPS("input foot vel",  footVelCmd);
    printDegPS("back-computed",   footVelBack);

    /* Jacobian check -------------------------------------------------- */
    mat2 J_an  = ankle_kinematics::Jmotor(cmd[0],cmd[1],true);
    mat2 J_fd  = J_fd_motor(cmd[0],cmd[1],true);

    double maxErr = 0.0;
    for(int r=0;r<2;++r)
        for(int c=0;c<2;++c)
            maxErr = std::max(maxErr,std::fabs(J_fd[r][c]-J_an[r][c]));
    printf("max|J_fd-J_anal|   = %.3e\n",maxErr);

    /* torque round-trip ---------------------------------------------- */
    vec2 tau_m{ 9.0 , -9.0 }; // test at maximum motor torque AK60-6 (example)
    vec2 tau_q = ankle_kinematics::ankleTorqueFromMotor(cmd[0],cmd[1],
                                            tau_m[0],tau_m[1],true);
    vec2 tau_m_back = ankle_kinematics::motorTorqueFromAnkle(cmd[0],cmd[1],
                                                 tau_q[0],tau_q[1],true);
    printf("ankle torques      : %+9.3f  %+9.3f (N·m)\n",tau_q[0],tau_q[1]);
    printf("back to motor      : %+9.3f  %+9.3f (N·m)\n",
           tau_m_back[0],tau_m_back[1]);

    vec2 tau_q_fd{
        J_fd[0][0]*tau_m[0] + J_fd[1][0]*tau_m[1],
        J_fd[0][1]*tau_m[0] + J_fd[1][1]*tau_m[1] };
    printf("tau_q (FD)         : %+9.3f  %+9.3f (N·m)\n",
           tau_q_fd[0],tau_q_fd[1]);
}