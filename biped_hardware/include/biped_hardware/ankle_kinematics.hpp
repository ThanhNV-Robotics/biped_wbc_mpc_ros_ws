#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdio>
namespace ankle_kinematics
{
using vec2 = std::array<double,2>;
using mat2 = std::array<std::array<double,2>,2>;

/* Mechanical range (design, 2026-10-23): roll ±15 deg, pitch ±30 deg, same as the URDF limits.
 * With the geometry below the linkage does not reach the corners of that box: full roll ±15 deg
 * only for pitch -22.7 .. +26.9 deg, at pitch -30 deg only about ±2 deg roll, at +30 deg about
 * ±8 deg (both legs, mirrored). clampToWorkspace() keeps commands inside the reachable set. */
constexpr double ROLL_LIMIT  = 15.0 * M_PI / 180.0;
constexpr double PITCH_LIMIT = 30.0 * M_PI / 180.0;

/* ───────── geometry ───────── */
struct Geom
{
    vec2 Px0, Py0, Pz0;
    vec2 d ,  a ,  h ,  l;
    vec2 sdir;
};


constexpr Geom LEFT {
    /*Px0*/ {0.043, 0.043},
    /*Py0*/ {0.023, -0.023},
    /*Pz0*/ {-0.02, -0.02},
    /*d  */ {.043, 0.043},
    /*a  */ {0.023,  0.023},
    /*h  */ {0.230 ,  0.145},
    /*l  */ {0.25, 0.165},
    /*s  */ { 1.0 ,-1.0 }
};

constexpr Geom RIGHT {
    /*Px0*/ {0.043, 0.043},
    /*Py0*/ {0.023, -0.023},
    /*Pz0*/ {-0.02, -0.02},
    /*d  */ {0.043, 0.043},
    /*a  */ {0.023,  0.023},
    /*h  */ {0.145,  0.23}, // swapped
    /*l  */ {0.165, 0.25}, // swapped
    /*s  */ { 1.0 ,-1.0 }
};
/* ───────── Helpers function───────── */
inline void rotate(double c5,double s5,double c6,double s6,
                   const vec2& Px0,const vec2& Py0,const vec2& Pz0,
                   vec2& Px, vec2& Py, vec2& Pz)
{
    for(std::size_t i=0;i<2;++i)
    {
        Px[i] =  c6*Px0[i] + s6*Pz0[i];
        Py[i] =  s5*s6*Px0[i] + c5*Py0[i] - s5*c6*Pz0[i];
        Pz[i] = -c5*s6*Px0[i] + s5*Py0[i] + c5*c6*Pz0[i];
    }
}

/* ---------- inverse map (pitch,roll) → (θm5,θm6) ---------------------- */
inline vec2 motorAngles(double roll,double pitch,const Geom& G)
{
    const auto& [Px0,Py0,Pz0,d,a,h,l,s] = G;
    const double c5=std::cos(roll),  s5=std::sin(roll);
    const double c6=std::cos(pitch), s6=std::sin(pitch);

    vec2 Px,Py,Pz;  rotate(c5,s5,c6,s6,Px0,Py0,Pz0,Px,Py,Pz);

    vec2 theta{};
    for(std::size_t i=0;i<2;++i)
    {
        double z = Pz[i]-h[i],  x = d[i]-Px[i];
        double u = 2*a[i]*Py[i], v = 2*a[i]*z;
        double w = (a[i]*a[i]+x*x+Py[i]*Py[i]+z*z-l[i]*l[i])*s[i];
        theta[i] = std::atan2(v,u) + std::acos(w/std::hypot(u,v));
    }
    return theta;
}
inline vec2 leftMotorAngles (double r,double p){return motorAngles(r,p,LEFT);}
inline vec2 rightMotorAngles(double r,double p){auto t=motorAngles(r,p,RIGHT); return {t[1],t[0]}; } // swap order

/* ---------- reachable set --------------------------------------------------------- */
/* True if both links reach (the acos argument of motorAngles stays inside [-1, 1]) */
inline bool reachable(double roll,double pitch,const Geom& G)
{
    const auto& [Px0,Py0,Pz0,d,a,h,l,s] = G;
    vec2 Px,Py,Pz;
    rotate(std::cos(roll),std::sin(roll),std::cos(pitch),std::sin(pitch),Px0,Py0,Pz0,Px,Py,Pz);
    for(std::size_t i=0;i<2;++i)
    {
        double z = Pz[i]-h[i],  x = d[i]-Px[i];
        double u = 2*a[i]*Py[i], v = 2*a[i]*z;
        double w = (a[i]*a[i]+x*x+Py[i]*Py[i]+z*z-l[i]*l[i])*s[i];
        if(!(std::fabs(w) <= std::hypot(u,v))) return false;
    }
    return true;
}

/* Clamp to the mechanical range, then, if the linkage cannot reach that pose, keep the pitch and
 * reduce the roll (bisection) until it can. Never returns a pose motorAngles() cannot solve. */
inline vec2 clampToWorkspace(double roll,double pitch,const Geom& G)
{
    roll  = std::clamp(roll, -ROLL_LIMIT,  ROLL_LIMIT);
    pitch = std::clamp(pitch,-PITCH_LIMIT, PITCH_LIMIT);
    if(reachable(roll,pitch,G)) return {roll,pitch};
    if(!reachable(0.0,pitch,G)) pitch = 0.0;   // not expected inside ±30 deg; stay safe
    double lo = 0.0, hi = 1.0;                  // scale of the roll: lo reachable, hi not
    for(int k=0;k<30;++k)
    {
        const double mid = 0.5*(lo+hi);
        if(reachable(mid*roll,pitch,G)) lo = mid; else hi = mid;
    }
    return {lo*roll,pitch};
}
inline vec2 leftClampToWorkspace (double r,double p){return clampToWorkspace(r,p,LEFT);}
inline vec2 rightClampToWorkspace(double r,double p){return clampToWorkspace(r,p,RIGHT);}

/* ---------- forward map (θm → pitch,roll)  –  FD Newton --------------- */
inline vec2 forwardOneLeg(const vec2& th,const Geom& G,
                          double roll0=0,double pitch0=0)
{
    const auto& [Px0,Py0,Pz0,d,a,h,l,s] = G;
    const vec2 c_m{std::cos(th[0]),std::cos(th[1])};
    const vec2 s_m{std::sin(th[0]),std::sin(th[1])};

    double roll=roll0, pitch=pitch0;
    for(int k=0;k<10;++k)   // converges in ~4 inside the range; more as margin near the edges
    {
        double c5=std::cos(roll),s5=std::sin(roll);
        double c6=std::cos(pitch),s6=std::sin(pitch);

        vec2 Px,Py,Pz;  rotate(c5,s5,c6,s6,Px0,Py0,Pz0,Px,Py,Pz);

        vec2 x,z,W,f;
        for(std::size_t i=0;i<2;++i)
        {
            z[i]=Pz[i]-h[i];  x[i]=d[i]-Px[i];
            W[i]=(a[i]*a[i]+x[i]*x[i]+Py[i]*Py[i]+z[i]*z[i]-l[i]*l[i])/(2*a[i]);
            f[i]=Py[i]*c_m[i]+z[i]*s_m[i]-s[i]*W[i];
        }
        /* Jacobian */
        vec2 dPy_dr,dPz_dr,dPy_dp,dPz_dp;
        for(std::size_t i=0;i<2;++i)
        {
            dPy_dr[i]= c5*s6*Px0[i]-s5*Py0[i]-c5*c6*Pz0[i];
            dPz_dr[i]= s5*s6*Px0[i]+c5*Py0[i]-s5*c6*Pz0[i];
            dPy_dp[i]= s5*c6*Px0[i]+s5*s6*Pz0[i];             // 
            dPz_dp[i]=-c5*c6*Px0[i]-c5*s6*Pz0[i];
        }
        vec2 dW_dr,dW_dp;
        for(std::size_t i=0;i<2;++i)
        {
            dW_dr[i]=(Py[i]*dPy_dr[i]+z[i]*dPz_dr[i])/a[i];
            double dPx_dp = -s6*Px0[i] + c6*Pz0[i];
            double dx_dp  = -dPx_dp;
            dW_dp[i]=(x[i]*dx_dp + Py[i]*dPy_dp[i] + z[i]*dPz_dp[i]) / a[i];
        }
        double J11=dPy_dr[0]*c_m[0]+dPz_dr[0]*s_m[0]-s[0]*dW_dr[0];
        double J12=dPy_dp[0]*c_m[0]+dPz_dp[0]*s_m[0]-s[0]*dW_dp[0];
        double J21=dPy_dr[1]*c_m[1]+dPz_dr[1]*s_m[1]-s[1]*dW_dr[1];
        double J22=dPy_dp[1]*c_m[1]+dPz_dp[1]*s_m[1]-s[1]*dW_dp[1];

        double det=J11*J22-J12*J21;
        double dx=( f[0]*J22-f[1]*J12)/det;
        double dy=(-f[0]*J21+f[1]*J11)/det;
        roll-=dx; pitch-=dy;
        if(std::fabs(dx)<1e-12 && std::fabs(dy)<1e-12) break;
    }
    return {std::clamp(roll,-ROLL_LIMIT,ROLL_LIMIT),
            std::clamp(pitch,-PITCH_LIMIT,PITCH_LIMIT)};
}
inline vec2 leftJointAngles (double t5,double t6,double r0=0,double p0=0)
{ return forwardOneLeg({t5,t6},LEFT ,r0,p0); }
inline vec2 rightJointAngles(double t5,double t6,double r0=0,double p0=0)
{ auto rp = forwardOneLeg({t6,t5},RIGHT,r0,p0); return rp; } // swap in/out

/* ───────────────────────── analytic J_inv  (\partial theta_m/\partial [roll,pitch]) ────────── */
inline mat2 jacobian(double roll,double pitch,const Geom& G)
{
    const auto& [Px0,Py0,Pz0,d,a,h,l,s] = G;

    double c5 = std::cos(roll),  s5 = std::sin(roll);
    double c6 = std::cos(pitch), s6 = std::sin(pitch);

    vec2 Px,Py,Pz;
    rotate(c5,s5,c6,s6,Px0,Py0,Pz0,Px,Py,Pz);

    /* 1st-order derivatives of the rotated points */
    vec2 dPy_dr,dPz_dr,dPy_dp,dPz_dp;
    double dPx_dp;
    for(std::size_t i=0;i<2;++i)
    {
        dPy_dr[i] =  c5*s6*Px0[i] - s5*Py0[i] - c5*c6*Pz0[i];
        dPz_dr[i] = s5*s6*Px0[i] + c5*Py0[i] - s5*c6*Pz0[i];

        dPy_dp[i] =  s5*c6*Px0[i] + s5*s6*Pz0[i];   
        dPz_dp[i] = -c5*c6*Px0[i] - c5*s6*Pz0[i];
        dPx_dp = -s6*Px0[i] + c6*Pz0[i];
    }

    mat2 J{};                   // result 2x2
    for(std::size_t i=0;i<2;++i)
    {
        /* shorthand */
        double x = d[i]  - Px[i];
        double y = Py[i];
        double z = Pz[i] - h[i];

        /* helper vectors u,v,w,R */
        double u = 2.0 * a[i] * y;
        double v = 2.0 * a[i] * z;
        double w = (a[i]*a[i] + x*x + y*y + z*z - l[i]*l[i]) * s[i];
        double R = std::hypot(u, v);

        /* γ = atan2(v,u)  — derivatives */
        double du_dr = 2.0 * a[i] * dPy_dr[i];
        double dv_dr = 2.0 * a[i] * dPz_dr[i];
        double du_dp = 2.0 * a[i] * dPy_dp[i];
        double dv_dp = 2.0 * a[i] * dPz_dp[i];

        double dgamma_dr = (u * dv_dr - v * du_dr) / (u*u + v*v); // 
        double dgamma_dp = (u * dv_dp - v * du_dp) / (u*u + v*v);

        /* s = w / R  — first derivatives (include sdir exactly once) */
        double dw_dr = (2.0*y*dPy_dr[i] + 2.0*z*dPz_dr[i]) * s[i];
        double dx_dp = -dPx_dp;
        double dw_dp = (2.0*x*dx_dp + 2.0*y*dPy_dp[i] + 2.0*z*dPz_dp[i]) * s[i];

        double dR_dr = (u*du_dr + v*dv_dr) / R;
        double dR_dp = (u*du_dp + v*dv_dp) / R;

        double ds_dr = (dw_dr*R - w*dR_dr) / (R*R);
        double ds_dp = (dw_dp*R - w*dR_dp) / (R*R);

        /* derivative of acos  :=  −ds / sqrt(1−s^2) */
        double root = std::sqrt(std::max(1e-14, 1.0 - (w/R)*(w/R)));
        double dacos_dr = -ds_dr / root;
        double dacos_dp = -ds_dp / root;

        J[i][0] = dgamma_dr + dacos_dr;   // \partial theta_m_i / \partial roll
        J[i][1] = dgamma_dp + dacos_dp;   // \partial theta_m_i / \partial pitch
    }
    return J;
}
/* ---------- Jacobian in motor order (swap rows right-leg) ------------- */
inline mat2 Jmotor(double roll,double pitch,bool left=true)
{
    return left ? jacobian(roll,pitch,LEFT)
                : mat2{ jacobian(roll,pitch,RIGHT)[1],
                        jacobian(roll,pitch,RIGHT)[0] };
}

/* ---------- velocity helpers ----------------------------------------- */
inline vec2 motorVelFromFoot(double r,double p,double rd,double pd,bool left=true)
{
    const mat2 J=Jmotor(r,p,left);
    return {J[0][0]*rd + J[0][1]*pd,
            J[1][0]*rd + J[1][1]*pd};
}
inline vec2 footVelFromMotor(double r,double p,double t5d,double t6d,bool left=true)
{
    const mat2 J=Jmotor(r,p,left);
    double det=J[0][0]*J[1][1]-J[0][1]*J[1][0];
    double inv=1.0/det;
    return { inv*( J[1][1]*t5d - J[0][1]*t6d),
             inv*(-J[1][0]*t5d + J[0][0]*t6d) };
}

/* ---------- torque helpers ------------------------------------------- */
inline vec2 ankleTorqueFromMotor(double r,double p,double tau5,double tau6,bool left=true)
{
    const mat2 J=Jmotor(r,p,left);
    return { J[0][0]*tau5 + J[1][0]*tau6,
             J[0][1]*tau5 + J[1][1]*tau6 };
}
inline vec2 motorTorqueFromAnkle(double r,double p,double tauR,double tauP,bool left=true)
{
    const mat2 J=Jmotor(r,p,left);
    double det=J[0][0]*J[1][1]-J[0][1]*J[1][0];
    mat2 Jinv{{ { J[1][1]/det, -J[0][1]/det },
                {-J[1][0]/det,  J[0][0]/det } }};
    return { Jinv[0][0]*tauR + Jinv[1][0]*tauP,
             Jinv[0][1]*tauR + Jinv[1][1]*tauP };
}
} // namespace 