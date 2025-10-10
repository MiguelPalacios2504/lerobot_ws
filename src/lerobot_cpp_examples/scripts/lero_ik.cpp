#include <iostream>
#include <cmath>
#include <cstring>
#include <vector>
#include <cstdio>
using namespace std;

// ---------- Parámetros DH (mm, rad) ----------
const double L0  = 54.2;
const double L1  = 30.5;
const double L2  = 116.1;
const double L3  = 134.5;
const double L45 = 160.0;

// ---------- LÍMITES ARTICULARES (° → rad) ----------
const double joint_min[5] = { -125, -35, -160, -90, -150 };
const double joint_max[5] = {  125, 200,   20,  90,  150 };

// ---------- Utilidades ----------
inline double rad(double deg){ return deg*M_PI/180.0; }
inline double deg(double r){ return r*180.0/M_PI; }
inline double wrap_pi(double a){
    a = fmod(a + M_PI, 2.0*M_PI);
    if (a < 0) a += 2.0*M_PI;
    return a - M_PI;
}

// ---------- Funciones de matrices ----------
void mat4_identity(double T[4][4]) {
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) T[i][j]=(i==j);
}

void mat4_mul(const double A[4][4], const double B[4][4], double C[4][4]){
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++){
            double s=0;
            for(int k=0;k<4;k++) s+=A[i][k]*B[k][j];
            C[i][j]=s;
        }
}

// DH estándar EXACTO como tu Python
void T_dh(double theta, double d, double a, double alpha, double T[4][4]){
    double ct=cos(theta), st=sin(theta);
    double ca=cos(alpha), sa=sin(alpha);
    T[0][0]=ct;   T[0][1]=-st*ca;  T[0][2]= st*sa;  T[0][3]=a*ct;
    T[1][0]=st;   T[1][1]= ct*ca;  T[1][2]=-ct*sa;  T[1][3]=a*st;
    T[2][0]=0;    T[2][1]=    sa;  T[2][2]=    ca;  T[2][3]=d;
    T[3][0]=0;    T[3][1]=      0; T[3][2]=      0; T[3][3]=1;
}

// === FK 5DOF exactamente como tu Python ===
void fk_5dof(const double q[5], double T05[4][4]){
    double T[4][4], A[4][4], Tmp[4][4];
    mat4_identity(T);

    T_dh(q[0],          L0,  L1,  M_PI/2, A); mat4_mul(T, A, Tmp); memcpy(T, Tmp, sizeof(Tmp));
    T_dh(q[1],           0,  L2,  0.0,     A); mat4_mul(T, A, Tmp); memcpy(T, Tmp, sizeof(Tmp));
    T_dh(q[2],           0,  L3,  0.0,     A); mat4_mul(T, A, Tmp); memcpy(T, Tmp, sizeof(Tmp));
    T_dh(q[3] - M_PI/2,  0,   0, -M_PI/2,  A); mat4_mul(T, A, Tmp); memcpy(T, Tmp, sizeof(Tmp));
    T_dh(q[4],        L45,   0,  0.0,     A); mat4_mul(T, A, Tmp); memcpy(T, Tmp, sizeof(Tmp));

    memcpy(T05, T, sizeof(double)*16);
}

void print_T(const string& name, const double T[4][4]){
    cout<<"\n"<<name<<":\n";
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++) printf("%10.4f ",T[i][j]);
        cout<<"\n";
    }
    printf("Posición: (X=%.1f, Y=%.1f, Z=%.1f) mm\n",T[0][3],T[1][3],T[2][3]);
}

// ---------- Jacobiano numérico SOLO POSICIÓN (3x5) ----------
void numerical_jacobian_pos(const double q[5], double J[3][5]){
    const double h=1e-6;
    double T0[4][4], T1[4][4];
    fk_5dof(q,T0);
    for(int i=0;i<5;i++){
        double qh[5]; memcpy(qh,q,sizeof(double)*5);
        qh[i]+=h;
        fk_5dof(qh,T1);
        for(int r=0;r<3;r++) J[r][i]=(T1[r][3]-T0[r][3])/h;
    }
}

// ---------- Clamp con límites físicos ----------
double clamp_with_limits(double q, int i) {
    double min_rad = rad(joint_min[i]);
    double max_rad = rad(joint_max[i]);
    if (q < min_rad) {
        printf("⚠️  Articulación %d saturada (%.1f° < min %.1f°)\n", i+1, deg(q), joint_min[i]);
        return min_rad;
    }
    if (q > max_rad) {
        printf("⚠️  Articulación %d saturada (%.1f° > max %.1f°)\n", i+1, deg(q), joint_max[i]);
        return max_rad;
    }
    return q;
}

// ---------- Solver IK (posición, DLS + límites) ----------
bool ik_solve_pos(const double T_target[4][4], const double q_init[5], double q_out[5]){
    double q[5]; memcpy(q,q_init,sizeof(double)*5);
    const double lambda=0.01;
    for(int it=0; it<300; ++it){
        double Tcur[4][4]; fk_5dof(q,Tcur);
        double e[3] = {
            T_target[0][3] - Tcur[0][3],
            T_target[1][3] - Tcur[1][3],
            T_target[2][3] - Tcur[2][3]
        };
        double err = sqrt(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]);
        if(err < 1e-3){ memcpy(q_out,q,sizeof(double)*5); return true; }

        double J[3][5]; numerical_jacobian_pos(q,J);

        // DLS: (J^T J + λ^2 I) dq = J^T e
        double JT[5][3]; for(int i=0;i<5;i++)for(int j=0;j<3;j++) JT[i][j]=J[j][i];
        double H[5][5]={0};
        for(int i=0;i<5;i++)
            for(int j=0;j<5;j++)
                for(int k=0;k<3;k++) H[i][j]+=JT[i][k]*J[k][j];
        for(int i=0;i<5;i++) H[i][i]+=lambda*lambda;
        double rhs[5]={0};
        for(int i=0;i<5;i++)
            for(int k=0;k<3;k++) rhs[i]+=JT[i][k]*e[k];

        // Resolver H * dq = rhs (Gauss-Jordan)
        double M[5][6];
        for(int i=0;i<5;i++){ for(int j=0;j<5;j++) M[i][j]=H[i][j]; M[i][5]=rhs[i]; }
        for(int c=0;c<5;c++){
            int piv=c; double best=fabs(M[c][c]);
            for(int r=c+1;r<5;r++){ double v=fabs(M[r][c]); if(v>best){best=v; piv=r;} }
            if (best<1e-12) continue;
            if (piv!=c) for(int j=c;j<=5;j++) swap(M[c][j],M[piv][j]);
            double pv=M[c][c];
            for(int j=c;j<=5;j++) M[c][j]/=pv;
            for(int r=0;r<5;r++){
                if(r==c) continue;
                double f=M[r][c];
                for(int j=c;j<=5;j++) M[r][j]-=f*M[c][j];
            }
        }
        double dq[5]; for(int i=0;i<5;i++) dq[i]=M[i][5];

        // Actualizar, normalizar y aplicar límites
        for(int i=0;i<5;i++){
            q[i] = wrap_pi(q[i] + dq[i]);
            q[i] = clamp_with_limits(q[i], i);
        }
    }
    memcpy(q_out,q,sizeof(double)*5);
    return false;
}

// ---------- MAIN (tests A y B como tu Python) ----------
int main(){
    cout<<"=== Prueba IK Piper 5DOF (con límites físicos) ===\n";

    double qA[5]={rad(90),rad(90),rad(0),rad(0),rad(0)};
    double qB[5]={rad(90),rad(90),rad(-90),rad(0),rad(0)};

    double TA[4][4], TB[4][4];
    fk_5dof(qA,TA);
    fk_5dof(qB,TB);

    print_T("FK A [90,90,0,0,0]",TA);
    print_T("FK B [90,90,-90,0,0]",TB);

    double q0[5]={rad(0),rad(10),rad(10),0,0};
    double qA_sol[5], qB_sol[5];

    bool okA = ik_solve_pos(TA,q0,qA_sol);
    bool okB = ik_solve_pos(TB,q0,qB_sol);

    auto print_sol=[&](const char* name,const double q[5],const double Ttar[4][4]){
        cout<<"\n"<<name<<"\n";
        cout<<"q_sol (deg) = [";
        for(int i=0;i<5;i++) cout<<deg(q[i])<<(i<4?", ":"");
        cout<<"]\n";
        double Tchk[4][4]; fk_5dof(q,Tchk);
        double ep = sqrt(pow(Tchk[0][3]-Ttar[0][3],2)+
                         pow(Tchk[1][3]-Ttar[1][3],2)+
                         pow(Tchk[2][3]-Ttar[2][3],2));
        printf("Error posición (mm): %.6f\n", ep);
        print_T("FK(q_sol)", Tchk);
    };

    print_sol("IK hacia A", qA_sol, TA);
    print_sol("IK hacia B", qB_sol, TB);
    
        // --- SALIDA SIMPLE PARA PYTHON ---
    printf("\nIK_RESULT ");
    for(int i=0;i<5;i++){
        printf("%.3f ", deg(qB_sol[i]));   // por ejemplo la solución hacia B
    }
    printf("\n");


    return 0;
}
