#include <osqp/osqp.h>

#include <ros/ros.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>

#include <iostream>
#include <math.h>
#include <array>
#include <tuple>

#include <iostream>
#include <stdexcept> 

#pragma once

using namespace Eigen;

class StateSpaceModel
{
    private:

    struct params{
        float DT;
        float d0;
        float Th;
        float a_max;
        float b_max;
        float del_a_max;
        float del_b_max;
        float t1;
        float k1;
        float k2;
        float t2;
        bool debug;
        float ra;
        float rb;
        float q1;   
        float q2;
        float q3;
    };

    struct models{
        MatrixXf A;
        MatrixXf B;
        MatrixXf C;
        MatrixXf Q;
        MatrixXf R;

        models() {}

        models(int nx, int nu, int ny)
        : A(nx, nx), B(nx, nu), C(ny, nx) , Q(nx , nx) , R(nu , nu) {}
    };

    struct state_space{
        models accel;
        models brake;
    };

    struct LaguerreMatrices {
        float a;       
        int N;          
        int Nc;         
        int n_in = 1;   
        int Np;

        MatrixXf L0;
        MatrixXf A1;
        MatrixXf Mu;
        MatrixXf Mdu;
        MatrixXf M;
        MatrixXf L0t;
        MatrixXf E;
        MatrixXf H;     
        MatrixXf h;     
        MatrixXf eta;   
    };

    // Renamed from OSQPData to avoid clashing with native OSQP struct
    struct SparseMatrixData {
        std::vector<c_float> val; 
        std::vector<c_int>   row; 
        std::vector<c_int>   col; 
        c_int nnz;
    };

    StateSpaceModel::params p;
    StateSpaceModel::state_space ss_model;
    StateSpaceModel::LaguerreMatrices L_a;
    StateSpaceModel::LaguerreMatrices L_b;

    public:

    StateSpaceModel(){
        ros::NodeHandle nh;

        // Fetching parameters directly from the ROS Parameter Server
        // Using nh.param() provides a safe default value (from your yaml) just in case the server misses a key
        nh.param("/acc_node/time/DT", p.DT, 0.1f);
        nh.param("/acc_node/Dist/min_dist", p.d0, 2.0f);
        nh.param("/acc_node/model/thw", p.Th, 1.0f);

        nh.param("/acc_node/acc/a_max", p.a_max, 1.0f);
        nh.param("/acc_node/brake/b_max", p.b_max, 1.0f);

        nh.param("/acc_node/acc/del_a_max", p.del_a_max, 0.1f);
        nh.param("/acc_node/brake/del_b_max", p.del_b_max, 0.1f);

        nh.param("/acc_node/acc/t1", p.t1, 0.2f);
        nh.param("/acc_node/acc/k1", p.k1, 1.0f);
        nh.param("/acc_node/brake/k2", p.k2, 1.0f);
        nh.param("/acc_node/brake/t2", p.t2, 0.2f);

        nh.param("/acc_node/weights/r_brake", p.ra, 1.0f);
        nh.param("/acc_node/weights/r_accel", p.rb, 1.0f);
        nh.param("/acc_node/weights/q1", p.q1, 1.2f);
        nh.param("/acc_node/weights/q2", p.q2, 0.5f);
        nh.param("/acc_node/weights/q3", p.q3, 1.5f);

        nh.param("/acc_node/debug", p.debug, true);

        if(p.debug) {
            ROS_INFO("MPC parameters successfully loaded from ROS Parameter Server.");
        }
    }

    void make_ss(){
        StateSpaceModel::models a_ss( 3 , 1 , 3);
        StateSpaceModel::models a_aug( 6 , 1 , 3);
        StateSpaceModel::models b_ss( 3 , 1 , 3);
        StateSpaceModel::models b_aug( 6 , 1 , 3);

        a_ss.A <<  0 , 1, -p.Th ,
                    0 , 0 , -1,
                    0 , 0 , -1/p.t1;
        a_ss.B << 0 , 0 , p.k1/p.t1 ;
        a_ss.C = MatrixXf::Identity(3,3);

        b_ss.A <<  0 , 1, -p.Th ,
                    0 , 0 , -1,
                    0 , 0 , -1/p.t2;
        b_ss.B << 0 , 0 , p.k2/p.t2 ;
        b_ss.C = MatrixXf::Identity(3,3);

        DiagonalMatrix<float,3> q (p.q1 , p.q2 , p.q3);
        a_ss.Q = q;
        b_ss.Q = q;

        MatrixXf M (4 , 4);
        M.setZero();
        M.block(0,0,3,3) = a_ss.A * p.DT;
        M.block(0,3,3,1) = a_ss.B * p.DT;

        MatrixXf Md = M.exp();
        a_ss.A = Md.block(0,0,3,3);
        a_ss.B = Md.block(0,3,3,1);

        a_aug.A = MatrixXf::Identity(6,6);
        a_aug.B = MatrixXf::Zero(6,1);
        a_aug.C = MatrixXf::Zero(3,6);

        a_aug.A.block(0,0,3,3) = a_ss.A;
        a_aug.A.block(3,0,3,3) = a_ss.C * a_ss.A;
        a_aug.B.block(0,0,3,1) = a_ss.B;
        a_aug.B.block(3,0,3,1) = a_ss.C * a_ss.B;
        a_aug.C.block(0,3,3,3) = a_ss.C;

        a_aug.R << p.ra ;
        a_aug.Q = a_aug.C.transpose()* a_ss.Q * a_aug.C;

        M.setZero();
        M.block(0,0,3,3) = b_ss.A * p.DT;
        M.block(0,3,3,1) = b_ss.B * p.DT;

        Md = M.exp();
        b_ss.A = Md.block(0,0,3,3);
        b_ss.B = Md.block(0,3,3,1);

        b_aug.A = MatrixXf::Identity(6,6);
        b_aug.B = MatrixXf::Zero(6,1);
        b_aug.C = MatrixXf::Zero(3,6);

        b_aug.A.block(0,0,3,3) = b_ss.A;
        b_aug.A.block(3,0,3,3) = b_ss.C * b_ss.A;
        b_aug.B.block(0,0,3,1) = b_ss.B;
        b_aug.B.block(3,0,3,1) = b_ss.C * b_ss.B;
        b_aug.C.block(0,3,3,3) = b_ss.C;

        b_aug.R << p.rb ;
        b_aug.Q = b_aug.C.transpose() * b_ss.Q * b_aug.C;

        ss_model.accel = a_aug;
        ss_model.brake = b_aug;
    }

    void get_L0( int N , float a , float Nc , float Np , const std::string &type) {
        LaguerreMatrices& L = (type=="accel") ? L_a : L_b;

        L.L0 = MatrixXf::Zero(N,1);
        L.a = a;
        L.N = N;
        L.Nc = Nc;
        L.Np = Np;

        for ( int i = 0; i <N; i++){
            L.L0(i,0) = pow(-a , i);
        }
        L.L0 = sqrt(1- a*a) * L.L0;

        // FIX: The A1 matrix subdiagonals must use (1 - a^2), NOT sqrt(1 - a^2)
        L.A1 = a * MatrixXf::Identity(N, N);
        MatrixXf Al_col = MatrixXf::Zero(N, 1);
        for (int i = 0; i < N; i++) {
            Al_col(i, 0) = pow(-a, i) * (1 - a * a);
        }
        for (int j = 0; j < N - 1; j++) {
            L.A1.block(j + 1, j, N - 1 - j, 1) = Al_col.block(0, 0, N - 1 - j, 1);
        }
    }

    void dmpc( const std::string & type ){
        LaguerreMatrices& L = (type=="accel") ? L_a : L_b;
        models& ss = (type=="accel") ? ss_model.accel : ss_model.brake;

        int N = L.N;        
        int n = ss.B.rows();

        MatrixXf E(N , N); E.setZero();
        MatrixXf H(N , n); H.setZero();

        MatrixXf Rpa = MatrixXf::Identity(N,N)*ss.R(0,0);  

        // FIX: Correct Convolution sequence for Prediction Matrices
        MatrixXf Phi = ss.B * L.L0.transpose();
        
        E = Phi.transpose() * ss.Q * Phi;
        H = Phi.transpose() * ss.Q * ss.A;

        MatrixXf A1_pow = L.A1;
        for(int i = 1 ; i < L.Np ; i++){
            // Properly accumulate the A * Phi term
            Phi = ss.A * Phi + (ss.B * L.L0.transpose()) * A1_pow.transpose();
            E = E + Phi.transpose() * ss.Q * Phi;
            H = H + Phi.transpose() * ss.Q * ss.A.pow(i + 1);
            A1_pow = A1_pow * L.A1;
        }
        E = E + Rpa;

        L.E = E ;
        L.H = H ;
    }

    void get_M(const std::string &type){
        LaguerreMatrices& L = (type=="accel") ? L_a : L_b;

        int n_in =1; 
        int N = L.N;
        int Nc = L.Nc;

        MatrixXf M (Nc , N);  M.setZero();
        MatrixXf M2 (Nc , N);  M2.setZero();
        MatrixXf Ms (n_in , N);  Ms.setZero();

        L.L0t = L.L0.transpose();

        for (int k =0 ; k<Nc ; k++){
            MatrixXf temp = L.A1.pow(k)*L.L0;
            Ms = Ms + temp.transpose();
            M.block(k,0,1,N) = temp.transpose();
            M2.block(k,0,1,N) = Ms;
        }

        L.Mdu = M;
        L.Mu = M2;

        L.M = MatrixXf(4*M.rows() , M.cols());
        L.M << M, -M , M2 , -M2;
    }

    SparseMatrixData eigenToOSQP(const MatrixXf& mat, bool extractUpperTriangular = false) {
        SparseMatrixData data;

        // Cast to c_float (double) required by OSQP 0.6.3
        Matrix<c_float, Dynamic, Dynamic> mat_double = mat.cast<c_float>();          
        SparseMatrix<c_float> sparse_mat;     
        
        if (extractUpperTriangular) {
            Matrix<c_float, Dynamic, Dynamic> temp = mat_double.triangularView<Upper>();
            sparse_mat = temp.sparseView();
        } else {
            sparse_mat = mat_double.sparseView();
        }

        sparse_mat.makeCompressed();                        

        data.nnz = (c_int)sparse_mat.nonZeros();           
        data.val.resize(data.nnz);
        data.row.resize(data.nnz);
        data.col.resize(sparse_mat.outerSize() + 1);

        for (int i = 0; i < data.nnz; ++i) {
            data.val[i] = sparse_mat.valuePtr()[i];
            data.row[i] = (c_int)sparse_mat.innerIndexPtr()[i];
        }

        for (int i = 0; i < sparse_mat.outerSize() + 1; ++i) {
            data.col[i] = (c_int)sparse_mat.outerIndexPtr()[i];
        }

        return data;
    }

    int OSQP(const std::string &type , const VectorXd &low , const VectorXd &up){
        LaguerreMatrices &L = (type == "accel") ? L_a : L_b;

        c_int n = L.N; 
        c_int m = 2*L.Nc; 

        SparseMatrixData P_data = eigenToOSQP(L.E, true);  

        MatrixXf M_mat (m , L.Mu.cols());
        M_mat << L.Mdu , L.Mu;
        SparseMatrixData A_data = eigenToOSQP(M_mat, false); 

        std::vector<c_float> q_vec(L.h.size());
        for(int i = 0; i < L.h.size(); ++i) q_vec[i] = (c_float)L.h(i);

        std::vector<c_float> l_vec(low.size());
        for(int i = 0; i < low.size(); ++i) l_vec[i] = (c_float)low(i);

        std::vector<c_float> u_vec(up.size());
        for(int i = 0; i < up.size(); ++i) u_vec[i] = (c_float)up(i);

        OSQPWorkspace *work = nullptr;
        OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
        OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

        if (settings) {
            osqp_set_default_settings(settings);
            settings->alpha = 1.0;
            settings->verbose = 0; 
        }

        if (data) {
            data->n = n;
            data->m = m;
            data->P = csc_matrix(data->n, data->n, P_data.nnz, P_data.val.data(), P_data.row.data(), P_data.col.data());
            data->q = q_vec.data();
            data->A = csc_matrix(data->m, data->n, A_data.nnz, A_data.val.data(), A_data.row.data(), A_data.col.data());
            data->l = l_vec.data();
            data->u = u_vec.data();
        }

        c_int setup_flag = osqp_setup(&work, data, settings);
        c_int solve_status = -99; // Default error state

        if (setup_flag == 0) {
            osqp_solve(work);
            solve_status = work->info->status_val; // CAPTURE ACTUAL SOLVER STATUS

            // 1 = OSQP_SOLVED, 2 = OSQP_SOLVED_INACCURATE
            if (solve_status == 1 || solve_status == 2) {
                L.eta = MatrixXf::Zero(n, 1);
                for (int i = 0; i < n; i++) {
                    L.eta(i, 0) = (float)work->solution->x[i];
                }
            } else {
                if(p.debug) std::cerr << "OSQP Failed with status: " << solve_status << std::endl;
            }
        }

        osqp_cleanup(work);
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        if (data) c_free(data);
        if (settings) c_free(settings);

        return (int)solve_status;
    }


    void init_controller(const std::vector<float> a , const std::vector<int> N , const std::vector<int> Nc ,  const std::vector<int> Np){
        make_ss();     
        std::vector<std::string> mode = {"accel" , "brake"};

        for(int i = 0 ; i < 2; i++){
            get_L0(N[i], a[i], Nc[i] , Np[i] , mode[i]);
            get_M(mode[i]);
            dmpc(mode[i]);
        }
    }

    // Notice we now return an int at the end of the tuple for the exitflag
    std::tuple<std::string, float, int> mpc_osqp(std::vector<float> fb){

        float u = fb[0];
        MatrixXf X0 (6,1);
        LaguerreMatrices* l;
        int exitflag;
        float del_u = 0;
        std::string mode;

        X0 << fb[1] , fb[2] , fb[3] , fb[4] , fb[5], fb[6];

        if (u >= 0) {
            l = &L_a;
            mode = "accel" ;
            VectorXd low (l->Nc*2);
            VectorXd up (l->Nc*2);
            VectorXd ones = VectorXd::Ones(l->Nc);
            l->h = l->H*X0;
            
            // ALLOW U TO SLIP SLIGHTLY NEGATIVE TO TRIGGER BRAKE MODE NEXT FRAME
            low << -ones*p.del_a_max , ones*(-u - 0.1f); 
            up << ones*p.del_a_max , ones*(p.a_max - u);
            exitflag = OSQP( mode , low , up );
        }
        else {
            l = &L_b;
            mode = "brake" ;
            VectorXd low (l->Nc*2);
            VectorXd up (l->Nc*2);
            VectorXd ones = VectorXd::Ones(l->Nc);
            l->h = l->H*X0;
            
            // ALLOW U TO SLIP SLIGHTLY POSITIVE TO TRIGGER ACCEL MODE NEXT FRAME
            low << -ones*p.del_b_max , ones*(-p.b_max - u);
            up << ones*p.del_b_max , ones*(-u + 0.1f);
            exitflag = OSQP( mode , low , up );
        }

        // Apply control ONLY if solved successfully
        if (exitflag == 1 || exitflag == 2) del_u = (l->L0t*l->eta)(0,0);

        return std::make_tuple(mode, u + del_u, exitflag);
    }

};