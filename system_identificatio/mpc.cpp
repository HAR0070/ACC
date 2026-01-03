// #include "osqp.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <Eigen/Eigenvalues> 

#include <iostream>
#include <math.h>
#include <array>

#include <yaml-cpp/yaml.h> 
#include <iostream>
#include <stdexcept> // Recommended for standard exceptions


using namespace Eigen; 

// Defining state space matrix and related identification functions
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
        float wx4;   // not weight - its windup error limit
        float wx5; 
        float wx6; 
    };

    struct models{
        MatrixXf A; 
        MatrixXf B; 
        MatrixXf C; 
        MatrixXf Q; 
        MatrixXf R;

        // default constructor while initializing
        models() {}

        models(int nx, int nu, int ny)
        : A(nx, nx), B(nx, nu), C(ny, nx) , Q(nx , nx) , R(nu , nu) {}
    };

    struct state_space{
        models accel;
        models brake;
    };

    struct LaguerreMatrices {
        float a;       // lagurre pole location
        int N;          // number of lagurre parameters
        int Nc;         // number of constrain horizon
        int n_in = 1;   // number of inputs
        int Np;

        MatrixXf L0;
        MatrixXf A1;
        MatrixXf Mu; 
        MatrixXf Mdu; 
        MatrixXf M;
        MatrixXf L0t;
        MatrixXf E;
        MatrixXf H;     // doesnt change with iteration 
        MatrixXf h;     // Changes every iteration 
        // MatrixXf Eta;   // doesnt change with iteration 
        MatrixXf eta;   // Changes every iteration 
    };

    StateSpaceModel::params p;
    StateSpaceModel::state_space ss_model;
    StateSpaceModel::LaguerreMatrices L_a;
    StateSpaceModel::LaguerreMatrices L_b;

    public:

    StateSpaceModel(){
        YAML::Node config  = YAML::LoadFile("params.yaml"); 

        try{
            p.DT        = config["time"]["DT"].as<float>(); 
            p.d0        = config["Dist"]["min_dist"].as<float>(); 
            p.Th        = config["time"]["headway_time"].as<float>(); 
            p.a_max     = config["acc"]["a_max"].as<float>(); 
            p.b_max     = config["brake"]["b_max"].as<float>(); 
            p.del_a_max = config["acc"]["del_a_max"].as<float>(); 
            p.del_b_max = config["break"]["del_b_max"].as<float>(); 
            p.t1        = config["acc"]["t1"].as<float>(); 
            p.k1        = config["acc"]["k1"].as<float>(); 
            p.k2        = config["brake"]["k2"].as<float>(); 
            p.t2        = config["brake"]["t2"].as<float>(); 
            p.ra        = config["weights"]["r_brake"].as<float>(); 
            p.rb        = config["weights"]["r_accel"].as<float>(); 
            p.debug     = config["debug"]["enable"].as<bool>();
            p.wx4       = config["windup"]["x4"].as<float>();
            p.wx5       = config["windup"]["x5"].as<float>();
            p.wx6       = config["windup"]["x6"].as<float>();
        }
        catch (const YAML::Exception& e) {
            std::cerr << "Error loading configuration: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE); 
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
        a_ss.B << 0 , 0 , -p.k1/p.t1 ; 
        a_ss.C = MatrixXf::Identity(3,3);

        b_ss.A <<  0 , 1, -p.Th ,
                    0 , 0 , -1, 
                    0 , 0 , -1/p.t2;
        b_ss.B << 0 , 0 , -p.k2/p.t2 ; 
        b_ss.C = MatrixXf::Identity(3,3);

        // ZOH Discretization
        MatrixXf M (4 , 4); 
        M.setZero();
        M.block(0,0,3,3) = a_ss.A * p.DT;
        M.block(0,3,3,1) = a_ss.B * p.DT; 
        
        MatrixXf Md = M.exp(); 
        a_ss.A = Md.block(0,0,3,3);
        a_ss.B = Md.block(0,3,3,1); 

        // Augmenting the system for integral error
        a_aug.A = MatrixXf::Identity(6,6);
        a_aug.B = MatrixXf::Zero(6,1); 
        a_aug.C = MatrixXf::Zero(3,6);

        a_aug.A.block(0,0,3,3) = a_ss.A;
        a_aug.A.block(3,0,3,3) = a_ss.C * a_ss.A; 

        a_aug.B.block(0,0,3,1) = a_ss.B; 
        a_aug.B.block(3,0,3,1) = a_ss.C * a_ss.B;

        a_aug.C.block(0,0,3,3) = a_ss.C; 

        a_aug.R << p.ra ; 
        a_aug.Q = a_aug.C.transpose() * a_aug.C;

        // Same for brake system
        // ZOH Discretization
        M.setZero();
        M.block(0,0,3,3) = b_ss.A * p.DT;
        M.block(0,3,3,1) = b_ss.B * p.DT; 
        
        Md = M.exp(); 
        b_ss.A = Md.block(0,0,3,3);
        b_ss.B = Md.block(0,3,3,1); 

        // Augmenting the system for integral error
        b_aug.A = MatrixXf::Identity(6,6);
        b_aug.B = MatrixXf::Zero(6,1); 
        b_aug.C = MatrixXf::Zero(3,6);

        b_aug.A.block(0,0,3,3) = b_ss.A;
        b_aug.A.block(3,0,3,3) = b_ss.C * b_ss.A; 

        b_aug.B.block(0,0,3,1) = b_ss.B; 
        b_aug.B.block(3,0,3,1) = b_ss.C * b_ss.B;

        b_aug.C.block(0,0,3,3) = b_ss.C; 
        b_aug.R << p.rb ; 
        b_aug.Q = b_aug.C.transpose() * b_aug.C;

        ss_model.accel = a_aug;
        ss_model.brake = b_aug;    

        if (p.debug){
            std::cout << "Acceleration Model A: \n" << ss_model.accel.A << "\n B: \n" << ss_model.accel.B << "\n C: \n" << ss_model.accel.C << std::endl;
            std::cout << "Brake Model A: \n" << ss_model.brake.A << "\n B: \n" << ss_model.brake.B << "\n C: \n" << ss_model.brake.C << std::endl;

            compute_eigenvalues(ss_model.accel.A);
            check_controllability(ss_model.accel.A , ss_model.accel.B);
            check_observability(ss_model.accel.A , ss_model.accel.C);

            compute_eigenvalues(ss_model.brake.A);
            check_controllability(ss_model.brake.A , ss_model.brake.B);
            check_observability(ss_model.brake.A , ss_model.brake.C);

        }

    }

    void compute_eigenvalues(const MatrixXf &A) {
        EigenSolver<MatrixXf> es(A);
        VectorXcf eigvals = es.eigenvalues();

        std::cout << "Eigenvalues of the system matrix A:\n";
        for (int i = 0; i < eigvals.size(); ++i) {
            std::cout << eigvals[i] << std::endl;
        }
    }

    void check_observability( const MatrixXf &A , const MatrixXf &C) {
        int n = A.rows(); 
        MatrixXf Ob( n* C.rows() , n); 

        Ob.block(0 , 0 ,C.rows() , n) = C;

        for (int i =1; i<n; ++i){
            Ob.block(i*C.rows() , 0 ,C.rows() , n) = Ob.block((i-1)*C.rows(), 0 ,C.rows() , n)*A; 
        }

        FullPivLU<MatrixXf> lu_decomp(Ob);
        int rank = lu_decomp.rank(); 

        if (rank == n){
            std::cout << "The system is observable." << std::endl;
        }
        else{
            std::cout << "The system is not observable." << "Rank: " << rank << " , States: " << n << std::endl;
            std::cout << "Observability Matrix: \n" << Ob << "its LU decomposition: \n" << lu_decomp.matrixLU() << std::endl;
        }
    }

    void check_controllability( const MatrixXf &A , MatrixXf &B){

        int n = A.rows(); 
        MatrixXf Co ( n , n*B.cols()); 

        for (int i =0; i<n; i++){
        Co.block(0 , i*B.cols() , n , B.cols()) = A.pow(i) * B;
        }

        FullPivLU<MatrixXf> lu_decomp(Co); 
        int rank = lu_decomp.rank();

        if (rank == n){
            std::cout << "The system is Controllable." << std::endl;
        }
        else{
            std::cout << "The system is not Controllable." << "Rank: " << rank << " , States: " << n << std::endl;
            std::cout << "Controllable Matrix: \n" << Co << "its LU decomposition: \n" << lu_decomp.matrixLU() << std::endl;
        }
    }

    void minimal_realisation( MatrixXf &A , MatrixXf &B , MatrixXf &C){
        // To be implemented -- if the model is not controllable or observable
        // Its slightly complex -- will do if req later
    
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

        // A1 matrix
        L.A1 = a* MatrixXf::Identity(N,N);
        for (int i = 1; i < N; i++){
            L.A1.block(i,i-1 , N-i , 1) = L.L0.block(0,0, N-i, 1); 
        }

        if (p.debug){
            std::cout << "Laguerre get_l0 called for type:" << type <<std::endl;
            std::cout << "Laguerre L0 matrix: \n" << L.L0 << std::endl;
            std::cout << "Laguerre A1 matrix: \n" << L.A1 << std::endl;
        }

    }

    void get_M(const std::string &type){
        /* 
        M is the data matrix for imposing constraints on the ﬁrst Nc samples on Δu(ki ). 
        The block matrix Lzerot is used for constructing the controlsignal.

        First call get_L0 to initialize L0 and A1
        then call this function to compute Mdu and Mu matrices
        */

        LaguerreMatrices& L = (type=="accel") ? L_a : L_b;

        int n_in =1; // number of inputs 
        int N = L.N; 
        int Nc = L.Nc;

        MatrixXf M (n_in , N*Nc);  M.setZero();
        MatrixXf M2 (n_in , N*Nc);  M2.setZero();
        MatrixXf Ms (n_in , N);  Ms.setZero();

        // for loop not req since n_in =1 
        L.L0t = L.L0.transpose();

        for (int k =0 ; k<Nc ; k++){
            MatrixXf temp = L.A1.pow(k)*L.L0;
            Ms = Ms + temp.transpose();
            M.block(0,k*N,1,N) = temp.transpose(); 
            M2.block(0,k*N,1,N) = Ms;
        }

        L.Mdu = M;
        L.Mu = M2;

        L.M = MatrixXf(4*M.rows() , M.cols());
        L.M << M, -M , M2 , -M2;

        if (p.debug){
            std::cout << "Laguerre Mdu matrix: \n" << L.Mdu.rows() << " " << L.Mdu.cols() << std::endl;
            std::cout << "Laguerre Mu matrix: \n" << L.Mu.rows() << " " << L.Mu.cols() << std::endl;
        }
    }

    void dmpc( const std::string & type ){
        /* 
        A_e;B_e define the extended state-space model when
        integrator is used
        %they can also be other forms of state-space models
        a contains the Laguerre pole locations for each input
        N the number of terms for each input
        Np prediction horizon
        Q weight on the state variables
        R weight on the input variables assumed to be diagonal.
        The cost function is J= eta ^T E eta +2 eta ^T H x(k_i)   -- Here E is the hessian -- Sorry for the naming 
        */

        LaguerreMatrices& L = (type=="accel") ? L_a : L_b;
        models& ss = (type=="accel") ? ss_model.accel : ss_model.brake;

        int N = L.N;        // The dim of eta
        int n_in = L.n_in, n = ss.B.rows();       

        MatrixXf E(N , N); E.setZero(); 
        MatrixXf H(N , n); H.setZero(); 

        MatrixXf Rpa = MatrixXf::Identity(N,N)*ss.R(0,0);  //  extended weight matrix for input

        MatrixXf S_in(n , N); S_in.setZero(); 
        MatrixXf S_sum(n , N); S_sum.setZero(); 

        S_in.block(0,0,n,N) = ss.B * L.L0.transpose() ; 

        E = S_in.transpose()*ss.Q*S_in; 
        H = S_in.transpose()*ss.Q*ss.A; 

        MatrixXf A1_pow = L.A1;
        for(int i = 1 ; i<L.Np ; i++){

            S_sum = ss.A*S_sum + S_in*(A1_pow.transpose()) ;
            A1_pow = A1_pow*L.A1;
            E = E + S_sum.transpose()*ss.Q*S_sum ; 
            H = H + S_sum.transpose()*ss.Q*(ss.A.pow(i)); 
        }
        E = E + Rpa; 

        L.E = E ;
        L.H = H ; 

    }

    void QPHild(const std::string &type , const MatrixXf &b){

        /*
        J= eta ^T E eta +2 eta ^T H x(k_i)
        A_const * eta <= b 

        E , H
        l.h = H*X0  or H* x(k_i)
        A_const = l.M  // full constrain matrix
        gamma=b;

        */

        LaguerreMatrices& l = (type=="accel") ? L_a : L_b;

        int n1 = l.M.rows() , m1 = l.M.cols(); 

        l.eta = -l.E.colPivHouseholderQr().solve(l.h);          // Unconstrained solution
        int k =0 ; 
        for(int i = 0; i< n1 ; i++){
            if ((l.M*l.eta)(0,0) > b(i,0)) k++;
        }

        if (k != 0) {
            
        MatrixXf P = l.M*(l.E.colPivHouseholderQr().solve(l.M.transpose()));
        MatrixXf d = l.M*(-1*l.eta) + b;
        int n = d.rows() , m = d.cols();

        MatrixXf lambda (n ,m); lambda.setZero();
        MatrixXf lambda_p (n ,m);

        int al = 10; 
        int iters = 40; 
        /* 
        find the elements in the solution vector one by one
        km could be larger if the Lagranger multiplier has a slow convergence rate.
        */

        for (int km = 0; km < iters; km++){
            lambda_p = lambda;

            for (int i =0; i< n; i++){
                float w = (P.row(i)*lambda)(0,0) - P(i,i)*lambda(i,0) + d(i,0); 
                float la = -w / P(i,i);
                lambda(i,0) = std::max(0.0f , la); 
            }

            al = (lambda - lambda_p).norm();
            if (al < 10e-8) break;
        } 

        l.eta = l.eta -l.E.colPivHouseholderQr().solve(l.M.transpose())*lambda; 

        }
        else{
            std::cout << "There is no Active constrain" << std::endl;
            // there is no change in eta - dmpc already gave global minima
        }


    }

    void init_controller(const std::vector<float> a , const std::vector<int> N , const std::vector<int> Nc ,  const std::vector<int> Np){
        // Intended use -- while initilizing class all the parameters will be called from yaml and state space model will be made

        // make the laguerre L0 for both accel and brake - based on N and a
        // get constrain matrix - no additional params req
        // make the dmpc matrix - for accel and brake each 

        make_ss();      // make state space for both accel and brake - parameters are in yaml file

        // accel init 
        if(p.debug){
            std::cout<<" accel params " << a[0] << " " << N[0] << " " << Nc[0] << " " << Np[0] << std::endl;
            std::cout<<" brake params " << a[1] << " " << N[1] << " " << Nc[1] << " "  << Np[0] << std::endl;
            std::cout<<" make sure the integral error is feed into the state space" << std::endl;
        }

        std::vector<std::string> mode = {"accel" , "brake"}; 

        for(int i = 0 ; i < 2; i++){
            get_L0(N[i], a[i], Nc[i] , Np[i] , mode[i]); 
            get_M(mode[i]); 
            dmpc(mode[i]);
        }

    }
    
    float mpc_constrained(std::vector<float> fb){
    
    // fb - [0] = u of previous step and the rest - x0 states
    float u = fb[0]; 
    MatrixXf X0 (6,1);
    LaguerreMatrices* l;

    // Integral error 
    if (fb[4] > p.wx4) fb[4] = p.wx4; 
    if (fb[5] > p.wx5) fb[5] = p.wx5; 
    if (fb[6] > p.wx6) fb[6] = p.wx6; 

    X0 << fb[1] , fb[2] , fb[3] , fb[4] , fb[5], fb[6];   // Reference is 0 - we still need integrator because model errs 
    // Error integrator will be done at user end of this code

    if (u > 0) {
        l = &L_a; 
        std::string mode = "accel" ;
        MatrixXf gamma (l->Nc*4 , 1); 
        MatrixXf ones = MatrixXf::Ones(l->Nc , 1); 
        l->h = l->H*X0; 
        gamma << ones*p.del_a_max , ones*(-p.del_a_max) , ones*(p.a_max - u) , ones*(-p.a_max + u);
        QPHild( mode , gamma );
    }
    else {
        l = &L_b;
        std::string mode = "brake" ;
        MatrixXf gamma (l->Nc*4 , 1); 
        MatrixXf ones = MatrixXf::Ones(l->Nc , 1); 
        l->h = l->H*X0; 
        gamma << ones*p.del_b_max , ones*(-p.del_b_max) , ones*(p.b_max - u) , ones*(-p.b_max +u);
        QPHild( mode , gamma );
    } 

    float del_u = (l->L0t*l->eta)(0,0); 

    return u + del_u; 

    }

};
