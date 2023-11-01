#pragma once 
#include <Eigen/Dense>
#include "slam_craft/math/S03.hpp"

#include <functional>
#include <iostream>
#include "slam_craft/defines.h"
#include "slam_craft/types/imu.h"
#include "slam_craft/modules/sc_module_base.h"
namespace SlamCraft
{
    class ESKF : private SCModuleBase
    {
    public:
        using Ptr = std::shared_ptr<ESKF>;
        struct State18
        {
            Eigen::Quaterniond rotation;
            Eigen::Vector3d position;
            Eigen::Vector3d velocity;
            Eigen::Vector3d bg;
            Eigen::Vector3d ba;
            Eigen::Vector3d gravity;
            State18(){
                rotation = Eigen::Quaterniond::Identity();
                position = Eigen::Vector3d::Zero();
                velocity = Eigen::Vector3d::Zero();
                bg = Eigen::Vector3d::Zero();
                ba = Eigen::Vector3d::Zero();
                gravity = Eigen::Vector3d::Zero();
            }
            friend std::ostream &operator<<( std::ostream &output, 
                                        const State18 &state18 )
            { 
                output<<"rotation: "<<state18.rotation.coeffs().transpose()
                    <<"\npostion: "<<state18.position.transpose()
                    <<"\nvelocity: "<<state18.velocity.transpose()
                    <<"\nbg: "<<state18.bg.transpose()
                    <<"\nba: "<<state18.ba.transpose()
                    <<"\ngrav: "<<state18.gravity.transpose()<<std::endl; ;
                return output;            
            }
        };
        class CalcZHInterface
        {
        public:
            virtual bool calculate(const State18&state,Eigen::MatrixXd & Z,Eigen::MatrixXd & H)=0;
        };

        std::shared_ptr<CalcZHInterface> calc_zh_ptr;

    private:
        State18 X,X_last;
        Eigen::Matrix<double,18,18> P;
        Eigen::Matrix<double,12,12> Q;
        int iter_times = 15;
        double abs_error_thre =  0.001;
        Eigen::Matrix3d  A_T(const Eigen::Vector3d& v){
            Eigen::Matrix3d res;
            double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            double norm = std::sqrt(squaredNorm);
            if(norm <1e-11){
                res = Eigen::Matrix3d::Identity();
            }
            else{
                res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skewSymmetric(v) + (1 - std::sin(norm) / norm) / squaredNorm * skewSymmetric(v) * skewSymmetric(v);
            }
            return res;
        }
        void initESKF();
    public:
        ESKF(std::string config_path= "",std::string prefix = "");

        ~ESKF();
        void set_iter_times(int n){
            iter_times = n;
        }
        struct State18 getX(){
            return X;
        }
        struct State18 getX_last(){
            return X_last;
        }
        void setX(const struct State18& in_x){
            X = in_x;
        }
        Eigen::Matrix<double,18,18> getP(){
            return P;
        }
        void setP(const Eigen::Matrix<double,18,18> &in_p){
            P = in_p;
        }
        Eigen::Matrix<double,12,12> getQ(){
            return Q;
        }
        void setQ(const Eigen::Matrix<double,12,12> &in_q){
            Q = in_q;
        }
        void predict(const SlamCraft::IMU & imu_data,double dt);
        
        bool update();
        static Eigen::Matrix<double,18,1> getErrorState18(const State18 &s1, const  State18 &s2){
            Eigen::Matrix<double,18,1> es;
            es.setZero();
            es.block<3,1>(0,0) = SlamCraft::SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix());
            es.block<3,1>(3,0) = s1.position - s2.position;
            es.block<3,1>(6,0) = s1.velocity - s2.velocity;
            es.block<3,1>(9,0) = s1.bg - s2.bg;
            es.block<3,1>(12,0) = s1.ba - s2.ba;
            es.block<3,1>(15,0) = s1.gravity - s2.gravity;
            return es;
        }
    };

    ESKF::ESKF(std::string config_path,std::string prefix):SCModuleBase(config_path,prefix,"Error State Kalman Filter"){initESKF();};

    ESKF::~ESKF()
    {
    }
    void ESKF::initESKF(){

        P.setIdentity();
        P(9,9)   = P(10,10) = P(11,11) = 0.0001;
        P(12,12) = P(13,13) = P(14,14) = 0.001;
        P(15,15) = P(16,16) = P(17,17) = 0.00001; 
        double cov_gyroscope,cov_acceleration,cov_bias_acceleration,cov_bias_gyroscope;
        readParam("cov_gyroscope",cov_gyroscope,0.1);
        readParam("cov_acceleration",cov_acceleration,0.1);
        readParam("cov_bias_acceleration",cov_bias_acceleration,0.1);
        readParam("cov_bias_gyroscope",cov_bias_gyroscope,0.1);
        Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d{cov_gyroscope,cov_gyroscope,cov_gyroscope};
        Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d{cov_acceleration,cov_acceleration,cov_acceleration};
        Q.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d{cov_bias_gyroscope,cov_bias_gyroscope,cov_bias_gyroscope};
        Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d{cov_bias_acceleration,cov_bias_acceleration,cov_bias_acceleration};
        X.ba.setZero();
        X.bg.setZero();
        X.gravity.setZero();
        X.position.setZero();
        X.rotation.setIdentity();
        X.velocity.setZero();
        printParamTable();
    }
    void ESKF::predict(const IMU & imu_data,double dt){
        auto imu = imu_data;
        imu.acceleration -= X.ba;
        imu.gyroscope -= X.bg;
        auto rotation = X.rotation.toRotationMatrix();
        X.rotation = Eigen::Quaterniond(X.rotation.toRotationMatrix()*so3Exp((imu.gyroscope)*dt));
        X.rotation.normalize();
        X.position += X.velocity*dt;
        X.velocity += (rotation*(imu.acceleration)+X.gravity)*dt;
        Eigen::Matrix<double,18,18> Fx;

        Eigen::Matrix<double,18 ,12>Fw;
        Fw.setZero();
        Fx.setIdentity();
        Fx.block<3,3>(0,0) = so3Exp(-1*imu.gyroscope*dt);

        Fx.block<3,3>(0,9) = -1*A_T(-imu.gyroscope*dt)*dt;

        Fx.block<3,3>(3,6) =  Eigen::Matrix3d::Identity()*dt;
        Fx.block<3,3>(6,0) = rotation*skewSymmetric(imu.acceleration)*dt*(-1);
        Fx.block<3,3>(6,12) = rotation*dt*(-1);
        Fx.block<3,3>(6,15) = Eigen::Matrix3d::Identity()*dt;
        Fw.block<3,3>(0,0) = -1*A_T(-imu.gyroscope*dt)*dt;
        Fw.block<3,3>(6,3) = -1*rotation*dt;
        Fw.block<3,3>(9,6) = Fw.block<3,3>(12,9) = Eigen::Matrix3d::Identity()*dt;
        P = Fx*P*Fx.transpose()+Fw*Q*Fw.transpose(); 

    }

    bool ESKF::update(){
        static int cnt_ = 0;
        auto x_k_k = X;
        auto x_k_last = X;

        Eigen::MatrixXd K;
        Eigen::MatrixXd H_k;
        Eigen::Matrix<double,18,18> P_in_update;
        bool converge =true;
        for (int i = 0; i < iter_times; i++)
        {

            Eigen::Matrix<double,18,1> error_state = getErrorState18(x_k_k,X);
            Eigen::Matrix<double,18,18> J_inv;

            J_inv.setIdentity();
            
            J_inv.block<3,3>(0,0) = A_T(error_state.block<3,1>(0,0));

            P_in_update = J_inv*P*J_inv.transpose();

            Eigen::MatrixXd z_k;
            
            Eigen::MatrixXd R_inv;
            calc_zh_ptr->calculate(x_k_k,z_k,H_k);
            Eigen::MatrixXd H_kt = H_k.transpose();
            K = (H_kt*H_k+(P_in_update/0.001).inverse()).inverse()*H_kt;
            Eigen::MatrixXd left = -1*K*z_k;

            Eigen::MatrixXd right = -1*(Eigen::Matrix<double,18,18>::Identity()-K*H_k)*J_inv*error_state; 

            Eigen::MatrixXd update_x = left+right;
            converge =true;
            for ( int idx = 0; idx < 18; idx++)
            {
                if (update_x(i,0)>0.001)
                {
                    
                    converge = false;
                    break;
                }
                
            }
            x_k_k.rotation = x_k_k.rotation.toRotationMatrix()*SlamCraft::so3Exp(update_x.block<3,1>(0,0));
            x_k_k.rotation.normalize();
            x_k_k.position = x_k_k.position+update_x.block<3,1>(3,0);
            x_k_k.velocity = x_k_k.velocity+update_x.block<3,1>(6,0);
            x_k_k.bg = x_k_k.bg+update_x.block<3,1>(9,0);
            x_k_k.ba = x_k_k.ba+update_x.block<3,1>(12,0);
            x_k_k.gravity = x_k_k.gravity+update_x.block<3,1>(15,0);
            if(converge){
                break;
            }
        }
        cnt_++;

        X_last = X;
        X = x_k_k;
        P = (Eigen::Matrix<double,18,18>::Identity()-K*H_k)*P_in_update;
        return converge;
    }
} // namespace SlamCraft