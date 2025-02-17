//includo le librerie necessarie per ROS2 e per la gestione della cinematica inversa
#include <rclcpp/rclcpp.hpp>  //libreria Ros per creare i nodi
#include <geometry_msgs/msg/pose.hpp> //messaggio per posizioni e orientamenti
#include <eigen3/Eigen/Dense> //libreria per algebra lineare 
#include <iostream>
#include <cmath>  


using namespace std;
/*timer*/
using namespace std::chrono_literals;  //permette di usare 500ms come tempo

/*creo una classe che estende rclcpp::Node*/
class RobotArmControl : public rclcpp::Node{
    public:
        /*costruttore*/
        RobotArmControl(): Node("robot_arm_control"){
            //creo un publisher per inviare la posizione del braccio sul topic arm position
            publisher_=this->create_publisher<geometry_msgs::msg::Pose>("arm_position",10);

            //creo un timer che mi esegue la funzione update_position ogni 500 ms
            timer_=this->create_wall_timer(500ms,bind(&RobotArmControl::update_position,this));

            //il timer si ferma dopo 5s
            stop_timer_=this->create_wall_timer(5s,bind(&RobotArmControl::stop_node,this));
            
            /*inizializzo i parametri della lunghezza del braccio*/
            l1=0.5;
            l2=0.5;
            l3=0.5;

            //stampa messaggio per indicare che il nodo è stato avviato
            RCLCPP_INFO(this->get_logger(), "Nodo RobotArmControl avviato");

        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_; //invia poszione del braccio al topic
        rclcpp::TimerBase::SharedPtr timer_; //chiama la funziona update position
        rclcpp::TimerBase::SharedPtr stop_timer_;// ferma il nodo dopo 5 secondi
         
        //lunghezze dei bracci
        double l1,l2,l3;

        //funzione per calcolare la jacobiana per un braccio con 3 joints
        void calculate_jacobian(const Eigen::Vector3d& joint_angles, Eigen::MatrixXd& J){
            
            double theta1 = joint_angles[0];
            double theta2 = joint_angles[1];
            double theta3 = joint_angles[2];
   
            // Log per vedere i valori degli angoli
            RCLCPP_INFO(this->get_logger(), "Angoli giunti: theta1=%.2f, theta2=%.2f, theta3=%.2f",
                        static_cast<double>(theta1), static_cast<double>(theta2), static_cast<double>(theta3));
        
            // Log per le lunghezze dei bracci
            RCLCPP_INFO(this->get_logger(), "Lunghezze: l1=%.2f, l2=%.2f, l3=%.2f", l1, l2, l3);
        
            //calcolo della jacobiana
            J(0,0)=-l1*sin(theta1)-l2*sin(theta1+theta2)-l3*sin(theta1+theta2+theta3);
            J(0,1)=-l2*sin(theta1+theta2)-l3*sin(theta1+theta2+theta3);
            J(0,2)=-l3*sin(theta1+theta2+theta3);
            
            J(1,0)=l1*cos(theta1)+l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3);
            J(1,1)=l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3);
            J(1,2)=l3*cos(theta1+theta2+theta3);


            // controllo se jac da valori nulli
            if (J.array().isNaN().any() || !J.array().isFinite().all()) {
                RCLCPP_ERROR(this->get_logger(), "La Jacobiana contiene NaN o Inf, interrompendo il calcolo.");
                return;  // Esci dalla funzione in caso di NaN o Inf
            }
            //verifica se la jacobiana è singolare
            if (J.determinant() < 1e-6) {
                RCLCPP_ERROR(this->get_logger(), "La Jacobiana è singolare, interrotto il calcolo.");
                return;
            }
        

            RCLCPP_INFO(this->get_logger(), "Jacobiana:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f",
            static_cast<double>(J(0, 0)), static_cast<double>(J(0, 1)), static_cast<double>(J(0, 2)),
            static_cast<double>(J(1, 0)), static_cast<double>(J(1, 1)), static_cast<double>(J(1, 2)));
        }

        /*FUNZIONE PER CALCOLARE LA PSEUDO-INVERSA DELLA JACOBIANA*/
        Eigen::MatrixXd pseudo_inverse(Eigen::MatrixXd J, double reg_param = 1e-6) {
            Eigen::MatrixXd Jt = J.transpose();
            Eigen::MatrixXd JtJ = Jt * J;
            
            // Aggiungi regolarizzazione alla matrice Jacobiana per evitare singolarità numeriche
            Eigen::MatrixXd JtJ_inv = (JtJ + reg_param * Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols())).inverse();
            return JtJ_inv * Jt;
        }
        

        Eigen::Vector3d forward_kinematics(const Eigen::Vector3d& joint_angles) {
        
            double x = l1 * cos(joint_angles[0]) + l2 * cos(joint_angles[0] + joint_angles[1]) + l3 * cos(joint_angles[0] + joint_angles[1] + joint_angles[2]);
            double y = l1 * sin(joint_angles[0]) + l2 * sin(joint_angles[0] + joint_angles[1]) + l3 * sin(joint_angles[0] + joint_angles[1] + joint_angles[2]);
            double z = 0; // Supponiamo che sia un braccio planare
        
            return Eigen::Vector3d(x, y, z);
        }
        
        /*INVERSE KINEMATICS*/
        //dato un punto nello spazio "target" mi deve restituire gli angoli necessari per raggiungerlo
        Eigen::Vector3d calculate_inverse_kinematics(const Eigen::Vector3d target){
            Eigen::MatrixXd J(3,3);  //jacobiana 3x3
            Eigen::Vector3d joint_angles(-0.785, 0.785, 0.785);
            Eigen::Vector3d current_position = forward_kinematics(joint_angles);  //angoli iniziali  
            
            //itero per calcolare gli angoli dei joints
            for(int i=0;i<1000;i++){
                Eigen::Vector3d position_error=target-current_position;  //calcolo l'errore nella posizione
                
                // Log per verificare l'errore
                RCLCPP_INFO(this->get_logger(), "Errore posizione: x=%.2f, y=%.2f, z=%.2f", position_error[0], position_error[1], position_error[2]);

                if(position_error.norm()<0.001){
                    break;
                }
                //calcolo la jacobiana per il joint corrente
                calculate_jacobian(joint_angles,J);

                // Verifica che la Jacobiana non contenga NaN o Inf
                if (J.array().isNaN().any() || !J.array().isFinite().all()) {
                    RCLCPP_ERROR(this->get_logger(), "La Jacobiana contiene NaN o Inf, non posso calcolare la pseudo-inversa.");
                    break;
                }

                //calcolo la speudo-inversa
                Eigen::MatrixXd J_inv= pseudo_inverse(J);

                //come variano gli angoli dei joints
                Eigen::Vector3d delta_joint_angles=J_inv*position_error;

                // Log per vedere il cambiamento degli angoli
                RCLCPP_INFO(this->get_logger(), "Delta joint angles: theta1=%.2f, theta2=%.2f, theta3=%.2f",
                delta_joint_angles[0], delta_joint_angles[1], delta_joint_angles[2]);

                //aggiorno gli angoli
                joint_angles+=delta_joint_angles;

                // Aggiorna la posizione attuale
                current_position = forward_kinematics(joint_angles);
               
            
            }
            return joint_angles;
        }

        
        void update_position(){
            //obiettivo(target) per la posizione del braccio(x,y,z)
            Eigen::Vector3d target_position(0.3,-0.3,0.7);

            // Verifica se la posizione target è raggiungibile
            double max_reach = l1 + l2 + l3;  // Raggio massimo raggiungibile dal braccio
            double target_distance = std::sqrt(target_position[0] * target_position[0] + target_position[1] * target_position[1]);
            
            if (target_distance > max_reach) {
                RCLCPP_ERROR(this->get_logger(), "La posizione target è fuori dal raggio raggiungibile.");
                return;  // Esci dalla funzione se il target è fuori dal raggio
            }
        
            //calcola gli angoli degli joints tramite l'inverse kinemtics
            Eigen::Vector3d joint_angles=calculate_inverse_kinematics(target_position);

            // Log per vedere gli angoli calcolati
            RCLCPP_INFO(this->get_logger(), "Angoli calcolati: theta1=%.2f, theta2=%.2f, theta3=%.2f",
            joint_angles[0], joint_angles[1], joint_angles[2]);
            
            //messaggio per inviare la posizione 
            auto message= geometry_msgs::msg::Pose();
            message.position.x=joint_angles[0];
            message.position.y=joint_angles[1];
            message.position.z=joint_angles[2];
            
            //restituisce sul topic ROS2 /arm_position
            publisher_->publish(message);

            //stampa della nuova posizione nella console
            RCLCPP_INFO(this->get_logger(),"Posizione aggiornata: x=%.2f, y:%.2f, z:%.2f",joint_angles[0],joint_angles[1],joint_angles[2]);

        }

        void stop_node(){
            //stoppa il nodo dopo 5 secondi
            RCLCPP_INFO(this->get_logger(),"sono passati 5 secondi, ferma il nodo");
            rclcpp::shutdown();
        }

};

int main(int argc, char* argv[]){
    //inizializzo ROS2
    rclcpp::init(argc,argv);

    //creo e avvio il nodo
    rclcpp::spin(make_shared<RobotArmControl>());

    //termino ROS2 alla chisura del nodo
    rclcpp::shutdown();
    return 0;
}

