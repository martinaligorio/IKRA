//includo le librerie necessarie per ROS2 e per la gestione della cinematica inversa
#include <rclcpp/rclcpp.hpp>  //libreria Ros per creare i nodi
#include <geometry_msgs/msg/pose.hpp> //messaggio per posizioni e orientamenti
#include <eigen3/Eigen/Dense> //libreria per algebra lineare 

/*timer*/
using namespace std::chrono_literals;  //permette di usare 500ms come tempo

/*creo una classe che estende rclcpp::Node*/
class RobotArmControl : public rclcpp::Node{
    public:
        /*costruttore*/
        RobotArmControl(): Node("robot_arm_control"){
            //creo un publisher per inviare la posizione del braccio
            publisher_=this->create_publisher<geometry_msgs::msg::Pose>("arm_position",10);

            //creo un timer che mi esegue la funzione update_position ogni 500 ms
            timer_=this->create_wall_timer(500ms,std::bind(&RobotArmControl::update_position,this));

            //il timer si ferma dopo 5s
            stop_timer_=this->create_wall_timer(5s,std::bind(&RobotArmControl::stop_node,this));
            //stmpa messaggio
            RCLCPP_INFO(this->get_logger(), "Nodo RobotArmControl avviato");

        }
    private:
        void update_position(){
            //obiettivo per la posizione del braccio
            Eigen::Vector3d target_position(1.0,1.0,1.0);

            //calcola gli angoli degli joints tramite l'inverse kinemtics
            Eigen::Vector3d joint_angles=calculate_inverse_kinematics(target_position);

            //messaggio per inviare la posizione 
            auto message= geometry_msgs::msg::Pose();
            message.position.x=joint_angles[0];
            message.position.y=joint_angles[1];
            message.position.y=joint_angles[2];
            
            //restituisce sul topic ROS2 /arm_position
            publisher_->publish(message);

            //stampa della nuova posizione nella console
            RCLCPP_INFO(this->get_logger(),"Posizione aggiornata: x=%.2f, y:%.2f, z:%.2f",joint_angles[0],joint_angles[1],joint_angles[2]);

        }

        /*INVERSE KINEMATICS*/
        //dato un punto nello spazio "target" mi deve restituire gli angoli necessari per raggiungerlo
        Eigen::Vector3d calculate_inverse_kinematics(const Eigen::Vector3d& target){
            Eigen::Vector3d joints;

            //approssimazione della posizoone target tra i tre giunti
            joints[0]=target[0]/2;
            joints[1]=target[1]/2;
            joints[2]=target[2]/2;
            
            return joints;
        }

        void stop_node(){
            //stoppa il nodo dopo 5 secondi
            RCLCPP_INFO(this->get_logger(),"sono passati 5 secondi, ferma il nodo");
            rclcpp::shutdown();
        }

        /*publisher ROS2*/
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;

        /*timer ROS2*/
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::TimerBase::SharedPtr stop_timer_;

};

int main(int argc, char* argv[]){
    //inizializzo ROS2
    rclcpp::init(argc,argv);

    //creo e avvio il nodo
    rclcpp::spin(std::make_shared<RobotArmControl>());

    //termino ROS2 alla chisura del nodo
    rclcpp::shutdown();
    return 0;
}