//includo le librerie necessarie per ROS2 e per la gestione della cinematica inversa
#include <rclcpp/rclcpp.hpp>  //libreria Ros per creare i nodi
#include <geometry_msgs/msg/pose.hpp> //messaggio per posizioni e orientamenti
#include <eigen3/Eigen/Dense> //libreria per algebra lineare 
#include <tinyxml2.h> // pr legger parametri DH da file XML

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
            timer_=this->create_wall_timer(500ms,std::bind(&RobotArmControl::update_position,this));

            //il timer si ferma dopo 5s
            stop_timer_=this->create_wall_timer(5s,std::bind(&RobotArmControl::stop_node,this));
            
            //carica i parametri dal file xml
            read_DH_parameters("dh_parameters.xml");
            
            //stampa messaggio per indicare che il nodo è stato avviato
            RCLCPP_INFO(this->get_logger(), "Nodo RobotArmControl avviato");

        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_; //invia poszione del braccio al topic
        rclcpp::TimerBase::SharedPtr timer_; //chiama la funziona update position
        rclcpp::TimerBase::SharedPtr stop_timer_;// ferma il nodo dopo 5 secondi
        std::vector<Eigen::Vector4d> dh_parameters_;//vettore di parametri per ogni giunto
        
        //funziona che carica dh parameters da xml
        void  read_DH_parameters(const std::string& file_name) {
            tinyxml2::XMLDocument doc; // crea oggetto XML document della librearia TinyXML2
            doc.LoadFile(file_name.c_str()); // carica file XML
            if(doc.Error()) {
                RCLCPP_ERROR(this->get_logger(), "errore nella lettura del file");
                return;
            }

            tinyxml2::XMLElement* root = doc.RootElement(); 
            for(int i=0; i<3; i++) { //leggi 3 giunti
                tinyxml2::XMLElement* joint = root->FirstChildElement("joint"); // leggi elemento joint dal file xml
                if(joint) {
                    double theta =std::stod(joint->Attribute("theta"));
                    double d =std::stod(joint->Attribute("d"));
                    double a =std::stod(joint->Attribute("a"));
                    double alpha =std::stod(joint->Attribute("alpha"));
                    dh_parameters_.push_back(Eigen::Vector4d(theta, d, a, alpha)); //i parametri DH di ogni joint vengono memorizzati in un vettore
                    root = joint->NextSiblingElement("joint"); // va al prossimo giunto
                }
            }  
            RCLCPP_INFO(this->get_logger(), "Parametr DH caricati con successo"); //conferma caricamento parametri
        }

        //funzione per calcolare la jacobiana per un braccio con 3 joints
        void calculate_jacobian(const Eigen::Vector3d& joint_angles, Eigen::MatrixXd& J){
            
            double l1=1.0;  //lunghezza primo braccio
            double l2=1.0;  //lunghezza secondo braccio
            double l3=1.0;  //lunghezza terzo braccio
            
            double theta1=joint_angles[0];
            double theta2=joint_angles[1];
            double theta3=joint_angles[2];

            //calcolo della jacobiana
            J(0,0)=-l1*sin(theta1)-l2*sin(theta1+theta2)-l3*sin(theta1+theta2+theta3);
            J(0,1)=-l2*sin(theta1+theta2)-l3*sin(theta1+theta2+theta3);
            J(0,2)=-l3*sin(theta1+theta2+theta3);
            
            J(1,0)=l1*cos(theta1)+l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3);
            J(1,1)=l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3);
            J(1,2)=l3*cos(theta1+theta2+theta3);

            J(2,0)=0;
            J(2,1)=0;
            J(2,2)=0;
        }

        /*FUNZIONE PER CALCOLARE LA PSEUDO-INVERSA DELLA JACOBIANA*/
        Eigen::MatrixXd pseudo_inverse(Eigen::MatrixXd J){
            Eigen::MatrixXd Jt=J.transpose();  //calcola la trasposta
            Eigen::MatrixXd JtJ_inv=(Jt*J).inverse();  //calcola l'inversa di Jt*J
            return JtJ_inv*Jt;   //return la pseudo-inversa
        }

        /*INVERSE KINEMATICS*/
        //dato un punto nello spazio "target" mi deve restituire gli angoli necessari per raggiungerlo
        Eigen::Vector3d calculate_inverse_kinematics(const Eigen::Vector3d target){
            Eigen::MatrixXd J(3,3);  //jacobiana 3x3
            Eigen::Vector3d joint_angles=Eigen::Vector3d::Zero();  //angoli iniziali  
            
            //itero per calcolare gli angoli dei joints
            for(int i=0;i<1000;i++){
                Eigen::Vector3d current_position=joint_angles;  //posizione attuale
                Eigen::Vector3d position_error=target-current_position;  //calcolo l'errore nella posizione

                //calcolo la jacobiana per il joint corrente
                calculate_jacobian(joint_angles,J);

                //calcolo la speudo-inversa
                Eigen::MatrixXd J_inv= pseudo_inverse(J);

                //come variano gli angoli dei joints
                Eigen::Vector3d delta_joint_angles=J_inv*position_error;

                //aggiorno gli angoli
                joint_angles+=delta_joint_angles;

                //se l'errore è piccolo possiamo fermare l'operazione
                if(position_error.norm()<0.001){
                    break;
                }
        
            }
            return joint_angles;
        }

        void update_position(){
            //obiettivo(target) per la posizione del braccio(x,y,z)
            Eigen::Vector3d target_position(1.0,1.0,1.0);

            //calcola gli angoli degli joints tramite l'inverse kinemtics
            Eigen::Vector3d joint_angles=calculate_inverse_kinematics(target_position);

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

