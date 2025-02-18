//includo le librerie necessarie per ROS2 e per la gestione della cinematica inversa
#include <rclcpp/rclcpp.hpp>  //libreria Ros per creare i nodi
#include <geometry_msgs/msg/pose.hpp> //messaggio per posizioni e orientamenti
#include <eigen3/Eigen/Dense> //libreria per algebra lineare 
#include <iostream>
#include <cmath>
#include <tinyxml2.h>  


using namespace std;
/*timer*/
using namespace std::chrono_literals;  //permette di usare 500ms come tempo
using namespace tinyxml2;

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

            // Carica i parametri DH da file
            read_DH_parameters("/home/marti/IKRA/src/robot_arm_control/src/dh_parameters.xml");

            //stampa messaggio per indicare che il nodo è stato avviato
            RCLCPP_INFO(this->get_logger(), "Nodo RobotArmControl avviato");

        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_; //invia poszione del braccio al topic
        rclcpp::TimerBase::SharedPtr timer_; //chiama la funziona update position
        rclcpp::TimerBase::SharedPtr stop_timer_;// ferma il nodo dopo 5 secondi
        bool printed_once_;

        struct DHParam{
            string theta;  //parametri q1,q2,q3
            double d;
            double a;
            double alpha;
        };
        //vettore per i parametri DH:[theta, d, a, alpha]
        vector<DHParam> dh_parameters;

        void  read_DH_parameters(const string& file_name) {
            XMLDocument doc; // crea oggetto XML document della librearia TinyXML2
            XMLError e= doc.LoadFile(file_name.c_str()); // carica file XML
            if(e!=XML_SUCCESS) {   
                RCLCPP_ERROR(this->get_logger(), "errore nella lettura del file");
                return;
            }

                XMLElement* root = doc.RootElement();  // <robot_arm>
            if (!root) {
                RCLCPP_ERROR(this->get_logger(), "Elemento root non trovato nel file XML");
                return;
            }
            
            // Pulisci eventuali dati precedenti
            dh_parameters.clear();
            
            XMLElement* joint = root->FirstChildElement("joint"); // leggi elemento joint dal file xml
            while(joint) {
                DHParam pDH;
                pDH.alpha = 0.0; // default
                pDH.d = 0.0;
                pDH.a = 0.0;
                pDH.theta = "";

                // Leggi <theta>
                XMLElement* alphaElem = joint->FirstChildElement("alpha");
                if(alphaElem && alphaElem->GetText()) {
                    // Converto la stringa in double, se possibile
                    // Se preferisci ignorare theta, lo puoi saltare
                    pDH.alpha = stod(alphaElem->GetText());
                }

                    // Leggi <d>
                XMLElement* dElem = joint->FirstChildElement("d");
                if(dElem && dElem->GetText()) {
                    pDH.d = stod(dElem->GetText());
                }

                // Leggi <a>
                XMLElement* aElem = joint->FirstChildElement("a");
                if(aElem && aElem->GetText()) {
                    pDH.a = stod(aElem->GetText());
                }
                // Leggi <alpha>
                // In questo esempio, nel file c'è "q1", "q2", "q3"
                // Salviamo come string
                XMLElement* thetaElem = joint->FirstChildElement("theta");
                if(thetaElem && thetaElem->GetText()) {
                    pDH.theta =thetaElem->GetText(); 
                }

                // Aggiungo questo giunto al vettore
                dh_parameters.push_back(pDH);

                // Passa al prossimo <joint>
                joint = joint->NextSiblingElement("joint");
                }
            
            // Log di debug
            for (size_t i = 0; i < dh_parameters.size(); i++){
                RCLCPP_INFO(this->get_logger(), 
                    "Giunto %zu: aplha=%.2f, d=%.2f, a=%.2f, theta=%s", 
                    i, dh_parameters[i].alpha, dh_parameters[i].d, dh_parameters[i].a, dh_parameters[i].theta.c_str()
                );
            }
        }

        Eigen::Vector3d analytical_IK_3joint_planar(double x, double y, double phi)
    {
        // Leggiamo a1, a2, a3 dai parametri
        if (dh_parameters.size() < 3) {
            RCLCPP_ERROR(get_logger(), "Non ho 3 giunti, impossibile calcolare IK");
            return Eigen::Vector3d::Zero();
        }

        double a1 = dh_parameters[0].a;
        double a2 = dh_parameters[1].a;
        double a3 = dh_parameters[2].a;

        double r = sqrt(x*x + y*y);
        double max_reach = a1 + a2 + a3;
        if (r > max_reach) {
            RCLCPP_ERROR(get_logger(), "Target fuori portata");
            return Eigen::Vector3d::Zero();
        }

        // braccio a 2 link (a1, a2+a3) => q1, q2
        double cos_q2 = (r*r - a1*a1 - (a2+a3)*(a2+a3)) / (2.0*a1*(a2+a3));
        if (cos_q2 > 1.0) cos_q2 = 1.0;
        if (cos_q2 < -1.0) cos_q2 = -1.0;
        double q2 = acos(cos_q2); // gomito su
        double sin_q2 = sqrt(1.0 - cos_q2*cos_q2);
        double beta = atan2((a2+a3)*sin_q2, a1 + (a2+a3)*cos_q2);
        double q1 = atan2(y, x) - beta;

        // q3 = phi - (q1 + q2)
        double q3 = phi - (q1 + q2);

        return Eigen::Vector3d(q1, q2, q3);
    }

    void update_position() {
        // Esempio di target
        double x = 0.3;
        double y = -0.3;
        double phi = 0.7; // orientamento finale
        
        // Calcolo IK analitico
        Eigen::Vector3d q = analytical_IK_3joint_planar(x, y, phi);

        // Creiamo un Pose da pubblicare (usiamo x= q1, y= q2, z= q3)
        geometry_msgs::msg::Pose msg;
        msg.position.x = q[0];
        msg.position.y = q[1];
        msg.position.z = q[2];

        publisher_->publish(msg);

        if(!printed_once_){
            RCLCPP_INFO(this->get_logger(), "Angoli analitici calcolati: q1=%.2f, q2=%.2f, q3=%.2f", 
                        q[0], q[1], q[2]);
            printed_once_ = true;
        }
    }


    void stop_node() {
        RCLCPP_INFO(get_logger(), "5 secondi trascorsi, chiudo il nodo");
        rclcpp::shutdown();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotArmControl>());
    rclcpp::shutdown();
    return 0;
}