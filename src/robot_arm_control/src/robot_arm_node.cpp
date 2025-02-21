/* nodo ROS2 che calcola la cinematica inversa di un braccio planare 2R utilizzando il metodo 
   newton e pubblica i risultati, prendendo i parametri dal file dh_parameters.xml*/

   //includo le librerie necessarie per ROS2 e per la gestione della cinematica inversa
#include <rclcpp/rclcpp.hpp>  //libreria Ros per creare i nodi
#include <geometry_msgs/msg/pose.hpp> //messaggio per posizioni e orientamenti
#include <eigen3/Eigen/Dense> //libreria per algebra lineare 
#include <iostream>
#include <cmath>
#include <tinyxml2.h>  //libreria per leggere i file xml


using namespace std;
using namespace tinyxml2;

/*creo una classe che estende rclcpp::Node*/
//Pubblica la posizione (angoli) del braccio su un topic ROS
//Legge i parametri DH da file
// Calcola la cinematica inversa con metodo iterativo di Newton
class RobotArmControl : public rclcpp::Node{
    public:
        /*costruttore*/
        RobotArmControl(): Node("robot_arm_control"){
            //creo un publisher per inviare la posizione del braccio sul topic arm position
            publisher_=this->create_publisher<geometry_msgs::msg::Pose>("arm_position",10);

             // variabile booleana (per stampare i risultati una sola volta)
            printed_once_ = false;

            // Carica i parametri DH da file
            read_DH_parameters("/home/marti/IKRA/src/robot_arm_control/src/dh_parameters.xml");

            //stampa messaggio per indicare che il nodo è stato avviato
            RCLCPP_INFO(this->get_logger(), "Nodo RobotArmControl avviato");

            update_position();
        }
    private:
        // Publisher ROS per inviare la posizione del braccio
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_; //invia poszione del braccio al topic
        bool printed_once_;

        /*struct per memorizzare i parametri DH*/
        struct DHParam{
            string theta;  //parametri q1,q2
            double d;
            double a;
            double alpha;
        };

        //vettore per i parametri DH:[theta, d, a, alpha]
        vector<DHParam> dh_parameters;

        //funzione per leggere i parametri dal file xml
        void  read_DH_parameters(const string& file_name) {
            XMLDocument doc; // crea oggetto XML document della librearia TinyXML2
            XMLError e= doc.LoadFile(file_name.c_str()); // carica file XML
            if(e!=XML_SUCCESS) {   
                RCLCPP_ERROR(this->get_logger(), "errore nella lettura del file");
                return;
            }
            //iniziallizzo un puntatore alla radice per verificare che il file non sia vuoto
            XMLElement* root = doc.RootElement();  // <robot_arm>
            if (!root) {
                RCLCPP_ERROR(this->get_logger(), "Elemento root non trovato nel file XML");
                return;
            }
            
            // Pulisci eventuali dati precedenti
            dh_parameters.clear();
            
            //leggo il primo elemento
            XMLElement* joint = root->FirstChildElement("joint"); // leggi elemento joint dal file xml
            while(joint) {
                DHParam pDH;
                pDH.alpha = 0.0;
                pDH.d = 0.0;
                pDH.a = 0.0;
                pDH.theta = "";

                // Leggi <theta> 
                XMLElement* alphaElem = joint->FirstChildElement("alpha");
                if(alphaElem && alphaElem->GetText()) {
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
                // Leggi <alpha> come stringa
                XMLElement* thetaElem = joint->FirstChildElement("theta");
                if(thetaElem && thetaElem->GetText()) {
                    pDH.theta =thetaElem->GetText(); 
                }

                // Aggiungo questo joint al vettore
                dh_parameters.push_back(pDH);

                // Passa al prossimo <joint>
                joint = joint->NextSiblingElement("joint");
                }
            
            // Log di debug
            for (size_t i = 0; i < dh_parameters.size(); i++){
                RCLCPP_INFO(this->get_logger(), 
                    "Joint %zu: aplha=%.2f, d=%.2f, a=%.2f, theta=%s", 
                    i, 
                    dh_parameters[i].alpha, 
                    dh_parameters[i].d, 
                    dh_parameters[i].a, 
                    dh_parameters[i].theta.c_str()
                );
            }
        }

        Eigen::Vector2d inverse_kinematics(double x_target, double y_target){
        // Leggiamo a1, a2 dai parametri, prima verifico se ci sono proprio due joint
        if (dh_parameters.size() < 2) {
            RCLCPP_ERROR(get_logger(), "Non ho 2 joints, impossibile calcolare IK");
            return Eigen::Vector2d::Zero();
        }

        double a1 = dh_parameters[0].a;
        double a2 = dh_parameters[1].a;

        Eigen::Vector2d q(0.5, -0.5); // inizializzo angoli

        const double tol   = 1e-3; // soglia di convergenza
        
        for (int i =0; i<20; i++){
            double q1 = q[0];
            double q2 = q[1];

            //calcolo la cinematica diretta
            double x_corr= a1*cos(q1)+ a2*cos(q1+q2);
            double y_corr= a1*sin(q1)+ a2*sin(q1+q2);

            //controllo quanto vale l'errore tra la posizione attuale e il target
            double dx= x_corr- x_target;
            double dy = y_corr-y_target;

            // Se l'errore è già piccolo, posso uscire
            if (sqrt(dx*dx + dy*dy) < tol) {
                RCLCPP_INFO(get_logger(), "Convergenza dopo %d iterazioni", i);
                return q;
            }

            //calcolo Jacobiana
            double j11 = -a1*sin(q1) - a2*sin(q1 + q2);
            double j12 = -a2*sin(q1 + q2);
            double j21 =  a1*cos(q1) + a2*cos(q1 + q2);
            double j22 =  a2*cos(q1 + q2);

            //costruisco la matrice
            Eigen::Matrix2d J;
            J<< j11, j12,
                j21, j22;

            // Costruisco il vettore errore d
            Eigen::Vector2d d_vec(dx, dy);
           
            //calcolo l'inversa della Jacobiana
            Eigen::Vector2d delta_q = -J.inverse()*d_vec;
            
            //aggiorno la configurazione
            q+= delta_q;

        }

        // Se arrivo qui, non ho convergenza
        RCLCPP_WARN(get_logger(), "Metodo di Newton: nessuna convergenza dopo 20 iterazioni");
        return q;
    }

    //dato un target chiama inverse kin e da in output i valori delle q
    void update_position() {
        // target
        double x = 0.4;
        double y = -0.3;
        
        // Calcolo inverse kinematics
        Eigen::Vector2d q = inverse_kinematics(x, y);

        // Creiamo un messaggio Pose 
        geometry_msgs::msg::Pose msg;
        msg.position.x = q[0];
        msg.position.y = q[1];
        
        //pubblico su arm_position
        publisher_->publish(msg);

        if(!printed_once_){
            RCLCPP_INFO(this->get_logger(), "Angoli calcolati: q1=%.3f, q2=%.3f", 
                        q[0], q[1]);
            printed_once_ = true;
        }
    }

};

int main(int argc, char* argv[])
{
    //inizializza Ros2
    rclcpp::init(argc, argv);

    //avvia il nodo
    rclcpp::spin(std::make_shared<RobotArmControl>());

    rclcpp::shutdown();
    return 0;
}