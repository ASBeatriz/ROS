#include "ros/ros.h"
/* #include "std_msgs/Int64.h" */
#include "TesteROS/MinhaMensagem.h"
#include <iostream>
#include <string>

/*void subscriberCallback(const std_msgs::Int64::ConstPtr& msg)*/
void subscriberCallback(const TesteROS::MinhaMensagem::ConstPtr& msg){
    /*std::cout << "[subscriber node] Valor recebido: " << msg->data << std::endl;*/
    std::cout << "[subscriber node] Primeiro inteiro recebido: " << msg->primeiroInteiro << std::endl;
    std::cout << "Segundo inteiro recebido: " << msg->segundoInteiro.data << std::endl;
    std::cout << "Texto recebido: " << msg->texto << "\n\n";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriberNode");
    ros::NodeHandle _nh;

    //referência ao nó que se "inscreve" no tópico "topicoExemplo" e chama a função subscriberCallBack sempre que receber algo do tópico
    ros::Subscriber topicoExemploRef = _nh.subscribe("topicoExemplo", 1, subscriberCallback);
    ros::spin();
}