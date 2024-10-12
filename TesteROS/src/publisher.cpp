#include <ros/ros.h>
/* #include "std_msgs/Int64.h" */
#include "TesteROS/MinhaMensagem.h"
#include <string>

int main (int argc, char **argv){
    ros::init(argc, argv, "publisherNode");
    ros::NodeHandle _nh;
    /* 
    ros::Publisher topicoExemploRef = _nh.advertise<std_msgs::Int64>("topicoExemplo", 1);
    std_msgs::Int64 mensagem;
    */

    //acessando uma estrutura criada por mim mesma (MinhaMensagem.msg) :

    //armazena a referência para o publicador do tópico "topicoExemplo" de fila tamanho 1
    ros::Publisher topicoExemploRef = _nh.advertise<TesteROS::MinhaMensagem>("topicoExemplo", 1); 

    // cria a mensagem a ser enviada e atribui valor aos seus campos
    TesteROS::MinhaMensagem mensagem;
    mensagem.primeiroInteiro = 2005;
    mensagem.segundoInteiro.data = 1977;
    mensagem.texto = std::string("IAAAAAAAAA, disse o Tarzan.");

    while(ros::ok()){
        //publica a mensagem
        topicoExemploRef.publish(mensagem);
        ros::spinOnce();
    }
}