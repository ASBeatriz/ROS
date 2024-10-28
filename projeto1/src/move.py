#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# Função auxiliar para trocar o estado de movimento
def troca(estado):
    if estado == "andando":
        return "parado"
    else:
        return "andando"

# Função pdo nó publicador
def publica():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)  
    rospy.init_node('publisher', anonymous=True)    

    # Cria a mensagem
    f = Twist()

    rate = rospy.Rate(10)  # Faz percorrer o loop 10 vezes por segundo

    # Armazena o tempo atual em segundos
    tempoInicial = rospy.get_time()  

    # Variável para armazenar o tempo relativo à ultima troca
    ultimaTroca = tempoInicial

    # Variável para indicar o estado do robô
    estado = "andando"

    while not rospy.is_shutdown():
        tempoAtual = rospy.get_time()

        # Muda o estado de andando/parado a cada 3 segundos
        if tempoAtual - ultimaTroca >= 3:
            estado = troca(estado)
            ultimaTroca = tempoAtual

        tempoPassado = tempoAtual - tempoInicial
        # Imprime as infos
        print("Timer : %.2f" %tempoPassado)
        print(f"Estado : {estado} \n")

        if estado == "andando":
            f.angular.z = 0.5
            f.linear.x = 1
        else:
            f.angular.z = 0
            f.linear.x = 0

        #rospy.loginfo(f)
        pub.publish(f)  # Publica a mensagem
        rate.sleep()
    
if __name__ == '__main__':
    try:
        publica()
    except rospy.ROSInterruptException:
        pass