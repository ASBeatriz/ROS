#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# Função para ler a velocidade do usuário
def ler_vel():
    vel = float(input("Velocidade: "))
    return vel

# Função auxiliar para trocar o estado de movimento
def troca(estado):
    if estado == "andando":
        return "parado"
    else:
        return "andando"

# Função do nó publicador
def publica():
    # simulação com gazebo: '/cmd_vel'
    # simulação sem gazebo (turtlesim_node): '/turtle1/cmd_vel'
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)  
    rospy.init_node('publisher', anonymous=True)    
    rate = rospy.Rate(10)
    
    # Cria a mensagem e inicializa seus componentes
    f = Twist()
    f.angular.x, f.angular.y, f.angular.z = [0,0,0]
    f.linear.x, f.linear.y, f.linear.z = [0,0,0]

    # Variável para indicar o estado do robô
    estado = "andando"

    vel = ler_vel()

    # Armazena o tempo atual em segundos
    tempoInicial = rospy.get_time()  

    # Variável para armazenar o tempo relativo à ultima troca
    ultimaTroca = tempoInicial

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
            # f.angular.z = 1
            f.linear.x = vel
        else:
            # f.angular.z = 0
            f.linear.x = 0  

        #rospy.loginfo(f)
        pub.publish(f)  # Publica a mensagem
        rate.sleep()
    
if __name__ == '__main__':
    try:
        publica()
    except rospy.ROSInterruptException:
        pass