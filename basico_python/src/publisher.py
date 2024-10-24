#!/usr/bin/env python3 
# ^ Assegura que o código seja interpretado como um script python

# Importa os bibliotecas
import rospy
from geometry_msgs.msg import Twist

# Função do nó publicador
def publica():
    # Cria um publicador (nome do tópico, tipo da mensagem, tam da fila)
    pub = rospy.Publisher('comandosTeste', Twist, queue_size = 10)  

    # Cria um nó (nome do nó, ...)
    rospy.init_node('publisher', anonymous=True)    

    # Cria a mendagem do tipo Twist
    f = Twist()
    f.linear.x = 2
    f.linear.y = 3
    f.linear.z = 3

    # Percorre o loop numa taxa de 10 vezes por segundo (com a ajuda do método sleep)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rospy.loginfo(f)
        pub.publish(f)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        publica()
    except rospy.ROSInterruptException:
        pass