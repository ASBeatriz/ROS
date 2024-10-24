#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback (data):
    rospy.loginfo('mensagem recebida (y): %s', data.linear.y)

def subscriber ():
    # Cria o nó
    rospy.init_node('subscriber', anonymous=True)
    # Inscreve o nó no tópico
    rospy.Subscriber('comandosTeste', Twist, callback)  # (nome do tópico, tipo da mensagem, função de callback)
    # Evita que o nó saia até que ele seja encerrado
    rospy.spin()

if __name__ == '__main__':
    subscriber()