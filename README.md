## Projetos com ROS
O intuito desse repositório é armazenar projetos destinados ao aprendizado e aplicação de ROS (Robot Operating System).  
Seguem abaixo anotações gerais para a criação de um projeto.

## Sumário
- [Anotações Gerais de ROS](#anotacoes-gerais)  
    - [Comandos para usar sempre](#comandos)  
    - [Criando um pacote ROS](#criando-pacote)  
- [Comando em C++](#cpp)  
    - [Bibliotecas](#biblioteca-cpp)  
    - [Criando um nó](#no-cpp)  
        - [Publicador](#pub-cpp)  
        - [Subscritor](#sub-cpp)  
    - [Configuração do CMakeLists](#cmake1)  
    - [Arquivo launch](#launch)  
    - [Criando mensagens customizadas](#msg-customizada)  
        - [Configuração do CMakeLists (novamente)](#cmake2)  
        - [Configuração do package.xml](#packagexml)  
        - [Modificando o arquivo fonte](#arquivo-fonte)  
- [Comando em Python](#python)  
    - [No terminal](#terminal)  
    - [Bibliotecas](#biblioteca-python)   
    - [Criando e manipulando um nó](#no-python)  
        - [Publicador](#pub-python)  
        - [Subscritor](#sub-python)  
    - [Separando em funções](#funcoes)  
    - ["Main"](#main) 
    - [Executando](#executando)  
---    

## Anotações Gerais de ROS <a name="anotacoes-gerais"></a>
Comandos e procedimentos básicos para criar um projeto com ROS.

### Comandos para usar sempre: <a name="comandos"></a>
`catkin_make`  
`source devel/setup.sh` - em cada terminal aberto

### Criando um pacote ROS <a name="criando-pacote"></a>
Na pasta **src**, utilizar o comando `catkin_create_pkg` com o nome do pacote e as dependências:
```
cd src
catkin_create_pkg nome_do_pacote dependencias
```
Exemplo:  
```
catkin_create_pkg helloWorld std_msgs rospy roscpp
```  

## Comando em C++ <a name="cpp"></a>
Anotações do Treinamento de ROS disponibilizado pelo grupo de extensão SEMEAR da USP São Carlos.  

### Bibliotecas <a name="biblioteca-cpp"></a>
Incluir a biblioteca `<ros/ros.h>` e outras, caso necessário.   
``` C++
#include <ros/ros.h>    // Biblioteca para o ROS
#include "std_msgs/Int64.h" // Exemplo de biblioteca para o tipo da mensagem (tipo padrão)
```

### Criando um nó <a name="no-cpp"></a>
Em um arquivo fonte (.cpp) dentro da pasta "src" do pacote. 
1. Inicializar o nó  
    ```C++
    ros::init(argc, argv, "publisherNode / subscriberNode");
    ```    

2. Manipulador do nó  
    ```C++
    ros::NodeHandle _nh;
    ```
    
#### Publicador: <a name="pub-cpp"></a>
3. Criar o publicador 
    ```C++
    ros::Publisher topicoExemploRef = _nh.advertise<tipo_da_mensagem>("topicoExemplo", tam_da_fila);
    ```
4. Instanciar uma mensagem e alterar seu conteúdo  
    ```C++
    tipo_da_mensagem mensagem;
    mensagem.data = 100;
    ```
5. Publicar a mensagem  
    ```C++
    topicoExemploRef.publish(mensagem);
    ```

#### Subscritor: <a name="sub-cpp"></a>
3. Criar o subscritor  
    ```C++
    ros::Subscriber topicoExemploRef = _nh.subscribe("topicoExemplo", tam_da_fila, subscriberCallBack)
    ```
4. Criar a função de callback  
    ```C++
    void subscriberCallBack(const tipo_da_mensagem::ConstPtr& msg){
        // Função a ser executada toda vez que uma mensagem chega. Ex:
        std::cout << "[SubscriberNode] Valor recebido: " << msg->data << std::endl;
    }
    ```

### Configuração do CMakeLists <a name="cmake1"></a>
Configurações mínimas **para cada nó criado**.  
1. `add_executable`  
    Procurar a seção que contém essa declaração (modelo) e adicionar em baixo:  
    ```cmake
    add_executable(nome_do_no
        caminho_para_o_no
    )
    ```
    Exemplo:  
    ```cmake
    add_executable(publisherNode
        src/publisher.cpp
    )
    ```

2. `target_linked_libraries`  
    Procurar a seção que contém essa declaração (modelo) e adicionar em baixo:
    ```cmake
    target_link_libraries(nome_do_no
        ${catkin_LIBRARIES}
    )
    ```
    Exemplo:  
    ```Cmake
    target_link_libraries(publisherNode
        ${catkin_LIBRARIES}
    )
    ```

### Arquivo launch  <a name="launch"></a>
Facilita a execução de vários nós, evitando executar um por vez.  
Dentro do pacote, criar um pasta "launch" e adicionar um arquivo .lauch (helloworld.lauch, por exemplo).    
Dentro do arquivo, adicionar os nós:  
```C++
 <node name="nome_do_no" pkg="nome_do_pacote" type="tipo_do_no" output="screen"/> 
 ```  
Exemplo:  
```C++
<node name="publisherNode" pkg="helloWorld" type="publisherNode" output="screen"/>
```  

- **Executando**:  
No terminal, digite o seguinte comando na pasta do workspace ("catkin_ws"):  
`roslaunch caminho_para_o_lauch` ou  
`roslaunch nome_projeto nome_arquivo_lauch`  
Exemplo:  
`roslaunch src/helloWorld/launch/helloWorld.launch` ou  
`roslaunch helloWorld helloWorld.launch`  

### Criando mensagens customizadas <a name="msg-customizada"></a>
No pacote em questão, criar uma pasta "msg" e adionar o arquivo .msg (minhaMensagem.msg, por exemplo).  
Dentro do arquivo, adicionar apenas a declaração de cada componente da mensagem, que é uma struct. Exemplo:  
``` msg
int inteiro1  

std_msgs/Int32 inteiro2  
#Inteiro usando uma mensagem pré-pronta

string palavra

bool verifica
...
```
#### Configuração do CMakeLists (novamente) <a name="cmake2"></a>
1. `find_package`  
Procurar a seção que contém essa declaração (modelo) e adicionar _message_generation_ na lista já existente:
    ```cmake
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation )
    ```
2. `add_message_files`  
    Procurar a seção que contém esse bloco, descomentar o bloco inteiro, apagar os nomes de mensagens genéricas e **manter** o termo FILES. Abaixo dele, insira o nome da mensagem. Exemplo:  
    ```cmake
    add_message_files(
        FILES
        MinhaMensagem.msg)
    ``` 
3. `generate_messages`  
    Procurar a seção que contém esse bloco, descomentar o bloco inteiro, apagar os nomes de mensagens genéricas e **manter** ou adicionar o termo DEPENDENCIES. Exemplo:  
    ```cmake
    generate_messages(
        DEPENDENCIES
        std_msgs)
    ``` 
    **Obs.:** No exemplo da mensagem customizada, além dos tipos primitivos, utilizamos o conjunto _std_msgs_, por isso ele foi adicionado aqui também.
4. `catkin_package`  
    Procurar a seção que contém esse bloco, descomentar o bloco inteiro e adicionar *message_runtime* na linha que se inicia com *CATKIN_DEPENDS*. Ex:
     ```cmake
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES helloWorld
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        DEPENDS system_lib
        )
    ``` 

#### Configuração do package.xml <a name="packagexml"></a>
No fim do arquivo _package.xml_ tem algumas tags escritas como:
``` 
<build_depend> </build_depend>

<build_export_depend> </build_export_depend>

<exec_depend> </exec_depend>
```
Para cada tipo, adicione mais uma tag contendo “message_runtime”. O resultado deve ser algo semelhante a:
```
<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>message_runtime</build_depend>

<build_export_depend>roscpp</build_export_depend>
<build_export_depend>rospy</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<build_export_depend>message_runtime</build_export_depend>

<exec_depend>roscpp</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>message_runtime</exec_depend>
```
#### Modificando o arquivo fonte <a name="arquivo-fonte"></a>
Para utilizar a estrutura de mensagem criada, adicionar:
``` c++
#include "nome_do_pacote/nome_da_mensagem.h"
```
Da mesma forma, deve-se mudar o tipo da mensagem na criação do publicador e subscritor e na instanciação da mensagem. Exemplo:
``` C++
ros::Publisher topicoExemploRef =
_nh.advertise<helloWorld::MinhaMensagem>("topicoExemplo", 1);

helloWorld::MinhaMensagem mensagem;
mensagem.inteiro1 = 10;
...
```

## Comandos em Python <a name="python"></a>
Anotações retiradas da documentação oficial.

### No terminal <a name="terminal"></a>
Para cada arquivo Python criado, faça-o executável:
```
chmod +x caminho_para_o_arquivo
```
Exemplo:
```
chmod +x src/file.py
```
### Bibliotecas <a name="biblioteca-python"></a>
Importar a biblioteca `rospy` e outras, caso necessário.
``` Python
import rospy    # biblioteca para o ROS
from geometry_msgs.msg import Twist # exemplo de biblioteca para o tipo da mensagem
```
### Criando um nó <a name="no-python"></a>
1. Cria o nó
    ``` Python
    rospy.init_node('nome_do_no', anonymous = True)
    ```

#### Publicador: <a name="pub-python"></a>
2. Criar o publicador 
    ``` Python
    pub = rospy.Publisher('comandosTeste', Twist, queue_size = 10)  
                       # (nome do tópico, tipo da mensagem, tam da fila)
    ```
3. Instanciar uma mensagem e alterar seu conteúdo  
    Exemplo:  
    ``` Python
    msg = Twist()
    msg.linear.x = 2
    msg.linear.y = 3
    msg.linear.z = 3
    ```
4. Publicar a mensagem  
    ``` Python
    pub.publish(msg)
    ```

**Obs:**  
Estabelecer uma taxa de frequência para percorrer um loop:
``` Python
rate = rospy.Rate(10)  # percorre o loop 10 vezes por segundo
```
E, dentro do loop:
``` Python
rate.sleep()
```
Exemplo:
``` Python
rate = rospy.Rate(10)  # 10hz   

while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()
```

#### Subscritor: <a name="sub-python"></a>
2. Chama o subscritor 
    ```Python
    # (nome do tópico, tipo da mensagem, função de callback)
    rospy.Subscriber('comandosTeste', Twist, callback) 
    ```
3. Criar a função de callback  
    Exemplo:
    ```Python
    def callback (msg):
        rospy.loginfo('mensagem recebida: %s', msg.data)
    ```
4. Método spin()
    Evita que o nó saia até que ele seja encerrado.
    ```Python
    rospy.spin()
    ```

### Separando em funções <a name="funcoes"></a>
Todos os comando anteriores vão em sua própria função (publisher(), subscriber(), callback(), etc).  
Exemplo de subscritor:  
``` Python
def callback (data):
    rospy.loginfo('mensagem recebida (y): %s', data.linear.y)

def subscriber ():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('comandosTeste', Twist, callback)
    rospy.spin()
```
### "Main" <a name="main"></a>
Após as funções estarem feitas, é construído o código que chama a função principal:  
**Publicador**
``` Python
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```
**Subscritor**
``` Python
if __name__ == '__main__':
    subscriber()
```

### Executando <a name="executando"></a>
1. Executar `roscore` no terminal

2. Em **outro terminal**, executar o código desejado com:  
`rosrun nome_pacote nome_arquivo`  
Exemplo:  
`rosrun helloWorld publisher.py`
