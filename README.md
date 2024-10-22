## Projetos com ROS
O intuito desse repositório é armazenar projetos destinados ao aprendizado e aplicação de ROS (Robot Operating System).  
Seguem abaixo anotações gerais para a criação de um projeto.

## Anotações Gerais de ROS
Anotações do Treinamento de ROS disponibilizado pelo grupo de extensão SEMEAR da USP São Carlos. 

### Comandos para usar sempre:
`catkin_make`  
`source devel/setup.sh`

### Criando um nó:
1. Inicializar o nó  
    ```C++
    ros::init(argc, argv, "publisherNode / subscriberNode");
    ```    

2. Manipulador do nó  
    ```C++
    ros::NodeHandle _nh;
    ```
    
#### Publicador:
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

#### Subscritor:
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

### Burocracia do CMakeLists (para cada nó criado)
Configurações mínimas para todo nó.  
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

### Arquivo lauch
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
`roslaunch caminho_para_o_lauch` ou `roslaunch nome_projeto nome_arquivo_lauch`  
Exemplo:  
`roslaunch src/helloWorld/launch/helloWorld.launch` ou `roslaunch helloWorld helloWorld.launch`  

### Criando mensagens customizadas
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
#### Burocracio do CMakeLists (novamente)
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

#### Burocracia do package.xml
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
#### Modificando o arquivo fonte
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
