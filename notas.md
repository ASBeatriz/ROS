## Anotações Gerais de ROS

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
    ```C++
    add_executable(nome_do_no
        caminho_para_o_no
    )
    ```
    Exemplo:  
    ```C++
    add_executable(publisherNode
        src/publisher.cpp
    )
    ```

2. `target_linked_libraries`  
    Procurar a seção que contém essa declaração (modelo) e adicionar em baixo:
    ```C++
    target_link_libraries(nome_do_no
        ${catkin_LIBRARIES}
    )
    ```
    Exemplo:  
    ```C++
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
`roslaunch caminho_para_o_lauch`  
Exemplo:  
`roslaunch src/helloWorld/launch/helloWorld.launch`
