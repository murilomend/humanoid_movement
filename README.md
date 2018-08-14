EDROM Humanoid Movement for ROS
===================================

Resumo
-----------------------------------

Bibliotecas baseadas no ROS para gerar movimentos para um robô humanoide com pernas com 6 graus de liberdade. O projeto contem:

 * `humanoid_general`: contem algumas funções de operações matemáticas básicas, xml e a classe `State.h`, que tem duas classes responsáveis por guardar trajetórias de Juntas e de Corpo. O arquivo `Enums.h` dá informações sobre diversas constantes de outras nodes.
 
 * `humanoid_msgs`: contem todas as mensagens, serviços, ações e arquivos de reconfiguração dinâmica, usados no ROS pelo movimento.

 * `humanoid_loadmap`: cria uma node para receber dados de arquivos .xml (olhe o diretório `config`), e cria um mapa para conversões entre tipos de id de juntar diferentes. O mapa é enviado quando um serviço é requisitado por outra node.

 * `humanoid_model`: lê um arquivo urdf contendo todos o dados dinâmicos e cinemáticos para um robô humanoide, e provém informação sobre o centro de massa, zmp, dinâmicas inversas e cinemáticas.

 * `humanoid_walking`: gera trajetórias cartesianas do andar, para serem seguidas por um pé humanoide a fim de alcançar um andar omnidirecional de loop aberto. É baseado em um pêndulo invertido linear e pode ser configurado através de serviços e tópicos.

 * `humanoid_interface`: providência a interface entre comandos de junta (`JointStateMsg`) com o Gazebo Simulation e o robô.
 * `humanoid_qt`: contem uma QT GUI para fácil configuração de parâmetros do robô humanoide e para visualização de dados.
 

Instalação das dependências do movimento
-----------------------------------

Vá para a pasta dependencies e extraia o .zip na sua pasta home (não faça isso dentro do repositório para evitar confusões no git).

* **Eigen:**<br>
        Entre na pasta do eigen que você extraiu e faça os seguintes comandos:
        
        ```
        mkdir build
        cd build
        cmake ..
        sudo make install
        ```

* **RBDL:**<br>
        Entre na pasta do RBDL que você extraiu e faça os seguintes comandos:
        
        ```
        mkdir build
        cd build
        cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=true -D RBDL_USE_ROS_URDF_LIBRARY=false ..
        make
        sudo make install
        ```


* *Instalando o pacote do QT para ROS:*
```
sudo apt install ros-kinetic-qt-build
```

* *Instalando pacotes do ROS Controllers:*
```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

* *Instalando pacotes do ROS Gazebo:*
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

* *Configurando o arquivo `has_binary_operator.hpp`:*
```
sudo gedit /usr/include/boost/type_traits/detail/has_binary_operator.hpp
```
  Encontre a linha :
  `namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {`<br>
  Em seguida, adicione essa linha de código **acima dela** ` #ifndef Q_MOC_RUN`<br>
  E esta linha de código **abaixo dela** `#endif`.<br>
  Salve o arquivo.

* *Instalando o setserial:*
```
sudo apt-get install setserial
```

* *Rode o arquivo setup-udev.sh:*
```
cd ~/edrom/src/humanoid_movement/3rdparty/phidgets_drivers/phidgets_api/share
./setup-udev.sh
```
