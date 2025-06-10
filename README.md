=================
 Ros-D4nc3r
=================

Um pacote ROS para robô diferencial do Projeto 10 Dimenções!


SOBRE O PROJETO
----------------
O ros-D4nc3r explorar o controle de robôs com o ROS (Robot Operating System).

Tecnologias Utilizadas:
* ROS (Robot Operating System): http://www.ros.org/
* Python: https://www.python.org/
* Micro-Ros : https://micro.ros.org/


COMEÇANDO
----------
Siga as instruções abaixo para colocar o Ros-D4nc3r em funcionamento no seu ambiente ROS.

Pré-requisitos:
* Um ambiente com ROS instalado (Humble), e Micro-ROS.
* Um robô d4nc3r.

Instalação:
1. Clone o repositório para a pasta `src` do seu espaço de trabalho Catkin:
   cd ~/d4nc3r_ws/src/
   git clone https://github.com/Rinaldots/Ros-D4nc3r.git

2. Compile o espaço de trabalho:
   cd ~/d4nc3r_ws/
   colcon build

3. Faça o "source" do arquivo de setup para que o ROS reconheça o novo pacote:
   source ~/d4nc3r_ws/install/setup.bash


COMO USAR
----------
Para iniciar o pacote, inicie 
