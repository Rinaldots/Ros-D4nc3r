# Ros-D4nc3r

<div align="center">
  <img src="https://cdn.simpleicons.org/ros/22314E" height="40" alt="ros logo"  />
  <img width="12" />
  <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" height="40" alt="python logo"  />
</div>

###

<p align="left">D4nc3r e um robô de direção diferencial totalmente open-source, projetado para aplicações artísticas e de baixo custo. Totalmente integrado com o ROS 2, ele serve como uma excelente plataforma para explorar e desenvolver habilidades na robótica. Com seu design aberto, qualquer pessoa pode modificar e personalizar o robô para atender às suas necessidades específicas no campo da arte.</p>

###

###

<br clear="both">

<h4 align="left">Plataforma :<br> º ROS 2: Humble Hawksbill<br> º Sistema Operacional:<br> º Ubuntu 22.04 Jammy Jellyfish<br>  - Dependências : <br>  º ROS 2 -humble <br>  º colcon</h4>

###
<p><strong>Plataforma :</strong></p>
<ul>
<li>
<p>ROS 2: Humble Hawksbill<br>
º Sistema Operacional:<br>
º Ubuntu 22.04 Jammy Jellyfish</p>
</li>
<li>
<p>Dependências :<br>
º ROS 2 - Humble<br>
º colcon<br>
Instalação:</p>
</li>
<li>
<p><strong>Crie o diretório do workspace:</strong><br>
<code>mkdir -p ~/ws/src</code></p>
</li>
<li>
<p><strong>Clone este repositório na pasta <code>src</code>:</strong><br>
<code>cd ~/ws/src git clone https://github.com/Ekumen-OS/andino.git</code></p>
</li>
<li>
<p><strong>Instale as dependências via <code>rosdep</code>:</strong><br>
<code>cd ~/ws rosdep install --from-paths src --ignore-src -i -y</code></p>
</li>
<li>
<p><strong>Compile os pacotes:</strong><br>
<code>colcon build</code></p>
</li>
<li>
<p><strong>Configure o ambiente com os pacotes compilados. Se estiver usando o <code>bash</code>, execute:</strong><br>
<code>source install/setup.bash</code></p>
</li>
</ul>
