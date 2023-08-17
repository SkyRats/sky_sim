BAIXAR ROS E MAVROS / CONECTAR POR SSH / SOLUÇÃO DE ERROS

########

Comandos para baixar o ros e mavros na raspberry pi:

Arrumar o horário:

```
sudo date -s "aaaa/mm/dd hh:mm:ss"
```

Instalar o ros noetic:

```
sudo apt update
sudo apt install ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


Instalar a mavros:

```
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

########

Pra ligar o hotspot do pc:

```
cd WifiConnection/
bash WIFICONNECTION.sh start
```

Pra conectar a rasp no Hotspot:

```
nmtui ("Activate a connection", procurar o hotspot)
```

Pra descobrir o ip da rasp:

```
ifconfig (na rasp)
- olhar o ip em
  wlan0: inet 10.42.0.65
  (em geral o último número é o que muda, no caso o 65)
```

Pra conectar por ssh (no pc):

```
ssh ubuntu@10.42.0.65
```

Caso um warning apareça, siga o comando ssh-keygen -f " ... etc

```
login: ubuntu
senha: caio123
```

#######

rodando a mavros:

```
roslaunch mavros apm.launch
```

Possível erro:

```
[FATAL] [1679940035.186375341]: UAS: GeographicLib exception:
File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm |
Run install_geographiclib_dataset.sh script in order to install Geoid Model dataset!
```

Solução:

```
cd /opt/ros/noetic/lib/mavros
sudo bash install_geographiclib_datasets.sh
```

Possível erro:

```
Não conectou (porta serial errada)
Edite o apm.launch para utilizar o dispositivo ttyUSB0
```

Solução:

```
cd /opt/ros/noetic/share/mavros/launch
sudo nano apm.launch
na linha 5, troque o default do fcu_url por "/dev/ttyUSB0:921600"
```
