# Introdução

Este repositório foi criado para expor todos os trabalhos feitos como ponderada para o módulo 6 da Inteli.

# Ponderada 1 - Desenhando com o turtlesim

Esse projeto serve para escrever "MVM" utilizando turtlesim e ROS 2. Ele funciona criando 3 tartarugas, cada uma escrevendo uma letra. Caso queira ver uma demonstração, [clique aqui](https://youtu.be/mZQxjUkAhuc/)

## Funcionalidades

- **Movimento**: Comanda as tartarugas à escrever M, V e M.
- **Serviços de Spawn e Kill**: Adiciona e remove tartarugas.
- **Configuração do Lápis**: Ajusta as características do traço deixado pelas tartarugas.

## Pré-requisitos

- ROS 2
- Python 3.8+
- turtlesim

Caso nao tenha os pré-requisitos listados, recomendo seguir [esse tutorial](https://rmnicola.github.io/m6-ec-encontros/E01/ros) para instalar todas os requisitos.

## Instalação

Aqui é relatado como rodar o projeto:

1. Clone o repositório no terminal:

```bash
      git clone https://github.com/MarioVenturaMedeiros/Ponderadas_M6
```
Este comando clona o repositório no seu computador, permitindo que você tenha acesso local a todos os arquivos do projeto.

2.Compile o código:

```bash
   cd workspace_ponderada
   colcon build
   source install/setup.bash
```

Esses comandos preparam o ambiente para execução do código. cd workspace_ponderada navega até o diretório do projeto. colcon build realiza a compilação do projeto, e source install/setup.bash atualiza o ambiente para usar a versão mais recente compilada.

3.Execute o código:

```bash 
    ros2 run ponderada-s1 ponderada_s1
```

Este comando inicia a execução do projeto. ros2 run é usado para rodar pacotes no ROS 2, especificamente aqui o pacote ponderada-s1 com o executável ponderada_s1.

# Ponderada 2 - Turtlebot teleoperado

Esse projeto serve para controlar um robô turtlebot3 burger. Ele funciona por meio de uma CLI criada, além de mostrar o usuário o estado do robô (se está parado, movendo e a direção que está se movendo). Caso queira ver uma demonstração, [clique aqui](https://www.youtube.com/embed/wLLeXSeqaAc?si=vfi_e0yc8g3TCSf2)

### Pré-requisitos

- ROS2 instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- [Pacote ROS do Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master) instalado no sistema operacional (Linux Ubuntu) da Raspberry do Turtlebot 3 e do computador usado para operá-lo remotamente

- Raspberry do Turtlebot 3 e computador usado para operá-lo remotamente conectados na mesma rede wi-fi

- Git instalado no computador usado para operar o robô remotamente

Caso nao tenha os pré-requisitos listados, recomendo seguir [esse tutorial](https://rmnicola.github.io/m6-ec-encontros/E01/ros) para instalar o ROS2 e [esse tutorial](hhttps://rmnicola.github.io/m6-ec-encontros/setupturtle) para configurar a Raspberry Pi e o pacote do ROS do turtlebot3, além de instalar o pacote do turtlebot3 na Raspberry Pi .

## Instalação

Aqui é relatado como rodar o projeto:

### Sistema operacional da Raspberry

1. No sistema operacional da Raspberry contida no Turtlebot 3 a ser controlado, abra uma janela de terminal e digite os seguintes comandos para limitar a comunicação via ROS a um domínio com ID 7 dentro da rede:

```bash
    echo 'export ROS_DOMAIN_ID=77' >> ~/.bashrc
    source ~/.bashrc
```

Esse comando tem o intuito de restringir a comunicação a somente os robôs configurados com esse ROS_DOMAIN_ID=5. Assim, reduz as chances de o mesmo código interagir com diversos robôs diferentes

2. Na mesma janela de terminal, digite o seguinte comando para iniciar a comunicação entre a Raspberry e o microcontrolador do robô, bem como torná-lo apto a receber comandos de movimentação remotamente:

```bash
    ros2 launch turtlebot3_bringup robot.launch.py
```

### Sistema operacional do computador

3. No sistema operacional do computador que será utilizado para controlar o robô de maneira remota, abra uma janela de terminal no diretório de sua preferência e clone o repositório através do seguinte comando:

```bash
    git clone https://github.com/MarioVenturaMedeiros/Ponderadas_M6
```

4. Na mesma janela de terminal, digite os seguintes comandos para iniciar o build do workspace:

```bash
    cd 2024-1B-T08-EC06-G05/workspace
    colcon build
```

5. Após isso, digite o seguinte comando para habilitar o uso do pacote criado:

`source install/local_setup.bash`

6. Digite os seguintes comandos para limitar a comunicação via ROS a um domínio com ID 77 dentro da rede:

```bash
    echo 'export ROS_DOMAIN_ID=77' >> ~/.bashrc
    source ~/.bashrc
    source install/local_setup.bash
```

7. Por fim, digite o seguinte comando para executar o script responsável por inicializar a CLI para controle de movimentação do robô:

```bash
    ros2 run turtlebot_teleoperado start_moving
```