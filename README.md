# Introdução

Este repositório foi criado para expor todos os trabalhos feitos no módulo 6 da Inteli.

# Ponderada 1 - Desenhando com o turtlesim

Esse projeto serve para escrever "MVM" utilizando turtlesim e ROS 2. Ele funciona criando 3 tartarugas, cada uma escrevendo uma letra. Para uma exemplificão maior, [clique aqui](https://youtu.be/mZQxjUkAhuc/)

## Funcionalidades

- **Movimento**: Comanda as tartarugas à escrever M, V e M.
- **Serviços de Spawn e Kill**: Adiciona e remove tartarugas.
- **Configuração do Lápis**: Ajusta as características do traço deixado pelas tartarugas.

## Pré-requisitos

- ROS 2
- Python 3.8+
- turtlesim

Caso nao tenha essas pré-requisitos recomendo seguir [esse tutorial](https://rmnicola.github.io/m6-ec-encontros/E01/ros)

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
   ros2 ros2 run ponderada-s1 ponderada_s1
```

Este comando inicia a execução do projeto. ros2 run é usado para rodar pacotes no ROS 2, especificamente aqui o pacote ponderada-s1 com o executável ponderada_s1.