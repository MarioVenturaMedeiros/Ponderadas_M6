# **DFS Reactive Navigation Algorithm**

---

## **Visão Geral**

Este projeto implementa um **algoritmo de navegação reativa baseado em Busca em Profundidade (DFS)** para um robô navegando em um ambiente de labirinto. O robô se move pelo grid usando informações sobre os arredores, buscando atingir um alvo enquanto evita obstáculos. O código foi desenvolvido em **C++** com o framework ROS2, utilizando um serviço customizado (`/move_command`) para gerenciar o movimento e os dados de posição.

O algoritmo garante:
- Navegação reativa ao explorar dinamicamente as células livres (`f`).
- Priorização imediata do alvo (`t`) ao detectá-lo.
- Evita loops infinitos ao manter um conjunto de nós visitados.

---

## **Como o Código Funciona**

### **Componentes Principais**

1. **Comunicação com Serviço**:
   - O robô se comunica com o serviço ROS2 `/move_command` para:
     - Obter sua posição atual (`robot_pos`).
     - Receber informações sobre as células ao redor (`up`, `down`, `left`, `right`).

2. **Busca em Profundidade (DFS)**:
   - O robô utiliza uma pilha para rastrear posições a serem exploradas.
   - Explora o máximo possível em uma direção antes de retroceder e tentar caminhos alternativos.
   - A pilha garante que o robô pode voltar para posições anteriores e explorar completamente o labirinto.

3. **Detecção do Alvo**:
   - Se o robô detecta o alvo (`t`) em qualquer direção, ele se move imediatamente para ele.
   - Outras direções são ignoradas quando o alvo é encontrado.

4. **Nós Visitados**:
   - O robô mantém um conjunto de posições visitadas para evitar revisitações desnecessárias, prevenindo loops infinitos.

---

### **Fluxo do Código**

1. **Inicialização**:
   - O nó ROS2 é criado e o serviço `/move_command` é inicializado.

2. **Solicitações ao Serviço**:
   - O robô envia requisições ao serviço para:
     - Obter sua posição atual.
     - Receber informações sobre as células em todas as quatro direções (`f` para livre, `t` para alvo, `b` para bloqueado).

3. **Exploração com DFS**:
   - Para cada posição:
     - O robô adiciona vizinhos livres (`f`) à pilha para futura exploração.
     - Se o alvo (`t`) for detectado, a pilha é limpa e o robô se move diretamente ao alvo.

4. **Movimento**:
   - O robô retira a próxima posição da pilha e envia uma solicitação de movimento ao serviço.
   - Se nenhum vizinho ou caminho estiver disponível, o robô relata estar "preso".

5. **Verificação do Alvo**:
   - O robô continuamente verifica se sua posição coincide com o alvo. Se sim, ele para e anuncia sucesso.

---

## **Por Que Não os Algoritmos Bug?**

Inicialmente, os algoritmos **Bug1**, **Bug2** e **Tangent Bug** foram implementados e testados. Esses algoritmos falharam em casos específicos:
- **Loops em Paredes Fechadas**:
  - Quando o robô encontrava paredes finas ou cantos, ele entrava em um loop infinito, movendo-se para frente e para trás sem progresso.
- **Problemas ao Seguir Paredes**:
  - O robô às vezes falhava em retornar ao modo de busca pelo alvo após seguir paredes, levando a caminhos ineficientes ou incorretos.

Devido a essas limitações, o algoritmo baseado em DFS foi escolhido por sua simplicidade, robustez e capacidade de explorar todos os caminhos possíveis de forma sistemática.

---

## **O Que o Código Faz**

1. **Navega no Labirinto**:
   - O robô explora dinamicamente o labirinto usando DFS, visitando sistematicamente cada posição possível até alcançar o alvo.

2. **Prioriza o Alvo**:
   - Ao detectar o alvo (`t`) em uma célula vizinha, o robô se move imediatamente para ele, ignorando outras direções.

3. **Evita Loops**:
   - Ao rastrear posições visitadas, o robô evita revisitar células, prevenindo loops infinitos.

4. **Lida com Labirintos Complexos**:
   - A abordagem DFS garante que, mesmo em labirintos complexos, o robô eventualmente encontrará um caminho para o alvo, se existir.

---

## **Como Executar o Código**

1. **Configurar o Ambiente ROS2**:
   - Instale o ROS2 e garanta que o workspace esteja devidamente configurado e ativado.

2. **Executar o Robô**:
   - Inicie o serviço ROS2 `/move_command` e execute o código de navegação do robô:
     ```bash
     ros2 run reactive_navigation reactive_navigation
     ```

3. **Monitorar os Logs**:
   - Verifique os logs no terminal para acompanhar em tempo real a posição do robô, detecção do alvo e decisões de movimento.

---

## **Possíveis Melhorias**

- **Otimização**:
  - Incorporar algoritmos baseados em heurísticas, como A*, para navegação mais rápida em grids maiores.
- **Melhoria na Detecção do Alvo**:
  - Usar sensores avançados para detectar alvos em uma faixa maior, reduzindo o tempo de exploração.
- **Ajuste Dinâmico de Caminho**:
  - Permitir ajustes dinâmicos do caminho do robô com base em alterações ao vivo no labirinto (e.g., novos obstáculos).

---

## **Conclusão**

O algoritmo de navegação reativa baseado em DFS implementado neste projeto navega com sucesso no labirinto e resolve as limitações dos algoritmos Bug. Combinando exploração sistemática com detecção em tempo real do alvo, o robô atinge uma navegação eficiente e confiável em ambientes complexos.
