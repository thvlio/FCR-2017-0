# fcr2017
=========

Entrada
-------
Para o mapeamento, nenhuma. A trejetória do robô no CIC é pré-programada. Depois do mapeamento, o usuário deve entrar com uma configuração final na forma <x, y, θ> por meio do nó user.

Saida
-----
Uma grade de ocupação feita pelo robô no terminal e um arquivo texto e um arquivo bitmap representando a grade de cada nó do grafo do CIC. Se o nó print for iniciado, várias imagens serão geradas representando o mapeamento conforme o robô foi cumprindo o trajeto.

Dependencias do ROS
-------------------
* roscpp
* std_msgs
* geometry_msgs
* sensor_msgs
* p2os_msgs
* tf

Dependencias fora do ROS
------------------------
* Nenhuma

Algoritmo - detecção e desvio de obstáculos
-------------------------------------------
O algoritmo para a detecção de obstáculos pelo Pioneer foi adaptado a partir dos artigos disponibilizados no moodle. Mais especificamente, grande parte da detecção foi inspirada no método VFH. Como este método é para robôs completamente autônomos, algumas modificações foram feitas nas ideias para que o algoritmo atuasse somente quando o robô estivesse muito próximo de algum obstáculo.
A ideia do algoritmo é criar um histograma polar a partir das medidas do sensor laser. Para simplificar, ao invés de montar um grade de ocupação e depois criar o histograma polar, o algoritmo cria diretamente o histograma polar. O nó itera no vetor de medidas do laser, já dividindo o vetor em setores, e associa um valor calculado com base na distancia e na quantidade de medidas a cada setor.

'''
para j de 0 até num_setores com passo 1
	para j*tam_setor até (j+1)*tam_setor com passo 1
		se (vet_med[i] <> inf)
			c = 1
			se ((i <> 0) e (abs(vet_med[i] - vet_med[i-1]) < 0.1))
				c = c + 2
			se ((i <> 719) e (abs(vet_med[i] - vet_med[i+1]) < 0.1))
				c = c + 2
			m = c * c * (a - b * vet_med[i]);
		senão
			m = 0
		fim se
		polar_obj_dens[j] = polar_obj_dens[j] + m
	fim para
fim para
'''

Veja que a e b são parâmetros arbitrários conforme descrito no artigo sobre VFH. Atualmente, os valores escolhidos sao 3.0 e 0.1, respectivamente. Veja que o algoritmo exposto aqui é somente uma simplificação sem os pormenores como as inicializações de variáveis. Depois de criar o vetor de densidade de objetos para cada setor, o algoritmo cria um vetor de densidade suavizado, seguindo a seguinte função.

'''
para i de 0 até num_setores com passo 1
	para j de -5 até 6 com passo 1 
		se (((i+j) > -1) e ((i+j) < num_setores))
			s_polar_obj_dens[i] = s_polar_obj_dens[i] + (6 - abs(j)) * polar_obj_dens[i+j]
		fim se
	fim para
	s_polar_obj_dens[i] = s_polar_obj_dens[i] / 36
fim para
'''

Agora, o algoritmo encontra uma sequência de setores vazios suficientemente grande para a passagem do robô. O tamanho mínimo dessa sequência foi definido a partir da experimentação com o simulador, assim como várias outras constantes no código. Enquanto o laço é executado, o nó tenta encontrar qual é o vale cuja orientação é a mais próxima da orientação sendo seguida atualmente pelo robô. Para isto, a posição dos setores é transformada num ângulo e o vale mais próximo do alvo é o escolhido.

'''
para i de 0 até num_setores com passo 1
	se (s_polar_obj_dens[i] < valley_thr)
		tam_atual = 0
		para j de i até num_setores com passo 1 e enquanto (s_polar_obj_dens[j] < valley_thr)
			tam_atual = tam_atual + 1
		fim para
		avgAngle = (i+tam_atual/2) * graus_por_setor - 135.0
		difference = avgAngle - atan2 (objetivo.y-pos_robo.y, objetivo.x-pos_robo.x)
		se ((abs(difference) < abs(smallest_difference)) e (tam_atual >= smax))
			closest_dir = avgAngle
			smallest_difference = difference
		fim se
		i = i + tam_atual - 1
	fim se
fim para
'''

O valor de valley_thr foi escolhido com base em experimentação e é algo em torno de 150.0 para uma detecção de vales razoável no prédio do CIC. Veja que graus_por_setor e smax foram definidas previamente no código para valores perto de 3 e 4, respectivamente. Para determinar quando o robô deve parar se seguir seu objetivo e desviar de algum obstáculo, a menor distância detectada pelo sensor foi determinada.

'''
para i de 0 a num_medidas com passo 1
	se (laser[i] < distance)
		distance = laser[i]
	fim se
fim para
'''

Num_medidas representa o número de medidas do sensor laser do Pioneer, que é igual a 720, e laser é o vetor de medidas obtido pelos sensores. Com base nessa menor distância é que se determina se o robô vai continuar seguindo seu objetivo ou vai desviar de algum obstáculo. Agora com um pouco mais de abstração do que os trechos de código anteriores, o comando de velocidade a ser executado pelo robô se dá pelo trecho a seguir.

'''
se (distance < safety_limit)
	se (closest_dir < 2.0)
		girar no sentido horário
	senão se (closest_dir > 2.0)
		girar no sentido anti-horário
	senão
		seguir em linha reta
	fim se
	se (distance < danger_limit)
		se (abs(closest_dir) > 2.0)
			reduzir sua velocidade linear para zero enquanto gira
		fim-se
	fim se
fim se
'''

Os valores de safety_limit, danger_limit foram também determinados experimentando com a simulação, e são em torno de 1.0m e 0.5m.

Algoritmo - planejamento e execução de caminho
----------------------------------------------
Para representar o mapa do prédio internamente na memória, o robô monta um grafo a partir de um arquivo texto. A estrutura do arquivo texto foi previamente definida e os valores das bordas das regiões de cada nó e o peso dos arcos que ligam os nós também foram previamente calculados.

O grafo é representado internamente como um vetor de nós, em que um Nó é uma classe que possui coordenadas que representam sua região e seu centro e possui um vetor de arcos que levam aos nós adjacentes. Como funções, a classe nó possui um método que permite determinar se um ponto está ou não dentro do nó. Além disso, os arcos são representados por uma classe Arco, que possui um membro que indica o peso do arco e um ponteiro para o nó vizinho.

Para percorrer o prédio do CIC e montar um mapa, o robô segue uma trajetória pré-programada. Sempre que o robô chega no centro de um nó ele vira na direção contrária ao próximo alvo para garantir que ele mapeou todo o nó. Isto só é útil porque existe certo overlap entre as grades, e por vezes algumas regiões ficam incertas atrás do robô quando ele entra num nó. A trajetória é lida na forma de um arquivo texto. O arquivo mudava constantemente a depender de que nós deveriam ser mapeados e de onde o robô começava, então pode ser que o arquivo enviado não seja o que foi efetivamente utilizado.

Depois de mapeados os corredores e com o grafo montado internamente, a partir do nó inicial e das coordenadas finais informadas pelo usuário, o menor caminho entre o nó inicial e o nó que contém a posição final é determinado conforme o algoritmo de Dijkstra. O algoritmo pode ser conferido [aqui.](https://en.wikipedia.org/wiki/Dijkstra's_algorithm). Os detalhes da implementação em C++ e as classes implementadas podem ser conferidos nos arquivos fonte.

Para a execução da trajetória, uma função determina um comando de velocidade para o robô enquanto ele tenta ir em direção a um ponto, tal qual o algoritmo abaixo. Difference armazena a diferença entre a guinada (yaw) do robô e a orientação da reta. Limit é o ângulo a partir do qual o robô começa a mudar suas velocidades angular e linear, e ele é algo em torno de 35º no código atual. 

'''
difference = yaw - atan2 (objetivo.y - pos_robo.y, objetivo.x - pos_robo.x)
se (abs(difference) < limit)
	velLinear = maxVelLinear * (1 - abs(difference)/limit)
	velAngular = maxVelAngular * -abs(difference)/limit
senão
	velLinear = 0
	se (difference > 0)
		angVel = -maxVelAngular
	senão
		angVel = maxVelAngular
	fim se
fim se
'''

Depois de receber o comando de velocidade das duas funções, o algoritmo pesa as contribuições de acordo com a distância ao obstáculos mais próximo, tal qual o algoritmo abaixo. Considere que safety_limit e danger_limit são tais quais definidas anteriormente, na seção de desvio de obstáculos.

'''
se (distance < danger_limit)
	vel_robo = vel_evasao
senão se (distance < safety_limit)
	vel_robo = vel_evasao + (vel_alvo - vel_evasao) * (distance - danger_limit) / (safety_limit - danger_limit)
senão
	vel_robo = vel_alvo	
'''

O comando vel_evasao se refere ao comando determinado pela função que trata do desvio de obstáculos e vel_alvo se refere ao comando dado pela função que faz o robô seguir em direção a um ponto alvo.

Visto que havia alguns problemas com o laser durante alguns testes, adicionei uma condição que para o robô se nada estiver sendo publicado no tópico /scan. O instante de tempo em que callback do laser é chamada é registrado e comparado com o instante atual no laço principal. Se a diferença no tempo for maior do que 0.5, a mensagem é muito antiga e o laser possivelmente está com problemas.

Algoritmo - mapeamento
----------------------
Para o mapeamento do prédio, foi criado um tipo Grade. Este tipo armazena uma matrix que representa a grade de ocupação a ser preenchida e algumas informações, como altura e largura da grade. Todas as grades de ocupação são alocadas e pré-preenchidas com uma probabilidade de 0.5 no início do programa.

A construção de grade é feita célula por célula. O procedimento envolve muita matemática vetorial e várias transformações de sistema de coordenadas. De um modo geral, o algoritmo funciona tal qual o pseudo-código abaixo.

'''
para j de 0 até altura_grade com passo 1
	para i de 0 até largura_grade com passo 1
		celula_rel_hokuyo = centro_celula[i][j] - pos_sensor_hokuyo_global
		angulo_rel_hokuyo = atan2 (celula_rel_hokuyo.y, celula_rel_hokuyo.x)
		indice_vetor_laser = (angulo_rel_hokuyo + 135) * 8/3
		para l de indice_vetor_laser-4 até indice_vetor_laser+5 com passo 1
			medida_rel_hokuyo = transforma_polar_para_retangular (laser[l])
			medida_global = medida_rel_hokuyo + pos_sensor_hokuyo_global
			se (medida_global dentro_de (celula[i][j]))
				valor da célula se aproxima de 1
			senão se (l = indice_vetor_laser e laser[k] > tamanho (celula_rel_hokuyo))
				valor da célula se aproxima de 0
			fim se
		fim para
		determina_posicao_robo_na_matriz
	fim para
fim para
'''

Os dados foram adquiridos e um filtro na forma de uma média ponderada móvel foi aplicado para aproximar um dado da célula de 0 ou 1. A cada vez que uma grade é construída, ela é impressa no terminal e uma matriz é publicada no tópico /grids para que o nó print consiga salvar a matriz numa imagem a cada dois segundos.

Como adicional, funções foram criadas para transformar uma grade numa imagem no formato bitmap. A implementação foi feita sem nenhuma blibioteca de processamento de imagens pronta e envolve muito código de baixo-nível que dificilmente pode ser adequadamente expresso em pseudo-código. Sua implementação pode ser conferida nos arquivos fonte.

Como um outro adicional, um nó print foi criado que salva a grade atual incluindo a posição do robô a cada dois segundos. Ele foi usado para gerar as imagens animadas incluídas no trabalho. Basta iniciar o nó numa pasta contendo duas subpastas bitmaps e grids e o nó lê as matrizes publicadas pelo programa principal e as converte em imagens. Sem este nó o programa principal também transforma as grades em imagens, mas só quando o robô passa de um nó para outro.

Descrição dos arquivos
----------------------
include/
    |--> fcr2017
            |--> obstacle_avoidance.h: header da classe que implementa o controle do Pioneer e a leitura dos sensores
			|--> my_classes.h: header que contém todas as classes adicionais criadas para auxiliar o programa principal
			|--> my_constants.h: header que contém todas as constantes usadas no código
			|--> my_functions.h: header que contém todas as funções usadas pelo programa principal
launch/
    |--> teleop.launch: launch file que abre o nó de teleoperação do Pioneer
    |--> pioneer3at.gazebo.launch: launch file que abre o Gazebo e carrega o Pioneer no mundo
src/
	bitmaps/: armazena as imagens geradas pelo programa principal
	grids/: armazena os arquivos texto contendo as grades de ocupação gerados pelo programa
    |--> obstacle_avoidance.cpp: source com o corpo dos métodos da classe ObstacleAvoidance definido em obstacle_avoidance.h
    |--> obstacle_avoidance_node.cpp: nó do ROS que recebe os comandos de teleoperação e controla o Pioneer na simulação
    |--> gazebo_sonar.cpp: código que publica as leituras traseiras e frontais do sonar num vetor só
	|--> user.cpp: nó que permite que o usuário entre com uma configuração final para o robô depois do mapeamento
	|--> print.cpp: nó que salva a grade atual numa imagem a cada dois segundos
	|--> graph.txt: arquivo texto contendo informações sobre os nós do grafo do CIC
	|--> nodes.txt: arquivo texto contendo informações sobre os nós num outro formato
	|--> sequence.txt: arquivo que contém a sequência de nós a ser seguida pelo robô durante a execução
		
CMakeLists.txt: Arquivo de configuração da build deste pacote
package.xml: Arquivo de configuração de dependecias deste pacote e informações de versão, autor e descrição
README.md: Este arquivo
